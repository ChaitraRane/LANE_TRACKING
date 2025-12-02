#!/usr/bin/env python3
import cv2
import urllib.request
import numpy as np
import socket
import time
import select
import sys

CAM_URL = "http://10.194.118.167/capture"
TCP_HOST = "10.194.118.167"
TCP_PORT = 8765

SAMPLE_FILE = "file:///mnt/data/c195be2b-d918-4bea-8ff9-126e5325c308.png"
TARGET_WIDTH = 640
TARGET_HEIGHT = 480
w, h = TARGET_WIDTH, TARGET_HEIGHT

NUM_ROIS = 4
roi_h = h // NUM_ROIS

DEFAULT_STRAIGHT = 90
MIN_SERVO = 40
MAX_SERVO = 130
BASE_SPEED = 150.0
MIN_SPEED = 40.0

Kp = 0.35
Ki = 0.002
Kd = 0.6

# behaviour tuning
STEERING_SCALE = 1.2
DERIVATIVE_ONLY_ABOVE_PIX = 70
INTEGRAL_WINDUP_LIMIT = 3000.0


DT_MAX = 0.2
ALPHA_ERR = 0.25
SERVO_SMOOTH = 0.0

SEND_RATE_HZ = 20
OVERLAY_ALPHA = 0.55
MIN_AREA_FRACTION = 0.005

OPEN_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
CLOSE_K = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

# optional rotate flags
ROTATE_180 = False
FLIP_VERTICAL = True
FLIP_HORIZONTAL = False
ROTATE_90_IF_TALL = True

DEBUG = True
COMMAND_NEWLINE = "\n"
FORCE_ROI = None

ROI_WEIGHTS = [0.5, 0.3, 0.15, 0.05]
assert len(ROI_WEIGHTS) == NUM_ROIS, "ROI_WEIGHTS length must equal NUM_ROIS"


SPEED_CONFIDENCE_MIN = 0.2
SPEED_CONFIDENCE_MAX = 1.0

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def nothing(x):
    pass

def setup_hsv_trackbar():
    cv2.namedWindow("HSV Tracker", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Tracker", 420, 260)
    cv2.createTrackbar("LH", "HSV Tracker", 0, 179, nothing)
    cv2.createTrackbar("LS", "HSV Tracker", 0, 255, nothing)
    cv2.createTrackbar("LV", "HSV Tracker", 0, 255, nothing)
    cv2.createTrackbar("UH", "HSV Tracker", 179, 179, nothing)
    cv2.createTrackbar("US", "HSV Tracker", 255, 255, nothing)
    cv2.createTrackbar("UV", "HSV Tracker", 50, 255, nothing)

def get_hsv_values():
    LH = cv2.getTrackbarPos("LH", "HSV Tracker")
    LS = cv2.getTrackbarPos("LS", "HSV Tracker")
    LV = cv2.getTrackbarPos("LV", "HSV Tracker")
    UH = cv2.getTrackbarPos("UH", "HSV Tracker")
    US = cv2.getTrackbarPos("US", "HSV Tracker")
    UV = cv2.getTrackbarPos("UV", "HSV Tracker")
    lower = np.array([LH, LS, LV])
    upper = np.array([UH, US, UV])
    return lower, upper

def fetch_frame_from_url(url, timeout=5):
    try:
        if url.startswith("file://"):
            path = url[len("file://"):]
            img = cv2.imread(path)
            if img is None:
                return None
            return cv2.resize(img, (w, h))
        resp = urllib.request.urlopen(url, timeout=timeout)
        data = resp.read()
        imgnp = np.frombuffer(data, np.uint8)
        img = cv2.imdecode(imgnp, cv2.IMREAD_COLOR)
        if img is None:
            return None
        return cv2.resize(img, (w, h))
    except Exception as e:
        if DEBUG:
            print("fetch_frame_from_url error:", e)
        return None

def tcp_connect(host, port, timeout=1.0):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((host, port))
        s.settimeout(0.8)
        if DEBUG:
            print(f"[TCP] connected to {host}:{port}")
        return s
    except Exception as e:
        if DEBUG:
            print(f"[TCP] connect failed: {e}")
        try:
            s.close()
        except Exception:
            pass
        return None

def _try_recv_ack(sock, timeout_s):
    try:
        r, _, _ = select.select([sock], [], [], timeout_s)
        if r:
            try:
                return sock.recv(128)
            except Exception:
                return None
        return None
    except Exception:
        return None

def send_tcp_command(sock, angle=None, speed=None, expect_ack=False, ack_timeout=0.1):
    try:
        if angle is not None:
            cmd = f"A{int(angle)}{COMMAND_NEWLINE}".encode()
            if DEBUG: print("[TCP SEND]", cmd)
            sock.sendall(cmd)
            if expect_ack:
                _ = _try_recv_ack(sock, ack_timeout)
        if speed is not None:
            cmd = f"F{int(speed)}{COMMAND_NEWLINE}".encode()
            if DEBUG: print("[TCP SEND]", cmd)
            sock.sendall(cmd)
            if expect_ack:
                _ = _try_recv_ack(sock, ack_timeout)
        return True
    except Exception as e:
        if DEBUG:
            print("[TCP SEND ERROR]", e)
        return False

def main():
    global Kp, Ki, Kd, STEERING_SCALE, SERVO_SMOOTH

    setup_hsv_trackbar()
    cv2.setTrackbarPos("LH", "HSV Tracker", 0)
    cv2.setTrackbarPos("LS", "HSV Tracker", 0)
    cv2.setTrackbarPos("LV", "HSV Tracker", 0)
    cv2.setTrackbarPos("UH", "HSV Tracker", 179)
    cv2.setTrackbarPos("US", "HSV Tracker", 255)
    cv2.setTrackbarPos("UV", "HSV Tracker", 50)

    sock = None
    last_send_time = 0.0
    prev_error = 0.0
    smoothed_error = 0.0
    integral = 0.0
    prev_time = time.time()
    prev_servo = DEFAULT_STRAIGHT
    min_area_px = int(MIN_AREA_FRACTION * (w * roi_h))

    print("Starting main loop.")
    print("Camera URL:", CAM_URL)
    print(f"Control TCP: {TCP_HOST}:{TCP_PORT}")
    print("Press 'q' in the image window to quit.")

    while True:
        frame = fetch_frame_from_url(CAM_URL)
        if frame is None:
            frame = fetch_frame_from_url(SAMPLE_FILE)
            if frame is None:
                if DEBUG:
                    print("No frame available. Retrying...")
                time.sleep(0.2)
                continue

        # optional rotation and flipping
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        if FLIP_VERTICAL:
            frame = cv2.flip(frame, 0)
        if FLIP_HORIZONTAL:
            frame = cv2.flip(frame, 1)
        if ROTATE_90_IF_TALL and frame.shape[0] > frame.shape[1]:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        frame = cv2.resize(frame, (w, h))
        vis = frame.copy()

        lower_black, upper_black = get_hsv_values()

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_black, upper_black)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_K)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_K)

        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        clean = np.zeros_like(mask)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= min_area_px:
                clean[labels == i] = 255
        mask = clean

        overlay = vis.copy()
        overlay[mask > 0] = (0, 200, 0)
        vis = cv2.addWeighted(overlay, OVERLAY_ALPHA, vis, 1 - OVERLAY_ALPHA, 0)

        roi_errors = [None] * NUM_ROIS  # None if not found, else signed px error (cx - center)
        roi_centroids = [None] * NUM_ROIS
        detected_count = 0
        weighted_error_sum = 0.0
        weight_sum_used = 0.0

        # FORCE_ROI still available for manual testing — if set, we still use all ROIs but mark target.
        target_roi_index = FORCE_ROI if FORCE_ROI is not None else 1  # keep default target for display

        for i in range(NUM_ROIS):
            # compute ROI coordinates; i==0 bottom-most (keeps original convention)
            y1 = h - (i + 1) * roi_h
            y2 = h - i * roi_h
            roi = mask[y1:y2, :]
            contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            error = None
            centroid = None
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) >= min_area_px:
                    M = cv2.moments(largest)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cx_global = cx
                        cy_global = int(M["m01"] / M["m00"]) + y1
                        centroid = (cx_global, cy_global)
                        error = cx_global - (w // 2)
                        # draw centroid and ROI box for visibility
                        cv2.circle(vis, centroid, 5, (0, 0, 255), -1)
                        cv2.line(vis, (w // 2, y1 + roi_h // 2), centroid, (255, 0, 0), 2)
                        cv2.rectangle(vis, (0, y1), (w, y2), (0, 255, 0), 1)
            roi_errors[i] = error
            roi_centroids[i] = centroid

            # annotate
            txt = f"ROI {i+1}: "
            txt += "N/A" if error is None else f"{error:+d}px"
            cv2.putText(vis, txt, (w - 220, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255) if error is None else (255, 255, 0), 1, cv2.LINE_AA)

            if error is not None:
                detected_count += 1
                w_i = ROI_WEIGHTS[i]
                weighted_error_sum += w_i * float(error)
                weight_sum_used += w_i

        combined_error_px = None
        if weight_sum_used > 0:
            combined_error_px = weighted_error_sum / weight_sum_used


        now = time.time()
        dt = now - prev_time
        if dt <= 0:
            dt = 1e-3
        if dt > DT_MAX:
            dt = DT_MAX
        prev_time = now

        if combined_error_px is not None:
            e = float(combined_error_px)
            smoothed_error = ALPHA_ERR * e + (1 - ALPHA_ERR) * smoothed_error
            integral += smoothed_error * dt
            integral = clamp(integral, -INTEGRAL_WINDUP_LIMIT, INTEGRAL_WINDUP_LIMIT)
            derivative = (smoothed_error - prev_error) / dt
            use_derivative = abs(e) > DERIVATIVE_ONLY_ABOVE_PIX

            correction = (Kp * smoothed_error +
                          Ki * integral +
                          (Kd * derivative if use_derivative else 0.0))

            angle_offset = correction * STEERING_SCALE
            raw_angle = DEFAULT_STRAIGHT + angle_offset
            angle = int(clamp(raw_angle, MIN_SERVO, MAX_SERVO))

            smooth_angle = int(clamp(prev_servo * SERVO_SMOOTH + angle * (1 - SERVO_SMOOTH),
                                     MIN_SERVO, MAX_SERVO))
            prev_servo = smooth_angle

            confidence = detected_count / float(NUM_ROIS)
            bottom_present = roi_errors[0] is not None
            if not bottom_present and detected_count > 0:
                confidence *= 0.6

            speed_fraction = SPEED_CONFIDENCE_MIN + (SPEED_CONFIDENCE_MAX - SPEED_CONFIDENCE_MIN) * confidence
            speed = int(clamp(BASE_SPEED * speed_fraction, 0, 255))

            if detected_count == 0:
                speed = 0

            if roi_errors[0] is not None and abs(roi_errors[0]) > (w * 0.35):
                speed = int(speed * 0.5)

            if (now - last_send_time) >= (1.0 / SEND_RATE_HZ):
                last_send_time = now
                if sock is None:
                    sock = tcp_connect(TCP_HOST, TCP_PORT)
                if sock:
                    ok = send_tcp_command(sock, angle=smooth_angle, speed=speed, expect_ack=False)
                    if not ok:
                        try:
                            sock.close()
                        except Exception:
                            pass
                        sock = None

            cv2.putText(vis, f"CombErr:{e:.1f}px  Angle:{smooth_angle}  Speed:{speed}  Conf:{confidence:.2f}",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            prev_error = smoothed_error

        else:
            # no detection anywhere -> stop (or optionally try recovery turn)
            cv2.putText(vis, "ALL ROIS: N/A - stopping", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if (now - last_send_time) >= (1.0 / SEND_RATE_HZ):
                last_send_time = now
                if sock is None:
                    sock = tcp_connect(TCP_HOST, TCP_PORT)
                if sock:
                    try:
                        if DEBUG: print("[TCP SEND] S (stop)")
                        sock.sendall(f"S{COMMAND_NEWLINE}".encode())
                        _ = _try_recv_ack(sock, 0.05)
                    except Exception as e:
                        if DEBUG: print("[TCP STOP SEND ERROR]", e)
                        try:
                            sock.close()
                        except Exception:
                            pass
                        sock = None

        # show images
        cv2.imshow("ESP32-CAM Line Follower (multi-ROI)", vis)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            if sock:
                try:
                    if DEBUG: print("[TCP SEND] S (stop) before quit")
                    sock.sendall(f"S{COMMAND_NEWLINE}".encode())
                except Exception:
                    pass
                try:
                    sock.close()
                except Exception:
                    pass
            break

    cv2.destroyAllWindows()
    if sock:
        try:
            sock.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()