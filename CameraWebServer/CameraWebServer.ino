#include "esp_camera.h"
#include <WiFi.h>

// ===== Select camera model =====
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// ===== Wi-Fi credentials =====
const char* ssid     = "iQOO Z3 5G";
const char* password = "chaitrarane";

void startCameraServer();  // Provided by camera server example

// ===== UART1 for Arduino Communication =====
// UART1 mapped as: TX = GPIO1, RX = GPIO3
HardwareSerial ArduinoSerial(1);

// ===== TCP server for receiving commands from laptop =====
WiFiServer controlServer(8765);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("Starting camera...");

  // ===== Activate UART TX (GPIO1 -> Arduino RX) =====
  // Parameters: (baud, config, rxPin, txPin)
  ArduinoSerial.begin(9600, SERIAL_8N1, 3, 1);  // RX = GPIO3, TX = GPIO1

  // ===== CAMERA CONFIG =====
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;   // Lower = better quality
  config.fb_count     = 1;

  // ===== Init camera =====
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // ===== Camera sensor tweaks =====
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);          // Flip vertically
    s->set_brightness(s, 1);     // Slightly brighter
    s->set_saturation(s, -2);    // Desaturate
    s->set_framesize(s, FRAMESIZE_QVGA);  // 320x240
  }

  // ===== Wi-Fi Connect =====
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  IPAddress ip = WiFi.localIP();
  Serial.print("IP address: ");
  Serial.println(ip);

  // ===== PRINT CAMERA + CONTROL URLs =====
  Serial.println();
  Serial.println("====================================");
  Serial.print("Camera Capture URL: http://");
  Serial.print(ip);
  Serial.println("/capture");
  Serial.print("Control TCP Port: ");
  Serial.print(ip);
  Serial.println(":8765");
  Serial.println("====================================");
  Serial.println();

  // ===== Start Camera HTTP Server =====
  startCameraServer();

  // ===== Start TCP Command Server =====
  controlServer.begin();
  Serial.println("TCP control server started on port 8765.");
  Serial.println("Waiting for PID commands from laptop...");
}

// ==== Forward command to Arduino (UART TX=GPIO1) ====
void forwardToArduino(const String &cmd) {
  if (cmd.length() == 0) return;

  char c0 = cmd.charAt(0);

  // Allow only A / F / S commands
  if (c0 == 'A' || c0 == 'F' || c0 == 'S') {
    String out = cmd;
    if (!out.endsWith("\n")) {
      out += "\n";
    }

    ArduinoSerial.print(out);
    Serial.print("-> Arduino: ");
    Serial.print(out);
  } else {
    Serial.print("Invalid command ignored: ");
    Serial.println(cmd);
  }
}

void loop() {
  // ==== Handle incoming TCP clients ====
  WiFiClient client = controlServer.available();
  if (client) {
    Serial.println("TCP client connected.");

    String buf = "";
    unsigned long lastActivity = millis();

    while (client.connected()) {
      while (client.available()) {
        char c = client.read();
        lastActivity = millis();

        if (c == '\r') continue;

        if (c == '\n') {
          String cmd = buf;
          cmd.trim();
          forwardToArduino(cmd);
          client.print("OK\n");
          buf = "";
        } else {
          buf += c;
          // Prevent buffer from growing too large
          if (buf.length() > 64) {
            buf = buf.substring(buf.length() - 64);
          }
        }
      }

      // Timeout if no activity for 6 seconds
      if (millis() - lastActivity > 6000) {
        break;
      }

      delay(1);
    }

    Serial.println("TCP client disconnected.");
    client.stop();
  }

  delay(10);
}
