#include <Servo.h>

// Motor Pins
#define ENA 5  
#define IN1 7
#define IN2 8
#define ENB 6  
#define IN3 9
#define IN4 10

// Servo
Servo steeringServo;
int servoPin = 11;

const int STRAIGHT_ANGLE = 90;

// ----- New state variables -----
int desiredSpeed = 150;        // default desired speed (0-255). change as you like.
int currentSpeed = 0;          // actual PWM applied to motors (may be reduced on sharp turns)
int servoAngle = STRAIGHT_ANGLE;

const float SHARP_REDUCTION_FACTOR = 0.5;  // reduce to 50% on sharp turns
const int SHARP_TURN_THRESHOLD_DEG = 25;   // threshold from straight to consider "sharp" (tunable)

bool movingForward = false;
bool movingReverse = false;

// --------------------------------
String inputString = "";  
bool stringComplete = false;

void setup() {
  Serial.begin(9600);

  // Motor setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  steeringServo.attach(servoPin);
  steeringServo.write(STRAIGHT_ANGLE);

  // initialize speeds
  desiredSpeed = 150; // default; you can change or set via F command
  currentSpeed = 0;
  stopMotors();
}

void loop() {
  if (stringComplete) {
    inputString.trim();
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  // nothing else needed in loop; motor speed updates happen inside processCommand when needed
}

// Serial receiver (line terminated)
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processCommand(String cmd) {
  if (cmd.length() == 0) return;

  // Servo command (A<number>) — update servoAngle but DO NOT change servo logic otherwise
  if (cmd.startsWith("A")) {   
    int angle = cmd.substring(1).toInt();
    angle = constrain(angle, 0, 180);
    servoAngle = angle;                       // track angle for sharp-turn detection
    steeringServo.write(angle);
    // If we are moving, immediately re-evaluate motor speed based on new servo angle
    applySpeedAccordingToTurn();
  }

  // Forward: set desiredSpeed (but actual motors will run at currentSpeed which may be reduced)
  else if (cmd.startsWith("F")) {   
    String numPart = cmd.substring(1);
    if (numPart.length() > 0) {
      int sp = numPart.toInt();
      sp = constrain(sp, 0, 255);
      desiredSpeed = sp;      // store in variable as requested
    }
    // set moving flags and apply computed currentSpeed
    movingForward = true;
    movingReverse = false;
    applySpeedAccordingToTurn();
  }

  // Reverse: set desiredSpeed and move reverse
  else if (cmd.startsWith("B")) {   
    String numPart = cmd.substring(1);
    if (numPart.length() > 0) {
      int sp = numPart.toInt();
      sp = constrain(sp, 0, 255);
      desiredSpeed = sp;
    }
    movingReverse = true;
    movingForward = false;
    // For reverse, we can also reduce speed on sharp turns (optional). Here we'll apply same reduction.
    applySpeedAccordingToTurn();
  }

  // Stop
  else if (cmd.startsWith("S")) {  
    movingForward = false;
    movingReverse = false;
    stopMotors();
  }
}

// Compute currentSpeed from desiredSpeed and servoAngle, then command motor driver accordingly
void applySpeedAccordingToTurn() {
  bool isSharp = abs(servoAngle - STRAIGHT_ANGLE) > SHARP_TURN_THRESHOLD_DEG;
  if (isSharp) {
    currentSpeed = (int)(desiredSpeed * SHARP_REDUCTION_FACTOR);
  } else {
    currentSpeed = desiredSpeed;
  }

  // If currently commanded to move forward/reverse, apply the motor commands with currentSpeed
  if (movingForward) {
    moveForward(currentSpeed);
  } else if (movingReverse) {
    moveReverse(currentSpeed);
  } else {
    // not moving; ensure motors are stopped
    stopMotors();
  }
}

// ---------- Motor functions ----------
void moveForward(int pwmSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, pwmSpeed);
  analogWrite(ENB, pwmSpeed);
}

void moveReverse(int pwmSpeed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, pwmSpeed);
  analogWrite(ENB, pwmSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}