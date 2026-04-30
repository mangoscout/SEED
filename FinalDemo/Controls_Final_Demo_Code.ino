// Team 10 - SEED Lab Code
// Controls Members: Lydia Tan and Kyle Ha
// Course: EENG350
// Date: 4/30/2026

/*
Final Demo: Marker Search, Alignment, Drive Forward, Turn 90 Degrees

Program behavior:
 - The robot searches for a visual marker using incremental turns if none is detected.
 - Once a marker is detected, it aligns to a target angle (~20 degrees) using feedback from a Raspberry Pi.
 - After alignment, the robot drives forward toward the marker.
 - The robot stops when the Pi-reported distance is approximately 1 meter (with tolerance).
 - If the robot fails to reach the marker (timeout), it restarts the cycle without turning.
 - After stopping, the robot reads the marker color from the Pi:
     - Green (1): turn RIGHT 90 degrees
     - Red (0): turn LEFT 90 degrees
     - No color (-1): do NOT turn (0 degrees)
 - The robot pauses briefly after the turn decision, then repeats the cycle.

Key improvements:
 - Prevents stale arrow/color data from triggering incorrect turns
 - Uses safe copying of Pi data (interrupt-protected) to avoid inconsistent readings
 - Adds tolerance to stopping distance to avoid missing exact 1.0 m threshold
 - Slows down turns near completion for improved consistency
 - Includes right motor PWM compensation for straighter driving

Hardware notes:
 - I2C communication is used to receive marker data from a Raspberry Pi.
     - GPIO 4: A4
     - GPIO 5: A5
     - GND: GND
 - Encoder A channels are interrupt-driven for tracking wheel motion
 - Motor driver enable pin must be HIGH for motion
 - Direction inversion flags may be required depending on wiring
*/

#include <Wire.h>
#include <Arduino.h>

// I2C SETTINGS
// Pi sends:
// color,turn_cmd,distance,angle
// color:
//   1  = green = turn RIGHT
//   0  = red   = turn LEFT
//  -1  = no color = turn 0 degrees / stop
 
#define SLAVE_ADDRESS 0x08
 
// MOTOR PINS
int ENABLE_PIN = 4;
int L_MOTOR_SIGN = 8;
int L_MOTOR_PWM  = 10;
int R_MOTOR_SIGN = 7;
int R_MOTOR_PWM  = 9;
 
// ENCODER PINS
int L_ENC_A = 2;
int R_ENC_A = 3;
int enc1PinB = 5;
int enc2PinB = 6;

 
// MOTOR SPEED SETTINGS
int motor_pwm = 60; // search turning speed
int align_pwm = 80; // alignment turning speed
int turn90_pwm = 60; // slower final turn for consistency
int forward_pwm = 80; // forward drive speed

// If robot curves right while driving forward
int RIGHT_PWM_BOOST = 10;

// ENCODER COUNTS
 
long left_count = 0;
long right_count = 0;

int left_dir_sign = -1;
int right_dir_sign = +1;

// ROBOT GEOMETRY
float wheel_radius_m = 0.0873125;
float wheel_base_m   = 0.15;
long counts_per_rev  = 3200;

// Lower this if robot over-turns.
// Raise it if robot under-turns.
float TURN_GAIN = 1.10;

// MOTOR DIRECTION INVERSION
bool L_DIR_INVERT = true;
bool R_DIR_INVERT = false;

// PI DATA
bool marker_detected = false;
bool new_packet_ready = false;

float beacon_distance_m = -1.0;
float beacon_angle_deg = -999.0;

int color_from_pi = -1;
int turn_cmd_from_pi = 0;

// ALIGNMENT SETTINGS
float TARGET_ANGLE = 20.0;
float MIN_ALIGN_RANGE = 0.0;
float MAX_ALIGN_RANGE = 37.0;
float ANGLE_TOLERANCE = 1.0;

int ALIGN_STEP_DEG = 1;
int SEARCH_STEP_DEG = 20;

// FORWARD SETTINGS
// Stop when Pi distance is about 1 meter.
// 1.05 gives a little tolerance so it does not miss 1.00.
float FORWARD_DISTANCE_M = 4.5 * 0.3048;
float STOP_DISTANCE_FROM_PI_M = 1.05;
// Pause after final turn decision
unsigned long AFTER_TURN_PAUSE_MS = 3000;

// I2C LINE BUFFER
// Pi sends chunked writes, so we collect characters
// until we receive '\n'.
String i2cLineBuffer = "";

// MOTOR HELPER
void setMotor(int signPin, int pwmPin, int pwm, bool forward) {
  bool cmdForward = forward;

  // Account for wiring differences
  if (signPin == L_MOTOR_SIGN && L_DIR_INVERT) cmdForward = !cmdForward;
  if (signPin == R_MOTOR_SIGN && R_DIR_INVERT) cmdForward = !cmdForward;

  int adjustedPwm = pwm;

  // Boost right wheel to help robot drive straighter
  if (signPin == R_MOTOR_SIGN) {
    adjustedPwm = pwm + RIGHT_PWM_BOOST;
  }

  digitalWrite(signPin, cmdForward ? HIGH : LOW);
  analogWrite(pwmPin, constrain(adjustedPwm, 0, 255));
}

void stopMotors() {
  analogWrite(L_MOTOR_PWM, 0);
  analogWrite(R_MOTOR_PWM, 0);
}

// ENCODER INTERRUPTS
void encoder1ISR() {
  left_count += left_dir_sign;
}

void encoder2ISR() {
  right_count += right_dir_sign;
}

// CLEAR PI DATA
// This prevents stale marker/color/arrow data.
 
void clearMarkerData() {
  noInterrupts();
  marker_detected = false;
  new_packet_ready = false;
  beacon_distance_m = -1.0;
  beacon_angle_deg = -999.0;
  color_from_pi = -1;
  turn_cmd_from_pi = 0;
  interrupts();
}

// PARSE PI PACKET
// Expected format:
// color,turn_cmd,distance,angle
//
// Example:
// 1,90,1.200,20.0
// 0,-90,1.000,20.0
// -1,0,1.000,20.0
 
void parsePiPacket(String line) {
  line.trim();
  if (line.length() == 0) return;

  int firstComma  = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  int thirdComma  = line.indexOf(',', secondComma + 1);

  if (firstComma <= 0 || secondComma <= 0 || thirdComma <= 0) {
    Serial.print("Bad packet: ");
    Serial.println(line);
    return;
  }

  String colorStr = line.substring(0, firstComma);
  String turnStr  = line.substring(firstComma + 1, secondComma);
  String distStr  = line.substring(secondComma + 1, thirdComma);
  String angleStr = line.substring(thirdComma + 1);

  int newColor = colorStr.toInt();
  int newTurnCmd = turnStr.toInt();
  float newDist = distStr.toFloat();
  float newAngle = angleStr.toFloat();

  noInterrupts();

  beacon_distance_m = newDist;
  beacon_angle_deg = newAngle;

  // Only allow valid color values.
  // This prevents old green/red values from being reused.
  if (newColor == 1 || newColor == 0 || newColor == -1) {
    color_from_pi = newColor;
  } else {
    color_from_pi = -1;
  }

  // Keep turn command for debugging, but final turning uses color.
  if (newTurnCmd == -90 || newTurnCmd == 90) {
    turn_cmd_from_pi = newTurnCmd;
  } else {
    turn_cmd_from_pi = 0;
  }

  if (newDist > 0.0 && newAngle > -180.0 && newAngle < 180.0) {
    marker_detected = true;
  } else {
    marker_detected = false;
    color_from_pi = -1;
    turn_cmd_from_pi = 0;
  }

  new_packet_ready = true;

  interrupts();

  Serial.print("Parsed -> color: ");
  Serial.print(newColor);
  Serial.print(" | turn_cmd: ");
  Serial.print(newTurnCmd);
  Serial.print(" | dist: ");
  Serial.print(newDist, 3);
  Serial.print(" m | angle: ");
  Serial.println(newAngle, 2);
}

 
// I2C RECEIVE EVENT
// Builds one full line from Pi, then parses it.
 
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = (char)Wire.read();

    if (c == '\0') continue;
    if (c == '\r') continue;

    if (c == '\n') {
      String completeLine = i2cLineBuffer;
      i2cLineBuffer = "";
      parsePiPacket(completeLine);
    } else {
      i2cLineBuffer += c;

      if (i2cLineBuffer.length() > 64) {
        i2cLineBuffer = "";
      }
    }
  }
}

// TURN FUNCTION
// Uses encoders to turn a target number of degrees.
// Slows down near the end to reduce overshoot.
 
void turnDegrees(char dir, int deg, int pwmSpeed, bool stopOnMarker = false) {
  deg = constrain(deg, -360, 360);

  if (deg < 0) {
    deg = abs(deg);
    dir = (dir == 'L') ? 'R' : 'L';
  }

  if (deg == 0) return;

  noInterrupts();
  left_count = 0;
  right_count = 0;
  interrupts();

  float factor = (wheel_base_m / (2.0 * wheel_radius_m)) * ((float)deg / 360.0);
  long targetCounts = (long)(TURN_GAIN * factor * counts_per_rev);

  if (targetCounts < 5) targetCounts = 5;

  if (toupper(dir) == 'L') {
    left_dir_sign  = +1;
    right_dir_sign = -1;
    setMotor(L_MOTOR_SIGN, L_MOTOR_PWM, pwmSpeed, true);
    setMotor(R_MOTOR_SIGN, R_MOTOR_PWM, pwmSpeed, false);
  } else {
    left_dir_sign  = -1;
    right_dir_sign = +1;
    setMotor(L_MOTOR_SIGN, L_MOTOR_PWM, pwmSpeed, false);
    setMotor(R_MOTOR_SIGN, R_MOTOR_PWM, pwmSpeed, true);
  }

  unsigned long start = millis();

  while (true) {
    bool seenMarker;

    noInterrupts();
    seenMarker = marker_detected;
    long lc = abs(left_count);
    long rc = abs(right_count);
    interrupts();

    if (stopOnMarker && seenMarker) break;

    long avg = (lc + rc) / 2;
    long remaining = targetCounts - avg;

    // Slow down near the end of the turn
    if (remaining < targetCounts * 0.25) {
      int slowPwm = constrain(pwmSpeed * 0.55, 0, 255);
      analogWrite(L_MOTOR_PWM, slowPwm);
      analogWrite(R_MOTOR_PWM, constrain(slowPwm + RIGHT_PWM_BOOST, 0, 255));
    }

    if (avg >= targetCounts) break;
    if (millis() - start > 5000) break;
  }

  stopMotors();
  delay(100);
}

 
// SEARCH FOR MARKER
// Rotates until Pi reports a valid marker.
void searchForMarker() {
  Serial.println("Starting search...");

  clearMarkerData();

  while (true) {
    bool seenMarker;

    noInterrupts();
    seenMarker = marker_detected;
    interrupts();

    if (seenMarker) break;

    Serial.println("Searching...");
    turnDegrees('R', SEARCH_STEP_DEG, motor_pwm, true);
    delay(1500);
  }

  stopMotors();
  Serial.println("Marker detected.");
}

 
// ALIGN TO MARKER
// Adjusts until the angle is around 20 degrees.
 
bool alignToMarker() {
  Serial.println("Starting alignment...");

  unsigned long start = millis();

  while (true) {
    bool seenMarker;
    float currentAngle;

    noInterrupts();
    seenMarker = marker_detected;
    currentAngle = beacon_angle_deg;
    interrupts();

    if (!seenMarker) {
      stopMotors();
      Serial.println("Marker lost.");
      return false;
    }

    Serial.print("Current angle: ");
    Serial.println(currentAngle);

    if (currentAngle < MIN_ALIGN_RANGE || currentAngle > MAX_ALIGN_RANGE) {
      stopMotors();
      Serial.println("Marker outside alignment range.");
      return false;
    }

    if (currentAngle >= (TARGET_ANGLE - ANGLE_TOLERANCE) &&
        currentAngle <= (TARGET_ANGLE + ANGLE_TOLERANCE)) {
      stopMotors();
      Serial.println("Aligned to 20 degrees.");
      return true;
    }

    if (currentAngle > TARGET_ANGLE) {
      Serial.println("Angle high -> shifting RIGHT");
      turnDegrees('L', ALIGN_STEP_DEG, align_pwm);
    } else {
      Serial.println("Angle low -> shifting LEFT");
      turnDegrees('R', ALIGN_STEP_DEG, align_pwm);
    }

    delay(150);

    if (millis() - start > 7000) {
      stopMotors();
      Serial.println("Alignment timeout.");
      return false;
    }
  }
}

 
// MOVE FORWARD
// Stops only when Pi says marker distance <= threshold.
// Returns:
//   true  = reached Pi stop distance
//   false = timed out
 
bool moveForwardDistance(float distance_m, int pwmSpeed) {
  Serial.println("Starting forward movement...");

  noInterrupts();
  left_count = 0;
  right_count = 0;
  interrupts();

  left_dir_sign = +1;
  right_dir_sign = +1;

  setMotor(L_MOTOR_SIGN, L_MOTOR_PWM, pwmSpeed, true);
  setMotor(R_MOTOR_SIGN, R_MOTOR_PWM, pwmSpeed, true);

  unsigned long start = millis();

  while (true) {
    float liveDistance;
    float liveAngle;
    bool seenMarker;

    // Copy Pi values safely before checking them
    noInterrupts();
    liveDistance = beacon_distance_m;
    liveAngle = beacon_angle_deg;
    seenMarker = marker_detected;
    interrupts();

    Serial.print("Pi distance = ");
    Serial.print(liveDistance, 3);
    Serial.print(" | angle = ");
    Serial.println(liveAngle, 2);

    // Main stop condition:
    // If marker is visible and distance is about 1 meter, stop.
    if (seenMarker && liveDistance > 0.0 && liveDistance <= STOP_DISTANCE_FROM_PI_M) {
      Serial.println("Reached Pi stop distance. STOPPING.");
      stopMotors();
      delay(200);
      return true;
    }

    // Safety timeout so robot does not drive forever
    if (millis() - start > 15000) {
      Serial.println("Forward motion timeout. Did NOT reach marker.");
      stopMotors();
      delay(200);
      return false;
    }

    delay(10);
  }
}

// FINAL TURN FROM COLOR
//
// color_from_pi:
//   1  = green = turn RIGHT
//   0  = red   = turn LEFT
//  -1  = no color = stop / turn 0 degrees
//
// NOTE:
// RIGHT uses turnDegrees('R')
// LEFT uses turnDegrees('L')
void finalTurnFromColor() {
  stopMotors();
  delay(300);

  int finalColor;

  noInterrupts();
  finalColor = color_from_pi;
  interrupts();

  if (finalColor == 1) {
    Serial.println("Green detected -> turn RIGHT 90 degrees");
    turnDegrees('L', 90, turn90_pwm);
  }
  else if (finalColor == 0) {
    Serial.println("Red detected -> turn LEFT 90 degrees");
    turnDegrees('R', 90, turn90_pwm);
  }
  else {
    Serial.println("No color detected -> stop, turn 0 degrees");
    stopMotors();
  }

  stopMotors();

  Serial.println("Pausing after turn decision...");
  delay(AFTER_TURN_PAUSE_MS);
}

 
// RUN ONE FULL CYCLE
bool runMarkerCycle() {
  Serial.println("=================================");
  Serial.println("Starting new marker cycle...");
  Serial.println("=================================");

  bool seenMarker;

  noInterrupts();
  seenMarker = marker_detected;
  interrupts();

  if (!seenMarker) {
    Serial.println("No marker visible. Searching...");
    searchForMarker();
  }

  if (!alignToMarker()) {
    Serial.println("Alignment failed. Searching again...");
    searchForMarker();

    if (!alignToMarker()) {
      Serial.println("Alignment still failed. Restarting cycle...");
      stopMotors();
      delay(300);
      return false;
    }
  }

  bool reachedMarker = moveForwardDistance(FORWARD_DISTANCE_M, forward_pwm);

  if (!reachedMarker) {
    Serial.println("Did not reach Pi stop distance. Restarting without turn.");
    stopMotors();
    delay(300);
    return false;
  }

  delay(300);

  // Give Pi a short moment to send the latest color result.
  // If it sends -1, finalTurnFromColor will turn 0 degrees.
  unsigned long colorWaitStart = millis();
  while (millis() - colorWaitStart < 1200) {
    int latestColor;

    noInterrupts();
    latestColor = color_from_pi;
    interrupts();

    if (latestColor == 1 || latestColor == 0 || latestColor == -1) {
      break;
    }
  }

  finalTurnFromColor();

  stopMotors();

  Serial.println("Cycle complete.");
  return true;
}

// SETUP
void setup() {
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_SIGN, OUTPUT);
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_SIGN, OUTPUT);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(enc1PinB, INPUT_PULLUP);
  pinMode(enc2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), encoder2ISR, CHANGE);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);

  clearMarkerData();

  Serial.println("Waiting 3 seconds before start...");
  delay(3000);

  Serial.println("Robot ready.");
}

 
// MAIN LOOP
 
void loop() {
  bool success = runMarkerCycle();

  if (!success) {
    delay(300);
  }
}