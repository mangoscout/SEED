// Team 10 - SEED Lab Code
// Controls Members: Lydia Tan and Kyle Ha
// Course: EENG350
// Date: 2/23/2026
/*
-- Demo 1: Combined Rotation + Distance Control --

Program behavior:
 - The robot first performs a commanded in-place rotation.
 - After the rotation is complete, it switches into distance-control mode.
 - In distance mode, the robot drives forward a preset distance using encoder feedback.
 - A position PI controller generates desired wheel speeds.
 - A velocity proportional controller converts speed error into motor voltage commands.
 - Encoder feedback is used during both phases, but processed differently in each mode.

Hardware notes:
 - LEFT encoder A channel uses pin 2, RIGHT encoder A channel uses pin 3.
 - Encoder B channels are used during distance mode for full quadrature decoding.
 - Motor driver enable pin must be set HIGH to allow motor output.
 - LEFT and RIGHT motor directions may require inversion depending on wiring.
 - *Note: No additional hardware was added compared to Mini Project Setup
*/

// Code Starts Here
#include <Wire.h> // Included for compatibility if I2C is later used
#include <Arduino.h> // Standard Arduino core library
#include <math.h> // Provides fabsf(), abs(), and other math functions

float pi_local = 3.14159f;   // Local approximation of pi
 
// ROTATION CODE SETTINGS  
// MOTOR PINS   
// Enable pin for the motor driver / shield
int ENABLE_PIN = 4;
// LEFT motor control pins
int L_MOTOR_SIGN = 8; // Direction pin for LEFT motor
int L_MOTOR_PWM = 10; // PWM pin for LEFT motor

// RIGHT motor control pins
int R_MOTOR_SIGN = 7; // Direction pin for RIGHT motor
int R_MOTOR_PWM = 9; // PWM pin for RIGHT motor

// ENCODER PINS   
// A channels are interrupt-capable and used to trigger encoder ISR updates
int L_ENC_A = 2; // LEFT encoder A channel
int R_ENC_A = 3; // RIGHT encoder A channel
// B channels are additionally used in distance mode for quadrature decoding
int enc1PinB = 5; // LEFT encoder B channel
int enc2PinB = 6; // RIGHT encoder B channel

// ROBOT CONSTANTS   
// Physical robot parameters used for turn calculations
float wheel_radius_m = 0.0873125; // Wheel radius in meters
float wheel_base_m = 0.15; // Distance between the two wheels in meters
long counts_per_rev = 3200; // Encoder counts per wheel revolution

//  TUNING   
// Turn gain used to calibrate real-world turning performance
float TURN_GAIN = 1.375;
// Optional direction inversion flags to compensate for motor wiring differences
bool L_DIR_INVERT = false;
bool R_DIR_INVERT = true;

//  MOTOR SETTINGS   
// PWM value used during the turn phase
int motor_pwm = 80;

// AUTO TURN COMMAND   
// Preset turn command for the demo
char TURN_DIRECTION = 'R'; // 'L' for left or 'R' for right
int TURN_DEGREES = 135; // Rotation angle in degrees

// ENCODER COUNTS FOR TURN   
// These counts are used during the rotation phase only
long left_count = 0; // LEFT wheel turn counter
long right_count = 0; // RIGHT wheel turn counter
// Signs applied during turn counting so counts increase in the intended direction
int left_dir_sign = -1;
int right_dir_sign = +1;

//  DISTANCE CODE SETTINGS:
// USER-SET DISTANCE VALUE
// Integer feet only: 0 to 10
// Preset forward travel distance in feet
float commanded_distance_ft = 2.82;

// DISTANCE OVERSHOOT CORRECTION
// Scale factor to reduce overshoot in measured travel distance
float DISTANCE_SCALE = 0.965f;
// Right wheel correction factor
// Used if the right wheel consistently under-travels or over-travels
float RIGHT_WHEEL_SCALE = 1.00f;
// Continuous wheel-matching proportional gain
// Helps keep LEFT and RIGHT wheel positions aligned while driving
float Kp_match = 70.0f;
// Ensures the distance command is only applied once after startup
bool distance_command_sent = false;

// Timing Variables
unsigned long desired_Ts_ms = 10; // Desired control loop sample period in milliseconds
unsigned long last_time_ms; // Time stamp of previous loop iteration
unsigned long start_time_ms; // Time stamp when distance mode begins
float current_time = 0; // Elapsed time in seconds
unsigned int i = 0; // General loop index

// Controller Gains / Parameters
float Kp_velocity = 2.0f; // Proportional gain for inner velocity loop
float battery_voltage = 7.8f; // Battery voltage used for PWM scaling

// Outer loop gains for position control
float Kp_pos = 10.0f; // Position proportional gain
float Ki_pos = 1.2f; // Position integral gain
float I_MAX = 10.0f; // Integrator anti-windup limit

// Limits / Thresholds
float MAX_DESIRED_SPEED = 6.0f; // Maximum commanded wheel speed in rad/s
float POS_STOP_BAND = 0.05f; // Position tolerance band for stopping
float FORWARD_EPS = 0.05f; // Small forward-wrap margin to avoid jitter

// Calibration:
// pi radians of wheel motion corresponds to about 0.758 ft of robot travel
float FEET_PER_PI_RAD = 0.758f;
// Velocity Control Variables
float desired_speed[2] = {0, 0}; // Desired speed for LEFT and RIGHT wheels
float actual_speed[2] = {0, 0}; // Measured speed for LEFT and RIGHT wheels
float voltage[2] = {0, 0}; // Motor voltage command for each wheel
float velocity_error[2] = {0, 0}; // Speed error for each wheel
unsigned int PWM[2] = {0, 0}; // PWM command derived from voltage

// Position Control Variables
float integral_error[2] = {0, 0}; // Accumulated position error for PI control
float pos_error[2] = {0, 0}; // Instantaneous position error
float desired_pos[2] = {0, 0}; // Target wheel positions in radians
float actual_pos[2] = {0, 0}; // Measured wheel positions in radians

// Encoder Variables for distance mode
long encoderPos[2] = {0, 0}; // Quadrature-decoded encoder counts
uint8_t state[2] = {0, 0}; // Previous state memory for quadrature decoding
long pos_counts[2] = {0, 0}; // Snapshot of encoder counts
float pos_rad[2] = {0, 0}; // Wheel position in radians
float pos_rad_old[2] = {0, 0}; // Previous wheel position for speed calculation

// Pin aliases to match original distance code naming
int enc1PinA = 2;
int enc2PinA = 3;
int shieldPin = 4;
int mot1Dir = 8; // LEFT motor direction pin
int mot1Pow = 10; // LEFT motor PWM pin
int mot2Dir = 7; // RIGHT motor direction pin
int mot2Pow = 9; // RIGHT motor PWM pin

//  PHASE CONTROL     
// Tracks whether the robot has finished the turn phase
bool rotation_done = false;

//  SHARED / HELPER FUNCTIONS    -
// Forward-only wrap helper
// Ensures a target angle is always ahead of the current angle by wrapping
// it forward by multiples of 2*pi if necessary.
float forwardWrapTarget(float current, float target) {
  while (target < current - FORWARD_EPS) {
    target += TWO_PI;
  }
  return target;
}

// Convert linear travel distance in feet into wheel rotation in radians
float feetToRadians(float distance_ft) {
  return distance_ft * pi_local / FEET_PER_PI_RAD;
}

// Apply the preset travel distance once
// This computes the target wheel positions for both wheels based on the
// commanded distance and calibration factors.
void applyPresetDistanceCommand() {
  float corrected_distance_ft = ((float)commanded_distance_ft) * DISTANCE_SCALE;
  float delta_theta = feetToRadians(corrected_distance_ft);

  // Set LEFT and RIGHT wheel targets relative to their current positions
  desired_pos[0] = forwardWrapTarget(actual_pos[0], actual_pos[0] + delta_theta);
  desired_pos[1] = forwardWrapTarget(actual_pos[1], actual_pos[1] + delta_theta * RIGHT_WHEEL_SCALE);

  // Reset the integrators so the new move starts cleanly
  integral_error[0] = 0;
  integral_error[1] = 0;

  // Print useful debug information to the Serial Monitor
  Serial.print("Commanded distance: ");
  Serial.print(commanded_distance_ft);
  Serial.println(" ft");

  Serial.print("Corrected distance: ");
  Serial.print(corrected_distance_ft, 3);
  Serial.println(" ft");

  Serial.print("Delta theta L: ");
  Serial.print(delta_theta, 4);
  Serial.println(" rad");

  Serial.print("Delta theta R: ");
  Serial.print(delta_theta * RIGHT_WHEEL_SCALE, 4);
  Serial.println(" rad");

  Serial.print("Targets -> L: ");
  Serial.print(desired_pos[0], 4);
  Serial.print(" rad, R: ");
  Serial.print(desired_pos[1], 4);
  Serial.println(" rad");
}

 
//  MOTOR HELPERS     
// Sets one motor's direction and PWM value.
// The 'forward' argument represents logical forward motion, and inversion
// flags are applied internally if needed for wiring correction.
void setMotor(int signPin, int pwmPin, int pwm, bool forward)
{
  bool cmdForward = forward;

  // Apply direction inversion if the motor wiring requires it
  if (signPin == L_MOTOR_SIGN && L_DIR_INVERT) cmdForward = !cmdForward;
  if (signPin == R_MOTOR_SIGN && R_DIR_INVERT) cmdForward = !cmdForward;

  // Set direction pin and PWM output
  digitalWrite(signPin, cmdForward ? HIGH : LOW);
  analogWrite(pwmPin, constrain(pwm, 0, 255));
}

// Immediately stop both motors by setting PWM to zero
void stopMotors()
{
  analogWrite(L_MOTOR_PWM, 0);
  analogWrite(R_MOTOR_PWM, 0);
}

 
//  ENCODER ISRS     -
// LEFT encoder ISR
// During rotation mode:
//   - simply increments or decrements a turn counter
// During distance mode:
//   - performs quadrature decoding for more accurate wheel position tracking
void encoder1ISR() {
  if (!rotation_done) {
    left_count += left_dir_sign;
    return;
  }
  uint8_t s = state[0] & 3;
  if (digitalRead(enc1PinA)) s |= 4;
  if (digitalRead(enc1PinB)) s |= 8;
  switch (s) {
    case 0: case 5: case 10: case 15: break;
    case 1: case 7: case 8: case 14: encoderPos[0]++; break;
    case 2: case 4: case 11: case 13: encoderPos[0]--; break;
    case 3: case 12: encoderPos[0] += 2; break;
    default: encoderPos[0] -= 2; break;
  }
  state[0] = (s >> 2);
}

// RIGHT encoder ISR
// Same behavior as the LEFT ISR, but for the right wheel
void encoder2ISR() {
  if (!rotation_done) {
    right_count += right_dir_sign;
    return;
  }
  uint8_t s = state[1] & 3;
  if (digitalRead(enc2PinA)) s |= 4;
  if (digitalRead(enc2PinB)) s |= 8;
  switch (s) {
    case 0: case 5: case 10: case 15: break;
    case 1: case 7: case 8: case 14: encoderPos[1]++; break;
    case 2: case 4: case 11: case 13: encoderPos[1]--; break;
    case 3: case 12: encoderPos[1] += 2; break;
    default: encoderPos[1] -= 2; break;
  }
  state[1] = (s >> 2);
}

 
//  TURN FUNCTION     
// Rotates the robot in place by the requested angle.
// dir = 'L' or 'R'
// deg = number of degrees to rotate
void turnDegrees(char dir, int deg)
{
  // Restrict the angle to a safe range
  deg = constrain(deg, -360, 360);

  // If the angle is negative, flip the turn direction and use its magnitude
  if (deg < 0) {
    deg = abs(deg);
    dir = (dir == 'L') ? 'R' : 'L';
  }
  // Nothing to do if the requested turn is zero
  if (deg == 0) return;
  // Reset turn counters before starting the move
  noInterrupts();
  left_count = 0;
  right_count = 0;
  interrupts();
  // Compute encoder counts needed for the requested turn
  // This uses robot geometry plus an experimentally tuned gain
  float factor = (wheel_base_m / (2.0 * wheel_radius_m)) * ((float)deg / 360.0);
  long targetCounts = (long)(TURN_GAIN * factor * (float)counts_per_rev);
  // Enforce a minimum count target for very small turns
  if (targetCounts < 5) targetCounts = 5;
  unsigned long start = millis(); // Used as a timeout safety

  // Set motor directions for left or right turn
  if (toupper(dir) == 'L')
  {
    left_dir_sign = +1;
    right_dir_sign = -1;
    setMotor(L_MOTOR_SIGN, L_MOTOR_PWM, motor_pwm, true);
    setMotor(R_MOTOR_SIGN, R_MOTOR_PWM, motor_pwm, false);
  }
  else
  {
    left_dir_sign = -1;
    right_dir_sign = +1;

    setMotor(L_MOTOR_SIGN, L_MOTOR_PWM, motor_pwm, false);
    setMotor(R_MOTOR_SIGN, R_MOTOR_PWM, motor_pwm, true);
  }
  // Wait until the average encoder count reaches the target or timeout occurs
  while (true)
  {
    long lc, rc;
    noInterrupts();
    lc = labs(left_count);
    rc = labs(right_count);
    interrupts();
    long avg = (lc + rc) / 2;
    if (avg >= targetCounts) break; // Turn complete
    if (millis() - start > 5000) break; // Safety timeout after 5 seconds
  }
  // Stop motors once the turn is complete
  stopMotors();
}

//  SETUP    -
// Runs once at startup
void setup()
{
  // Configure encoder pins as inputs with pull-up resistors
  pinMode(enc1PinA, INPUT_PULLUP);
  pinMode(enc1PinB, INPUT_PULLUP);
  pinMode(enc2PinA, INPUT_PULLUP);
  pinMode(enc2PinB, INPUT_PULLUP);

  // Configure motor driver pins as outputs
  pinMode(shieldPin, OUTPUT);
  pinMode(mot1Dir, OUTPUT);
  pinMode(mot2Dir, OUTPUT);
  pinMode(mot1Pow, OUTPUT);
  pinMode(mot2Pow, OUTPUT);

  // Start serial communication for debugging output
  Serial.begin(9600);

  // Enable the motor shield
  digitalWrite(shieldPin, HIGH);

  // Attach interrupt service routines to the A channels of the encoders
  attachInterrupt(digitalPinToInterrupt(enc1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2PinA), encoder2ISR, CHANGE);

  delay(1000); // Small pause before motion begins

  //  ROTATION FIRST   
  // Perform the commanded turn before starting forward distance control
  Serial.println("Starting rotation...");
  turnDegrees(TURN_DIRECTION, TURN_DEGREES);
  stopMotors();

  //  SWITCH TO DISTANCE MODE   
  // Clear encoder and controller state before beginning the distance phase
  noInterrupts();
  left_count = 0;
  right_count = 0;
  encoderPos[0] = 0;
  encoderPos[1] = 0;
  state[0] = 0;
  state[1] = 0;
  interrupts();

  // Tell the program to use distance-control logic from now on
  rotation_done = true;

  // Reset controller memory
  integral_error[0] = 0;
  integral_error[1] = 0;
  pos_rad[0] = 0;
  pos_rad[1] = 0;
  pos_rad_old[0] = 0;
  pos_rad_old[1] = 0;

  desired_pos[0] = 0;
  desired_pos[1] = 0;
  actual_pos[0] = 0;
  actual_pos[1] = 0;

  // Set default forward directions for distance mode
  // LEFT is treated as forward-only in this phase
  digitalWrite(mot1Dir, LOW);
  digitalWrite(mot2Dir, HIGH);

  // Initialize timing values for the control loop
  last_time_ms = millis();
  start_time_ms = last_time_ms;

  Serial.println("Rotation done.");
  Serial.println("Starting distance mode...");
}

 
// LOOP
// Runs repeatedly after setup()
void loop() {
  // Only run the distance-control algorithm after rotation has finished
  if (!rotation_done) {
    return;
  }
  // Apply the preset distance target once, right after entering distance mode
  if (!distance_command_sent) {
    applyPresetDistanceCommand();
    distance_command_sent = true;
  }
  // Read encoder counts atomically so ISR updates do not interrupt the read
  long eL, eR;
  noInterrupts();
  eL = encoderPos[0];
  eR = encoderPos[1];
  interrupts();
  pos_counts[0] = eL;
  pos_counts[1] = eR;
  // Convert encoder counts into wheel position in radians
  pos_rad[0] = (float)TWO_PI * (float)pos_counts[0] / 3200.0f;
  pos_rad[1] = (float)TWO_PI * (float)pos_counts[1] / 3200.0f;
  actual_pos[0] = pos_rad[0];
  actual_pos[1] = pos_rad[1];
  // Estimate wheel speed using finite difference:
  // speed = change in position / sample time
  float Ts = (float)desired_Ts_ms / 1000.0f;
  actual_speed[0] = (pos_rad[0] - pos_rad_old[0]) / Ts;
  actual_speed[1] = (pos_rad[1] - pos_rad_old[1]) / Ts;
  // Save current positions for the next speed estimate
  pos_rad_old[0] = pos_rad[0];
  pos_rad_old[1] = pos_rad[1];
  // Position PI loop:
  // Generates desired wheel speed from position error
  for (i = 0; i < 2; i++) {
    pos_error[i] = desired_pos[i] - actual_pos[i];
    integral_error[i] += pos_error[i] * Ts;
    integral_error[i] = constrain(integral_error[i], -I_MAX, I_MAX);
    desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];
  }

  // Continuous wheel matching:
  // Forces the two wheel positions to stay close together during forward travel
  float match_error = actual_pos[0] - actual_pos[1];
  desired_speed[0] -= Kp_match * match_error;
  desired_speed[1] += Kp_match * match_error;

  // Clamp desired speed so neither wheel exceeds the allowed maximum
  for (i = 0; i < 2; i++) {
    desired_speed[i] = constrain(desired_speed[i], -MAX_DESIRED_SPEED, MAX_DESIRED_SPEED);
  }
  // Velocity P loop:
  // Converts desired speed into a motor voltage command
  for (i = 0; i < 2; i++) {
    velocity_error[i] = desired_speed[i] - actual_speed[i];
    voltage[i] = constrain(Kp_velocity * velocity_error[i], -8.2f, 8.2f);
  }

  // Stop logic:
  // If either wheel is within the stop band, stop both wheels
  if (fabsf(pos_error[0]) <= POS_STOP_BAND || fabsf(pos_error[1]) <= POS_STOP_BAND) {
    voltage[0] = 0;
    voltage[1] = 0;

    desired_speed[0] = 0;
    desired_speed[1] = 0;

    integral_error[0] = 0;
    integral_error[1] = 0;
  }

  // LEFT motor control:
  // LEFT wheel is forced to forward-only motion during distance mode
  if (voltage[0] < 0) voltage[0] = 0;
  digitalWrite(mot1Dir, LOW);
  PWM[0] = (unsigned int)(255.0f * fabsf(voltage[0]) / battery_voltage);
  analogWrite(mot1Pow, constrain(PWM[0], 0, 255));

  // RIGHT motor control:
  // RIGHT wheel is allowed to move forward or backward
  if (voltage[1] > 0) {
    digitalWrite(mot2Dir, HIGH);
  } else {
    digitalWrite(mot2Dir, LOW);
  }
  PWM[1] = (unsigned int)(255.0f * fabsf(voltage[1]) / battery_voltage);
  analogWrite(mot2Pow, constrain(PWM[1], 0, 255));

  // Compute elapsed time since distance mode started
  current_time = (float)(last_time_ms - start_time_ms) / 1000.0f;

  // Wait until the next sample period to maintain a fixed control rate
  while (millis() < last_time_ms + desired_Ts_ms) {
    // wait
  }

  // Update loop timestamp for the next iteration
  last_time_ms = millis();
}
// Code Ends Here