// Team 10 - SEED Lab Code
// Controls Members: Lydia Tan and Kyle Ha
// Course: EENG350
// Date: 2/23/2026

/*
-- Arduino Motor Control (Mini Project) --
Updated for hardware configuration:
 - LEFT and RIGHT wheels both support forward and reverse movement.
 - I2C communication allows the Raspberry Pi to control the wheel positions in real-time.
 - The Raspberry Pi sends commands to the Arduino in the format: write_i2c_block_data(0x08, 0x01, [lw, rw]).
 - The Arduino receives these commands and applies them to the motors.
 - Connect A4, A5, and the GND Pin to GPIO 2, GPIO 3, and the Raspberry Pi's GND
*/

#include <Wire.h> // Include the I2C library for communication with the Raspberry Pi.
#include <math.h> // Include math library for functions like floorf and fabsf

  float pi_local = 3.14159f; // Define a local variable for Pi value

// I2C Communication Configuration
  uint8_t I2C_SLAVE_ADDR = 0x08; // Arduino I2C address
  uint8_t I2C_CMD_ID = 0x01;// Command ID to differentiate between different commands sent by Pi

// Variables to hold received data
  bool i2c_new_cmd = false; // Flag to indicate new command received from Pi
  uint8_t i2c_lw = 0; // Left wheel control (0 or 1)
  uint8_t i2c_rw = 0; // Right wheel control (0 or 1)

// I2C Receive Interrupt Service Routine
void onI2CReceive(int numBytes) {
  if (numBytes <= 0) return;  // If no data is received, return immediately.

  uint8_t first = (uint8_t)Wire.read(); // Read the first byte
  numBytes--; // Decrement the byte count

  uint8_t lw = 0, rw = 0; // Variables to store left and right wheel control data
  bool gotBits = false; // Flag to confirm if valid data was received

  // Expected format from Pi: [0x01, lw, rw] where 0x01 is the command ID
  if (first == I2C_CMD_ID && numBytes >= 2) {
    lw = (uint8_t)Wire.read(); numBytes--; // Read left wheel control
    rw = (uint8_t)Wire.read(); numBytes--; // Read right wheel control
    gotBits = true;  // Set flag indicating valid data received
  }
  // Fallback format: [lw, rw] if no command ID is included
  else if (numBytes >= 1) {
    lw = first; // Assign the first byte to left wheel control
    rw = (uint8_t)Wire.read(); numBytes--; // Read right wheel control
    gotBits = true;
  }

  // Drain any remaining bytes (if any)
  while (numBytes-- > 0) {
    (void)Wire.read(); // Read and discard excess bytes
  }

  // Validate that the received bits are correct (either 0 or 1 for both wheels)
  if (gotBits && (lw <= 1) && (rw <= 1)) {
    i2c_lw = lw; // Store the left wheel control data
    i2c_rw = rw; // Store the right wheel control data
    i2c_new_cmd = true; // Set flag indicating new command is available
  }
}
// END I2C Communication Configuration 

// Timing Variables for controlling the sample rate
unsigned long desired_Ts_ms = 10; // Desired sample time in milliseconds (100Hz rate)
unsigned long last_time_ms; // Stores the last time the loop ran
unsigned long start_time_ms; // Stores the start time of the program
float current_time = 0; // Variable to store the current time in seconds
unsigned int i = 0; // Loop counter

// Control Gains for the system
float Kp_velocity = 4.0f; // Proportional gain for velocity control
float battery_voltage = 7.8f; // Battery voltage, used to scale PWM values

// Position control gains
float Kp_pos = 10.0f; // Proportional gain for position control
float Ki_pos = 1.0f; // Integral gain for position control
float I_MAX = 10.0f; // Maximum value for integral error to prevent windup

// Limits for controlling maximum speed and error tolerance
float MAX_DESIRED_SPEED = 6.0f; // Maximum desired speed in rad/s
float POS_STOP_BAND = 0.01f; // Small tolerance around zero position to stop motor
float FORWARD_EPS = 0.05f; // Small epsilon to avoid jittery position wrap behavior

// Velocity control variables for each wheel
float desired_speed[2] = {0, 0}; // Desired speed for the left and right wheels
float actual_speed[2] = {0, 0}; // Actual speed based on encoder feedback
float voltage[2] = {0, 0}; // Voltage commands for each motor (converted to PWM)
float velocity_error[2] = {0, 0}; // Error between desired and actual speed for each wheel
unsigned int PWM[2] = {0, 0}; // PWM values for controlling the motor speed

// Position control variables
float integral_error[2] = {0, 0}; // Integral error for each wheel to accumulate position error
float pos_error[2] = {0, 0}; // Position error between desired and actual positions
float desired_pos[2] = {0, 0}; // Desired position for each wheel
float actual_pos[2] = {0, 0}; // Actual position based on encoder readings

// Encoder variables
  long encoderPos[2] = {0, 0}; // Encoder position counters for left and right wheels
uint8_t state[2] = {0, 0}; // Encoder state for left and right wheels
long pos_counts[2] = {0, 0}; // Encoder counts for each wheel
float pos_rad[2] = {0, 0}; // Position in radians for each wheel
float pos_rad_old[2] = {0, 0}; // Previous position in radians for velocity calculations

// Command bits to control wheel positions
uint8_t piDir[2] = {0, 0}; // Direction control for each wheel (0 or 1)

// Pin assignments for encoders and motors
  int enc1PinA = 2; // LEFT encoder A (interrupt capable)
  int enc1PinB = 5; // LEFT encoder B
  int enc2PinA = 3; // RIGHT encoder A (interrupt capable)
  int enc2PinB = 6; // RIGHT encoder B

// Pin assignments for motor control
  int shieldPin = 4; // Motor driver enable pin
  int mot1Dir = 8; // LEFT motor direction pin
  int mot2Dir = 7; // RIGHT motor direction pin
  int mot1Pow = 10; // LEFT motor PWM control pin
  int mot2Pow = 9; // RIGHT motor PWM control pin

// LEFT helper: force target to be forward of current (never reverse)
float forwardWrapTarget(float current, float target) {
  while (target < current - FORWARD_EPS) {  // Ensure the target is always forward
    target += TWO_PI;  // Wrap the target around to avoid reverse motion
  }
  return target;
}

// Apply command to desired positions based on received I2C data
void applyCommandBits(uint8_t b0, uint8_t b1) {
  piDir[0] = b0;
  piDir[1] = b1;

  // Base (absolute) targets: 0 or π based on received command
  float baseL = 0.0f;
  float baseR = 0.0f;

  if (piDir[0] == 0 && piDir[1] == 0) {
    baseL = 0; baseR = 0;
  } else if (piDir[0] == 0 && piDir[1] == 1) {
    baseL = 0; baseR = pi_local;
  } else if (piDir[0] == 1 && piDir[1] == 1) {
    baseL = pi_local; baseR = pi_local;
  } else { // 10
    baseL = pi_local; baseR = 0;
  }

  // LEFT wheel: Forward or reverse
  desired_pos[0] = forwardWrapTarget(actual_pos[0], baseL);

  // RIGHT wheel: Normal nearest wrap (forward/backward allowed)
  desired_pos[1] = forwardWrapTarget(actual_pos[1], baseR);

  // Reset integrators when a new command is received
  integral_error[0] = 0;
  integral_error[1] = 0;

  // Output command for debugging
  Serial.print("I2C CMD: ");
  Serial.print(piDir[0]); Serial.print(piDir[1]);
  Serial.print(" -> targetL="); Serial.print(desired_pos[0], 3);
  Serial.print(" targetR="); Serial.println(desired_pos[1], 3);
}

// ISR for Encoder 1 (LEFT) - Increment or decrement encoder position based on encoder state
void encoder1ISR() {
  uint8_t s = state[0] & 3;
  if (digitalRead(enc1PinA)) s |= 4;
  if (digitalRead(enc1PinB)) s |= 8;
  switch (s) {
    case 0: case 5: case 10: case 15: break;
    case 1: case 7: case 8: case 14:  encoderPos[0]++; break;
    case 2: case 4: case 11: case 13: encoderPos[0]--; break;
    case 3: case 12:                  encoderPos[0] += 2; break;
    default:                          encoderPos[0] -= 2; break;
  }
  state[0] = (s >> 2);
}

// ISR for Encoder 2 (RIGHT) - Increment or decrement encoder position based on encoder state
void encoder2ISR() {
  uint8_t s = state[1] & 3;
  if (digitalRead(enc2PinA)) s |= 4;
  if (digitalRead(enc2PinB)) s |= 8;
  switch (s) {
    case 0: case 5: case 10: case 15: break;
    case 1: case 7: case 8: case 14:  encoderPos[1]++; break;
    case 2: case 4: case 11: case 13: encoderPos[1]--; break;
    case 3: case 12:                  encoderPos[1] += 2; break;
    default:                          encoderPos[1] -= 2; break;
  }
  state[1] = (s >> 2);
}

// Setup function that runs once when the program starts
void setup() {
  // Initialize encoder pins as input with internal pull-ups
  pinMode(enc1PinA, INPUT_PULLUP);
  pinMode(enc1PinB, INPUT_PULLUP);
  pinMode(enc2PinA, INPUT_PULLUP);
  pinMode(enc2PinB, INPUT_PULLUP);

  // Attach interrupts to encoder A pins for both wheels
  attachInterrupt(digitalPinToInterrupt(enc1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2PinA), encoder2ISR, CHANGE);

  // Initialize motor control pins as output
  pinMode(shieldPin, OUTPUT);
  pinMode(mot1Dir, OUTPUT);
  pinMode(mot2Dir, OUTPUT);
  pinMode(mot1Pow, OUTPUT);
  pinMode(mot2Pow, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);

  // I2C Communication Setup
  // Arduino acts as an I2C slave so the Pi can push commands.
  Wire.begin(I2C_SLAVE_ADDR); // Initialize I2C as slave at address 0x08
  Wire.onReceive(onI2CReceive); // Set the function that handles received data
  // END I2C Communication Setup 

  // Enable the motor shield by setting the shield pin high
  digitalWrite(shieldPin, HIGH);

  // Default directions
  digitalWrite(mot1Dir, LOW); // LEFT motor direction set to forward (LOW)
  digitalWrite(mot2Dir, HIGH); // RIGHT motor direction set to forward (HIGH)

  // Initialize encoder positions to 0
  noInterrupts();
  encoderPos[0] = 0;
  encoderPos[1] = 0;
  state[0] = 0;
  state[1] = 0;
  interrupts();

  // Initialize variables for position, velocity, and integral errors
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

  // Record the time when the program starts
  last_time_ms = millis();
  start_time_ms = last_time_ms;

  // Print a message indicating that the system is ready
  Serial.println("Ready. I2C slave @0x08. Pi sends 00,01,10,11 as 2 bits.");
}

// Main loop of the program where the logic runs continuously
void loop() {
  // I2C Communication
  // If a new command has been received from the Raspberry Pi, process it
  if (i2c_new_cmd) {
    noInterrupts();
    uint8_t lw = i2c_lw; // Store the left wheel control data
    uint8_t rw = i2c_rw; // Store the right wheel control data
    i2c_new_cmd = false; // Reset the command flag
    interrupts();

    applyCommandBits(lw, rw); // Apply the command to set wheel positions
  }

  // END I2C Communication 

  // Atomic encoder read (safely read encoder positions without interruption)
  long eL, eR;
  noInterrupts();
  eL = encoderPos[0];
  eR = encoderPos[1];
  interrupts();
  pos_counts[0] = eL;
  pos_counts[1] = eR;

  // Continuous position (unwrapped) calculation
  pos_rad[0] = (float)TWO_PI * (float)pos_counts[0] / 3200.0f;  // Calculate left wheel position in radians
  pos_rad[1] = (float)TWO_PI * (float)pos_counts[1] / 3200.0f;  // Calculate right wheel position in radians
  actual_pos[0] = pos_rad[0];  // Store the actual position of the left wheel
  actual_pos[1] = pos_rad[1];  // Store the actual position of the right wheel

  // Speed estimation using the change in position and sample time
  float Ts = (float)desired_Ts_ms / 1000.0f;  // Convert sample time to seconds
  actual_speed[0] = (pos_rad[0] - pos_rad_old[0]) / Ts;  // Calculate left wheel speed in rad/s
  actual_speed[1] = (pos_rad[1] - pos_rad_old[1]) / Ts;  // Calculate right wheel speed in rad/s
  pos_rad_old[0] = pos_rad[0];  // Store current position for next calculation
  pos_rad_old[1] = pos_rad[1];  // Store current position for next calculation

  // Position control loop to calculate desired speed based on position error
  for (i = 0; i < 2; i++) {
    pos_error[i] = desired_pos[i] - actual_pos[i]; // Calculate position error
    integral_error[i] += pos_error[i] * Ts; // Integrate position error over time
    integral_error[i] =  rain(integral_error[i], -I_MAX, I_MAX);  // Prevent integrator windup

    desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];  // Calculate desired speed for each wheel
    desired_speed[i] =  rain(desired_speed[i], -MAX_DESIRED_SPEED, MAX_DESIRED_SPEED);  // Limit desired speed
  }

  // Velocity control loop to calculate motor voltage (PWM) based on speed error
  for (i = 0; i < 2; i++) {
    velocity_error[i] = desired_speed[i] - actual_speed[i]; // Calculate velocity error
    voltage[i] =  rain(Kp_velocity * velocity_error[i], -8.2f, 8.2f); // Calculate motor voltage
  }

  // Stop logic for both wheels: if within stop band, stop the motor
  for (i = 0; i < 2; i++) {
    if (fabsf(pos_error[i]) <= POS_STOP_BAND) {
      voltage[i] = 0; // Stop motor
      desired_speed[i] = 0; // Stop speed
      integral_error[i] = 0; // Reset integral error
    }
  }

  // LEFT motor: Forward and reverse control
  digitalWrite(mot1Dir, voltage[0] > 0 ? LOW : HIGH); // Set direction based on voltage polarity
  PWM[0] = (unsigned int)(255.0f * fabsf(voltage[0]) / battery_voltage); // Calculate PWM value based on voltage
  analogWrite(mot1Pow,  rain(PWM[0], 0, 255)); // Apply PWM to LEFT motor

  // RIGHT motor: Forward and reverse control
  digitalWrite(mot2Dir, voltage[1] > 0 ? HIGH : LOW); // Set direction based on voltage polarity
  PWM[1] = (unsigned int)(255.0f * fabsf(voltage[1]) / battery_voltage); // Calculate PWM value based on voltage
  analogWrite(mot2Pow,  rain(PWM[1], 0, 255)); // Apply PWM to RIGHT motor

  // Timing control: ensure the loop runs at the desired sample rate
  current_time = (float)(last_time_ms - start_time_ms) / 1000.0f; // Calculate elapsed time in seconds

  // Wait until the next sample time (using the millis() function for non-blocking timing)
  while (millis() < last_time_ms + desired_Ts_ms) {
    // Wait for the next sample time
  }
  last_time_ms = millis();  // Update last_time_ms to the current time for the next loop cycle
}