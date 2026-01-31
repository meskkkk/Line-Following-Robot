// ======================
// L298N Motor Driver Pins Configuration
// ======================

// These pins control the motor direction for the left and right motors
#define IN1 7  // Direction control pin for the left motor
#define IN2 6  // Direction control pin for the left motor
#define IN3 5  // Direction control pin for the right motor
#define IN4 4  // Direction control pin for the right motor

// These pins control the motor speed using PWM signals
#define ENA 3  // PWM speed control pin for the left motor
#define ENB 9  // PWM speed control pin for the right motor

// ======================
// IR Sensor Pins Configuration
// ======================

// These pins read analog signals from the IR sensors used for line detection
#define S1 A0  // Far Left IR sensor
#define S2 A1  // Mid Left IR sensor
#define S3 A2  // Mid Right IR sensor
#define S4 A3  // Far Right IR sensor

// ======================
// Motor Speed Configuration & PID Constants
// ======================

// BASE_SPEED represents the constant speed for both motors during straight movement
#define BASE_SPEED 100      // Base speed for straight movement (value between 0 and 255)

// MAX_CORRECTION sets the limit for the speed differential applied to correct the robot's direction
#define MAX_CORRECTION 100  // Increased maximum correction for sharper turns

// THRESHOLD is the reference value used to distinguish between the black line and white background read by IR sensors
#define THRESHOLD 500       // Sensor threshold for detecting line

// ERROR_DEADBAND defines the tolerance band for error; no correction is applied if the error is within this range
#define ERROR_DEADBAND 2    // Deadband range to prevent overcorrection for small errors

// PID constants used for calculating corrective actions
float Kp = 0.3;             // Proportional gain: how strongly to react to the error
float Ki = 10.0;            // Integral gain: accumulates past errors to correct steady-state errors
float Kd = 0.005;           // Derivative gain: reacts to changes in error to improve stability

// ======================
// Status LEDs & Buzzer Configuration
// ======================

// Indicator components used to provide feedback when the robot loses the line
#define RED_LED 10   // LED turns ON when line detection fails
#define GREEN_LED 11 // LED turns ON during normal operation
#define BUZZER 12    // Buzzer sounds when line detection fails

// ======================
// Variables for Error Calculation and Recovery
// ======================

// lastLinePos tracks the last known position of the line (used for recovery from lost line scenarios)
int lastLinePos = 0;         // -1 = Left direction (ACW), 1 = Right direction (CW), 0 = Center

// PID error terms
float integralError = 0;     // Accumulated sum of errors (integral term)
float prevError = 0;         // Previous error value (used for calculating the derivative term)

// ======================
// Arduino Setup
// ======================

void setup() {
  // Define motor pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Define PWM speed control pins as OUTPUT
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Define indicator pins (LEDs and buzzer) as OUTPUT
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

 // ======================
// PID Calculation Function
// ======================
float calculatePID(float error) {
  // Accumulate errors for the integral term
  integralError += error;  

  // Prevent integral windup by clamping the accumulated error within a range
  integralError = constrain(integralError, -30, 30);

  // Calculate the derivative term as the change in error since the last loop iteration
  float derivative = error - prevError;

  // Compute the total correction using the PID formula: correction = Kp*error + Ki*integralError + Kd*derivative
  float correction = (Kp * error) + (Ki * integralError) + (Kd * derivative);

  // Update prevError for the next iteration
  prevError = error;

  // Return the calculated correction for use in motor speed adjustment
  return correction;
}

// ======================
// Main Loop
// ======================
void loop() {
  // Track current timestamp for debugging purposes
  float currentTimeSec = millis() / 1000.0;

  // Read analog values from the IR sensors positioned at different locations
  int v1 = analogRead(S1);
  int v2 = analogRead(S2);
  int v3 = analogRead(S3);
  int v4 = analogRead(S4);

  // Convert analog sensor values into binary indicators (1 = line detected, 0 = no line)
  int s1 = (v1 > THRESHOLD);
  int s2 = (v2 > THRESHOLD);
  int s3 = (v3 > THRESHOLD);
  int s4 = (v4 > THRESHOLD);

  // Compute a weighted error score based on the binary sensor values to determine line position
  float error = (-3 * s1) + (-1 * s2) + (1 * s3) + (3 * s4);

  // Update lastLinePos based on the calculated error, used for recovery during lost line scenarios
  if (error < 0) lastLinePos = -1; // Line detected to the left
  if (error > 0) lastLinePos = 1;  // Line detected to the right

  // Handle cases when no line is detected (all sensors OFF)
  if (!s1 && !s2 && !s3 && !s4) {
    // Turn RED LED and buzzer ON to signal lost line detection
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BUZZER, HIGH);

    // Recover direction based on last known position
    if (lastLinePos == -1) {
      spinRight(100);  // Rotate clockwise to search for the line
    } else {
      spinLeft(100);   // Rotate anticlockwise to search for the line
    }
    // Skip the remaining loop logic until the line is recovered
    return;
  }

  // Turn GREEN LED ON and RED LED/Buzzer OFF to indicate normal operation when line is detected
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BUZZER, LOW);

  // Declare variables for calculated motor speeds
  int leftSpeed, rightSpeed;

  // Handle errors within the deadband range: Drive straight without correction
  if (abs(error) <= ERROR_DEADBAND) {
    Kp = 0.4; // Increase Kp for faster response on straight paths
    leftSpeed = BASE_SPEED + 20;  // Boost speed for straight movement
    rightSpeed = BASE_SPEED + 20;
  } else {
    Kp = 0.3; // Use normal Kp for curves and corrections
    // Calculate corrective action using PID
    float correction = calculatePID(error);

    // Clamp correction to prevent excessive motor speed changes
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

    // Adjust left and right motor speeds based on the calculated correction
    leftSpeed = BASE_SPEED + correction;
    rightSpeed = BASE_SPEED - correction;
  }

  // Clamp motor speeds to prevent invalid values (below 0 or above 255 for PWM)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Set the motors to move forward with the corrected speeds
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Output debugging information via serial
  Serial.print("Time: "); Serial.print(currentTimeSec, 2);
  Serial.print(", Error: "); Serial.print(error);
  Serial.print(", Left Speed: "); Serial.print(leftSpeed);
  Serial.print(", Right Speed: "); Serial.println(rightSpeed);
}

// ======================
// Spin Functions for Recovery
// ======================

// Function to spin the robot anticlockwise
void spinLeft(int speed) {
  digitalWrite(IN1, HIGH); // Left motor backward
  digitalWrite(IN2, LOW);  // Left motor stop
  digitalWrite(IN3, LOW);  // Right motor stop
  digitalWrite(IN4, HIGH); // Right motor forward
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// Function to spin the robot clockwise
void spinRight(int speed) {
  digitalWrite(IN1, LOW);  // Left motor stop
  digitalWrite(IN2, HIGH); // Left motor forward
  digitalWrite(IN3, HIGH); // Right motor backward
  digitalWrite(IN4, LOW);  // Right motor stop
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}