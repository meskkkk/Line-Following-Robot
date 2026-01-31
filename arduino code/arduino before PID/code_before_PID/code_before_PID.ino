// ======================
// L298N Motor Driver Pins Configuration
// ======================

// These pins control the direction of the motors for turning, forward, or backward motion
#define IN1 7  // Left motor direction control pin
#define IN2 6  // Left motor direction control pin
#define IN3 5  // Right motor direction control pin
#define IN4 4  // Right motor direction control pin

// These pins control the speed of the left and right motors using PWM signals
#define ENA 3  // PWM speed control pin for the left motor
#define ENB 9  // PWM speed control pin for the right motor

// ======================
// IR Sensor Pins Configuration
// ======================

// These pins read analog signals from line-following IR sensors
#define S1 A0  // Far Left IR sensor, detects far-left line position
#define S2 A1  // Mid Left IR sensor, detects mid-left line position
#define S3 A2  // Mid Right IR sensor, detects mid-right line position
#define S4 A3  // Far Right IR sensor, detects far-right line position

// ======================
// Motor Speed & PID Constants
// ======================

// BASE_SPEED determines the baseline speed of both motors during straight movement
#define BASE_SPEED 100     // Base motor speed (value between 0 and 255)

// MAX_CORRECTION limits the maximum speed difference applied to correct the robot's direction
#define MAX_CORRECTION 80  // Maximum speed adjustment for correcting movement

// THRESHOLD is the IR sensors' sensitivity cutoff, differentiating between black line and white background
#define THRESHOLD 500      // IR sensor threshold for line detection

// ERROR_DEADBAND is the tolerance range within which small directional errors are ignored (drives straight)
#define ERROR_DEADBAND 2   // Tolerance level to prevent micro-corrections

// ======================
// Status LEDs & Buzzer Configuration
// ======================

// These components visually and audibly indicate the robot’s state
#define RED_LED 10   // LED lights up when the robot loses the line
#define GREEN_LED 11 // LED lights up when the robot is following the line
#define BUZZER 12    // Buzzer sounds when the robot loses the line

// ======================
// Memory Variables
// ======================

// `lastLinePos` stores the last known position of the line (-1 = Left, 1 = Right, 0 = Center)
int lastLinePos = 0;

void setup() {
  // Motor pins setup as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Speed control pins setup as OUTPUT
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Status indicator pins (LEDs and buzzer) set as OUTPUT
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // Begin serial communication for debugging purposes
  Serial.begin(9600);
}

// ======================
// Motor Control Functions
// ======================

// Function: Move both motors forward at a specified speed
void forward(int speed) {
  digitalWrite(IN2, HIGH);  // Left motor moves forward
  digitalWrite(IN1, LOW);   // Left motor direction LOW
  digitalWrite(IN4, HIGH);  // Right motor moves forward
  digitalWrite(IN3, LOW);   // Right motor direction LOW
  analogWrite(ENA, speed);  // Set left motor speed
  analogWrite(ENB, speed);  // Set right motor speed
}

// Function: Turn left by stopping the right motor and reducing left motor speed
void turnLeft(int speed) {
  digitalWrite(IN1, LOW);   // Left motor LOW direction
  digitalWrite(IN2, HIGH);  // Left motor forward
  analogWrite(ENA, speed);  // Reduce left motor speed
  digitalWrite(IN3, LOW);   // Right motor stopped
  digitalWrite(IN4, LOW);   // Right motor stopped
  analogWrite(ENB, 0);      // Stop right motor
}

// Function: Turn right by stopping the left motor and reducing right motor speed
void turnRight(int speed) {
  digitalWrite(IN1, LOW);   // Left motor stop
  digitalWrite(IN2, LOW);   // Left motor stop
  analogWrite(ENA, 0);      // Stop left motor
  digitalWrite(IN3, LOW);   // Right motor LOW
  digitalWrite(IN4, HIGH);  // Right motor forward
  analogWrite(ENB, speed);  // Reduce right motor speed
}

// Function: Spin anticlockwise (left motor backward, right motor forward)
void spinLeft(int speed) {
  digitalWrite(IN1, HIGH);  // Left motor backward
  digitalWrite(IN2, LOW);   // Left motor direction LOW
  digitalWrite(IN3, LOW);   // Right motor stop
  digitalWrite(IN4, HIGH);  // Right motor forward
  analogWrite(ENA, speed);  // Adjust left motor speed
  analogWrite(ENB, speed);  // Adjust right motor speed
}

// Function: Spin clockwise (left motor forward, right motor backward)
void spinRight(int speed) {
  digitalWrite(IN1, LOW);   // Left motor forward
  digitalWrite(IN2, HIGH);  // Left motor direction HIGH
  digitalWrite(IN3, HIGH);  // Right motor backward
  digitalWrite(IN4, LOW);   // Right motor direction LOW
  analogWrite(ENA, speed);  // Adjust left motor speed
  analogWrite(ENB, speed);  // Adjust right motor speed
}

// ======================
// Main Loop
// ======================

void loop() {
  float currentTimeSec = millis() / 1000.0;  // Track elapsed time in seconds

  // Read IR sensor analog values
  int v1 = analogRead(S1);  // Far Left sensor
  int v2 = analogRead(S2);  // Mid Left sensor
  int v3 = analogRead(S3);  // Mid Right sensor
  int v4 = analogRead(S4);  // Far Right sensor

  // Convert analog values to binary (1 for line detected, 0 for no line)
  int s1 = (v1 > THRESHOLD);
  int s2 = (v2 > THRESHOLD);
  int s3 = (v3 > THRESHOLD);
  int s4 = (v4 > THRESHOLD);

  // Calculate weighted error based on sensor positions to center the robot on the line
  float error = (-3 * s1) + (-1 * s2) + (1 * s3) + (3 * s4);

  // `lastLinePos` tracks the last known error direction for recovery
  if (error < 0) lastLinePos = -1;  // Line is to the left
  if (error > 0) lastLinePos = 1;   // Line is to the right

  // Handle lost line scenario (all sensors OFF)
  if (!s1 && !s2 && !s3 && !s4) {
    // Turn RED LED and buzzer ON to signal lost line detection
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BUZZER, HIGH);

    // Attempt recovery based on last known line position
    if (lastLinePos == -1) {
      spinRight(100);  // Rotate clockwise to find the line
    } else {
      spinLeft(100);   // Rotate anticlockwise to find the line
    }
    return;  // Skip remaining loop logic until the line is recovered
  }

  // Turn GREEN LED ON and RED LED/Buzzer OFF to indicate normal operation
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BUZZER, LOW);

  int leftSpeed, rightSpeed;

  // Apply deadband logic: Drive straight if the error is within tolerance
  if (abs(error) <= ERROR_DEADBAND) {
    leftSpeed = BASE_SPEED;  // Maintain uniform speed on straight paths
    rightSpeed = BASE_SPEED;
  } else {
    // Calculate correction factor for adjusting motor speeds
    int correction = error * 15;  // Proportional gain factor for correcting turns
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);  // Limit correction to valid range

    // Adjust motor speeds using proportional correction logic
    leftSpeed = BASE_SPEED + correction;  // Modify left speed based on error
    rightSpeed = BASE_SPEED - correction; // Modify right speed based on error
  }

  // Ensure motor speeds are within valid PWM range (0 to 255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  int steering_cmd = leftSpeed - rightSpeed;  // Difference between left and right motor speeds

  // Drive forward with the corrected motor speeds
  digitalWrite(IN2, HIGH);  // Left motor forward
  digitalWrite(IN1, LOW);   // Left motor direction LOW
  digitalWrite(IN4, HIGH);  // Right motor forward
  digitalWrite(IN3, LOW);   // Right motor direction LOW

  // Apply corrected speeds using PWM
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Output debugging information
  Serial.print(currentTimeSec, 2);   // Log time
  Serial.print(",");                // Separator
  Serial.print(steering_cmd);       // Log steering command
  Serial.print(",");
  Serial.println(error);            // Log error
}