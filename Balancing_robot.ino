#include <Wire.h>
#include <MPU6050.h>

// Create an MPU6050 object
MPU6050 mpu;

// Variables for angle calculation
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Filter constants
float angleKalman = 0;  // Angle estimated by the Kalman filter
float bias = 0;         // Gyroscope bias estimated by the Kalman filter
float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix
float Q_angle = 0.001;  // Process noise for the angle
float Q_bias = 0.003;   // Process noise for the bias
float R_measure = 0.03; // Measurement noise

// PID constants
float Kp = 1.2;   // Proportional gain
float Ki = 0.05;  // Integral gain
float Kd = 0.25;  // Derivative gain

// PID variables
float setPoint = 180;  // Desired angle (upright position)
float input, output;
float error, prevError;
float integral, derivative;

// Pins for L298N motor driver
const int enA = 9;   // PWM pin for motor A
const int in1 = 8;   // Direction pin 1 for motor A
const int in2 = 7;   // Direction pin 2 for motor A
const int enB = 10;  // PWM pin for motor B
const int in3 = 6;   // Direction pin 1 for motor B
const int in4 = 5;   // Direction pin 2 for motor B

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  Serial.println("Initializing the MPU6050...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
  Serial.println("MPU6050 connected successfully!");

  // Set up motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize PID variables
  integral = 0;
  prevError = 0;
}

void loop() {
  // Calculate the robot's angle using the Kalman filter
  input = getFilteredAngleY();

  // PID calculation
  error = setPoint - input;
  integral += error * 0.150;  // Assuming a loop time of around 150ms
  integral = constrain(integral, -255, 255);  // Integral windup prevention
  derivative = (error - prevError) / 0.150;
  output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -255, 255);  // Constrain PID output
  prevError = error;

  // Control motors based on PID output
  setMotorSpeed(output);

  // Debugging
  Serial.print("Angle Y: ");
  Serial.print(input);
  Serial.print(" Output: ");
  Serial.print(output);
  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print(" Integral: ");
  Serial.print(integral);
  Serial.print(" Derivative: ");
  Serial.println(derivative);

  delay(150);  // Delay to reduce sampling frequency
}

// Function to implement the Kalman filter to calculate tilt angle
float kalmanFilter(float newAngle, float newRate, float dt) {
  // State prediction
  angleKalman += dt * (newRate - bias);

  // Update the error covariance matrix
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Calculate Kalman Gain
  float S = P[0][0] + R_measure;  // Estimate error
  float K[2];                     // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Update the estimated angle with accelerometer's measured angle
  float y = newAngle - angleKalman;  // Angle difference
  angleKalman += K[0] * y;
  bias += K[1] * y;

  // Update error covariance matrix
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angleKalman;
}

// Function to get the filtered angle using Kalman filter
float getFilteredAngleY() {
  // Read data from accelerometer and gyroscope
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate the raw angle from the accelerometer relative to the Y-axis
  float accAngleY = atan2(ax, az) * 180 / PI;

  // Convert gyroscope rate to degrees per second
  float gyroRateY = gy / 131.0;  // MPU6050 scale of 131 LSB per degree/s

  // Calculate time elapsed (delta t) in seconds
  float dt = 0.150;  // Assuming a loop delay of 150ms

  // Apply Kalman filter
  return kalmanFilter(accAngleY, gyroRateY, dt);
}

void setMotorSpeed(float speed) {
  int motorSpeed = constrain(abs(speed), 0, 255);  // Limit speed within PWM range

  if (speed > 0) {
    // Motors move forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (speed < 0) {
    // Motors move backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    // Stop motors if speed is zero
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}


