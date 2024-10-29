# Balancing Robot

This project is an Arduino-based self-balancing robot that uses a combination of an MPU6050 gyro/accelerometer sensor, an L298N motor driver, and PID control to maintain an upright position. The robot continuously adjusts its motors based on tilt data to achieve stability. This is a great project for exploring control systems, robotics, and sensor fusion techniques, including the use of the Kalman Filter.

## Features

- **Self-Balancing**: The robot automatically balances itself by adjusting motor speed and direction based on tilt angle.
- **PID Control**: A Proportional-Integral-Derivative (PID) controller is implemented to calculate precise adjustments to motor output based on the angle error, integral of past errors, and the rate of change in error.
- **Kalman Filter**: The Kalman filter is used to improve the accuracy of the angle readings from the MPU6050 sensor, combining gyroscope and accelerometer data for a stable measurement.

## Components Used

1. **Arduino Board** (e.g., Arduino Uno or Nano): The main microcontroller that processes sensor data and controls the motors.
2. **MPU6050 Gyroscope and Accelerometer**: Provides tilt and rotation data. This sensor is crucial for determining the angle of the robot relative to the ground.
3. **L298N Motor Driver**: Controls the speed and direction of the motors based on the PID output.
4. **DC Motors with Wheels**: Provide movement and stabilization.
5. **Battery**: Power supply for the entire system.

## How It Works

1. **Angle Measurement**:
   - The MPU6050 sensor provides both gyroscope and accelerometer data. The accelerometer measures the tilt angle relative to gravity, while the gyroscope measures the rate of change in angle.
   - These readings are combined using a **Kalman Filter** to create a more accurate and stable angle measurement, filtering out noise from the accelerometer and compensating for gyroscope drift over time.

2. **Kalman Filter**:
   - The Kalman filter is applied to stabilize the angle measurement by fusing gyroscope and accelerometer data. This filter predicts the next state (angle and bias) and then corrects it based on new measurements, minimizing error. The result is a smooth and reliable angle reading, essential for precise control.

3. **PID Control**:
   - The PID controller continuously calculates the error between the current angle and the target angle (usually set to 0 degrees for an upright position).
   - The controller adjusts motor speed based on three components:
     - **Proportional (P)**: Corrects based on the current angle error.
     - **Integral (I)**: Accounts for cumulative errors over time.
     - **Derivative (D)**: Reacts to the rate of change in error.
   - The combined output from the PID controller adjusts the motors to counteract any tilt and keep the robot balanced.

4. **Motor Adjustment**:
   - Based on the PID output, the L298N motor driver controls the motors' speed and direction to keep the robot balanced. If the robot tilts forward, the motors spin in a direction to bring it back upright, and vice versa.

## Code Explanation

- **Kalman Filter Implementation**: The code uses a Kalman filter function that fuses accelerometer and gyroscope data for accurate angle estimation.
- **PID Control Loop**: In the main loop, the PID controller calculates adjustments for the motors based on the filtered angle measurement.
- **Motor Control**: The motor speed and direction are adjusted in real-time to balance the robot.

## How to Run the Code

1. Upload the provided `.ino` file to your Arduino board.
2. Ensure that the MPU6050, L298N motor driver, and motors are correctly connected to the Arduino.
3. Power the robot with a suitable battery and place it on a flat surface.
4. The robot should begin balancing automatically. Adjust PID values if necessary for optimal performance.

## Tips for Calibration

- **PID Tuning**: Adjust `Kp`, `Ki`, and `Kd` values to improve the balancing performance. Start with `Kp` for basic correction, then add `Kd` for stability, and finally, use `Ki` sparingly to correct any persistent tilt over time.
- **Kalman Filter**: The Kalman filter parameters (such as process and measurement noise) might need minor adjustments depending on your setup for optimal stability.

## Challenges and Future Improvements

- **Surface Sensitivity**: The robot may perform differently on various surfaces. Further tuning might be needed for different terrains.
- **Battery Management**: For consistent performance, consider adding a voltage regulator or battery monitor to prevent drops in power.
- **Obstacle Avoidance**: Adding sensors (like ultrasonic or infrared) could allow the robot to avoid obstacles.

This balancing robot project is a rewarding way to dive into the world of control systems, sensor fusion, and robotics. The Kalman filter and PID controller are powerful tools for any engineer interested in creating stable and responsive systems.

