# Longsword IMU BLE sensors

Working demo: https://www.youtube.com/watch?v=v7hvOyPQ0EM

The .ino arduino (Genuino 101) src code, that acquires raw IMU data from I2C sensors and sends them via Bluetooth Low Energy to the Nvidia Jetson TX2. The sampling frequency 1/25 Hz. It publishes data to the BLE characteristics from 3-axis: Internal Accelerometer, Gyroscope & step counter, MPU-6050 Accelerometer + Gyroscope, HMC5883L Magnetometer.

## Technologies

- Inertial Measurement Units (IMU): Accelerometer, Magnetometer, Gyroscope, Step counter
- Bluetooth Low Energy (BLE) byte stream

## Hardware
- Genuino 101
- MPU-6050
- HMC5883L

## SDKs & Libraries
- Wire
- Imu
- BLE