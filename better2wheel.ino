// A second attempt at making a 2-wheel balancing robot work well with an
// Arduino.
//
// My configuration:
//  - A slightly reconfigured version of the OSEPP two-wheel balancing kit.
//  - Arduino Uno.
// Orientation:
//  Right Side - electronics
// Pins:
//   MPU6050:
//      A5 - SCL
//      A4 - SDA
//      D2 - INT (interrupt pin)
//   Wheel encoders:
//      A0 - Left side, channel B
//      A1 - Left side, channel A
//      A2 - Right side, channel A
//      A3 - Right side, channel B
//   Motors:
//      D3 - Left side, speed pin
//      D4 (D8 if using shield with Uno) - Left side, direction pin
//      D5 - Right side, speed pin
//      D6 - Right side, direction pin

#include <Arduino.h>
#include <wiring.h>

#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"

const bool DEBUG_OUTPUT = true;
const float SPEED_EXP_MOVING_AVG_ALPHA = 0.4;
const float MILLIS_PER_SECOND = 1000;

IMU imu;

Motor LeftMotor;
Motor RightMotor;

Encoder LeftEncoder;
Encoder RightEncoder;

PidInfo pidAverageSpeed(0, 0.1, 0, 5.0 / 180.0f * PI);
PidInfo pidAngle(1145, 57, 0, 5);
PidInfo pidLeftSpeed(10, 0.05, 20, 5);
PidInfo pidRightSpeed(10, 0.05, 20, 5);

// User input.
float targetAvgSpeed = 0;   // Revolutions per second.
float targetTurnSpeed = 0;  // Wheel speed difference in revolutions per second.

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  imu.Setup(2);  // Digital pin 2 for the I2C interrupt.
  LeftMotor.SetupPins(4, 3);
  RightMotor.SetupPins(6, 5);
  LeftEncoder.Setup(A0, A1);
  RightEncoder.Setup(A2, A3);

  Serial.println("Boot Complete");
}

void loop() {
  static float prevTime = 0;
  static float prevLeftOdometer = 0;
  static float prevRightOdometer = 0;
  static float averageSpeed = 0;

  // Get elapsed time.
  float currTime = millis();
  float elapsedTime = currTime - prevTime;
  prevTime = currTime;
  if (elapsedTime < 1) return;

  // Get tilt angle in degrees (straight up is zero; positive angle is forward).
  float angle = imu.GetAngle();
  if (abs(angle) > 30.0) {
    // Check for unrecoverable tilt.
    LeftMotor.SetPower(0);
    RightMotor.SetPower(0);
    pidAverageSpeed.reset();
    pidAngle.reset();
    pidLeftSpeed.reset();
    pidRightSpeed.reset();
    return;
  }

  // Get (exponential moving average of) wheel speed in revolutions per second
  // (positive value is forward).
  float leftOdometer = LeftEncoder.GetOdometer();
  float rightOdometer = RightEncoder.GetOdometer();
  float leftSpeed =
      (leftOdometer - prevLeftOdometer) / elapsedTime * MILLIS_PER_SECOND;
  float rightSpeed =
      (rightOdometer - prevRightOdometer) / elapsedTime * MILLIS_PER_SECOND;
  prevLeftOdometer = leftOdometer;
  prevRightOdometer = rightOdometer;
  averageSpeed = SPEED_EXP_MOVING_AVG_ALPHA * (leftSpeed + rightSpeed) / 2.0f +
                 (1 - SPEED_EXP_MOVING_AVG_ALPHA) * averageSpeed;

  float commandAngle = pidAverageSpeed.update(targetAvgSpeed - averageSpeed);
  float commandSpeed = pidAngle.update(commandAngle - angle);

commandSpeed = 1;

  const float SPEED_POWER_RATIO = 255.0 / 5.0;
  float leftPower =
      SPEED_POWER_RATIO * commandSpeed +
      pidLeftSpeed.update(commandSpeed - leftSpeed + targetTurnSpeed);
  float rightPower =
      SPEED_POWER_RATIO * commandSpeed +
      pidRightSpeed.update(commandSpeed - rightSpeed - targetTurnSpeed);

  LeftMotor.SetPower(-leftPower);
  RightMotor.SetPower(rightPower);

  if (DEBUG_OUTPUT) {
    // Serial.print(elapsedTime);
    // Serial.print("\t");
    // Serial.print(angle);
    // Serial.print("\t");
    Serial.print(commandSpeed);
    // Serial.print("\t");
    // Serial.print(rightPower / 255.0 * 100.0);
    // Serial.print("\t");
    // Serial.print(leftSpeed);
    // Serial.print("\t");
    // Serial.print(rightSpeed);
    Serial.print("\t");
    Serial.print(averageSpeed);
    Serial.println();
  }
}