// A second attempt at making a 2-wheel balancing robot work well with an
// Arduino.
//
// My configuration:
//  - A slightly reconfigured version of the OSEPP two-wheel balancing kit.
//  - OSEPP T6612 motor shield, without the Uno.
//  - Nucleo-32 L432KC.
// Pins:
//   MPU6050:
//      A5 - SCL
//      A4 - SDA
//      D12 - INT (interrupt pin)
//   Wheel encoders:
//      A0 - Left side, channel B
//      A1 - Left side, channel A
//      A2 - Right side, channel A
//      A3 - Right side, channel B
//   Motors:
//      D2 - Left side, EN1
//      D3 - Left side, EN2
//      D6 - Left side, PWM
//      D9 - Right side, PWM
//      D10 - Right side, EN1
//      D11 - Right side, EN2
//   Ground:
//      Common ground with OSEPP motor shield.
// L432KC notes:
//   D0/D1 are for serial RX/TX only.
//   D4/D5 cannot be used if A4/A5 are being used for I2C.
//   D7/D8 cannot be used because they are needed by the oscillator.
//   D11 (and D12?) has no PWM support.
//   (And yes, I found all this out the hard way until I finally read the spec.)

#include <Arduino.h>
#include <wiring.h>

#include "encoder.h"
#include "imu.h"
#include "pid.h"
#include "wheel.h"

const bool DEBUG_OUTPUT = true;
const float SPEED_EXP_MOVING_AVG_ALPHA = 0.4;
const float MILLIS_PER_SECOND = 1000;
const float WHEEL_DIAMETER_MM = 50;

IMU imu;

Wheel leftWheel(11*45, WHEEL_DIAMETER_MM, 5*WHEEL_DIAMETER_MM/1000, D6, D2, D3, A0, A1);
Wheel rightWheel(11*45, WHEEL_DIAMETER_MM, 5*WHEEL_DIAMETER_MM/1000, D9, D10, D11, A2, A3);

PidInfo pidAverageSpeed(1, 0, 0, 5.0);
PidInfo pidAngle(0.5, 0.5/20.0, 1.5, 5);

// User input.
float targetAvgSpeed = 0;   // Revolutions per second.
float targetTurnSpeed = 0;  // Wheel speed difference in revolutions per second.

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  imu.Setup(D12);

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
    leftWheel.Stop();
    rightWheel.Stop();
    pidAverageSpeed.reset();
    pidAngle.reset();
    return;
  }

  // Get (exponential moving average of) wheel speed in revolutions per second
  // (positive value is forward).
  // float leftOdometer = LeftEncoder.GetOdometer();
  // float rightOdometer = RightEncoder.GetOdometer();
  // float leftSpeed =
  //     (leftOdometer - prevLeftOdometer) / elapsedTime * MILLIS_PER_SECOND;
  // float rightSpeed =
  //     (rightOdometer - prevRightOdometer) / elapsedTime * MILLIS_PER_SECOND;
  // prevLeftOdometer = leftOdometer;
  // prevRightOdometer = rightOdometer;
  // averageSpeed = SPEED_EXP_MOVING_AVG_ALPHA * (leftSpeed + rightSpeed) / 2.0f +
  //                (1 - SPEED_EXP_MOVING_AVG_ALPHA) * averageSpeed;

  // float commandAngle = pidAverageSpeed.update(targetAvgSpeed - averageSpeed);
  // float commandSpeed = -pidAngle.update(commandAngle - angle);

  // const float SPEED_POWER_RATIO = 255.0 / 5.0;
  // float leftPower =
  //     SPEED_POWER_RATIO * commandSpeed +
  //     pidLeftSpeed.update(commandSpeed - leftSpeed + targetTurnSpeed);
  // float rightPower =
  //     SPEED_POWER_RATIO * commandSpeed +
  //     pidRightSpeed.update(commandSpeed - rightSpeed - targetTurnSpeed);

  // LeftMotor.SetPower(-leftPower);
  // RightMotor.SetPower(rightPower);

  // if (DEBUG_OUTPUT) {
  //   // Serial.print(elapsedTime);
  //   // Serial.print("\t");
  //   Serial.print(commandAngle * 10);
  //   Serial.print("\t");
  //   Serial.print(angle * 10);
  //   // Serial.print("\t");
  //   // Serial.print(commandSpeed * 10);
  //   // Serial.print("\t");
  //   // Serial.print(rightPower / 255.0 * 100.0);
  //   // Serial.print("\t");
  //   // Serial.print(leftSpeed * 10);
  //   // Serial.print("\t");
  //   // Serial.print(rightSpeed * 10);
  //   Serial.print("\t");
  //   Serial.print(averageSpeed * 10);
  //   Serial.println();
  // }
}