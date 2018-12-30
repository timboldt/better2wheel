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
//      D10 (D8 if using shield with Uno) - Left side, direction pin
//      D11 - Right side, direction pin
//      D12 - Right side, speed pin

#include <Arduino.h>
#include <wiring.h>

#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"

IMU imu;

Motor LeftMotor;
Motor RightMotor;

Encoder LeftEncoder;
Encoder RightEncoder;

PidInfo pidDistance(0, 0.1, 0, 5.0 / 180.0f * PI); 
PidInfo pidAngle(1145, 57, 0, 255);
// The differential PID controller handles differences in wheel speeds, and
// corrects for unintentional differential steering. The P-term is essential,
// but I'm not sure about the I-term.
PidInfo pidDifferential(100, 0.1, 0, 512);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  imu.Setup(2);  // Digital pin 2 for the I2C interrupt.
  LeftMotor.SetupPins(10, 3);
  RightMotor.SetupPins(12, 11);
  LeftEncoder.Setup(A0, A1);
  RightEncoder.Setup(A2, A3);

  Serial.println("Boot Complete");
}

void loop() {
  // Read inputs.
  float angle = imu.GetAngle();
  float leftOdometer = LeftEncoder.GetOdometer();
  float rightOdometer = RightEncoder.GetOdometer();

  // Check for unrecoverable tilt.
  if (abs(angle) > PI / 4.0f) {
    LeftMotor.SetSpeed(0);
    RightMotor.SetSpeed(0);
    pidDistance.reset();
    pidAngle.reset();
    pidDifferential.reset();
    Serial.println("Over-tilt!");
    return;
  }

  // Compute the speed of both wheels.
  float distance = (leftOdometer + rightOdometer) / 2.0f;
  float targetAngle = pidDistance.update(-distance);
  const float maxAngleOffset = 10.0 / 180.0f * PI;
  if (targetAngle > maxAngleOffset) {
    targetAngle = maxAngleOffset;
  } else if (targetAngle < -maxAngleOffset) {
    targetAngle = maxAngleOffset;
  }
  float speed = pidAngle.update(angle - targetAngle);
Serial.print((angle - targetAngle)/PI*180);
Serial.print("\t");
Serial.println(speed);
  float speedAdjust = pidDifferential.update(rightOdometer - leftOdometer);
  LeftMotor.SetSpeed(speedAdjust + speed);
  RightMotor.SetSpeed(speedAdjust - speed);
}