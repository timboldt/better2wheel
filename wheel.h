#pragma once

#include "L298N.h"
#include "encoder.h"
#include "pid.h"

class Wheel {
 public:
  // Ticks per rev is the number of interrupts on one encoder pin per revolution.
  // Wheel diameter is in millimeters.
  // Max speed is in meters per second.
  // Requires 3 pins for the motor: enable (PWM), in1 (forward) and in2
  // (reverse). Requires 2 pins for the encoder: A and B.
  Wheel(uint16_t ticks_per_rev, float diameter_in_mm, float max_speed_in_mps, uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2,
        uint8_t pinEncoderA, uint8_t pinEncoderB);

  // Speed is in meters per second.
  void SetTargetSpeed(float meters_per_second);

  void Run();
  void Stop();

 private:
  Encoder encoder_;
  L298N motor_;
  PidInfo pid_;

  // pidLeftSpeed.reset();
  // pidRightSpeed.reset();
  // PidInfo pidLeftSpeed(10, 0.05, 20, 5);
  // PidInfo pidRightSpeed(10, 0.05, 20, 5);
  //       LeftMotor.SetupPins(6, 3);
  //   RightMotor.SetupPins(10, 9);
  //   LeftEncoder.Setup(A0, A1);
  //   RightEncoder.Setup(A2, A3);
};