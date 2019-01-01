// This file derived from the examples at https://www.osepp.com/robotic-kits.

#include "motor.h"

const uint8_t max_power = 255;

void Motor::SetupPins(uint8_t dir_pin, uint8_t pwm_pin) {
  // NOTE: On stm32duino, we can't do this in the constructor.
  dir_pin_ = dir_pin;
  pwm_pin_ = pwm_pin;

  pinMode(dir_pin_, OUTPUT);
  digitalWrite(dir_pin_, LOW);
  pinMode(pwm_pin_, OUTPUT);
  analogWrite(pwm_pin_, 0);
}

void Motor::SetPower(int power) {
  // Limit the rate of change.
  // const int max_diff = 100;
  // int diff = power - prev_power_;
  // if (diff > max_diff) {
  //   diff = max_diff;
  // }
  // if (diff < -max_diff) {
  //   diff = -max_diff;
  // }
  // power = prev_power_ + diff;
  // prev_power_ = power;

  // Clear the previous request.
  analogWrite(pwm_pin_, 0);

  // If the power is negative, run the motor in reverse.
  if (power < 0) {
    // Reverse.
    power = -power;
    digitalWrite(dir_pin_, LOW);
  } else {
    // Forward.
    digitalWrite(dir_pin_, HIGH);
  }

  // Clip the power at the maximum value.
  if (power > max_power) {
    power = max_power;
  }

  // Set the motor direction and power, as requested.
  analogWrite(pwm_pin_, power);
}
