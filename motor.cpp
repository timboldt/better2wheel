// This file derived from the examples at https://www.osepp.com/robotic-kits.

#include "motor.h"

const uint8_t max_power = 255;

void Motor::SetupPins(uint8_t dir_pin, uint8_t pwm_pin) {
  // NOTE: On stm32duino, we can't do this in the constructor.
  _dir_pin = dir_pin;
  _pwm_pin = pwm_pin;

  pinMode(_dir_pin, OUTPUT);
  pinMode(_pwm_pin, OUTPUT);
  analogWrite(_pwm_pin, 0);
}

void Motor::SetPower(int power) {
  // Clear the previous request.
  analogWrite(_pwm_pin, 0);

  // If the power is negative, run the motor in reverse.
  if (power < 0) {
    // Reverse.
    power = -power;
    digitalWrite(_dir_pin, LOW);
  } else {
    // Forward.
    digitalWrite(_dir_pin, HIGH);
  }

  // Clip the power at the maximum value.
  if (power > max_power) {
    power = max_power;
  }

  // Set the motor direction and power, as requested.
  analogWrite(_pwm_pin, power);
}
