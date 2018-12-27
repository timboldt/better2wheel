// This file derived from the examples at https://www.osepp.com/robotic-kits.

#include "motor.h"

const uint8_t max_speed = 255;

void Motor::SetupPins(uint8_t dir_pin, uint8_t pwm_pin) {
  // NOTE: On stm32duino, we can't do this in the constructor.
  _dir_pin = dir_pin;
  _pwm_pin = pwm_pin;

  pinMode(_dir_pin, OUTPUT);
  digitalWrite(_pwm_pin, LOW);
}

void Motor::SetSpeed(int speed) {
  // Clear the previous request.
  analogWrite(_pwm_pin, 0);

  // If the speed is negative, run the motor in reverse.
  if (speed < 0) {
    // Reverse.
    speed = -speed;
    digitalWrite(_dir_pin, LOW);
  } else {
    // Forward.
    digitalWrite(_dir_pin, HIGH);
  }

  // Clip the speed at the maximum value.
  if (speed > max_speed) {
    speed = max_speed;
  }

  // Set the motor direction and speed, as requested.
  analogWrite(_pwm_pin, speed);
}
