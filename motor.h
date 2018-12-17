// This file derived from the examples at https://www.osepp.com/robotic-kits.

#include <Arduino.h>

// This class has been tested with the OSEPP DC motors that come with the Tank
// Kit and the 2-Wheel Balancing Kit.
class Motor {
 public:
  // 'dir_pin' is the digital output pin that controls the motor direction.
  // 'pwm_pin' is the analog output pin that controls the motor speed.
  // Typical values on the OSEPP motor shield are (12, 11) for motor #1 and (8,
  // 3) for motor #2.
  Motor(uint8_t dir_pin, uint8_t pwm_pin);

  // Sets the motor speed.
  // 'speed' should be between -255 and +255. A negative speed runs the motor in
  // reverse.
  void SetSpeed(int speed);

 private:
  uint8_t _dir_pin, _pwm_pin;
};
