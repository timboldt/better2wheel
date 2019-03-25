#include "wheel.h"

Wheel::Wheel(uint16_t ticks_per_rev, float diameter_in_mm, float max_speed,
             uint8_t motor_pwm, uint8_t motor_forward_pin,
             uint8_t motor_backward_pin, uint8_t encoder_pin_a,
             uint8_t encoder_pin_b)
    : motor_(motor_pwm, motor_forward_pin, motor_backward_pin),
      encoder_(encoder_pin_a, encoder_pin_b),
      pid_(10, 0.05, 20, 5) {
}

void Wheel::SetTargetSpeed(float meters_per_second) {}

void Wheel::Run() {
    // TODO
    motor_.run(100);
}

void Wheel::Stop() {
  motor_.stop();
  pid_.reset();
}