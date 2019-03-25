#pragma once

#include <Arduino.h>

// The Encoder class manages a hall-effect wheel encoder with two inputs, A and
// B. The pattern of highs and lows determines the direction.
class Encoder {
 public:
  Encoder(uint8_t pinA, uint8_t pinB);

  float get_odometer();

 private:
  volatile int32_t odometer_;

  void interrupt_handler(uint8_t other_pin, uint8_t high_low);
};
