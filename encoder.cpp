#include "encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(pinA, [this, pinB] { this->interrupt_handler(pinB, HIGH); },
                  RISING);
  attachInterrupt(pinB, [this, pinA] { this->interrupt_handler(pinA, LOW); },
                  RISING);
}

float Encoder::get_odometer() {
  const float TICKS_PER_REVOLUTION = 909.0f;
  return odometer_ / TICKS_PER_REVOLUTION;
}

void Encoder::interrupt_handler(uint8_t otherPin, uint8_t highLow) {
  if (digitalRead(otherPin) == highLow) {
    odometer_++;
  } else {
    odometer_--;
  }
}
