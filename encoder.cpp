#include "encoder.h"

void Encoder::Setup(uint8_t pinA, uint8_t pinB) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(pinA, [this, pinB] { this->InterruptHandler(pinB, HIGH); },
                  RISING);
  attachInterrupt(pinB, [this, pinA] { this->InterruptHandler(pinA, LOW); },
                  RISING);
}

float Encoder::GetOdometer() {
  const float TICKS_PER_REVOLUTION = 909.0f;
  return odometer / TICKS_PER_REVOLUTION;
}

void Encoder::InterruptHandler(uint8_t otherPin, uint8_t highLow) {
  if (digitalRead(otherPin) == highLow) {
    odometer++;
  } else {
    odometer--;
  }
}
