#pragma once

#include <Arduino.h>

class PidInfo {
 public:
  PidInfo(float p, float i, float d, float limit) {
    kp = p;
    ki = i;
    kd = d;
    previous = 0;
    accumulator = 0;
    accumulatorLimit = limit / i;
  }

  float update(float e);

  void reset();

 private:
  float kp, ki, kd;
  float previous;          // Previous error.
  float accumulator;       // Cummulative error.
  float accumulatorLimit;  // Maximum absolute value of accumulator.
};
