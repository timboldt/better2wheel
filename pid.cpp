#include "pid.h"

float PidInfo::update(float e) {
  accumulator += e;
  if (accumulator > accumulatorLimit) {
    accumulator = accumulatorLimit;
  } else if (accumulator < -accumulatorLimit) {
    accumulator = -accumulatorLimit;
  }
  float r = kp * e + ki * accumulator + kd * (e - previous);
  previous = e;
  return r;
}

void PidInfo::reset() { accumulator = 0; }