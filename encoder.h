#include <Arduino.h>

// The Encoder class manages a hall-effect wheel encoder with two inputs, A and
// B. The pattern of highs and lows determines the direction.
class Encoder {
 public:
  void Setup(uint8_t pinA, uint8_t pinB);
  float GetOdometer();

 private:
  volatile int32_t odometer;

  void InterruptHandler(uint8_t otherPin, uint8_t highLow);
};
