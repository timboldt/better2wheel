#include <Arduino.h>

// The IMU class manages an inertial measurement unit and returns the tilt angle
// of the robot. It is currently implemented as a singleton (but this isn't
// enforced, so don't try to create more than one instance of this class). This
// class currently only works with an MPU6050 6-axis gyro/accelerometer.
class IMU {
 public:
  void Setup(uint8_t digitalInterruptPin);
  float GetAngle();
};
