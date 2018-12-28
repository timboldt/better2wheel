#include "imu.h"

#include <pins_arduino.h>
// Use Jeff Rowberg's excellent MPU6050 library which uses highly-efficient
// interrupt-driven data capture.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// Indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void imuISR() { mpuInterrupt = true; }

void IMU::Setup(uint8_t digitalInterruptPin) {
  uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
  uint8_t devStatus;  // return status after each device operation (0 = success,
                      // !0 = error)
  bool dmpReady = false;  // set true if DMP init was successful
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock.
  mpu.initialize();
  pinMode(digitalInterruptPin, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    digitalPinToInterrupt(digitalInterruptPin);
    attachInterrupt(digitalPinToInterrupt(digitalInterruptPin), imuISR, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// This function will busy-wait until the next interrupt, but presumably without
// new information, there is nothing to do. Based on some quick testing,
// interrupts happen once every 7ms or so, giving a control loop frequency of
// about 145Hz.
float IMU::GetAngle() {
  static float angle = 0.0;

  // MPU control/status vars
  uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
  uint8_t devStatus;  // return status after each device operation (0 = success,
                      // !0 = error)
  bool dmpReady = false;   // set true if DMP init was successful
  uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;      // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64];  // FIFO storage buffer

  // orientation/motion vars
  Quaternion q;    // [w, x, y, z]         quaternion container
  VectorInt16 aa;  // [x, y, z]            accel sensor measurements
  VectorInt16
      aaReal;  // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16
      aaWorld;  // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;  // [x, y, z]            gravity vector
  float euler[3];       // [psi, theta, phi]    Euler angle container
  float ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity
                 // vector

  // If DMP programming failed, don't try to do anything.
  if (!dmpReady) return angle;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too
  // inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) ||
      fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen
    // frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angle = ypr[2] > 0 ? PI - ypr[2] : -1 * (PI + ypr[2]);
  }

  return angle;
}
