// A second attempt at making a 2-wheel balancing robot work well with an
// Arduino.
//
// My configuration:
//  - A slightly reconfigured version of the OSEPP two-wheel balancing kit.
//  - Arduino Uno.
// Orientation:
//  Right Side - electronics
// Pins:
//   MPU6050:
//      A5 - SCL
//      A4 - SDA
//      D2 - INT (interrupt pin)
//   Wheel encoders:
//      A0 - Left side, channel A
//      A1 - Left side, channel B
//      A2 - Right side, channel A
//      A3 - Right side, channel B
//   Motors:
//      D3 - Left side, speed pin
//      D10 (D8 if using shield with Uno) - Left side, direction pin
//      D11 - Right side, direction pin
//      D12 - Right side, speed pin

#include <Arduino.h>
#ifdef __AVR__
#include <avr/wdt.h>
#endif
#include <pins_arduino.h>
// Use Jeff Rowberg's excellent MPU6050 library which uses highly-efficient
// interrupt-driven data capture.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "motor.h"

// ================================================================
//   MPU6050 IMU code
// ================================================================

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
                      // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
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
float
    ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void imuISR() { mpuInterrupt = true; }

void imuSetup() {
  const int INTERRUPT_PIN = 2;

  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock.
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  //   mpu.setXGyroOffset(220);
  //   mpu.setYGyroOffset(76);
  //   mpu.setZGyroOffset(-85);
  //   mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    digitalPinToInterrupt(INTERRUPT_PIN);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), imuISR, RISING);
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
float imuGetAngle() {
  static float angle = 0.0;

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

// ================================================================
//   Motor Control
// ================================================================

Motor LeftMotor;
Motor RightMotor;

void motorSetup() {
#ifdef __AVR__
  // Change the timer frequency
  // To avoid the frequency of hearing.
  TCCR2B &= ~7;
  TCCR2B |= 1;
#endif

  LeftMotor.SetupPins(10, 3);
  RightMotor.SetupPins(12, 11);
}

// ================================================================
//   Encoder Reader
// ================================================================

#define LEFT_ENCODER_A (1 << 0)
#define LEFT_ENCODER_B (1 << 1)
#define RIGHT_ENCODER_B (1 << 2)  // Intentionally reversed from physical pins.
#define RIGHT_ENCODER_A (1 << 3)  // Intentionally reversed from physical pins.

volatile long int rightOdometer = 0;
volatile long int leftOdometer = 0;

void encoderISR() {
  // TODO
}

#ifdef __AVR__
ISR(PCINT1_vect) {
  static char oldPin = 0xff;
  char in = PINC;
  char change = in ^ oldPin;
  if (change & RIGHT_ENCODER_A) {
    if (in & RIGHT_ENCODER_A) {
      if (in & RIGHT_ENCODER_B) {
        rightOdometer++;
      } else {
        rightOdometer--;
      }
    } else {
      if (in & RIGHT_ENCODER_B) {
        rightOdometer--;
      } else {
        rightOdometer++;
      }
    }
  } else if (change & (RIGHT_ENCODER_B)) {
    if (in & (RIGHT_ENCODER_B)) {
      if (in & RIGHT_ENCODER_A) {
        rightOdometer--;
      } else {
        rightOdometer++;
      }
    } else {
      if (in & RIGHT_ENCODER_A) {
        rightOdometer++;
      } else {
        rightOdometer--;
      }
    }
  }

  if (change & LEFT_ENCODER_A) {
    if (in & LEFT_ENCODER_A) {
      if (in & LEFT_ENCODER_B) {
        leftOdometer++;
      } else {
        leftOdometer--;
      }
    } else {
      if (in & LEFT_ENCODER_B) {
        leftOdometer--;
      } else {
        leftOdometer++;
      }
    }
  } else if (change & (LEFT_ENCODER_B)) {
    if (in & (LEFT_ENCODER_B)) {
      if (in & LEFT_ENCODER_A) {
        leftOdometer--;
      } else {
        leftOdometer++;
      }
    } else {
      if (in & LEFT_ENCODER_A) {
        leftOdometer++;
      } else {
        leftOdometer--;
      }
    }
  }
  oldPin = in;
}
#endif

void encoderSetup() {
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
#ifdef __AVR__
  PCIFR = 1 << 1;
  PCMSK1 = RIGHT_ENCODER_A | RIGHT_ENCODER_B | LEFT_ENCODER_A | LEFT_ENCODER_B;
  PCICR = 1 << 1;
  sei();
#endif
}

// ================================================================
//   PID Control
// ================================================================

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

// ================================================================
//   Mainline Code
// ================================================================

PidInfo pidDistance(0, 0, 0, 0.035);  // old=(0, 1.0, 0, 0.035)
PidInfo pidAngle(1145, 57, 3438, 255);

// The differential PID controller handles differences in wheel speeds, and
// corrects for unintentional differential steering. The P-term is essential,
// but I'm not sure about the I-term.
PidInfo pidDifferential(1, 1e-4, 0, 512);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

#ifdef __AVR__
  // Setup a watchdog
  // When the battery voltage is insufficient / program unexpected
  // Watchdog reset chip
  wdt_enable(WDTO_1S);
  wdt_reset();
#endif

  imuSetup();
  motorSetup();
  //!!!encoderSetup();
  Serial.println("Boot Complete");
}

void loop() {
  static float targetAngle = 0;

#ifdef __AVR__
  wdt_reset();
#endif

  float angle = imuGetAngle();
  {
    // Serial.print(targetAngle);
    // Serial.print("\t");
    Serial.println(angle * 180 / PI);
    // Serial.print("\t");
    // Serial.println(rightOdometer);
  }
  if (abs(angle) > PI / 6) {
    // Game over. Stop the motors.
    LeftMotor.SetSpeed(0);
    RightMotor.SetSpeed(0);
    pidDistance.reset();
    pidAngle.reset();
    pidDifferential.reset();
    Serial.println("Over-tilt!");
    return;
  }
  float distance = leftOdometer / 1e6;
  targetAngle = pidDistance.update(-distance);
  const float twoDegrees = 0.035;
  if (targetAngle > twoDegrees) {
    targetAngle = twoDegrees;
  } else if (targetAngle < -twoDegrees) {
    targetAngle = twoDegrees;
  }
  float speed = pidAngle.update(angle - targetAngle);
  float speedAdjust = pidDifferential.update(leftOdometer - rightOdometer);
  // XXXX
  speedAdjust = 0;
  LeftMotor.SetSpeed(speedAdjust + speed);
  RightMotor.SetSpeed(speedAdjust - speed);
}