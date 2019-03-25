/*
  L298N.cpp - Libreria per gestire i movimenti di un motore DC con il modulo L298N
  Autore:   Andrea Lombardo
  Sito web: http://www.lombardoandrea.com  
*/

#ifndef L298N_h
#define L298N_h

#include "Arduino.h"


typedef void (*CallBackFunction) ();

class L298N{
   public:
      typedef enum
      {
            FORWARD  = 0,
            BACKWARD = 1
      } Direction;
      L298N(uint8_t pinEnable, uint8_t pinIN1, uint8_t pinIN2);
      void setSpeed(unsigned short pwmVal);
      unsigned short getSpeed();
      void forward();
      void forwardFor(unsigned long delay, CallBackFunction callback);
      void forwardFor(unsigned long delay);
      void backward();
      void backwardFor(unsigned long delay, CallBackFunction callback);
      void backwardFor(unsigned long delay);
      void run(uint8_t direction);
      void stop();
      void reset();
      boolean isMoving();
   private:
      byte _pinEnable;
      byte _pinIN1;
      byte _pinIN2;
      byte _pwmVal;
      unsigned long _lastMs;
      boolean _canMove;
      boolean _isMoving;
      static void fakeCallback();
};

#endif