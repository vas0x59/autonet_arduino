#pragma once

#include <Arduino.h>

class Encoder {
public:
  Encoder(int pa, int pb);
  void init(int i1, int i2);
  void reset();
  long get();
  void A();
  void B();

private:
  
  void setCount(int state);
  int pinA, pinB;
  volatile long pause = 1;
  volatile long lastTurn = 0;
  volatile long count = 0;
  long actualcount = 0;
  volatile int state = 0;
  volatile int pinAValue = 0;
  volatile int pinBValue = 0;
};