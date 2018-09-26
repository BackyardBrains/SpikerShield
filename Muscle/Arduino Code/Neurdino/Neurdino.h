/*
  Neurdino.h
*/
#ifndef Neurdino_h
#define Neurdino_h

#include "Arduino.h"

class Neurdino
{
  public:
    Neurdino(void);
    void writeLEDs(void);
    void writeLEDs(byte outByte);
    void writeLED(int led, bool state);
  private:
    byte _shiftRegState = 0;
};

#endif