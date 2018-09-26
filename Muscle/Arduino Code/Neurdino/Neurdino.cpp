/*
  Neurdino.cpp
*/

#include "Arduino.h"
#include "Neurdino.h"


#define SHIFT_LATCH_PIN B00000100                       //latch pin for shift register        RCK - PB2
#define I_SHIFT_LATCH_PIN B11111011 
#define SHIFT_CLOCK_PIN B00000010                       //clock pin for shift register              PB1
#define I_SHIFT_CLOCK_PIN B11111101 
#define SHIFT_DATA_PIN  B00001000                        //serial data pin for shift register SER - PB3
#define I_SHIFT_DATA_PIN  B11110111 
#define BITMASK_ONE B00000001

Neurdino::Neurdino(void)
{
  //MISO as digital pin 14
  //MOSI as digital pin 16
  //SCK as digital pin 15
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
}

void Neurdino::writeLEDs(void)
{
  PORTB &=  I_SHIFT_LATCH_PIN;

  byte tempBitmask;
  tempBitmask = BITMASK_ONE;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(_shiftRegState & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      

   PORTB |=  SHIFT_LATCH_PIN;
}



void Neurdino::writeLEDs(byte outByte)
{
  PORTB &=  I_SHIFT_LATCH_PIN;
  _shiftRegState = outByte;
  byte tempBitmask;
  tempBitmask = BITMASK_ONE;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      tempBitmask = tempBitmask<<1;

      if(outByte & tempBitmask)
      {
          PORTB |=  SHIFT_DATA_PIN;
      }
      else
      {
          PORTB &=  I_SHIFT_DATA_PIN;
      }
    
      //pulse the clock for shift
      PORTB |=  SHIFT_CLOCK_PIN;
      PORTB &=  I_SHIFT_CLOCK_PIN;
      

   PORTB |=  SHIFT_LATCH_PIN;
}




void Neurdino::writeLED(int led, bool state)
{
  byte bitMask = BITMASK_ONE;
  if(led>7 || led <0)
  {
    return;
  }
  if(state)
  {
    _shiftRegState |= bitMask<<(7-led);
  }
  else
  {
     _shiftRegState &= ~(bitMask<<(7-led));
  }
  writeLEDs();
}