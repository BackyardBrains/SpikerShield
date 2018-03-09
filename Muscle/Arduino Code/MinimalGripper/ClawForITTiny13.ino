/*
 * Backyard Brains 2018
 * 
Claw code for ITTINY 13A

We used MicroCore library from
https://github.com/MCUdude/MicroCore

Written by Stanislav Mircic

 */

unsigned int adcValue;
unsigned int trigger = 20;
unsigned int servoPeriod;
#define BOTTOM_THRESHOLD 50
#define SERVO_PERIOD 7230
#define BASE_DURATION 565
#define MAX_DURATION 880

void setup() {
  // initialize digital pin 13 as an output.
  pinMode(4, OUTPUT);
}

void loop() 
{
  servoPeriod++;
  if(servoPeriod == SERVO_PERIOD)
  {
    adcValue = analogRead(3);
    servoPeriod = 0;  
    
    if(adcValue>BOTTOM_THRESHOLD)
    {
      adcValue = adcValue - BOTTOM_THRESHOLD;  
    }
    else
    {
      adcValue = 0;
    }
    trigger = adcValue>>1;
    trigger = BASE_DURATION + trigger;

    if(trigger>MAX_DURATION)
    {
      trigger = MAX_DURATION;  
    }
    digitalWrite(4, HIGH);   
  }
  if(servoPeriod==trigger)
  {
    digitalWrite(4, LOW);
  }
}
