
#include "Neurdino.h"

Neurdino board;

void setup() {

}

void loop() 
{

  //Write each LED separately 
  delay(1000);
  board.writeLED(0,HIGH);
  delay(1000);
  board.writeLED(1,HIGH);
  delay(1000);
  board.writeLED(2,HIGH);
  delay(1000);
  board.writeLED(3,HIGH);
  delay(1000);
  board.writeLED(4,HIGH);
  delay(1000);

  //Write all LEDs at once. Faster. 
  board.writeLEDs(B11111111);
  board.writeLEDs(B11111111);
  board.writeLEDs(B00111111);
  board.writeLEDs(B00011111);
  board.writeLEDs(B10001111);
  board.writeLEDs(B00001101);
  board.writeLEDs(B00000000);
  
  board.writeLED(0,LOW);
  board.writeLED(1,LOW);
  board.writeLED(2,LOW);
  board.writeLED(3,LOW);
  board.writeLED(4,LOW);
  board.writeLED(5,LOW);
  board.writeLED(6,LOW);
  board.writeLED(7,LOW);
}
