//
// This code is made for communication between mouse and SpikeRecorder
// Code works ONLY with Arduino Due 
//

#include <MouseController.h>

// Initialize USB Controller
USBHost usb;

// Attach mouse controller to USB
MouseController mouse(usb);

byte outputFrameBuffer[4]; 
bool newDataArrived;
void setup(){
  Serial.begin(210800);
  InitTimer();
}

void loop(){
  usb.Task();
  if(newDataArrived)
  {
    
    newDataArrived = false;  
  }
}
uint16_t mousex;
uint16_t mousey;

void mouseMoved() {
  mousex = (uint16_t)(mouse.getXChange()+512);
  mousey = (uint16_t)(mouse.getYChange()+512);
}

void InitTimer()
{

  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);     // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC7);   // enable peripheral clock TC7

  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */TC2,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4); 
  TC_SetRC(TC2, 1, 132);
  TC_Start(TC2, 1);

  // enable timer interrupts on the timer
  TC2->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;   // IER = interrupt enable register
  TC2->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;  // IDR = interrupt disable register

  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number (=(1*3)+1) for timer1 channel1 */
  NVIC_EnableIRQ(TC7_IRQn);
}

void TC7_Handler()
{
  // We need to get the status to clear it and allow the interrupt to fire again
  TC_GetStatus(TC2, 1);
  newDataArrived = true;


  outputFrameBuffer[0]= (mousex>>7)| 0x80;           //convert data to frame according to protocol
  outputFrameBuffer[1]=  mousex & 0x7F;              //first bit of every byte is used to flag start of the frame
  outputFrameBuffer[2]= (mousey>>7)& 0x7F;           //so first bit is set only on first byte of frame (| 0x80)
  outputFrameBuffer[3]=  mousey & 0x7F;
  Serial.write(outputFrameBuffer,4);                
  mousex = 512;
  mousey = 512;
}
