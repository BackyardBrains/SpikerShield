/*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains 22. March. 2018
  * 
  * TENS device code with Spike Recorder
  * 
  * Made for Arduino Leonardo
  * A0 - EMG input
  * A1 - ---
  * A2 - ---
  * A3 - Pulse width potentiometer
  * A4 - Pulse frequency potentiometer
  * A5 - Natery voltage input
  * 
  * 
  * 
  * D0  - --- 
  * D1  - aux button
  * D2  - VU LED 2 
  * D3  - ---
  * D4  - Low Battery indicator
  * D5  - TENS ON/OFF control output
  * D6  - power ON LED
  * D7  - Sensitivity selection button input
  * D8  - VU LED 1
  * D9  - Pulse generator output for TENS
  * D10 - VU LED 3
  * D11 - VU LED 4
  * D12 - VU LED 5
  * D13 - VU LED 6
  * 
  * 
  * V0.1
  * Written by Stanislav Mircic
  *
  * ----------------------------------------------------------------------------------------------------
  */

                                                //Clear/reset bit in register "cbi" macro
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
                                                //Set bit in register "sbi" macro 
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#define CURRENT_SHIELD_TYPE "HWT:MUSCLESS;"     //type of the board. Used for detection of shield 
                                                //by desktop Spike Recorder application
#define NOISE_FLOOR_FOR_ENVELOPE 530            //must be greater than 512 and less than 1023
#define ANTI_FLICKERING_TIME_IN_MS 50           //relay refresh period in ms
#define BATTERY_THRESHOLD_VOLTAGE 712           //voltage threshold for empty battery (0-1024 ADC units)


uint16_t antiFlickeringCounterMax;              //relay refresh period in sample periods
uint16_t antiFlickeringTimerForOutput;          //timer for relay state refresh in sample periods
                                                //it decreases every sample period and when we hit zero 
                                                //we update relay state

#define SENSITIVITY_BUTTON_PIN 7                //pin for button that changes sensitivity

#define SIZE_OF_COMMAND_BUFFER 30               //command buffer size (for commands received from serial)
char commandBuffer[SIZE_OF_COMMAND_BUFFER];     //receiving command buffer
byte commandBufferIndex = 0;                    //index of writing head in receiving command buffer
byte messageReceived = 0;                       //flag signals main loop that we have received message that
                                                //needs to be processed


volatile  byte ADCSequenceIndex=0;            //index of analog channel that will be measured next by ADC 



#define ESCAPE_SEQUENCE_LENGTH 6                //length of escape sequence that is used when sending messages to PC

byte escapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,128,255};            //start escape sequence
byte endOfescapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,129,255};       //end escape sequence

byte incomingByte;
volatile uint16_t emgADCResult;     //main buffer that contains real measurements

int incrementsForLEDThr[] = {28, 53, 81, 108, 135, 161};      //threshold intervals for different sensitivities (sens-30)/6
                                                              //for example:for value = 28. EMG envelope needs to increase 28 
                                                              //AD units to light up one more LED  
                                                              
                                                              //we set this from sensitivities[] 
int lastSensitivitiesIndex = 2;                               //index of sensitivity from sensitivities[] that is currently in use
byte sensitivityButtonPressed = 0;                            //used to ignore glitches from button bounce
uint16_t sensitivityVisualFeedbackCounter = 0;                //timer counter that holds selected sensitivity LED ON when user changes
                                                              //sensitivity with button press  
#define SENSITIVITY_LED_FEEDBACK 5000                         //length of interval (expressed in semple periods) sensitivity feedback LED will be ON

volatile byte outputBufferReady = 0;                          //variable that signals to main loop that output data frame buffer is ready for sending
byte outputFrameBuffer[64];                                   //Output frame buffer that contains measured EMG data 
                                                              //formated according to SpikeRecorder serial protocol

#define LINE_FEED 10                                          //\n character

#include <avr/io.h>
#include <avr/interrupt.h>
volatile uint16_t envelopeFirstChanel = 0;                    //value of calculated envelope of first EMG channel
uint16_t movingThresholdSum;                                  //temp calculation variable for thresholds for LEDs and servo
uint16_t incrementForLEDThreshold = 81;                       //increment for LED threshold (how much EMG needs to change to light up one more LED)
                                                              //depends on selected sensitivity

char* command;                                                //temp variable for parsing received commands
char* separator;                                              //temp variable for parsing received commands

byte readyToDoAuxComputation = 0;                             //flag that enables auxiliary computation only once per sampling timer interrupt

uint16_t widthOfPulseADCResult = 30;
uint16_t frequencyOfPulseADCResult = 30;
uint16_t batteryVoltage = 1024;

uint16_t tempEnvValue;
byte envelopeDecrementCounter = 0;
uint16_t temp1, temp2, temp3, temp4;
#define MAX_ENV_DECREMENT_COUNTER 5

//----------------------------------- SETUP FUNCTION --------------------------------------------------------------------------------------------
void setup()
{
    pinMode(5, OUTPUT);                                   //enable stimulation outout
    PORTC &= B10111111;                                   //turn Stimulation OFF pin 5 (PC6)
     
    pinMode(8, OUTPUT);                                   //LED pin
    pinMode(2, OUTPUT);                                   //LED pin
    pinMode(10, OUTPUT);                                  //LED pin
    pinMode(11, OUTPUT);                                  //LED pin
    pinMode(12, OUTPUT);                                  //LED pin
    pinMode(13, OUTPUT);                                  //LED pin

    pinMode(9, OUTPUT);                                   //pulse generator

    pinMode(6, OUTPUT);                                   //Power ON LED
    pinMode(4, OUTPUT);                                   //Low battery
    
    //debug pins
    //pinMode(5, OUTPUT);
    //pinMode(6, OUTPUT);

    pinMode(1, OUTPUT);     //debug          //extra button PD3
    pinMode(SENSITIVITY_BUTTON_PIN, INPUT);     //debug        //sensitivity button PE6
   
    
    //setup serial communication
    Serial.begin(230400);       //begin Serial comm
    delay(300);                 //whait for init of serial
    Serial.setTimeout(2);

   

    cli();                                                //stop interrupts

    //------------------------------------ setup ADC -----------------------------------------------------------------------
    cbi(ADMUX,REFS0);                                     //set ADC reference to AVCC
    cbi(ADMUX,ADLAR);                                     //left Adjust the result
    sbi(ADCSRA,ADEN);                                     //enable ADC
    sbi(ADCSRA,ADIE);                                     //enable ADC Interrupt
  
                                                          //set ADC clock division to 16
    sbi(ADCSRA,ADPS2);                                    //1
    cbi(ADCSRA,ADPS1);                                    //0
    cbi(ADCSRA,ADPS0);                                    //0


    //---------------------- Set timer for pulse generator at timer 1 digital pin 9 ------------------------------------------
    TCCR1B = 0;
    //WGM = B1110
    //Fast PWM ICRn=TOP
    //Page 133 http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf
    TCCR1B = (1 << WGM12) | (1 << WGM13);
    //channel A enabled (and second part of WGM updated)
    TCCR1A = B10000010;
    //set prescaler to 64 = CS1 0:3 = B011 so that timer frequency is 16MHz/64 = 250kHz
    TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11) | (1<<CS10))) | (1<<CS11) | (1<<CS10);
   
   



     //------------------------- Set timer for 10kHz sample rate at timer 4 ---------------------------------------------------
    //Page 171 http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf
    // Timer 4 10bit counter

    //TCCR4B  prescaler
    //PWM4X PSR4 DTPS41 DTPS40 CS43 CS42 CS41 CS40
    TCCR4B = B00000111;

    //TCCR4C enable PWM bit
    //COM4A1S COM4A0S COM4B1S COMAB0S COM4D1 COM4D0 FOC4D PWM4D
    TCCR4C = B00000001;
    
    //TCCR4D
    //FPIE4 FPEN4 FPNC4 FPES4 FPAC4 FPF4 WGM41 WGM40
    TCCR4D = B00000000;
    
    //OCR4C is TOP
    //set frequency to 10kHz
    OCR4C = 24;

    //TIMSK4 interupt enable
    //OCIE4D OCIE4A OCIE4B   -    -    TOIE4   -    - 
    TIMSK4 = B10000000; 
    //OCR4A = 200;
    
    pinMode(1, OUTPUT);

    //---------------------------------------------------------------------------------------------------------------------

    antiFlickeringCounterMax =  ANTI_FLICKERING_TIME_IN_MS*10;
    antiFlickeringTimerForOutput = antiFlickeringCounterMax;
    //set timers and maximum values of timers to new value according to sample rate
    

    //set sensitivity and all variables that depend on sensitivity depending on lastSensitivitiesIndex 
    incrementForLEDThreshold = incrementsForLEDThr[lastSensitivitiesIndex];

    set_freq(4,28);
    sei();                                                //enable Global Interrupts
    digitalWrite(6,HIGH);// power ON led
    
}


//------------------------------------------------ MAIN LOOP ----------------------------------------------------
//   Here we:
//    - initiate sending of data frame by setting UDR0
//    - parse and execute received messages (for number of channels and board type)
//    - calculate EMG envelope
//    - refresh LEDs based on EMG envelope
//    - check for button press (sensitivity and gripper mode)
//    - refresh relay state if EMG envelope crosses threshold
//    - calculate servo PWM based on EMG envelope
//---------------------------------------------------------------------------------------------------------------
void loop()
{

  //do the auxiliary computation once per timer interrupt
   if(readyToDoAuxComputation==1)
   {
        readyToDoAuxComputation = 0;
        
        if(sensitivityVisualFeedbackCounter==0)//disable update of LEDs when we display selected sensitivity level
        {

                  //--------------- Calculate envelope ------------------------
                  tempEnvValue = emgADCResult;
                  if(tempEnvValue<NOISE_FLOOR_FOR_ENVELOPE)
                  {
                    tempEnvValue = 0;  
                  }
                  else
                  {
                    tempEnvValue = tempEnvValue - NOISE_FLOOR_FOR_ENVELOPE;  
                  }
                  tempEnvValue = tempEnvValue<<1;
                  //--------------- Calculate envelope ------------------------
                  if(envelopeFirstChanel<tempEnvValue)
                  {
                    envelopeFirstChanel=tempEnvValue;
                  }

                  /*
                  * Pins mapping to Leonardo port registers
                  *
                  * Pin 8 PB4
                  * Pin 2 PD1
                  * Pin 10 PB6
                  * Pin 11 PB7
                  * Pin 12 PD6
                  * Pin 13 PC7
                  *
                  */
                  //-------------- Refresh LED states ------------------------
                  movingThresholdSum = 30;//set initial threshold for first LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00010000;//Pin 8 PB4
                  }
                  else
                  {
                      PORTB &= B11101111;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold; //increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTD |= B00000010;//turn ON LED Pin 2 PD1
                  }
                  else
                  {
                      PORTD &= B11111101;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B01000000;//turn ON LED Pin 10 PB6
                  }
                  else
                  {
                      PORTB &= B10111111;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B10000000;//turn ON LED Pin 11 PB7
                  }
                  else
                  {
                      PORTB &= B01111111;//turn OFF LED
                  }


                  //---------------------------------------- TRIGGER STIMMULATION ----------------------------------
                  //check if we should activate relay
                  antiFlickeringTimerForOutput--;
                  if(antiFlickeringTimerForOutput==0)//if anough time passed update relay state
                  {
                      antiFlickeringTimerForOutput = antiFlickeringCounterMax;
                      if(envelopeFirstChanel>movingThresholdSum)
                      {
                          PORTC |= B01000000;//turn Stimulation ON pin 5 (PC6)
                      }
                      else
                      {
                          PORTC &= B10111111;//turn Stimulation OFF pin 5 (PC6)
                      }
                  }
          
                  //-------------------------------------- END OF TRIGGER STIMULATION ----------------------------------
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTD |= B01000000;//turn ON LED Pin 12 PD6
                  }
                  else
                  {
                      PORTD &= B10111111;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTC |= B10000000;//turn ON LED Pin 13 PC7
                  }
                  else
                  {
                      PORTC &= B01111111;//turn OFF LED
                  }
                 
          
                  //---------------------------------- SENSITIVITY BUTTON -----------------------------------------------
        
                  //check if button is pressed (HIGH)
                  if (digitalRead(SENSITIVITY_BUTTON_PIN))
                  { 
                      if(sensitivityButtonPressed == 0)//if button was not pressed when we scan last time
                      {
                            sensitivityButtonPressed = 1;
                            //change sensitivity and all other variables that depend on sensitivity
                            lastSensitivitiesIndex++;
                            if(lastSensitivitiesIndex==6)
                            {
                              lastSensitivitiesIndex = 0;
                            }
                          
                            //get current sensitivity value
                         
                            incrementForLEDThreshold = incrementsForLEDThr[lastSensitivitiesIndex];

                            PORTB &= B00101111;//turn OFF all leds
                            PORTD &= B10111101;//turn OFF all leds
                            PORTC &= B01111111;//turn OFF all leds
                            
                            //prepare to light up only one LED that represent sensitivity level

                            if(lastSensitivitiesIndex ==1)
                            {
                              digitalWrite(2,HIGH);
                            }
                            else
                            {
                              digitalWrite(8+lastSensitivitiesIndex,HIGH);
                            }
                            
                            sensitivityVisualFeedbackCounter = SENSITIVITY_LED_FEEDBACK;//set timer for visual feedback
                            
                           PORTC &= B10111111;//turn Stimulation OFF pin 5 (PC6) 
                      }    
                  }
                  else
                  {
                      sensitivityButtonPressed = 0;
                  }



                  // Battery indicator LED 
                  if(batteryVoltage<BATTERY_THRESHOLD_VOLTAGE)
                  {
                    PORTD |= B00010000;
                  }
                  else
                  {
                    PORTD &= B11101111;  
                  }
        }//end of sensitivity visual LED feedback counter
        else
        {
          //decrement sensitivityVisualFeedbackCounter counter. When it hits  
          //zero we are finished with signaling selected sensitivity level
          //and LEDs will continue to update and display EMG level 
          sensitivityVisualFeedbackCounter--;  
          PORTC &= B10111111;//turn Stimulation OFF pin 5 (PC6)
        }





        //----------------------------------- Set frequency and pulse width ----------------------------------

 
         //set frequency 250000/f = ICR1
         //from 4-150Hz
         temp1 = frequencyOfPulseADCResult>>3;
         temp2 = 127-temp1;
         temp3= 475*temp2;
         temp4 = 1666 + temp3;
         ICR1 = static_cast<uint16_t>(temp4);
         
         //set pulse width it is 4us resolution and zero value is 4us
         //so we have to substract 4 and divide with 4
         //from 30-260us
         temp1 = widthOfPulseADCResult>>4;
         if(temp1<7)
         {
          temp1 = 7;
          }
         OCR1A = temp1;


        
      

       //---------------------------------------- DECAY OF ENVELOPE ------------------------------------------
        envelopeDecrementCounter++;

       if(envelopeDecrementCounter == MAX_ENV_DECREMENT_COUNTER)
       { 
            envelopeDecrementCounter = 0;
            
            if(envelopeFirstChanel>0)
            {
              //envelope decay is 1 AD unit per one sample period. At 10kHz sampling rate
              // it is 10000*(5V/1024) per second. So it will linearly decay 5V in 10ms.
              //It is fast. Maybe we should slower down it here.
                envelopeFirstChanel--;  
            }

       }
      
   }



    //----------------------------------SENDING DATA ----------------------------------------------------------
    if(outputBufferReady == 1)//if we have new data
    {

        Serial.write(outputFrameBuffer, 2);
        outputBufferReady = 0;
        readyToDoAuxComputation = 1;
    }
      //--------------------------------- PARSING OF INCOMING MESSAGES --------------------------------------

    while (Serial.available() > 0) 
    {
          // read the incoming byte:
          
          incomingByte = Serial.read();
  
          if(incomingByte==LINE_FEED)                             //if received byte is \n than we are at the end of message
          {
            commandBuffer[commandBufferIndex] = 0;                    //null terminate string
            messageReceived = 1;                                      //set flag so that main loop knows that we have new messages to parse
            commandBufferIndex =0;                                    //rewind index of buffer to begining
          }
          else
          {
            commandBuffer[commandBufferIndex] = incomingByte;     //if we are not at the end of the command just add another character to buffer
            commandBufferIndex++;                                     //increment writing head index
            if(commandBufferIndex==SIZE_OF_COMMAND_BUFFER)            //if data is longer than buffer 
            {
              
              commandBufferIndex=0;                                   //start writing from begining
              break;
            }
          }
    }


  
    
    if(messageReceived)
    {
            messageReceived = 0;
            commandBufferIndex = 0;
            
            // breaks string str into a series of tokens using delimiter ";"
            // Namely split strings into commands
            char* command = strtok(commandBuffer, ";");
            while (command != 0)
            {
                // Split the command in 2 parts: name and value
                char* separator = strchr(command, ':');
                if (separator != 0)
                {
                    // Actually split the string in 2: replace ':' with 0
                    *separator = 0;
                    --separator;
                    if(*separator == 'b')//if we received command for impuls
                    {
                      TIMSK4 = B00000000; //disable timer for sampling                                 
                      PORTC &= B10111111;//turn Stimulation OFF pin 5 (PC6) 
                      sendMessage(CURRENT_SHIELD_TYPE);
                      TIMSK4 = B10000000; //enable timer for sampling
                    }
                }
                // Find the next command in input string
                command = strtok(0, ";");
            }
            //calculate sampling rate
    }//message processing if
}//end of main loop


//------------------------------------------ SAMPLING TIMER INTERRUPT --------------------------------------------------
ISR(TIMER4_COMPD_vect) 
{ 
  ADMUX =  B01000111;                                           //Start ADC Conversions A0 channel
  ADCSRA |=B01000000;                                           //do this at the begining since ADC can work in
                                                                //paralel with this timer handler

  outputFrameBuffer[0]= (emgADCResult>>7)| 0x80;           //convert data to frame according to protocol
  outputFrameBuffer[1]=  emgADCResult & 0x7F;              //first bit of every byte is used to flag start of the frame
                                                                //so first bit is set only on first byte of frame (| 0x80)
  outputBufferReady = 1;                                        //signal main loop to send frame
}


//---------------------------ADC INTERRUPT HANDLER ----------------------------------------------------------------------
//This is called when ADC conversion is complete.
ISR(ADC_vect)           
{
      //set the channel
      switch (ADCSequenceIndex) {
            case 0: 
              emgADCResult = ADCL | (ADCH << 8);      // store lower and higher byte of ADC
              ADMUX =  B01000100;//pot width of pulse
              ADCSRA |=B01000000; //start sampling
              ADCSequenceIndex++;
            break;
            case 1:
              widthOfPulseADCResult = ADCL | (ADCH << 8); 
              ADMUX =  B01000111;//record EMG again
              ADCSequenceIndex++;
            break;
            case 2:
              emgADCResult = ADCL | (ADCH << 8);      // store lower and higher byte of ADC
              ADMUX =  B01000001;//pot frequency of pulses
              ADCSRA |=B01000000; //start sampling
              ADCSequenceIndex++;
            break;
            case 3:
              frequencyOfPulseADCResult = ADCL | (ADCH << 8); 
              ADMUX =  B01000111;//record EMG again
              ADCSequenceIndex++;
            break;
            case 4:
              emgADCResult = ADCL | (ADCH << 8);      // store lower and higher byte of ADC
              ADMUX =  B01000000;//batery voltage
              ADCSRA |=B01000000; //start sampling
              ADCSequenceIndex++;
            break;
            case 5:
              batteryVoltage = ADCL | (ADCH << 8); 
              ADMUX =  B01000111;//record EMG again
              ADCSequenceIndex = 0;
            break;
      }

} 



//------------------------------- SEND MESSAGE WITH ESCAPE SEQUENCE ----------------------------------------------------
//timer for sampling must be dissabled when 
//we call this function

//push message to main sending buffer
//timer for sampling must be dissabled when 
//we call this function
void sendMessage(const char * message)
{

  int i;
  int head = 0;
  //send escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      outputFrameBuffer[head++] = escapeSequence[i];
     
  }

  //send message
  i = 0;
  while(message[i] != 0)
  {
      outputFrameBuffer[head++] = message[i++];
      
  }

  //send end of escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      outputFrameBuffer[head++] = endOfescapeSequence[i];
      
  }
   Serial.write(outputFrameBuffer, head);
   
}


//
//PWM frequency and pulse width setter
//Clock is 16000000Hz/64 = 250 000Hz
//Min frequency = 250000/65536 = 3.814Hz
//Max period 262ms (for 3.814Hz)
//Resolution 1/250000 = 4us
//We will make frequency from 4Hz to 150Hz
//Goal for pulse width 30us to 250us
//28 - 260ms
//
void set_freq(float f, float pulseWidthMicroSec)
{
    uint16_t n;
  
   //set frequency 250000/f = ICR1
   ICR1 = static_cast<uint16_t>(250000/f);
   //set pulse width it is 4us resolution and zero value is 4us
   //so we have to substract 4 and divide with 4
   OCR1A = (pulseWidthMicroSec-4)/4;   
}



