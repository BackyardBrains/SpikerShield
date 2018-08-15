/*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains 15. Aug. 2018
  * 
  * Made for HHI SpikerBox V1(0.7) 
  * Based on Arduino UNO ATMEGA 328
  * 
  * TENS device with communication with Spike Recorder
  * 
  * Parameters of stimulation are fixed: 
  * Pulse width: 100 micro seconds
  * Frequency:   120Hz 
  * 
  * Code sends RAW emg to Spike Recorder via serial interface. It is calculating envelope based on RAW EMG
  * that is expected to float on Vcc/2. Based on envelope it updates VU meter and creates stimulation square 
  * wave for TENS when envelope crosses the threshold.
  * Threshold can be changed with press of the button on D7.
  * 
  * 
  * A0 - EMG input
  * A1 - ---
  * A2 - ---
  * A3 - ---
  * A4 - ---
  * A5 - Batery voltage input
  * 
  * 
  * 
  * D0  - Rx
  * D1  - Tx
  * D2  - VU LED 2
  * D3  - aux button
  * D4  - Red LED - Low Battery indicator
  * D5  - TENS ON/OFF control output
  * D6  - Green LED - power ON LED
  * D7  - Sensitivity selection button input
  * D8  - VU LED 1
  * D9  - Pulse generator output for TENS
  * D10 - VU LED 3
  * D11 - VU LED 4
  * D12 - VU LED 5
  * D13 - VU LED 6
  * 
  * 
  * V0.4
  * History:
  * V0.2 Glitches in communication fixed in V0.2 amd sampling frequency lower.
  * V0.3 Serial communication implemented with Serial.write and batery voltage is measured every 3 sec
  * V0.4 Block of TENS after long activation
  * Written by Stanislav Mircic
  * 
  * ----------------------------------------------------------------------------------------------------
  */
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
float lowPowerVoltage = 8.1;//In volts
float shutDownVoltage = 7.1;//In volts
//if this % of six second period stimulation is over threshold we will disable TENS and 
//wait for TENS to be at least 3 without stimullation over threshold and than enable TENS again
float percentageOfStimulationInSixSeconds = 50.0; 
#define FREQUENCY_OF_STIMULATION 120.0 //In Hz

#define POWER_STATE_GOOD 0
#define POWER_STATE_LOW 1
#define POWER_STATE_SHUT_DOWN 2
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
#define SENSITIVITY_BUTTON_PIN 7                //pin for button that changes sensitivity
#define MAX_NUMBER_OF_CHANNELS 2                //maximum number of analog channels

int interrupt_Number = 198;//198;                     // Output Compare Registers  value = (16*10^6) / (Fs*8) - 1  
                                                // Set to 198 for 10000 Hz max sampling rate
                                                // Used for main timer that defines period of measurements

#define MESSAGE_BUFFER_SIZE 100                 //size of buffer used to send messages to serial (to PC)
                                                //it needs to be big enough to hold start escape sequence,
                                                //message and end escape sequence
byte messageBuffer[MESSAGE_BUFFER_SIZE];        //buffer used to send messages to serial (to PC)
#define ESCAPE_SEQUENCE_LENGTH 6                //length of escape sequence that is used when sending messages to PC

byte escapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,128,255};            //start escape sequence
byte endOfescapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,129,255};       //end escape sequence

byte messageSending = 0;                        //flag that signals TX handler to send data from "messageBuffer"           
byte lengthOfMessasge = 0;                      //length of message in message buffer

volatile uint16_t samplingBuffer[MAX_NUMBER_OF_CHANNELS];     //main buffer that contains real measurements


int incrementsForLEDThr[] = {28, 53, 81, 108, 135, 161};      //threshold intervals for different sensitivities (sens-30)/6
                                                              //for example:for value = 28. EMG envelope needs to increase 28 
                                                              //AD units to light up one more LED  
                                                        
int lastSensitivitiesIndex = 2;                               //index of sensitivity from sensitivities[] that is currently in use
byte sensitivityButtonPressed = 0;                            //used to ignore glitches from button bounce
uint16_t sensitivityVisualFeedbackCounter = 0;                //timer counter that holds selected sensitivity LED ON when user changes
                                                              //sensitivity with button press  
#define SENSITIVITY_LED_FEEDBACK 5000                         //length of interval (expressed in semple periods) sensitivity feedback LED will be ON

uint16_t sensitivityVisualFeedbackCounterMax = SENSITIVITY_LED_FEEDBACK;      //max value of sensitivityVisualFeedbackCounter. It changes with sample freq.

volatile byte outputBufferReady = 0;                          //variable that signals to main loop that output data frame buffer is ready for sending
byte outputFrameBuffer[MAX_NUMBER_OF_CHANNELS*2];             //Output frame buffer that contains measured EMG data 
                                                              //formated according to SpikeRecorder serial protocol

#include <avr/io.h>
#include <avr/interrupt.h>
volatile uint16_t envelopeFirstChanel = 0;                    //value of calculated envelope of first EMG channel
uint16_t movingThresholdSum;                                  //temp calculation variable for thresholds for LEDs and servo
uint16_t incrementForLEDThreshold = 81;                       //increment for LED threshold (how much EMG needs to change to light up one more LED)
                                                              //depends on selected sensitivity
byte tempCalcByteMask = 0;                                    //general purpose bitmask for shifting bits

byte readyToDoAuxComputation = 0;                             //flag that enables auxiliary computation only once per sampling timer interrupt

uint16_t tempEnvValue;
byte envelopeDecrementCounter = 0;
#define MAX_ENV_DECREMENT_COUNTER 5

byte emgCrossedTheThreshold = 0;


uint16_t periodOfStimulationExpressedInPeriodsOfSampling = 0;
uint16_t counterForPeriodOfStimulation = 0;

uint16_t shutDownVoltageInADUnits = 0;
uint16_t lowPowerVoltageInADUnits = 0;

byte powerState = POWER_STATE_GOOD;
uint16_t powerInADUnits = 1023;

uint16_t measurementTimerForBatery = 0;

uint16_t stimulationTimeCounter = 0;
uint16_t maxStimulationSamples = 0;
bool stimulationEnabled = true;
//----------------------------------- SETUP FUNCTION --------------------------------------------------------------------------------------------
void setup()
{
    pinMode( 8, OUTPUT);                                  //VU LED 1 PB0
    pinMode( 2, OUTPUT);                                  //VU LED 2 PD2
    pinMode(10, OUTPUT);                                  //VU LED 3 PB2
    pinMode(11, OUTPUT);                                  //VU LED 4 PB3
    pinMode(12, OUTPUT);                                  //VU LED 5 PB4
    pinMode(13, OUTPUT);                                  //VU LED 6 PB5

    pinMode(5, OUTPUT);                                   //TENS ON/OFF control output - PD5
    pinMode(9, OUTPUT);                                   //Pulse generator output     - PB1

    pinMode(SENSITIVITY_BUTTON_PIN, INPUT);               //sensitivity button pin
    pinMode(3, INPUT);                                    //aux. button

    pinMode(4, OUTPUT);                                   //Red LED output  - low power - PD4
    pinMode(6, OUTPUT);                                   //Green LED output - power ON - PD6


    lowPowerVoltageInADUnits = (uint16_t)((lowPowerVoltage/15.0)*1023);
    shutDownVoltageInADUnits = (uint16_t)((shutDownVoltage/15.0)*1023);

    if(percentageOfStimulationInSixSeconds>100)
    {
      percentageOfStimulationInSixSeconds = 100.0;  
    }
    maxStimulationSamples = (uint16_t)(60000*(percentageOfStimulationInSixSeconds/100.0));//6sec *(max%/100%)

    
    powerState = POWER_STATE_GOOD; 
    //put initial measurement of voltage to highest to avoid going in sleep mode         
    samplingBuffer[1] = 1023;
    //turn ON Green LED
    PORTD &= B11101111;
    PORTD |= B01000000;

     
                                                          //setup serial communication
    Serial.begin(230400); //Serial communication baud rate (alt. 115200)
    delay(300); //whait for init of serial
    Serial.println("StartUp!");
    Serial.setTimeout(2);

    cli();                                                //stop interrupts

                                                          //setup ADC
    cbi(ADMUX,REFS0);                                     //set ADC reference to AVCC
    cbi(ADMUX,ADLAR);                                     //left Adjust the result
    cbi(ADMUX,ADATE);                                     //left Adjust the result
    sbi(ADCSRA,ADEN);                                     //enable ADC
    sbi(ADCSRA,ADIE);                                     //enable ADC Interrupt
 
                                                          //set ADC clock division to 16
    sbi(ADCSRA,ADPS2);                                    //1
    cbi(ADCSRA,ADPS1);                                    //0
    cbi(ADCSRA,ADPS0);                                    //0


    //set timer1 interrupt
    TCCR1A = 0;                                           //set entire TCCR1A register to 0
    TCCR1B = 0;                                           //same for TCCR1B
    TCNT1  = 0;                                           //initialize counter value to 0;
    OCR1A = interrupt_Number;                             //output compare registers 
    TCCR1B |= (1 << WGM12);                               //turn on CTC mode
    TCCR1B |= (1 << CS11);                                //set CS11 bit for 8 prescaler
    TIMSK1 |= (1 << OCIE1A);                              //enable timer compare interrupt



    //set timers and maximum values of timers to new value according to sample rate
    sensitivityVisualFeedbackCounterMax = SENSITIVITY_LED_FEEDBACK;

    //set sensitivity and all variables that depend on sensitivity depending on lastSensitivitiesIndex
    incrementForLEDThreshold = incrementsForLEDThr[lastSensitivitiesIndex];

    periodOfStimulationExpressedInPeriodsOfSampling = 10000/FREQUENCY_OF_STIMULATION;
    counterForPeriodOfStimulation = periodOfStimulationExpressedInPeriodsOfSampling;
    
    sei();                                                //enable Global Interrupts
}

//------------------------------------------------ MAIN LOOP ----------------------------------------------------
//   Here we:
//    - initiate sending of data frame by setting UDR0
//    - parse and execute received messages (for number of channels and board type)
//    - calculate EMG envelope
//    - refresh LEDs based on EMG envelope
//    - check for button press (sensitivity and gripper mode)
//---------------------------------------------------------------------------------------------------------------


void loop()
{


  //do the auxiliary computation once per timer interrupt
   if(readyToDoAuxComputation==1)
   {
        readyToDoAuxComputation = 0;

        //----------------------- POWER MANAGEMENT --------------------------
        powerInADUnits =  samplingBuffer[1];// = 1023;
        
        if(powerState == POWER_STATE_GOOD)
        {
          if(powerInADUnits<lowPowerVoltageInADUnits)
          {
            powerState = POWER_STATE_LOW;
            //red
            PORTD &= B10111111;
            PORTD |= B00010000;
          }
        }
        if(powerState == POWER_STATE_LOW)
        {
          if(powerInADUnits<shutDownVoltageInADUnits)
          {
            powerState = POWER_STATE_SHUT_DOWN;
            //shut down
            sleep_enable();//Enabling sleep mode
            PORTB = B0;
            PORTD = B0;
            cli();   
            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            sleep_cpu();

            
          }
          else if(powerInADUnits>lowPowerVoltageInADUnits+20)
          {
              powerState = POWER_STATE_GOOD;
              //green

              PORTD &= B11101111;
              PORTD |= B01000000;
          }
        }
      
        
        //----------------------- POWER MANAGEMENT --------------------------

          
        if(sensitivityVisualFeedbackCounter==0)//disable update of LEDs when we display selected sensitivity level
        {

                  //--------------- CALCULATE ENVELOPE ------------------------
                  tempEnvValue = samplingBuffer[0];
                  if(tempEnvValue<NOISE_FLOOR_FOR_ENVELOPE)
                  {
                    tempEnvValue = 0;  
                  }
                  else
                  {
                    tempEnvValue = tempEnvValue - NOISE_FLOOR_FOR_ENVELOPE;  
                  }
                  tempEnvValue = tempEnvValue<<1;
                  
                  if(envelopeFirstChanel<tempEnvValue)
                  {
                    envelopeFirstChanel=tempEnvValue;
                  }
                  //-----------------------------------------------------------
                  

                  //-------------- REFRESH LED STATES -------------------------
                  movingThresholdSum = 30;//set initial threshold for first LED
                   //VU LED 1 PB0
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00000001;//we set directly bit on port since it is faster than digitalWrite
                  }
                  else
                  {
                      PORTB &= B11111110;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold; //increment threshold for next LED
                  //VU LED 2 PD2
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTD |= B00000100;
                  }
                  else
                  {
                      PORTD &= B11111011;
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  //VU LED 3 PB2
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00000100;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11111011;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  //VU LED 4 PB3
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00001000;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11110111;//turn OFF LED
                  }
                  
                  //---------------------------------------- STIMULATION THRESHOLD ----------------------------------
                  //check if we should activate stimulation

                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      emgCrossedTheThreshold = 1;
                      stimulationTimeCounter++;
                      if(stimulationTimeCounter>maxStimulationSamples)
                      {
                          stimulationEnabled = false;
                          stimulationTimeCounter = maxStimulationSamples;
                      }
                  }
                  else
                  {
                      emgCrossedTheThreshold = 0;
                      if(stimulationTimeCounter>0)
                      {
                        stimulationTimeCounter--;
                      }
                      else
                      {
                        stimulationEnabled = true;
                      }
                  }
                  //-------------------------------------------------------------------------------------------------
          
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  //VU LED 5 PB4
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                    PORTB |= B00010000;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11101111;//turn OFF LED 
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  //VU LED 6 PB5
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00100000;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11011111;//turn OFF LED
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

                            //turn OFF all leds
                            PORTB &= B11000010;
                            PORTD &= B11111011;

                            //turn OFF TENS
                            PORTD &= B11011111;// PD5
                            PORTB &= B11111101;//PB1

                            //Turn ON only one LED that represent sensitivity level
                            if(lastSensitivitiesIndex ==1)
                            {
                                PORTD |= B00000100;
                            }
                            else
                            {
                                tempCalcByteMask = 1<<lastSensitivitiesIndex;
                                PORTB |= tempCalcByteMask;//light up one LED for visual feedback of sensitivity
                            }
                            sensitivityVisualFeedbackCounter = sensitivityVisualFeedbackCounterMax;//set timer for visual feedback 
                      }    
                  }
                  else
                  {
                      sensitivityButtonPressed = 0;
                  }
        }//end of sensitivity visual LED feedback counter
        else
        {
          //decrement sensitivityVisualFeedbackCounter counter. When it hits  
          //zero we are finished with signaling selected sensitivity level
          //and LEDs will continue to update and display EMG level 
          sensitivityVisualFeedbackCounter--;  
        }


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
   }//end of aux. computation




  //----------------------------------SENDING DATA ----------------------------------------------------------
   if(outputBufferReady == 1  )//if we have new data
   {

      outputBufferReady = 0;//this will be zero until we send whole frame buffer and fill it again
      //since we want to do aux computation (LEDs, relay, servo) only once per sample period
      //and main loop is called multiple times per period (anytime the code is not in interrupt handler)
      //we set this flag here so that aux comp. is done only once after this initialization of frame sending 
      readyToDoAuxComputation = 1;

      //Sends first byte of frame. The rest is sent by TX handler.
       Serial.write(outputFrameBuffer,2);
   }//end of detection of fresh frame data


  //----------------------------------SENDING MESSAGE ----------------------------------------------------------
   if(messageSending ==1)
   {
      messageSending = 0;
      Serial.write(messageBuffer,lengthOfMessasge-1);   
   }

   
}//end of main loop
//---------------------------------------------- END OF MAIN LOOP ---------------------------------------------------------


void serialEvent() 
{
    String inString = Serial.readStringUntil('\n'); 
    cli();//dissable interrupts
    //turn OFF TENS
    PORTD &= B11011111;// PD5
    PORTB &= B11111101;//PB1
    sendMessage(CURRENT_SHIELD_TYPE);//send message with escape sequence
}


//------------------------------------------ SAMPLING TIMER INTERRUPT --------------------------------------------------
ISR(TIMER1_COMPA_vect) {

 if(measurementTimerForBatery==0)
 {
  ADMUX =  B01000101;                                          //Start ADC Conversions
 }
 else
 {
   ADMUX =  B01000000;                                           //Start ADC Conversions
 }
  ADCSRA |=B01000000;                                           //do this at the begining since ADC 
                                                                //can work in paralel with other stuff

  outputFrameBuffer[0]=  (samplingBuffer[0]>>7)| 0x80;           //convert data to frame according to protocol
  outputFrameBuffer[1]=  samplingBuffer[0] & 0x7F;              //first bit of every byte is used to flag start of the framr
    outputBufferReady = 1;                                        //signal main loop to send frame

  if(emgCrossedTheThreshold && stimulationEnabled)
  {
        counterForPeriodOfStimulation--;
        if(counterForPeriodOfStimulation==0)
        {
            counterForPeriodOfStimulation = periodOfStimulationExpressedInPeriodsOfSampling;
            PORTB |= B00000010;//PB1
        }
        else
        {
            PORTB &= B11111101;//PB1
        }
        PORTD |= B00100000; 
  }
  else
  {
    counterForPeriodOfStimulation = periodOfStimulationExpressedInPeriodsOfSampling;
    PORTD &= B11011111;// PD5
    PORTB &= B11111101;//PB1
  }
}



//---------------------------ADC INTERRUPT HANDLER ----------------------------------------------------------------------
//This is called when ADC conversion is complete.
ISR(ADC_vect)           
{
    if(measurementTimerForBatery==0)
    {
      samplingBuffer[1] = ADCL | (ADCH << 8);      // store lower and higher byte of ADC
    }
    else
    {
       samplingBuffer[0] = ADCL | (ADCH << 8);      // store lower and higher byte of ADC
    }
    measurementTimerForBatery++;                     
} 



//------------------------------- SEND MESSAGE WITH ESCAPE SEQUENCE ----------------------------------------------------
//timer for sampling must be dissabled when 
//we call this function

void sendMessage(const char * message)
{
  int i;
  lengthOfMessasge = 0;
                                                                        //add escape sequence to buffer
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      messageBuffer[lengthOfMessasge++] = escapeSequence[i];
      if(lengthOfMessasge==MESSAGE_BUFFER_SIZE)
      {
        lengthOfMessasge = 0;
      }
  }

                                                                        //add message to buffer
  i = 0;
  while(message[i] != 0)
  {
      messageBuffer[lengthOfMessasge++] = message[i++];
      if(lengthOfMessasge==MESSAGE_BUFFER_SIZE)
      {
        lengthOfMessasge = 0;
      }
  }

                                                                        //add end of escape sequence to buffer
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      messageBuffer[lengthOfMessasge++] = endOfescapeSequence[i];
      if(lengthOfMessasge==MESSAGE_BUFFER_SIZE)
      {
        lengthOfMessasge = 0;
      }
  }
  messageSending =1;                                                   //set flag that we are sending message and not data frames
     
         sei();
         
}


  
