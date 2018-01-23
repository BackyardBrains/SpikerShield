/*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains 23. Jan. 2018
  * 
  * Muscle SpikerShield with HHI, Claw, LEDs and communication with Spike Recorder integrated.
  * This code does not need envelope in order to function. It calculates envelope for LEDs, HHI and the claw
  * V0.3
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

#define ANTI_FLICKERING_TIME_IN_MS 50           //relay refresh period in ms
uint16_t antiFlickeringCounterMax;              //relay refresh period in sample periods
uint16_t antiFlickeringTimerForOutput;          //timer for relay state refresh in sample periods
                                                //it decreases every sample period and when we hit zero 
                                                //we update relay state

#define GRIPPER_STATE_BUTTON_PIN 4              //pin for button that switches defult state of the gripper (opened/closed)
#define SENSITIVITY_BUTTON_PIN 7                //pin for button that changes sensitivity

#define SIZE_OF_COMMAND_BUFFER 30               //command buffer size (for commands received from serial)
char commandBuffer[SIZE_OF_COMMAND_BUFFER];     //receiving command buffer
byte commandBufferIndex = 0;                    //index of writing head in receiving command buffer
byte tempReceivedByte;                          //temp calc. variable
byte messageReceived = 0;                       //flag signals main loop that we have received message that
                                                //needs to be processed


#define MAX_NUMBER_OF_CHANNELS 6                //maximum number of analog channels

byte numberOfChannels = 1;                      //How many analog channels we are currently sending via 
                                                //serial to SpikeRecorder
byte tempNumberOfChanels;                       //temp calc. variable

volatile  byte regularChannelsIndex;            //index of analog channel that will be measured next by ADC 


int interrupt_Number = 198;                     // Output Compare Registers  value = (16*10^6) / (Fs*8) - 1  
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
byte messageSendingIndex = 0;                   //index of next byte in messageBuffer that needs to be sent

volatile uint16_t samplingBuffer[MAX_NUMBER_OF_CHANNELS];     //main buffer that contains real measurements

int sensitivities[] = {200, 350, 520, 680, 840, 1000};        //maximum value of EMG in AD units, saturation value
                                                              //this basicaly defines sensitivity of LEDs and relay
int incrementsForLEDThr[] = {28, 53, 81, 108, 135, 161};      //threshold intervals for different sensitivities (sens-30)/6
                                                              //for example:for value = 28. EMG envelope needs to increase 28 
                                                              //AD units to light up one more LED  

//threshold intervals for different sensitivities (sens-120)/16. We are controlling servo by generating PWM signal. Signal has 
//period of 20ms. To fully close claw PWM needs to be 1.6ms ON. To fully open claw PWM needs to be 2.4ms ON and rest should be OFF. 
//Since our timers are already used we had to find another periodic handler that can be used to generate PWM.
//Our code is constantly sending data and TX handler is called "periodicaly" every 50us. That gives us enought time resolution
//to generate PWM signal for servo (20ms/50us = 400). Difference in PWM between fully open and fully closed positions expressed 
//in TX periods is (2.4ms-1.6ms)/50us = 16. So we need to divide EMG interval (from minimum treshold to saturation value defined
//by sensitivities[]) with 16 to get how much AD units EMG envelope needs to change for PWM to extend one TX period (50us).
int incrementsForClosedClaw[] = {5, 15, 25, 35, 45, 55};       
                                                              
                                                              
int incrementsForOpenClaw[] = {10, 20, 30, 40, 50, 60};       //threshold intervals for different sensitivities (sens-40)/16
                                                              // see incrementsForClosedClaw[] explanation for details
uint16_t emgSaturationValue = 0;                              //we rectify EMG value if it crosses emgSaturationValue before we
                                                              //calculate LEDs, relay and servo
                                                              //we set this from sensitivities[] 
int lastSensitivitiesIndex = 2;                               //index of sensitivity from sensitivities[] that is currently in use
byte sensitivityButtonPressed = 0;                            //used to ignore glitches from button bounce
uint16_t sensitivityVisualFeedbackCounter = 0;                //timer counter that holds selected sensitivity LED ON when user changes
                                                              //sensitivity with button press  
#define SENSITIVITY_LED_FEEDBACK 5000                         //length of interval (expressed in semple periods) sensitivity feedback LED will be ON

uint16_t sensitivityVisualFeedbackCounterMax = SENSITIVITY_LED_FEEDBACK;      //max value of sensitivityVisualFeedbackCounter. It changes with sample freq.

volatile byte outputBufferReady = 0;                          //variable that signals to main loop that output data frame buffer is ready for sending
byte outputFrameBuffer[MAX_NUMBER_OF_CHANNELS*2];             //Output frame buffer that contains measured EMG data 
                                                              //formated according to SpikeRecorder serial protocol

#define LINE_FEED 10                                          //\n character

byte sendBufferIndex = 0;                                     //index of output frame buffer that contains data
#define USART_BAUDRATE 230400                                 //baud rate of serial communication
#define BAUD_PRESCALE (F_CPU / 4 / USART_BAUDRATE - 1) / 2    //value for UBRR0H and UBRR0L registers that controll baud rate

#include <avr/io.h>
#include <avr/interrupt.h>
volatile uint16_t envelopeFirstChanel = 0;                    //value of calculated envelope of first EMG channel
uint16_t movingThresholdSum;                                  //temp calculation variable for thresholds for LEDs and servo
uint16_t incrementForLEDThreshold = 81;                       //increment for LED threshold (how much EMG needs to change to light up one more LED)
                                                              //depends on selected sensitivity
uint16_t incrementForOpenClawThreshold = 30;                  //increment for PWM. How much EMG needs to change to change PWM for 50us (one TX period)
                                                              //used only in default open claw state. Depends on sensitivity
uint16_t incrementForClosedClawThreshold = 25;                //increment for PWM. How much EMG needs to change to change PWM for 50us (one TX period)
                                                              //used only in default closed claw state. Depends on sensitivity
byte tempCalcByteMask = 0;                                    //general purpose bitmask for shifting bits

#define OPEN_CLAW_MODE 0
#define CLOSED_CLAW_MODE 1
byte currentFunctionality = OPEN_CLAW_MODE;                   //current mode of claw operation (default closed/open)

#define LENGTH_OF_SERVO_PERIOD 400                            //20ms/50us (look explanation for incrementsForClosedClaw[] array)
uint16_t servoPeriodCounter = LENGTH_OF_SERVO_PERIOD;         //counter that keeps current time inside one PWM period (expressed in TX periods)
byte tempActiveServoPeriod = 48;                              //how many 50us periods PWM needs to be ON during one PWM period
byte activeServoPeriod = 48;                                  //how many 50us periods PWM needs to be ON during one PWM period
byte disableServoMeasure = 4;                                 //how many PWM 20ms periods wee need to whait before we update servo value based on EMG
                                                              //we change servo value every 100ms to avoid motor "shaking"
#define GRIPPER_BUTTON_DEBOUNCE 1000                          //100ms debounce time for gripper mode (expressed in sample rate @10k)
uint16_t gripperButtonDebounceCounter = GRIPPER_BUTTON_DEBOUNCE;
uint16_t gripperButtonDebounceCounterMax;                     //100ms debounce time for gripper mode (expressed in surrent sample rate)
byte gripperButtonEnabled = 1;                                //flag used to avoid multiple button press with one real button press 

char* command;                                                //temp variable for parsing received commands
char* separator;                                              //temp variable for parsing received commands

byte readyToDoAuxComputation = 0;                             //flag that enables auxiliary computation only once per sampling timer interrupt

uint16_t tempEnvValue;
byte envelopeDecrementCounter = 0;
#define MAX_ENV_DECREMENT_COUNTER 9


//----------------------------------- SETUP FUNCTION --------------------------------------------------------------------------------------------
void setup()
{
    
    pinMode(8, OUTPUT);                                   //LED pin
    pinMode(9, OUTPUT);                                   //LED pin
    pinMode(10, OUTPUT);                                  //LED pin
    pinMode(11, OUTPUT);                                  //LED pin
    pinMode(12, OUTPUT);                                  //LED pin
    pinMode(13, OUTPUT);                                  //LED pin

    pinMode(2, OUTPUT);                                   //step motor
    pinMode(3, OUTPUT);                                   //relay output
    
    //debug pins
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);

    pinMode(SENSITIVITY_BUTTON_PIN, INPUT);               //sensitivity button pin
    pinMode(GRIPPER_STATE_BUTTON_PIN, INPUT);             //mode of operation button
    
                                                          //setup serial communication
    uint16_t baud_setting = BAUD_PRESCALE;                //get baud rate
    UCSR0B = (1 << RXEN0 ) | (1 << TXEN0 );               //turn on the transmission and reception circuitry
    UCSR0C = 0x06;                                        //8 bit, stop bit, no parity 
    UCSR0A = 1 << U2X0;                                   //baud rate mode
    UBRR0H  = ( baud_setting >> 8) ;                      //load upper 8- bits of the baud rate value into the high byte of the UBRR register
    UBRR0L = baud_setting ;                               //load lower 8 - bits of the baud rate value into the low byte of the UBRR register
    UCSR0B |= (1 << RXCIE0 );                             //enable the USART Recieve Complete interrupt ( USART_RXC )
    UCSR0B |= (1 << TXCIE0 );                             //enable the USART Transmite Complete interrupt ( USART_RXC )

    sendBufferIndex = 0;                                  //set index (head position) of sending buffer

    cli();                                                //stop interrupts

                                                          //setup ADC
    cbi(ADMUX,REFS0);                                     //set ADC reference to AVCC
    cbi(ADMUX,ADLAR);                                     //left Adjust the result
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

    numberOfChannels = 1;                                 //set initial number of channels

    //set timers and maximum values of timers to new value according to sample rate
    antiFlickeringCounterMax =  ((ANTI_FLICKERING_TIME_IN_MS*10)/numberOfChannels);
    antiFlickeringTimerForOutput = antiFlickeringCounterMax;
    gripperButtonDebounceCounterMax = GRIPPER_BUTTON_DEBOUNCE/numberOfChannels;
    sensitivityVisualFeedbackCounterMax = SENSITIVITY_LED_FEEDBACK/numberOfChannels;

    //set sensitivity and all variables that depend on sensitivity depending on lastSensitivitiesIndex
    emgSaturationValue = sensitivities[lastSensitivitiesIndex]; 
    incrementForLEDThreshold = incrementsForLEDThr[lastSensitivitiesIndex];
    incrementForOpenClawThreshold = incrementsForOpenClaw[lastSensitivitiesIndex];
    incrementForClosedClawThreshold = incrementsForClosedClaw[lastSensitivitiesIndex];

    sei();                                                //enable Global Interrupts
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
      
                  outputFrameBuffer[0]= (samplingBuffer[0]>>7)| 0x80;           //convert data to frame according to protocol
                  outputFrameBuffer[1]=  samplingBuffer[0] & 0x7F;              //first bit of every byte is used to flag start of the frame

                 // outputFrameBuffer[0]= (envelopeFirstChanel>>7)| 0x80;           //convert data to frame according to protocol
                 // outputFrameBuffer[1]=  envelopeFirstChanel & 0x7F;
                  
                  outputFrameBuffer[2]= (samplingBuffer[1]>>7)& 0x7F;           //so first bit is set only on first byte of frame (| 0x80)
                  outputFrameBuffer[3]=  samplingBuffer[1] & 0x7F;
                  outputFrameBuffer[4]= (samplingBuffer[2]>>7)& 0x7F;
                  outputFrameBuffer[5]=  samplingBuffer[2] & 0x7F;
                  outputFrameBuffer[6]= (samplingBuffer[3]>>7)& 0x7F;
                  outputFrameBuffer[7]=  samplingBuffer[3] & 0x7F;
                  outputFrameBuffer[8]= (samplingBuffer[4]>>7)& 0x7F;
                  outputFrameBuffer[9]=  samplingBuffer[4] & 0x7F;
                  outputFrameBuffer[10]= (samplingBuffer[5]>>7)& 0x7F;
                  outputFrameBuffer[11]=  samplingBuffer[5] & 0x7F;
        
        
        if(sensitivityVisualFeedbackCounter==0)//disable update of LEDs when we display selected sensitivity level
        {

                  tempEnvValue = samplingBuffer[0];
                  if(tempEnvValue<512)
                  {
                    tempEnvValue = 0;  
                  }
                  else
                  {
                    tempEnvValue = tempEnvValue - 512;  
                  }
                  tempEnvValue = tempEnvValue<<1;
                  //--------------- Calculate envelope ------------------------
                  if(envelopeFirstChanel<tempEnvValue)
                  {
                    envelopeFirstChanel=tempEnvValue;
                  }


                  //-------------- Refresh LED states ------------------------
                  movingThresholdSum = 30;//set initial threshold for first LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00000001;//we set directly bit on port since it is faster than digitalWrite
                  }
                  else
                  {
                      PORTB &= B11111110;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold; //increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00000010;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11111101;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00000100;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11111011;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00001000;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11110111;//turn OFF LED
                  }
                  //---------------------------------------- RELAY THRESHOLD ----------------------------------
                  //check if we should activate relay
                  antiFlickeringTimerForOutput--;
                  if(antiFlickeringTimerForOutput==0)//if anough time passed update relay state
                  {
                      antiFlickeringTimerForOutput = antiFlickeringCounterMax;
                      if(envelopeFirstChanel>movingThresholdSum)
                      {
                          PORTD |= B00001000;//turn ON LED
                      }
                      else
                      {
                          PORTD &= B11110111;//turn OFF LED
                      }
                  }
          
          
                  
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
                  if(envelopeFirstChanel>movingThresholdSum)
                  {
                      PORTB |= B00010000;//turn ON LED
                  }
                  else
                  {
                      PORTB &= B11101111;//turn OFF LED
                  }
                  movingThresholdSum+=incrementForLEDThreshold;//increment threshold for next LED
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
                            emgSaturationValue = sensitivities[lastSensitivitiesIndex]; 
                            incrementForLEDThreshold = incrementsForLEDThr[lastSensitivitiesIndex];
                            incrementForOpenClawThreshold = incrementsForOpenClaw[lastSensitivitiesIndex];
                            incrementForClosedClawThreshold = incrementsForClosedClaw[lastSensitivitiesIndex];
                            PORTB &= 11000000;//turn OFF all leds
                            //prepare to light up only one LED that represent sensitivity level
                            tempCalcByteMask = 1<<lastSensitivitiesIndex;
                            PORTB |= tempCalcByteMask;//light up one LED for visual feedback of sensitivity
                            sensitivityVisualFeedbackCounter = sensitivityVisualFeedbackCounterMax;//set timer for visual feedback
                            
                            PORTD &= B11110111;//reset HHI relay in case it is active, we don't want to fry people
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




          //---------------------------------- GRIPPER MODE BUTTON -----------------------------------------------

          //check if button is pressed (HIGH)
          if (digitalRead(GRIPPER_STATE_BUTTON_PIN))
          { 
            if(gripperButtonEnabled==1)
            {
              if(gripperButtonDebounceCounter == 0)
              {
                gripperButtonEnabled = 0;
                gripperButtonDebounceCounter = gripperButtonDebounceCounterMax;
                //swap default gripper position
                if(currentFunctionality == OPEN_CLAW_MODE)
                {
                  currentFunctionality = CLOSED_CLAW_MODE;
                }
                else
                {
                  currentFunctionality = OPEN_CLAW_MODE;
                }
              }    
            }
          }
          else
          {
              gripperButtonEnabled = 1;
          }
          if(gripperButtonDebounceCounter>0)
          {
            gripperButtonDebounceCounter--;  
          }



        //-------------------------------------------------- SERVO MEASURE --------------------------------------
        if(disableServoMeasure==0)
        {
              disableServoMeasure = 4;//update servo value every 4 servo periods
        
              if(currentFunctionality == OPEN_CLAW_MODE)
              {
                //open claw default position PWM is 2.4ms ON out of 20ms period
                tempActiveServoPeriod = 48;//PWM ON period = 48*50us = 2400us
                for(movingThresholdSum = 40;movingThresholdSum <emgSaturationValue;movingThresholdSum+=incrementForOpenClawThreshold)
                {
                  if(  movingThresholdSum>envelopeFirstChanel)
                  {
                      tempActiveServoPeriod--;//decrease active PWM period if EMG is stronger (min is 1.6ms)
                  }
                }
                activeServoPeriod = tempActiveServoPeriod;//set actual PWM period
                
              }
              else
              {
                //closed claw default position PWM is 1.6ms ON out of 20ms period
                tempActiveServoPeriod = 32;//PWM ON period = 32*50us = 1600us
                for(movingThresholdSum = 120;movingThresholdSum <emgSaturationValue;movingThresholdSum+=incrementForClosedClawThreshold)
                {
                  if(  movingThresholdSum>envelopeFirstChanel)
                  {
                      tempActiveServoPeriod++;//increase active PWM period if EMG is stronger (max is 2.4ms)
                  }
                }
                activeServoPeriod = tempActiveServoPeriod;//set actual PWM period
              }
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
      
                                                               //debug


       //--------------------------------- PARSING OF INCOMING MESSAGES --------------------------------------
      if(messageReceived ==1)
      {
        
            messageReceived =0;
        
            command = strtok(commandBuffer, ";");
            while (command != 0)
            {
                // Split the command in 2 parts: name and value
                separator = strchr(command, ':');
                if (separator != 0)
                {
                    // Actually split the string in 2: replace ':' with 0
                    *separator = 0;
                    --separator;
                    if(*separator == 'c')//if we received command for number of channels
                    {
                      separator = separator+2;
                      tempNumberOfChanels = (byte)atoi(separator);//read number of channels
                        //calculate sampling rate
                      TIMSK1 &= ~(1 << OCIE1A);//disable timer for sampling
                        numberOfChannels = tempNumberOfChanels;
                       regularChannelsIndex = 0;
                       antiFlickeringCounterMax =  ((ANTI_FLICKERING_TIME_IN_MS*10)/numberOfChannels);
                       gripperButtonDebounceCounterMax = GRIPPER_BUTTON_DEBOUNCE/numberOfChannels;
                       sensitivityVisualFeedbackCounterMax = SENSITIVITY_LED_FEEDBACK/numberOfChannels;
                       antiFlickeringTimerForOutput = antiFlickeringCounterMax;
                      OCR1A = (interrupt_Number+1)*numberOfChannels - 1;
                      TIMSK1 |= (1 << OCIE1A);//enable timer for sampling
                    }
                    else if(*separator == 'b')//if we received command for board name
                    {
                       cli();//dissable interrupts
                       PORTD &= B11110111;//turn off HHI relay, we don't want to stim. people
                       sendMessage(CURRENT_SHIELD_TYPE);//send message with escape sequence
                    }
                }
                // Find the next command in input string
                command = strtok(0, ";");
              }  
          
            }//end of message received parsing
            
   }

  //----------------------------------SENDING DATA ----------------------------------------------------------
   if(outputBufferReady == 1 && sendBufferIndex ==0)//if we have new data
   {

      outputBufferReady = 0;//this will be zero until we send whole frame buffer and fill it again
      //since we want to do aux computation (LEDs, relay, servo) only once per sample period
      //and main loop is called multiple times per period (anytime the code is not in interrupt handler)
      //we set this flag here so that aux comp. is done only once after this initialization of frame sending 
      readyToDoAuxComputation = 1;

      //Sends first byte of frame. The rest is sent by TX handler.
      
      UDR0 = outputFrameBuffer[sendBufferIndex];
      
      sendBufferIndex++;


     
  
      
      
   }//end of detection of fresh frame data
}//end of main loop
//---------------------------------------------- END OF MAIN LOOP ---------------------------------------------------------

//-------------------------------------------- SERIAL RX INTERRUPT HANDLER ------------------------------------------------
ISR (USART_RX_vect)
{
  
    tempReceivedByte = UDR0;                                    //get data from serial register
    if(tempReceivedByte==LINE_FEED)                             //if received byte is \n than we are at the end of message
    {
      commandBuffer[commandBufferIndex] = 0;                    //null terminate string
     
      messageReceived = 1;                                      //set flag so that main loop knows that we have new messages to parse
      
      commandBufferIndex =0;                                    //rewind index of buffer to begining
    }
    else
    {
      commandBuffer[commandBufferIndex] = tempReceivedByte;     //if we are not at the end of the command just add another character to buffer
      commandBufferIndex++;                                     //increment writing head index
      if(commandBufferIndex==SIZE_OF_COMMAND_BUFFER)            //if data is longer than buffer 
      {
        commandBufferIndex=0;                                   //start writing from begining
      }
    }
 
}

//-------------------------------------------- SERIAL TX INTERRUPT HANDLER ------------------------------------------------
ISR (USART_TX_vect)
{
  
    if(messageSending==1)                                       //if we are sending message (board name)
    {                                               
        UDR0 = messageBuffer[messageSendingIndex];              //put one byte to serial from message buffer                     
        messageSendingIndex++;
        if(messageSendingIndex == lengthOfMessasge)             //if we sent all bytes of message                           
        {
            messageSending = 0;                                 //reset flags so that code can continue sending data frames
            messageSendingIndex = 0;
            outputBufferReady = 1;
            sendBufferIndex =0;
        }
    }
    else
    {
        
            if(sendBufferIndex==(numberOfChannels<<1))               //we have 2 * numberOfChannels bytes in one frame
            {
                sendBufferIndex = 0;
                
            }
            else
            {
              readyToDoAuxComputation = 1;  
            }
              
              UDR0 = outputFrameBuffer[sendBufferIndex];
               
              sendBufferIndex++;


              //----------------------------------------------- UPDATE SERVO PWM ----------------------------------------
              //servo PWM code is embeded here since we don't have and can not use another timer just for that so 
              //we use TX semi periodic calls (every 50us) to generate PWM  
              if(servoPeriodCounter>0)                          //if we are inside one PWM period
              {
                    servoPeriodCounter--;                       //decrease time inside on PWM period
                    if(servoPeriodCounter<=activeServoPeriod)
                    {
                      PORTD |= B00000100;                       //set PWM pin HIGH
                    }
                    else
                    {
                      PORTD &= B11111011;                       //set PWM pin LOW
                    }
              }
              else                                              //if we are at the end of PWM period
              {
                servoPeriodCounter = LENGTH_OF_SERVO_PERIOD;    //reset time counter
                PORTD &= B11111011;                             //put PWM pin to LOW
                if(disableServoMeasure>0)                       //decrement countdown timer for servo measure 
                {
                  disableServoMeasure--;
                }
                
              }
              //-------------------------------------------- END OF SERVO PWM UPDATE ------------------------------------
        
       
    }
     
}



//------------------------------------------ SAMPLING TIMER INTERRUPT --------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  
  ADMUX =  B01000000;                                           //Start ADC Conversions
  ADCSRA |=B01000000;                                           //do this at the begining since ADC can work in

  outputBufferReady = 1;                                        //signal main loop to send frame
}



//---------------------------ADC INTERRUPT HANDLER ----------------------------------------------------------------------
//This is called when ADC conversion is complete.
ISR(ADC_vect)           
{
        
                  
         samplingBuffer[regularChannelsIndex] = ADCL | (ADCH << 8);      // store lower and higher byte of ADC

         regularChannelsIndex++;
         if(regularChannelsIndex == numberOfChannels)
         {
            regularChannelsIndex = 0;
         }
         else
         {
            ADMUX =  B01000000 | regularChannelsIndex;                   //Select ADC Channel/ REFS0 set means AVcc is reference
            ADCSRA |=B01000000; 
         }
         
        
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
  messageSendingIndex = 0;                                             //rewind index
  sei();                                                               //enable interrupts
}
