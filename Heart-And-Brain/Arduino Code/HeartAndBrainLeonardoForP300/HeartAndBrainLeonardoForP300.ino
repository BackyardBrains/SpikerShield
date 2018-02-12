//
// Plant SpikerBox based P300 experiment on Leonardo
// 
// V1.0
// Backyard Brains
// Stanislav Mircic
// https://backyardbrains.com/
//
// D2 - Speaker output
// D3 - Aux. button input (generates Event 3 on rising edge)
//

#define CURRENT_SHIELD_TYPE "HWT:HBLEOSB;"
#define TONE_PERIOD_MS 2000                 //max 6000
#define TONE_DURATION_MS 300             //max BEEP_PERIOD_MS
#define CHANCE_OF_ODD_TONE_PERCENT 10     //max 100
#define FREQUENCY_OF_NORMAL_TONE_HZ 300     
#define FREQUENCY_OF_ODD_TONE_HZ 500


#define BUFFER_SIZE 256  //sampling buffer size
#define SIZE_OF_COMMAND_BUFFER 30 //command buffer size

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int buffersize = BUFFER_SIZE;
int head = 0;//head index for sampling circular buffer
int tail = 0;//tail index for sampling circular buffer
byte writeByte;
char commandBuffer[SIZE_OF_COMMAND_BUFFER];//receiving command buffer
byte reading[BUFFER_SIZE]; //Sampling buffer
#define ESCAPE_SEQUENCE_LENGTH 6
byte escapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,128,255};
byte endOfescapeSequence[ESCAPE_SEQUENCE_LENGTH] = {255,255,1,1,129,255};

#define SPEAKER_OUTPUT 2
#define EVENT_INPUT_3 3



unsigned int periodOfToneInTimerIntervals = 10000;
unsigned int durationOfTone = 3000;
unsigned int chancesOfOddToneInMaxRND = 6553;
unsigned int periodOfNormalWave = 33;
unsigned int periodOfOddWave = 20;

unsigned int counterForTonePeriod = 0;
unsigned int counterForToneDuration = 0;
unsigned int currentPeriodOfToneWave = 0;
unsigned int counterForWavePeriod = 0;


int interrupt_Number=198; // 10000Hz timer compare register
int numberOfChannels = 1; // current number of channels sampling
int tempSample = 0; 
int commandMode = 0;      //flag for command mode. Don't send data when in command mode
unsigned int randNumber;
float tempCalculationNumber;
float tempCalculationNumber2;
void setup(){ 
  Serial.begin(230400); //Serial communication baud rate (alt. 115200)
  //while (!Serial)
  //{}  // wait for Serial comms to become ready
  delay(300); //whait for init of serial
  Serial.println("StartUp!");
  Serial.setTimeout(2);

  //Inputs for events 
  pinMode(SPEAKER_OUTPUT, OUTPUT);
  pinMode(EVENT_INPUT_3, INPUT);


  periodOfToneInTimerIntervals = TONE_PERIOD_MS*10;
  
  tempCalculationNumber = ((float)CHANCE_OF_ODD_TONE_PERCENT)/100.0;
  chancesOfOddToneInMaxRND = tempCalculationNumber*65535.0;
  
  periodOfNormalWave = 5000/FREQUENCY_OF_NORMAL_TONE_HZ;
  periodOfOddWave = 5000/FREQUENCY_OF_ODD_TONE_HZ;
  
  counterForTonePeriod = periodOfToneInTimerIntervals;
  currentPeriodOfToneWave = periodOfNormalWave;
  counterForWavePeriod = currentPeriodOfToneWave;
  durationOfTone = 10*TONE_DURATION_MS;
  counterForToneDuration = 0;
  
  // TIMER SETUP- the timer interrupt allows preceise timed measurements of the reed switch
  //for mor info about configuration of arduino timers see http://arduino.cc/playground/Code/Timer1
  cli();//stop interrupts

  //Make ADC sample faster. Change ADC clock
  //Change prescaler division factor to 16
  sbi(ADCSRA,ADPS2);//1
  cbi(ADCSRA,ADPS1);//0
  cbi(ADCSRA,ADPS0);//0

  //set timer1 interrupt at 10kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0;
  OCR1A = interrupt_Number;// Output Compare Registers 
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts
  //END TIMER SETUP
  TIMSK1 |= (1 << OCIE1A);
}

int buttonEnabled = 1;
int buttonDebounceTimer = 0;
#define MAX_DEBOUNCE_TIME 2000


//
// Create a random integer from 0 - 65535
//
unsigned int rng() {
  static unsigned int y = 0;
  y += micros(); // seeded with changing number
  y ^= y << 2; y ^= y >> 7; y ^= y << 7;
  return (y);
} 

ISR(TIMER1_COMPA_vect) {
   //Interrupt at the timing frequency you set above to measure to measure AnalogIn, and filling the buffers

    //---------------------------- GENERATE 5kHz MODULATION ------------------------
    PORTC ^= B10000000;//5kHz modulation tone generator


   //----------------------------- MEASURE EEG -------------------------------------
   if(commandMode!=1)
   {
     //Put samples in sampling buffer "reading". Since Arduino Leonardo has 10bit ADC we will split every sample to 2 bytes
     //First byte will contain 3 most significant bits and second byte will contain 7 least significat bits.
     //First bit in all byte will not be used for data but for marking begining of the frame of data (array of samples from N channels)
     //Only first byte in frame will have most significant bit set to 1
     
       //Sample first channel and put it into buffer
       tempSample = analogRead(A0);
       reading[head] =  (tempSample>>7)|0x80;//Mark begining of the frame by setting MSB to 1
       head = head+1;
       if(head==BUFFER_SIZE)
       {
         head = 0;
       }
       reading[head] =  tempSample & 0x7F;
       head = head+1;
       if(head==BUFFER_SIZE)
       {
         head = 0;
       }
       
   }

    //------------------------------ GENERATE SOUND ------------------------------
    randNumber = rng();
    counterForTonePeriod--;
    if(counterForTonePeriod==0)
    {
        counterForTonePeriod = periodOfToneInTimerIntervals;
        if(randNumber<chancesOfOddToneInMaxRND)
        {
          sendMessage("EVNT:2;");
          currentPeriodOfToneWave = periodOfOddWave;
        }
        else
        {
          sendMessage("EVNT:1;");
          currentPeriodOfToneWave = periodOfNormalWave;
        }
        PORTD ^= B00000010;
        counterForWavePeriod = currentPeriodOfToneWave;
        counterForToneDuration = durationOfTone;
    }

    if(counterForToneDuration>0)
    {
      counterForToneDuration--;
      counterForWavePeriod--;
      if(counterForWavePeriod ==0)
      {
        //flip tone output
        PORTD ^= B00000010;
        counterForWavePeriod = currentPeriodOfToneWave;
      }
      
    }
    else
    {
      //Turn off tone generator  
      PORTD &= B11111101;
    }

    //--------------------------- CHECK BUTTON ----------------------------------
    //check if button is pressed (HIGH)
    if (digitalRead(EVENT_INPUT_3))
    { 
          if(buttonEnabled==1)
          {
            if(buttonDebounceTimer == 0)
            {
              buttonEnabled = 0;
              buttonDebounceTimer = MAX_DEBOUNCE_TIME;
              //swap default gripper position
              sendMessage("EVNT:3;");
            }    
          }
    }
    else
    {
        buttonEnabled = 1;
    }
    if(buttonDebounceTimer>0)
    {
      buttonDebounceTimer--;  
    }
   
}
  

//push message to main sending buffer
//timer for sampling must be dissabled when 
//we call this function
void sendMessage(const char * message)
{

  int i;
  //send escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      reading[head++] = escapeSequence[i];
      if(head==BUFFER_SIZE)
      {
        head = 0;
      }
  }

  //send message
  i = 0;
  while(message[i] != 0)
  {
      reading[head++] = message[i++];
      if(head==BUFFER_SIZE)
      {
        head = 0;
      }
  }

  //send end of escape sequence
  for(i=0;i< ESCAPE_SEQUENCE_LENGTH;i++)
  {
      reading[head++] = endOfescapeSequence[i];
      if(head==BUFFER_SIZE)
      {
        head = 0;
      }
  }
  
}




void loop(){
    
    while(head!=tail && commandMode!=1)//While there are data in sampling buffer whaiting 
    {
      Serial.write(reading[tail]);
      //Move thail for one byte
      tail = tail+1;
      if(tail>=BUFFER_SIZE)
      {
        tail = 0;
      }
    }

    if(Serial.available()>0)
    {
                     // digitalWrite(6, HIGH);
                   //digitalWrite(6, LOW);
                  commandMode = 1;//frag that we are receiving commands through serial
                  //TIMSK1 &= ~(1 << OCIE1A);//disable timer for sampling
                  // read untill \n from the serial port:
                  String inString = Serial.readStringUntil('\n');
                
                  //convert string to null terminate array of chars
                  inString.toCharArray(commandBuffer, SIZE_OF_COMMAND_BUFFER);
                  commandBuffer[inString.length()] = 0;
                  
                  
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
                          if(*separator == 'c')//if we received command for number of channels
                          {
                            separator = separator+2;
                            numberOfChannels = 1;//atoi(separator);//read number of channels
                          }
                           if(*separator == 's')//if we received command for sampling rate
                          {
                            //do nothing. Do not change sampling rate at this time.
                            //We calculate sampling rate further below as (max Fs)/(Number of channels)
                          }

                          if(*separator == 'b')//if we received command for impuls
                          {
                            sendMessage(CURRENT_SHIELD_TYPE);
                          }
                      }
                      // Find the next command in input string
                      command = strtok(0, ";");
                  }
                  //calculate sampling rate
                  
                  //TIMSK1 |= (1 << OCIE1A);//enable timer for sampling
                  commandMode = 0;
      }
    
}
