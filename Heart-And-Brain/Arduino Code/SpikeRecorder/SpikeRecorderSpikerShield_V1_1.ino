//
// Spike Recorder Arduino code 30. Sep. 2015
// V1.1
// Backyard Brains
// Stanislav Mircic
// https://backyardbrains.com/
// This code is made for Heart and Brain SpikerShield and similar products that need to communicate with
// Spike Recorder desktop software via USB (virtual serial port).
// Sample rate depends on number of channels that are enabled. It is 10kHz divided with number of channels
// So, if only one channel is enabled sample rate is 10kHz. If two channels are enabled sample rate will be 5kHz etc.
//

#define CURRENT_SHIELD_TYPE "HWT:HEARTSS;"

#define EKG A0 //we are reading from AnalogIn 0
#define BUFFER_SIZE 100  //sampling buffer size
#define SIZE_OF_COMMAND_BUFFER 30 //command buffer size
#define LENGTH_OF_MESSAGE_IMPULS 100 // length of message impuls in ms
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

int messageImpulsPin = 5;
int messageImpulseTimer = 0;

long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

int redButton = 4;
int greenButton = 7;

int redLED = 13;
int redLEDCounter = 0;
int greenLED = 8;
int greenLEDCounter = 0;
int redButtonReady = 1;
int greenButtonReady = 1;


////This sets up serial communication values can 9600, 14400, 19200, 28800, 31250, 38400, 57600, and 115200, also 300, 600, 1200, 2400, 4800, but that's too slow for us
/// Interrupt number - very important in combination with bit rate to get accurate data
int interrupt_Number=198;// Output Compare Registers  value = (16*10^6) / (Fs*8) - 1  set to 1999 for 1000 Hz sampling, set to 3999 for 500 Hz sampling, set to 7999 for 250Hz sampling, 199 for 10000 Hz Sampling
int numberOfChannels = 1;//current number of channels sampling
int tempSample = 0; 
int commandMode = 0;//flag for command mode. Don't send data when in command mode

void setup(){ 
  Serial.begin(230400); //Serial communication baud rate (alt. 115200)
  delay(300); //whait for init of serial
  Serial.println("StartUp!");
  Serial.setTimeout(2);
  pinMode(messageImpulsPin, OUTPUT);
   
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
}




ISR(TIMER1_COMPA_vect) {
   //Interrupt at the timing frequency you set above to measure to measure AnalogIn, and filling the buffers
  
   if(messageImpulseTimer>0)
   {
     messageImpulseTimer--;
     if(messageImpulseTimer==0)
     {
         digitalWrite(messageImpulsPin, LOW);
     }
   }
   
   
   if(commandMode!=1)
   {
     
     //Put samples in sampling buffer "reading". Since Arduino Mega has 10bit ADC we will split every sample to 2 bytes
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
       if(numberOfChannels>1)
       {
           //Sample 2. channel and put it into buffer
           tempSample = analogRead(A1);
           reading[head] =  (tempSample>>7) & 0x7F;
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
       if(numberOfChannels>2)
       {
           //Sample 3. channel and put it into buffer
           tempSample = analogRead(A2);
           reading[head] =  (tempSample>>7) & 0x7F;
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
       if(numberOfChannels>3)
       {
           //Sample 4. channel and put it into buffer
           tempSample = analogRead(A3);
           reading[head] =  (tempSample>>7) & 0x7F;
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
       if(numberOfChannels>4)
       {
           //Sample 5. channel and put it into buffer
           tempSample = analogRead(A4);
           reading[head] =  (tempSample>>7) & 0x7F;
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
       if(numberOfChannels>5)
       {
           //Sample 6. channel and put it into buffer
           tempSample = analogRead(A5);
           reading[head] =  (tempSample>>7) & 0x7F;
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
   }
   
   
}
   
void serialEvent() 
{
  commandMode = 1;//frag that we are receiving commands through serial
  TIMSK1 &= ~(1 << OCIE1A);//disable timer for sampling
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
            numberOfChannels = atoi(separator);//read number of channels
          }
           if(*separator == 's')//if we received command for sampling rate
          {
            //do nothing. Do not change sampling rate at this time.
            //We calculate sampling rate further below as (max Fs)/(Number of channels)
          }
          
          if(*separator == 'p')//if we received command for impuls
          {
            //Set impuls
            separator = separator+2;
            digitalWrite(messageImpulsPin, HIGH);
            messageImpulseTimer = (LENGTH_OF_MESSAGE_IMPULS * 10)/numberOfChannels;
          }
          if(*separator == 'b')//if we received command for impuls
          {
            //sendMessage("HWT:PLANTSS;");
            //sendMessage("HWT:MUSCLESS;");
            sendMessage(CURRENT_SHIELD_TYPE);
          }
      }
      // Find the next command in input string
      command = strtok(0, ";");
  }
  //calculate sampling rate
  OCR1A = (interrupt_Number+1)*numberOfChannels - 1;
  TIMSK1 |= (1 << OCIE1A);//enable timer for sampling
  commandMode = 0;
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
      if(tail==BUFFER_SIZE)
      {
        tail = 0;
      }
    }
}
