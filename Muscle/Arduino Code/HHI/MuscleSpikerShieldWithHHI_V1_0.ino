 /*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains 2015
  * Muscle SpikerShield Arduino UNO Code for Human-Human-Interface
  *
  * Code monitors amplitude of EMG envelope, displays EMG strength on LED bar and controls 
  * relay that turns on/off a TENS (Transcutaneous Electrical Nerve Stimulation) device.
  * 
  * V1.0
  * Written by Marcio Amorim
  * Updated by Stanislav Mircic
  *
  * Tested with Muscle SpikerShield V2.31
  * ----------------------------------------------------------------------------------------------------
  */  
  
  #define RELAY_PIN 3                         //pin for relay that controls TENS device
  #define SENSITIVITY_BUTTON_PIN 7            //pin for button that selects sesitivity
  #define NUM_LED 6                           //number of LEDs in LED bar
  #define RELAY_THRESHOLD 4                   //defines sensitivity of relay

  byte ledPins[] = {8, 9, 10, 11, 12, 13};    //pins for LEDs in LED bar
  
  //EMG saturation values (when EMG reaches this value the TENS relay will be activated)
  int sensitivities[] = {200, 350, 520, 680, 840, 1000};
  int lastSensitivitiesIndex = 2;             //set initial sensitivity index
  
  int emgSaturationValue = 0;                 //selected sensitivity/EMG saturation value
  int analogReadings;                         //measured value for EMG
  byte ledbarHeight = 0;                      //temporary variable for led bar height


  //-----------------------------------------------------------------------------------
  //   Setup inputs and outputs
  // ----------------------------------------------------------------------------------
  void setup(){

    //init button pin to input                                
    pinMode(SENSITIVITY_BUTTON_PIN, INPUT); 
    //init relay pin to output    
    pinMode(RELAY_PIN, OUTPUT); 
    digitalWrite(RELAY_PIN, LOW);
    //initialize all LED pins to output
    for(int i = 0; i < NUM_LED; i++){ 
      pinMode(ledPins[i], OUTPUT);
    }
    
    //get current sensitivity
    emgSaturationValue = sensitivities[lastSensitivitiesIndex];
  }



  //-----------------------------------------------------------------------------------
  //   Main loop
  //
  //   - Checks state of sesitivity button
  //   - Measure EMG
  //   - Shows EMG strength on LED bar
  //   - Turns ON or OFF the relay for TENS device
  // ----------------------------------------------------------------------------------
  void loop()
  {
   
        //-----------------------  Switch sensitivity ------------------------------------
    
        //check if button is pressed (HIGH)
        if (digitalRead(SENSITIVITY_BUTTON_PIN))
        { 
            //turn off all the LEDs in LED bar
            for(int j = 0; j < NUM_LED; j++)
            {  
              digitalWrite(ledPins[j], LOW);
            }
          
            //increment sensitivity index
            lastSensitivitiesIndex++;
            if(lastSensitivitiesIndex==NUM_LED)
            {
              lastSensitivitiesIndex = 0;
            }
          
            //get current sensitivity value
            emgSaturationValue = sensitivities[lastSensitivitiesIndex]; 
            
            //light up LED at lastSensitivitiesIndex position for visual feedback
            digitalWrite(ledPins[lastSensitivitiesIndex], HIGH);
           
            //wait user to release button
            while (digitalRead(SENSITIVITY_BUTTON_PIN)) 
            {  
              delay(10);
            }       
            //whait a bit more so that LED light feedback is always visible
            delay(100);        
        }
        

        //-----------------------------  Measure EMG -----------------------------------------------
    
        analogReadings = analogRead(A0);//read EMG value from analog input A0
        
        
        //---------------------- Show EMG strength on LED ------------------------------------------
        
        //turn OFF all LEDs on LED bar
        for(int j = 0; j < NUM_LED; j++)
        {  
          digitalWrite(ledPins[j], LOW);
        }
         
        //calculate what LEDs should be turned ON on the LED bar
        analogReadings= constrain(analogReadings, 30, emgSaturationValue);
        ledbarHeight = map(analogReadings, 30, emgSaturationValue, 0, NUM_LED);
        
        //turn ON LEDs on the LED bar
        for(int k = 0; k < ledbarHeight; k++)
        {
          digitalWrite(ledPins[k], HIGH);
        }
        
        
        //----------------------- Turn ON/OFF relay for TENS ---------------------------------------
        
        //Turn ON relay if EMG is greater than threshold value
        //(threshold is expressed in LED bar height units)
        if(ledbarHeight>RELAY_THRESHOLD)
        {
          digitalWrite(RELAY_PIN, HIGH);
          delay(50);
        }
        else
        {
          digitalWrite(RELAY_PIN, LOW);
        }
}
