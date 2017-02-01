
  /*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains 2015
  * Muscle SpikerShield Arduino UNO Code for RF controll of remote Transcutaneous electrical nerve 
  * stimulation (TENS) devices. 
  *
  * Code monitors amplitude of EMG envelope, displays EMG strength on LED bar and generates PWM
  * signal that is transmitted via 433Mhz RF serial module to multiple receivers
  * 
  * V1.2
  * Written by Stanislav Mircic
  *
  * ----------------------------------------------------------------------------------------------------
  */
  #define NUMBER_OF_LEDS 6                    //Number of LEDs in LED bar
  #define OUTPUT_PIN 2                        //Output pin (this pin is connected directly to RF serial transmitter)
  #define OPERATION_MODE_BUTTON_PIN 7         //pin for button that controls operation mode
  #define SENSITIVITY_BUTTON_PIN 4            //pin for button that controls sensitivity
  #define INDICATOR_LED_PIN 13                //pin for LED that indicates state of output

  int emgMeasurement = 0;                     //current value of measured EMG
  int buttonState;                            //temporary variable for reading of button state
  
  int currentOperationMode = 2;

  //EMG saturation values (when EMG reaches this value the output will be activated/deactivated)
  int sensitivities[] = {200, 350, 520, 680, 840, 1000};
  int lastSensitivitiesIndex = 2;             //set initial sensitivity index
  int sensitivity;                            //current sensitivity threshold
  
  byte ledbarHeight = 0;                      //temporary variable for led bar height
  byte ledPins[] = {8, 9, 10, 11, 12, 13};    //pins for LEDs in LED bar


  //-----------------------------------------------------------------------------------
  //   Setup inputs and outputs
  // ----------------------------------------------------------------------------------
  void setup()
  {
    //set realy pin as output
    pinMode(OUTPUT_PIN, OUTPUT);
    
    //set button pins as inputs
    pinMode(OPERATION_MODE_BUTTON_PIN, INPUT);//not used in this project
    pinMode(SENSITIVITY_BUTTON_PIN, INPUT);
  
    //set all pins for LED bar to output
    for(int i = 0; i < NUMBER_OF_LEDS; i++)
    { 
      pinMode(ledPins[i], OUTPUT);
    }
    digitalWrite(OUTPUT_PIN, HIGH);
    //get the current sensitivity
    sensitivity = sensitivities[lastSensitivitiesIndex];
  }




  //-----------------------------------------------------------------------------------
  //   Main loop
  //
  //   - Checks state of sesitivity button
  //   - Measure EMG
  //   - Generate PWM signal that will be sent through serial RF device
  //   - Shows EMG strength on LED bar  
  // ----------------------------------------------------------------------------------
  void loop()
  {

   //-----------------------  Switch sensitivity ------------------------------------
      
    //check if button is pressed (HIGH)
    if (digitalRead(SENSITIVITY_BUTTON_PIN))
    { 
        turnOffAllLeds();
      
        //increment sensitivity index
        lastSensitivitiesIndex++;
        if(lastSensitivitiesIndex==NUMBER_OF_LEDS)
        {
          lastSensitivitiesIndex = 0;
        }
      
        //get current sensitivity value
        sensitivity = sensitivities[lastSensitivitiesIndex]; 
        
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
  
  
    //--------------------------- Measure EMG signal --------------------------------------
    
    emgMeasurement = analogRead(A0);
  
  
    //---------------------- Generate PWM signal at output pin -----------------------------

    //If EMG reach threshold generate ON PWM output (83% duty cycle)
    //otherwise generate OFF PWM output (16% duty cycle)
    if(emgMeasurement <= sensitivity )
    {
        digitalWrite(OUTPUT_PIN, HIGH);
        delay(4);
        digitalWrite(OUTPUT_PIN, LOW);
        delay(20);
       
    }
    else
    {
        digitalWrite(OUTPUT_PIN, HIGH);
        delay(20);
        digitalWrite(OUTPUT_PIN, LOW);
        delay(4);
       
    }
       
     //---------------------- Display EMG strength on LED bar ------------------------------       
    turnOffAllLeds();
    emgMeasurement= constrain(emgMeasurement, 0, sensitivity);
    ledbarHeight = map(emgMeasurement, 0, sensitivity, 0, NUMBER_OF_LEDS);
    
    for(int k = 0; k < ledbarHeight; k++)
    {
        digitalWrite(ledPins[k], HIGH);
    }
  } 
  
  
  
  
  //---------------------------------------------------------------------------
  //
  //                 Helper functions
  //
  //---------------------------------------------------------------------------
  
  

  //---------------------------------------------------------------------------
  // Turn OFF all LEDs in LED bar
  //---------------------------------------------------------------------------
  void turnOffAllLeds()
  {
    for(int k = 0; k < NUMBER_OF_LEDS; k++)
    {
      digitalWrite(ledPins[k], LOW);
    }
  }
  

  

