  /*
  * ----------------------------------------------------------------------------------------------------
  * Backyard Brains 2016
  * Muscle SpikerShield Arduino UNO Code for Testing Gripper Funcionality.
  *
  * V1.0
  * Written by Tim Marzullo
  *
  * Tested with Muscle SpikerShield V2.6
  * ----------------------------------------------------------------------------------------------------
  */

  #include <Servo.h>  
  #define SERVO_PIN 2                         //pin for servo motor
  Servo Gripper;                              //servo for gripper


  //-----------------------------------------------------------------------------------
  //   Setup servo
  // ----------------------------------------------------------------------------------
  void setup(){
    //init servo
    Gripper.attach(SERVO_PIN); 
  }


  //-----------------------------------------------------------------------------------
  //   Main loop
  //   Moves Gripper between Open and Closed Position to Test Gripper Functionality
  // ----------------------------------------------------------------------------------
  void loop() {
    Gripper.write(105); //close Gripper
    delay(500);         // delay 500 ms
    Gripper.write(190); //open Gripper
    delay(2000);        // delay 2000 ms
}


