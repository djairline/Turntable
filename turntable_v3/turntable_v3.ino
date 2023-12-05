#define CONST_SPEED 8000//(TURNTABLE_SPEED*STEPS_PER_TABLE_ROTATION/60) // in steps/s
#define ACCELERATION 2000//(CONST_SPEED*CONST_SPEED/(2*NB_OF_TRACKS_ACCEL*STEPS_PER_TRACK)) // in steps/s^2
#define RX_PIN 8
#define TX_PIN 10 

#include "C:\Users\dj_ai\Desktop\Turntable\turntable_v3\src\FastAccelStepper.h"
#include <TMC2208Stepper.h>
TMC2208Stepper driver = TMC2208Stepper(RX_PIN,TX_PIN);
Stepper stepper;

void setup() {
  /*****************************Serial bluetooth****************************************/
  Serial.begin(250000);
  while(!Serial);
  /*****************************Move controller*****************************************/
  stepper.init(&Serial);
  stepper.setAcceleration(ACCELERATION);
  stepper.setSpeed(CONST_SPEED);
  
  /*********************************Driver*********************************************/
  driver.beginSerial(115200);
  
  driver.push();                // Reset registers
  //Serial.println(driver.MSCNT(&test)); 
  //Serial.println(test & 0xFF);
  driver.pdn_disable(true);     // Use PDN/UART pin for communication
  driver.I_scale_analog(false); // Use internal voltage reference
  driver.rms_current(500);      // Set driver current = 500mA, 0.5 multiplier for hold current and RSENSE = 0.11.
  driver.intpol(false);
  driver.mstep_reg_select(1);
  driver.microsteps(MICROSTEPS);
  driver.dedge(true);

  uint32_t drv_status;
  driver.DRV_STATUS(&drv_status);
  //Serial.print("drv_status=");
  //Serial.println(drv_status, HEX);

  switch (driver.test_connection()) {
    case 1: Serial.println("Connection error: F");break;
    case 2: Serial.println("Connection error: 0");break;
    default: Serial.println("Connection OK");
  }
  uint32_t test;
  driver.toff(0);
  /*while(test !=0){
    driver.MSCNT(&test);
    digitalWrite(STEP_PIN, HIGH); 
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(10);
  }*/
  delay(1000);
  //Serial.println(test);
  driver.toff(2);               // Enable driver in software
  Serial.println("ready");
}


void loop() {
  if (Serial.available()) {
    //Serial.print("received command : ");
    char command = Serial.read()&0x7F;
    //Serial.println(command);
    switch (command) {
      case 'A': //clockwise
        stepper.moveForNTracks(1, 1);
        Serial.println("Received A");
        break;
      case 'B': //anticlockwise
        stepper.moveForNTracks(1, 0);
        Serial.println("Received B");
        break;
      case 'C': //continuous clockwise
        //stepper.moveAtConstSpeed(1);
        break;
      case 'D': //continuous anticlockwise
        //stepper.moveAtConstSpeed(0);
        break;
      case 'E':
        //stepper.stop();
        digitalWrite(EN_PIN, HIGH);
        break;
      case 'F':
        digitalWrite(EN_PIN, LOW);
        break;
      case 'G':
        {
          Serial.print("Received G");
          while (!Serial.available());
          byte nTracks = Serial.read()&0x7F - '0';
          while (!Serial.available());
          bool dir = Serial.read()&0x7F - '0';
          Serial.print(nTracks);
          Serial.println(dir);
          stepper.moveForNTracks(nTracks, dir);
          break;
        }
        
      case 'H':
        {
          Serial.println("received H");
          while(!Serial.available());
          if(Serial.available()==2){
            byte track = Serial.read() - '0';
            stepper.moveTo(track);
          }
          else if(Serial.available()==3){
            byte dectrack = Serial.read() - '0';
            byte unittrack = Serial.read() - '0';
            byte track = dectrack*10+unittrack;
            stepper.moveTo(track);
          }
        }
        break;
    }
  }
 
  stepper.handleCommands();
}
