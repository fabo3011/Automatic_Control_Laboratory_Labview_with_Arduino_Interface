/*
  LabviewDataHandler.cpp - Library for interacting with serial messages coming from the Labview Interface.
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "LabviewDataHandler.h"

LabviewDataHandler::LabviewDataHandler(){}

void LabviewDataHandler::setBaudRate(unsigned long baud){
    Serial.begin(baud);
}

int LabviewDataHandler::getIncomingFrameFromLabview(ControllerInfo *controllerInfo){

  // -- Serial communication --
  // Example frames: 
  /*
   *    OnOff Ref 15                :  #1,15,
   *    OnOff Ref 10                :  #1,10,
   *    Hyste R: 10, W: 50          :  #2,10,50,
   *    Hyste R: 10, W: 10          :  #2,10,10,
   *    P     R: 10, kP:5           :  #3,10,5,
   *    P     R: 15, kP:8           :  #3,15,8,
   *    PI R: 15, kP:0.5, Ki: 3.3   :  #4,15,0.5,3.3,
   */
   
  /*
   * 1. If data is available at serial port, read
   * 2. If byte read is the header (#), read in order:
   *        Controller
   *        Reference
   *        Window  / Kp  
   *        Ki
   *        ...(Fuzzy)...
   */

   if(Serial.available() > 0){
        incomingByte = Serial.read();
        // If header found
        if(incomingByte == '#'){
          // Read Controller type
          controllerInfo->controllerType = Serial.parseFloat();
          incomingByte = Serial.read();
          // Read Reference
          controllerInfo->reference = Serial.parseFloat();
          incomingByte = Serial.read();
          //TEMPORAL direct assign of the samplingPeriod variable
          controllerInfo->samplingPeriodInSeconds = 0.13;
          // Read controller configs
          switch((int)controllerInfo->controllerType){
            case 2:     // On/Off Hyst
              controllerInfo->hysteresisPercentage = Serial.parseFloat();
              break;
            case 3:      // P Controller
              controllerInfo->kp = Serial.parseFloat();
              break;
            case 4:     // PI Controller
              controllerInfo->kp = Serial.parseFloat();
              incomingByte = Serial.read();
              controllerInfo->ki = Serial.parseFloat();
              break;
            case 5:     // Fuzzy Controller Input
              // Membership function 1
              controllerInfo->inputMFDescriptor[0][0];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[0][1];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[0][2];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[0][3];
              incomingByte = Serial.read();
              // Membership function 2
              controllerInfo->inputMFDescriptor[1][0];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[1][1];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[1][2];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[1][3];
              incomingByte = Serial.read();
              // Membership function 3
              controllerInfo->inputMFDescriptor[2][0];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[2][1];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[2][2];
              incomingByte = Serial.read();
              controllerInfo->inputMFDescriptor[2][3];
            case 6:     // Fuzzy Controller Output
              // Membership function 1
              controllerInfo->outputMFDescriptor[0][0];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[0][1];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[0][2];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[0][3];
              incomingByte = Serial.read();
              // Membership function 2
              controllerInfo->outputMFDescriptor[1][0];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[1][1];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[1][2];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[1][3];
              incomingByte = Serial.read();
              // Membership function 3
              controllerInfo->outputMFDescriptor[2][0];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[2][1];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[2][2];
              incomingByte = Serial.read();
              controllerInfo->outputMFDescriptor[2][3];
              break;
            case 7:     // Open loop
              incomingByte = Serial.read();
              break;  
          }
         }
         return 1;
      }
    return 0;
}

void LabviewDataHandler::setReferenceLinearityRegionTo5V(ControllerInfo *controllerInfo, float m, float b, float setToZeroThreshold){
    //Set reference to 0 with values below setToZeroThreshold
    if(controllerInfo->reference < setToZeroThreshold){
        controllerInfo->reference = 0;
        return;
    }
    // Convert reference to 5v logic
    controllerInfo->reference = (controllerInfo->reference-b)/m;  
    // Take the maximum between the reference and 5V
    controllerInfo->reference = max(5.0,controllerInfo->reference);
}
