/*
  Controller.h - Library to model distinct types of controllers according to the infromation provided by the struct ControllerInfo e.g. OnOff, P, PI, Fuzzy, Open Loop
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Controller.h"
#include "LabviewDataHandler.h"
#include "ADCDataHandler.h"

Controller::Controller(){
    previousUK = 0.0;
    previousEK = 0.0;
}

void Controller::calculateControlSignalResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo){
    currentEK = controllerInfo->reference-adcInfo->yKFromADC;
    switch((int)controllerInfo->controllerType){
        case 2:     // On/Off Hyst
          controlSignal = onOffController.onOffControllerResponse(controllerInfo, adcInfo, &currentEK);
          break;
        case 3:      // P Controller
          //controllerInfo->kp = Serial.parseFloat();
          break;
        case 4:     // PI Controller
          //controllerInfo->kp = Serial.parseFloat();
          //incomingByte = Serial.read();
          //controllerInfo->ki = Serial.parseFloat();
          break;
        case 5:     // Fuzzy Controller
        
        case 6:     // Open loop
          break;
    }
    previousEK = currentEK;
    previousUK = adcInfo->uKFromADC;
}