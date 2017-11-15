/*
  Controller.h - Library to model distinct types of controllers according to the infromation provided by the struct ControllerInfo e.g. OnOff, P, PI, Fuzzy, Open Loop
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Controller.h"
#include "../LabviewDataHandler/LabviewDataHandler.h"
#include "../ADCDataHandler/ADCDataHandler.h"
#include "OnOff/OnOffController.h"
#include "P/PController.h"
#include "PI/PIController.h"

Controller::Controller(){
    previousUK = 0.0;
    previousEK = 0.0;
    pIController.setPIControllerScalingFactor(100000.0);
}

void Controller::calculateControlSignalResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo){
    currentEK = controllerInfo->reference-adcInfo->yKFromADC;
    switch((int)controllerInfo->controllerType){
        case 2:     // On/Off Hyst
          controlSignal = onOffController.onOffControllerResponse(controllerInfo, &currentEK);
          break;
        case 3:      // P Controller
          controlSignal = pController.pControllerResponse(controllerInfo, &currentEK);
          break;
        case 4:     // PI Controller
          controlSignal = pIController.pIControllerResponse(controllerInfo, &currentEK, &previousUK, &previousEK);
          break;
        case 5:     // Fuzzy Controller
        
        case 6:     // Open loop
          controlSignal = controllerInfo->reference;
          break;
    }
    previousEK = currentEK;
    previousUK = adcInfo->uKFromADC;
}