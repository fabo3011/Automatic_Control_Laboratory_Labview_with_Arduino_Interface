/*
  PController.h - Library to model a P Controller
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "PController.h"
#include "../LabviewDataHandler/LabviewDataHandler.h"

PController::PController(){}

float PController::pControllerResponse(ControllerInfo *controllerInfo, float *currentEK){
    controlSignal = controllerInfo->kp * currentEK;
    controlSignal = max(controlSignal,0.0);  // Lower Saturation Limit
    controlSignal = min(5.0,controlSignal);  // Upper Satutaration Limit
    return controlSignal;
}
