/*
  OnOffController.h - Library to model an OnOff Controller with Hysteresis
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "OnOffController.h"
#include "../LabviewDataHandler/LabviewDataHandler.h"

OnOffController::OnOffController(){}

float OnOffController::onOffControllerResponse(ControllerInfo *controllerInfo, float *currentEK){
    threshold = controllerInfo->reference * (controllerInfo->hysteresisPercentage/100.0);
    if(currentEK > threshold){              // Upper Limit Hysteresis
        return 5.0;
    } else if (currentEK > -threshold){     // Lower Limit Hysteresis
        return 0.0;
    }
}
