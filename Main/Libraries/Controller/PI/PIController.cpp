/*
  PIController.h - Library to model a PI Controller
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "PIController.h"

PIController::PIController(){}

void  PIController::setPIControllerScalingFactor(float F){
    scalingFactor = F;
}
float PIController::pIControllerResponse(ControllerInfo *controllerInfo, float *currentEK, float *previousUK, float *previousEK){
    //Coefficients needed to calculate the control signal
    a = 1   *   scalingFactor;
    b = ( controllerInfo->kp + ( controllerInfo->ki * controllerInfo->samplingPeriodInSeconds ) / 2 )    *   scalingFactor;
    c = ( ( controllerInfo->ki * controllerInfo->samplingPeriodInSeconds ) / 2 - controllerInfo->kp )    *   scalingFactor;

    controlSignal = ( a * *previousUK + b * *currentEK + c * *previousEK ) / scalingFactor;
    controlSignal = max(controlSignal,0.0);  // Lower Saturation Limit
    controlSignal = min(5.0,controlSignal);  // Upper Satutaration Limit
    return controlSignal;
}
