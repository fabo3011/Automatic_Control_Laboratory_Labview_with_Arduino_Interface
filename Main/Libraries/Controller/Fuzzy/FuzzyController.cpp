/*
  FuzzyController.h - Library to model a Fuzzy Controller with 3 different defuzzification methods
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "FuzzyController.h"

FuzzyController::FuzzyController(){}

float FuzzyController::fuzzyControllerResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo, float *currentEK){
    
    Fuzzify_and_Polyline(currentEK);
    dU = Singleton_Def();

    return controlSignal;
}
