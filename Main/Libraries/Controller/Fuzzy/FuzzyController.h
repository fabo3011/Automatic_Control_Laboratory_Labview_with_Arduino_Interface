/*
  FuzzyController.h - Library to model a Fuzzy Controller with 3 different defuzzification methods
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef FuzzyController_h
#define FuzzyController_h

#include "Arduino.h"
#include "../../LabviewDataHandler/LabviewDataHandler.h"
#include "../../ADCDataHandler/ADCDataHandler.h"

class  FuzzyController{
  public:
    FuzzyController();
    float fuzzyControllerResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo, float *currentEK);
  private:
    float controlSignal;
    // Differential in the control signal obtained by the defuzzifier
    float dU;
};
#endif