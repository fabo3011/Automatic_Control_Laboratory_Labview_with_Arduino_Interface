/*
  PIController.h - Library to model a PI Controller
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef PIController_h
#define PIController_h

#include "Arduino.h"
#include "../../LabviewDataHandler/LabviewDataHandler.h"

class  PIController{
  public:
    PIController();
    void  setPIControllerScalingFactor(float F);
    float pIControllerResponse(ControllerInfo *controllerInfo, float *currentEK, float *previousUK, float *previousEK);
  private:
    float controlSignal;
    float scalingFactor;
    float a;
    float b;
    float c;
};
#endif