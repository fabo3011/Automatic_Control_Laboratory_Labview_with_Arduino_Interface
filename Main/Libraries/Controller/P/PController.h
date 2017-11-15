/*
  PController.h - Library to model a P Controller with Hysteresis
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef PController_h
#define PController_h

#include "Arduino.h"
#include "../../LabviewDataHandler/LabviewDataHandler.h"

class  PController{
  public:
    PController();
    float pControllerResponse(ControllerInfo *controllerInfo, float *currentEK);
  private:
    float controlSignal;
};
#endif