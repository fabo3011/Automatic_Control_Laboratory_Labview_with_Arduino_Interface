/*
  OnOffController.h - Library to model an OnOff Controller with Hysteresis
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef OnOffController_h
#define OnOffController_h

#include "Arduino.h"
#include "../../LabviewDataHandler/LabviewDataHandler.h"

class  OnOffController{
  public:
    OnOffController();
    float onOffControllerResponse(ControllerInfo *controllerInfo, float *currentEK);
  private:
    float threshold;
};
#endif