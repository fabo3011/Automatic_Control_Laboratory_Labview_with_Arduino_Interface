/*
  Controller.h - Library to model distinct types of controllers according to the infromation provided by the struct ControllerInfo e.g. OnOff, P, PI, Fuzzy, Open Loop
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef Controller_h
#define Controller_h

#include "Arduino.h"
#include "LabviewDataHandler.h"
#include "ADCDataHandler.h"

class  Controller{
  public:
    Controller();
    void calculateControlSignalResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo);
  private:
    float controlSignal;
    float currentEK;
    float previousUK;
    float previousEK;
};
#endif