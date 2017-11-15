/*
  Controller.h - Library to model distinct types of controllers according to the infromation provided by the struct ControllerInfo e.g. OnOff, P, PI, Fuzzy, Open Loop
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef Controller_h
#define Controller_h

#include "Arduino.h"

class  Controller{
  public:
    Controller();
    void setLowPassButterworthFilterCutoffFrequency(float cutoffFrequency);
  private:
    float controlSignal;
};
#endif