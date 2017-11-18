/*
  Synchronizer.h - Library to synchronize the period of the Main program to match the Controller's sampling rate
  Created by fgg, November 18, 2017.
  Released into the public domain.
*/
#ifndef Synchronizer_h
#define Synchronizer_h

#include "Arduino.h"
#include "../LabviewDataHandler/LabviewDataHandler.h"

class Synchronizer{
  public:
    Synchronizer();
    void setSamplingSignalPinAsOutput(int pin);
    void setSamplingSignalPinToHIGH(int pin);
    void setSamplingSignalPinToLOW(int pin);
    void assignTimestampInMicrosToT1();
    void assignTimestampInMicrosToT2();
    void waitUntilNextSamplingPeriod(ControllerInfo *controllerInfo);
  private:
    unsigned long t1;
    unsigned long t2;
};
#endif