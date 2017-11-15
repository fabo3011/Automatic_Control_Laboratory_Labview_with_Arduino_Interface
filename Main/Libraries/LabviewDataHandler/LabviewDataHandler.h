/*
  LabviewDataHandler.h - Library for interacting with serial messages coming from the Labview Interface.
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef LabviewDataHandler_h
#define LabviewDataHandler_h

#include "Arduino.h"

//Struct Declaration
//Controller Information Structure to Store Data Recieved by the Labview Interface
typedef struct ControllerInfo ControllerInfo;
struct ControllerInfo{
    float reference;
    float samplingPeriodInSeconds;
    float controllerType;
    float hysteresisPercentage;
    float kp;
    float ki;
    float inputMFDescriptor[3][4];
    float outputMFDescriptor[3][4];
};

class LabviewDataHandler{
  public:
    LabviewDataHandler();
    void setBaudRate(unsigned long baud);
    int  getIncomingFrameFromLabview(ControllerInfo *controllerInfo);
    void setReferenceLinearityRegionTo5V(ControllerInfo *controllerInfo, float m, float b, float setToZeroThreshold);
  private:
    char incomingByte;
};
#endif