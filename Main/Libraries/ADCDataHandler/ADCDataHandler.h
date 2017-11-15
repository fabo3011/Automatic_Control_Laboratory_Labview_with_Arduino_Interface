/*
  ADCDataHandler.h - Library for interacting with ADC signals coming from uK (control signal) and yK (output signal)
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef ADCDataHandler_h
#define ADCDataHandler_h

#include "Arduino.h"
#include "Filters.h"

//Struct Declaration
//ADC Information Structure to Store Data Recieved by the Analog to Digital Converters which contain uK and yK as inputs
typedef struct ADCInfo ADCInfo;
struct ADCInfo{
    float uKFromADC;
    float yKFromADC;
};

class  ADCDataHandler{
  public:
    ADCDataHandler(int _UKAnalogPin, int _YKAnalogPin);
    void setLowPassButterworthFilterCutoffFrequency(float cutoffFrequency);
    void readUKFromADC(ADCInfo *adcInfo);
    void readYKFromADC(ADCInfo *adcInfo);
    void filterYK(ADCInfo *adcInfo);
  private:
    //lowPassFilter:     Filter object for Y(k) and u(k)
    FilterTwoPole lowPassFilter;    
    int UKAnalogPin;
    int YKAnalogPin;
};
#endif