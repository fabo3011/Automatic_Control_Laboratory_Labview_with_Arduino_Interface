/*
  ADCDataHandler.h - Library for interacting with ADC signals coming from uK (control signal) and yK (output signal)
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ADCDataHandler.h"
#include "Filters.h"

ADCDataHandler::ADCDataHandler(int _UKAnalogPin, int _YKAnalogPin){
    UKAnalogPin = _UKAnalogPin;
    YKAnalogPin = _YKAnalogPin;
}
void ADCDataHandler::setLowPassButterworthFilterCutoffFrequency(float cutoffFrequency){
    // yK input digital filter configuration
    lowPassFilter.setAsFilter(LOWPASS_BUTTERWORTH,cutoffFrequency);
}
void ADCDataHandler::readUKFromADC(ADCInfo *adcInfo){
    adcInfo->uKFromADC = analogRead(UKAnalogPin)*5.0/1023.0;
}
void ADCDataHandler::readYKFromADC(ADCInfo *adcInfo){
    adcInfo->yKFromADC = analogRead(YKAnalogPin)*5.0/1023.0;
}
void ADCDataHandler::filterYK(ADCInfo *adcInfo){
    adcInfo->yKFromADC = lowPassFilter.input(adcInfo->yKFromADC);
}