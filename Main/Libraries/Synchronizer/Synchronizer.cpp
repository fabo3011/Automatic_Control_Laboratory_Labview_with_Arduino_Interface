/*
  Synchronizer.h - Library to synchronize the period of the Main program to match the Controller's sampling rate
  Created by fgg, November 18, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Synchronizer.h"

Synchronizer::Synchronizer(){}

void Synchronizer::setSamplingSignalPinAsOutput(int pin){
    pinMode(pin,OUTPUT);
}
void Synchronizer::setSamplingSignalPinToHIGH(int pin){
    digitalWrite(pin,HIGH);
}
void Synchronizer::setSamplingSignalPinToLOW(int pin){
    digitalWrite(pin,LOW);
}
void Synchronizer::assignTimestampInMicrosToT1(){
    t1 = micros();
}
void Synchronizer::assignTimestampInMicrosToT2(){
    t2 = micros();
}
void Synchronizer::waitUntilNextSamplingPeriod(ControllerInfo *controllerInfo){
    delayMicroseconds( ( (unsigned long) ( controllerInfo->samplingPeriodInSeconds ) ) * (unsigned long) (1000000) - t2 + t1);    
}