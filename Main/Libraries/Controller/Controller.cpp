/*
  Controller.h - Library to model distinct types of controllers according to the infromation provided by the struct ControllerInfo e.g. OnOff, P, PI, Fuzzy, Open Loop
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Controller.h"

Controller::Controller(){
    previousUK = 0.0;
    previousEK = 0.0;
    pIController.setPIControllerScalingFactor(100000.0);
}

void Controller::setControlSignalResponsePWMPinAsOutput(int pin){
    pinMode(pin,OUTPUT); 
}
void Controller::writeControlSignalResponseToPWMPin(int pin){
    analogWrite(pin,round((controlSignal/5.0)*255.0));
}
void Controller::calculateControlSignalResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo){
    currentEK = controllerInfo->reference - adcInfo->yKFromADC;
    switch((int)controllerInfo->controllerType){
        case 2:     // On/Off Hyst
          controlSignal = onOffController.onOffControllerResponse(controllerInfo, &currentEK);
          break;
        case 3:      // P Controller
          controlSignal = pController.pControllerResponse(controllerInfo, &currentEK);
          break;
        case 4:     // PI Controller
          controlSignal = pIController.pIControllerResponse(controllerInfo, &currentEK, &previousUK, &previousEK);
          break;
        case 5: case 6:     // Fuzzy Controller
          controlSignal = fuzzyController.fuzzyControllerResponse(controllerInfo, adcInfo, &currentEK, &previousUK);
          break;
        case 7:     // Open loop
          controlSignal = controllerInfo->reference;
          break;
    }
    previousEK = currentEK;
    previousUK = controlSignal;
}
void Controller::retrieveLinearityRegionForYKAndUK(ADCInfo *adcInfo, int m, int b, int setToMaxThreshold, bool isUKFromADCSignal ){
  adcInfo->yKFromADC = max( adcInfo->yKFromADC * m + b, 0 );
  adcInfo->yKFromADC = min( adcInfo->yKFromADC, setToMaxThreshold );
  if( isUKFromADCSignal ){
    adcInfo->uKFromADC = max( adcInfo->uKFromADC * m + b, 0 );
    adcInfo->uKFromADC = min( adcInfo->uKFromADC, setToMaxThreshold );
  } else{
    controlSignal = max( controlSignal * m + b, 0 );
    controlSignal = min( controlSignal, setToMaxThreshold );
  }
}
void Controller::sendYKAndUKToLabview( ADCInfo *adcInfo, bool isUKFromADCSignal ){
  float tempY = round( max( adcInfo->yKFromADC * 100.0, 0 ) ) / 100.0;
  float tempU;
  if( isUKFromADCSignal ){
    tempU = round( max( adcInfo->uKFromADC * 100.0, 0 ) ) / 100.0;
  } else{
    tempU = round( max( controlSignal * 100.0, 0 ) ) / 100.0;
  }
  // Send header
  Serial.print('#');
  // Send u
  Serial.print('u');
  if(tempU < 10) Serial.print('0');
  Serial.print(tempU,2);
  // Sedn y
  Serial.print('y');
  if(tempY < 10) Serial.print('0');
  Serial.print(tempY,2);
}
void Controller::printToSerialForArduinoDebugging( ControllerInfo *controllerInfo, ADCInfo *adcInfo, bool isUKFromADCSignal ){
  // Print Ref  
  Serial.print( controllerInfo->reference, 2 );
  Serial.print(',');
  // Print Y  
  Serial.print( adcInfo->yKFromADC, 2 );
  Serial.print(',');
  // Print U 
  if( isUKFromADCSignal ){
    Serial.print( adcInfo->uKFromADC, 2 );
  } else{
    Serial.print( controlSignal, 2 );
  } 
  Serial.print(',');
  // Print E
  Serial.print( currentEK, 2 );
  Serial.println();
}