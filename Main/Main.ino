#include "Libraries/Synchronizer/Synchronizer.h"
#include "Libraries/Synchronizer/Synchronizer.cpp"
#include "Libraries/LabviewDataHandler/LabviewDataHandler.h"
#include "Libraries/LabviewDataHandler/LabviewDataHandler.cpp"
#include "Libraries/ADCDataHandler/ADCDataHandler.h"
#include "Libraries/ADCDataHandler/ADCDataHandler.cpp"
#include "Libraries/Controller/Controller.h"
#include "Libraries/Controller/Controller.cpp"
#include "Libraries/Controller/OnOff/OnOffController.h"
#include "Libraries/Controller/OnOff/OnOffController.cpp"
#include "Libraries/Controller/P/PController.h"
#include "Libraries/Controller/P/PController.cpp"
#include "Libraries/Controller/PI/PIController.h"
#include "Libraries/Controller/PI/PIController.cpp"
#include "Libraries/Controller/Fuzzy/FuzzyController.h"
#include "Libraries/Controller/Fuzzy/FuzzyController.cpp"

#define U_SIGNAL 1
#define Y_SIGNAL 0
#define pwm_pin 9
#define SAMPLE_SIGNAL 6
#define ADC_U_SIGNAL true

// Controller Information Structure to store data recieved by the Labview Interface
ControllerInfo controllerInfo;
// Labview Data Handler Object to manage the information recieved through the serial port by the Labview Interface
LabviewDataHandler labviewDataHandler;

// ADC Information Structure to store data recieved by the analog to digital converters which contain uK and yK as inputs
ADCInfo adcInfo;
// ADC Data Handler Object to manage the information recieved through the ADC converters which contain uK and yK as inputs
ADCDataHandler adcDataHandler = ADCDataHandler( U_SIGNAL, Y_SIGNAL );

// Controller
Controller controller;

// Synchronizer Object to sync the period of the Main program to match the Controller's sampling rate
Synchronizer sync;

void setup() {
  // Initialize serial communication with custom baudrate
  labviewDataHandler.setBaudRate( 921600 );
  // Set the cutoff frequency of the digital filter for yK
  adcDataHandler.setLowPassButterworthFilterCutoffFrequency( 5.0 );
  // Set filter pin as output (PWM)
  controller.setControlSignalResponsePWMPinAsOutput( pwm_pin );
  // Set sample singal pin as output
  sync.setSamplingSignalPinAsOutput( SAMPLE_SIGNAL );
}

void loop() {
  // Set t1 as time before starting program
  sync.assignTimestampInMicrosToT1();
  // Set sample_signal as high (used to obtain period)
  sync.setSamplingSignalPinToHIGH( SAMPLE_SIGNAL );
  
  // Get data from Serial
  int frameRecieved = labviewDataHandler.getIncomingFrameFromLabview( &controllerInfo );
  if(frameRecieved){
      // Transform reference to a 0-5V range (for first order system)
      labviewDataHandler.setReferenceLinearityRegionTo5V( &controllerInfo, 4.5583, -1.0322, 0.8 );
  }

  // Read ADC U_k, Y_k and Filter Y_k
  adcDataHandler.readUKFromADC( &adcInfo );
  adcDataHandler.readYKFromADC( &adcInfo );
  adcDataHandler.filterYK( &adcInfo );

  // Calculate U_k and write it to the PWM pin
  controller.calculateControlSignalResponse( &controllerInfo, &adcInfo );
  controller.writeControlSignalResponseToPWMPin( pwm_pin );

  // Transofrm Y_k and U_k to a 0-20V range (for first order system)
  controller.retrieveLinearityRegionForYKAndUK( &adcInfo, 4.5583, -1.0322, 20.0, ADC_U_SIGNAL ); 
  
  float temp,temp2;
  float y_k,u_k;
  temp =round(max(abs(y_k)*100.0,0))/100.0;
  Serial.print('#');

  // Send u(k) to serial and pad 0
  Serial.print('u');
  temp2 =round(max(abs(u_k)*100.0,0))/100.0;
  //temp2 =round(max(abs((u_k* 4.5583)-1.032),0)*100.0)/100.0;
  if(temp2 < 10){
    Serial.print('0');
  }
  Serial.print(temp2,2);
  
  Serial.print('y');
  if(temp < 10){
    Serial.print('0');
  }
  Serial.print(temp,2);
  /*
  temp =max(ref,0);
  Serial.print(temp,2);
  Serial.print(',');
  temp =max(y_k,0);
  Serial.print(temp,2);
  Serial.print(',');
  temp =max(u_k,0);
  Serial.println(temp,2);
  //Serial.println();*/

  // Set t2 as time before starting program
  sync.assignTimestampInMicrosToT2();
  // Implements delayMicroseconds to wait until the next sampling period
  sync.waitUntilNextSamplingPeriod( &controllerInfo );
  // Set sample_signal as low (used to obtain period)
  sync.setSamplingSignalPinToLOW( SAMPLE_SIGNAL );
}
