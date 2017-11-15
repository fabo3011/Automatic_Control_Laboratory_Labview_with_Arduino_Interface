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

#include <string.h>
#include <math.h>

#define U_SIGNAL 1
#define Y_SIGNAL 0
#define pwm_pin 9
#define SAMPLE_SIGNAL 6

unsigned long t1,t2,T;

// Controller Information Structure to store data recieved by the Labview Interface
ControllerInfo controllerInfo;
// Labview Data Handler Object to manage the information recieved through the serial port by the Labview Interface
LabviewDataHandler labviewDataHandler;

// ADC Information Structure to store data recieved by the analog to digital converters which contain uK and yK as inputs
ADCInfo adcInfo;
// ADC Data Handler Object to manage the information recieved through the ADC converters which contain uK and yK as inputs
ADCDataHandler adcDataHandler = ADCDataHandler(U_SIGNAL,Y_SIGNAL);

// Controller
Controller controller;

void setup() {
  
  // Initialize serial communication with custom baudrate
  labviewDataHandler.setBaudRate(921600);
  // Set the cutoff frequency of the digital filter for yK
  adcDataHandler.setLowPassButterworthFilterCutoffFrequency(5.0);
  
  // Set filter pin as output (PWM)
  pinMode(pwm_pin,OUTPUT); 
  // Set sample singal pin as output
  pinMode(SAMPLE_SIGNAL,OUTPUT);
  /*
  int mf,rect;
  for(mf = 0; mf < 3; ++mf) {
        for(rect = 0; rect < 3; ++rect) {
            //a
            inp_rect[mf][rect].a = inp_mf[mf][rect+1]-inp_mf[mf][rect];
            //b
            inp_rect[mf][rect].b  = y_val[rect]-y_val[rect+1];
            //c
            inp_rect[mf][rect].c  = -inp_rect[mf][rect].a*y_val[rect]-inp_rect[mf][rect].b*inp_mf[mf][rect];
        }
    }
    //printf("OUTPUT\n\n");
    //output
    for(mf = 0; mf < 3; ++mf) {
        for(rect = 0; rect < 3; ++rect) {
            //a
            outp_rect[mf][rect].a = outp_mf[mf][rect+1]-outp_mf[mf][rect];
            //b
            outp_rect[mf][rect].b = y_val[rect]-y_val[rect+1];
            //c
            outp_rect[mf][rect].c = -outp_rect[mf][rect].a*y_val[rect]-outp_rect[mf][rect].b*outp_mf[mf][rect];
        }
    }*/
  
}

void loop() {
  // Set t1 as time before starting program
  t1=micros();
  // Set sample_signal as high (used to obtain period)
  digitalWrite(SAMPLE_SIGNAL,HIGH);
  

/*
  if(millis()>5000){
    
    dataControl = 5;
    tmpRef = 2;
    //tmpRef = (tmpRef+1.0322)/4.5583;
    ref = tmpRef;
  }*/
  

  int frameRecieved = labviewDataHandler.getIncomingFrameFromLabview( &controllerInfo );
  if(frameRecieved){
      labviewDataHandler.setReferenceLinearityRegionTo5V( &controllerInfo, 4.5583, -1.0322, 0.8 );
  }

  adcDataHandler.readUKFromADC( &adcInfo );
  adcDataHandler.readYKFromADC( &adcInfo );
  adcDataHandler.filterYK( &adcInfo );

  controller.calculateControlSignalResponse( &controllerInfo, &adcInfo );


  // Read current output signal
  //u_k = analogRead(U_SIGNAL)*5.0/1024.0;
  //analogWrite(pwm_pin,round((ref/5.0)*255.0));
  
  //Serial.print(lowPassFilter.input(max(((u_k* 4.5583)-1.032),0)),2);
  // Send y(k) to serial and pad 0
  //temp =round(max(abs((y_k* 4.5583)-1.032),0)*100.0)/100.0;
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
  
  t2=micros();
  delayMicroseconds(((unsigned long)(T))*(unsigned long)(1000000)-t2+t1);
   // delayMicroseconds(4800-t2+t1);

  digitalWrite(max(SAMPLE_SIGNAL,0),LOW);
  
}
