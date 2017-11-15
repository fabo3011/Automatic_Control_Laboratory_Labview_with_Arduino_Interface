/*
  FuzzyController.h - Library to model a Fuzzy Controller with 3 different defuzzification methods
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "FuzzyController.h"

FuzzyController::FuzzyController(){
    eps = 1e-6;
}

void  FuzzyController::Fuzzify_and_Polyline(ControllerInfo *ctrlInfo, float *currentEK){
    //Fuzzification and PolyLine
    //Calculate lvl of membership by each input function using crisp variable,
    for(int mf = 0; mf < 3; ++mf) {
        for(int rect = 0; rect < 3; ++rect) {
            if (fabs(inp_rect[mf][rect].a) > eps) {//a != 0
                if (((x_test-inp_mf[mf][rect]) > -eps) && ((x_test-inp_mf[mf][rect+1]) < eps)) { //x_test is in range of a line
                    mf_level[mf] = (-inp_rect[mf][rect].c-inp_rect[mf][rect].b*x_test)/inp_rect[mf][rect].a;
                }
            }
        }
    }

    if(mf_level[0]>mf_level[2])
        mf_level[2] = 0.0;
    if(mf_level[2]>mf_level[0])
        mf_level[0] = 0.0;
     //Mf output points
     for(int mf = 0; mf < 3; ++mf) {
        outp_points[mf][0].x = outp_mf[mf][0];
        outp_points[mf][3].x = outp_mf[mf][3];
        outp_points[mf][0].y = 0;
        outp_points[mf][1].y = mf_level[mf];
        outp_points[mf][2].y = mf_level[mf];
        outp_points[mf][3].y = 0;

        //Calculation of intersection point in mf given level of mf
        rect = 0;
        if (fabs(outp_rect[mf][rect].b) > eps) { //b != 0
            outp_points[mf][1].x = (-outp_rect[mf][rect].c-outp_rect[mf][rect].a*mf_level[mf])/outp_rect[mf][rect].b;
        }
        else   {
            outp_points[mf][1].x = outp_mf[mf][1];
        }

        rect = 2;
        if (fabs(outp_rect[mf][rect].b) > eps) { //b != 0
            outp_points[mf][2].x = (-outp_rect[mf][rect].c-outp_rect[mf][rect].a*mf_level[mf])/outp_rect[mf][rect].b;
        }
        else   {
            outp_points[mf][2].x = outp_mf[mf][2];
        }
     }

    //Construct polyline of multiple membership functions based on cases
    if(fabs(mf_level[1]) < eps) {//mf 1 == 0
        if(fabs(mf_level[0]) > eps) {//mf 0 != 0
            Polygon(&idx,0);
        }
        else { //mf 2 != 0
            Polygon(&idx,2);
        }
    }
    else{ //mf 1 != 0
        if(fabs(mf_level[0]) > eps) {//mf 0 != 0
            Polygon_Conv(&idx,0,1);
        }
        else {
            if(fabs(mf_level[2]) > eps){ //mf 2 != 0
                Polygon_Conv(&idx,1,2);
            }
            else{ //only mf 1 != 0
                Polygon(&idx,1);
            }
        }
    }
}

float FuzzyController::fuzzyControllerResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo, float *currentEK){
    
    Fuzzify_and_Polyline(currentEK);
    dU = Singleton_Def();
    controlSignal = adcInfo->uKFromADC + dU;
    controlSignal = max(controlSignal,0.0);  // Lower Saturation Limit
    controlSignal = min(5.0,controlSignal);  // Upper Satutaration Limit
    return controlSignal;
}
