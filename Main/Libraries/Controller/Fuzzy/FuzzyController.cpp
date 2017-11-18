/*
  FuzzyController.h - Library to model a Fuzzy Controller with 3 different defuzzification methods
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/

#include "Arduino.h"
#include "FuzzyController.h"

FuzzyController::FuzzyController(){
    eps = 1e-6;
    y_val[0] = 0.0;
    y_val[1] = 1.0;
    y_val[2] = 1.0;
    y_val[3] = 0.0;
}

// Function to Check Counter Clock Wise Turn using cross product
float FuzzyController::ccw(const struct Point* a, const struct Point* b, const struct Point* c){
    return (c->y-a->y)*(b->x-a->x) > (b->y-a->y)*(c->x-a->x);
}
// Function that uses CCW function to test wether 2 line segments intersect (AB with CD)
int FuzzyController::intersect(const struct Point* a, const struct Point* b, const struct Point* c, const struct Point* d){
    return ((ccw(a,c,d)!=ccw(b,c,d))&&(ccw(a,b,c)!=ccw(a,b,d)));
}
// Intersection of 2 rects given a,b,c for one line and e,d,f for other line
struct Point FuzzyController::intersection(const Line* l1, const Line* l2){
    struct Point tmp;
    float det = -1/(l1->a*l2->b-l1->b*l2->a);
    tmp.x = det*(l1->c*l2->b-l1->b*l2->c);
    tmp.y = det*(l2->c*l1->a-l1->c*l2->a);
    return tmp;
}
// Calculates the combined points of 2 mf that intersect each other
void FuzzyController::Polygon_Conv(int* poly_size, int poly1, int poly2){
    *poly_size = 0;
    poly[(*poly_size)++] = outp_points[poly1][0];
    poly[(*poly_size)++] = outp_points[poly1][1];
    // check intersection with rect 1 of poly1 and rect 0 of poly2
    if( intersect( &outp_points[poly1][1], &outp_points[poly1][2], &outp_points[poly2][0], &outp_points[poly2][1] ) ){
        Line tmpLine;
        tmpLine.a = outp_points[poly1][2].x - outp_points[poly1][1].x;
        tmpLine.b = 0;
        tmpLine.c =  - tmpLine.a * outp_points[poly1][1].y;
        poly[(*poly_size)++] = intersection( &tmpLine, &outp_rect[poly2][0] );
        poly[(*poly_size)++] = outp_points[poly2][1];
        poly[(*poly_size)++] = outp_points[poly2][2];
        poly[(*poly_size)++] = outp_points[poly2][3];
    }
    else{
        // check intersection with rect 2 of poly1 and rect 0 of poly2
        if( intersect( &outp_points[poly1][2], &outp_points[poly1][3], &outp_points[poly2][0], &outp_points[poly2][1] ) ){
            poly[(*poly_size)++] = outp_points[poly1][2];
            poly[(*poly_size)++] = intersection( &outp_rect[poly1][2], &outp_rect[poly2][0] );
            poly[(*poly_size)++] = outp_points[poly2][1];
            poly[(*poly_size)++] = outp_points[poly2][2];
            poly[(*poly_size)++] = outp_points[poly2][3];
        }
        else{ // defualt intersection with rect 2 of poly1 and rect 1 of poly2
            poly[(*poly_size)++] = outp_points[poly1][2];
            Line tmpLine;
            tmpLine.a = outp_points[poly2][2].x - outp_points[poly2][1].x;
            tmpLine.b = 0;
            tmpLine.c = - tmpLine.a * outp_points[poly2][1].y;
            poly[(*poly_size)++] = intersection( &outp_rect[poly1][2], &tmpLine );
            poly[(*poly_size)++] = outp_points[poly2][2];
            poly[(*poly_size)++] = outp_points[poly2][3];
        }
    }
}
//Extract Points of a single Polygon
void FuzzyController::Polygon(int* poly_size, int poly0){
    *poly_size = 4;
    for(int i = 0; i < *poly_size; ++i){
        poly[i] = outp_points[poly0][i];
    }
}
// Use mebership functions vertes coordinates to define the equations of lines for each MF in the form ay+bx+c=0
void  FuzzyController::SetValuesForLineEquationsInMF(ControllerInfo *ctrlInfo){
    // Input
    for(int mf = 0; mf < 3; ++mf) {
        for(int rect = 0; rect < 3; ++rect) {
            // a
            inp_rect[mf][rect].a = ctrlInfo->inputMFDescriptor[mf][rect+1] - ctrlInfo->inputMFDescriptor[mf][rect];
            // b
            inp_rect[mf][rect].b  = y_val[rect] - y_val[rect+1];
            // c
            inp_rect[mf][rect].c  = - inp_rect[mf][rect].a * y_val[rect] - inp_rect[mf][rect].b * ctrlInfo->inputMFDescriptor[mf][rect];
        }
    }
    // Output
    for(int mf = 0; mf < 3; ++mf) {
        for(int rect = 0; rect < 3; ++rect) {
            // a
            outp_rect[mf][rect].a = ctrlInfo->outputMFDescriptor[mf][rect+1] - ctrlInfo->outputMFDescriptor[mf][rect];
            // b
            outp_rect[mf][rect].b = y_val[rect] - y_val[rect+1];
            // c
            outp_rect[mf][rect].c = - outp_rect[mf][rect].a * y_val[rect] - outp_rect[mf][rect].b * ctrlInfo->outputMFDescriptor[mf][rect];
        }
    }

}
void  FuzzyController::Fuzzify_and_Polyline(ControllerInfo *ctrlInfo, float *currentEK){
    // Fuzzification and PolyLine
    // Calculate lvl of membership by each input function using crisp variable,
    for(int mf = 0; mf < 3; ++mf) {
        for(int rect = 0; rect < 3; ++rect) {
            if ( fabs( inp_rect[mf][rect].a ) > eps ) {//a != 0
                if ( ( ( *currentEK - ctrlInfo->inputMFDescriptor[mf][rect] ) > -eps) && ( ( *currentEK - ctrlInfo->inputMFDescriptor[mf][rect+1] ) < eps ) ) { // currentEK is in range of a line
                    mf_level[mf] = ( - inp_rect[mf][rect].c - inp_rect[mf][rect].b * *currentEK ) / inp_rect[mf][rect].a;
                }
            }
        }
    }
    // Ifs to ensure that one of the extreme membership function is set to 0
    if( mf_level[0] > mf_level[2] )
        mf_level[2] = 0.0;
    if( mf_level[2] > mf_level[0] )
        mf_level[0] = 0.0;
     // Mf output points
     for(int mf = 0; mf < 3; ++mf) {
        outp_points[mf][0].x = ctrlInfo->outputMFDescriptor[mf][0];
        outp_points[mf][3].x = ctrlInfo->outputMFDescriptor[mf][3];
        outp_points[mf][0].y = y_val[0];
        outp_points[mf][1].y = mf_level[mf];
        outp_points[mf][2].y = mf_level[mf];
        outp_points[mf][3].y = y_val[3];

        // Calculation of intersection point in mf given level of mf
        int rect = 0;
        if ( fabs( outp_rect[mf][rect].b ) > eps ) { // b != 0
            outp_points[mf][1].x = ( - outp_rect[mf][rect].c - outp_rect[mf][rect].a * mf_level[mf] ) / outp_rect[mf][rect].b;
        }
        else   {
            outp_points[mf][1].x = ctrlInfo->outputMFDescriptor[mf][1];
        }

        rect = 2;
        if ( fabs( outp_rect[mf][rect].b ) > eps ) { // b != 0
            outp_points[mf][2].x = ( - outp_rect[mf][rect].c - outp_rect[mf][rect].a * mf_level[mf] ) / outp_rect[mf][rect].b;
        }
        else   {
            outp_points[mf][2].x = ctrlInfo->outputMFDescriptor[mf][2];
        }
     }
    // Construct polyline of multiple membership functions based on cases
    if( fabs( mf_level[1] ) < eps ) { // mf 1 == 0
        if( fabs( mf_level[0] ) > eps ) { // mf 0 != 0
            Polygon(&idx,0);
        }
        else { // mf 2 != 0
            Polygon(&idx,2);
        }
    }
    else{ // mf 1 != 0
        if( fabs( mf_level[0] ) > eps ) { // mf 0 != 0
            Polygon_Conv(&idx,0,1);
        }
        else {
            if( fabs( mf_level[2] ) > eps ){ // mf 2 != 0
                Polygon_Conv(&idx,1,2);
            }
            else{ // only mf 1 != 0
                Polygon(&idx,1);
            }
        }
    }
}
// Defuzzify by Singleton
float FuzzyController::Singleton_Def(){
   // Singleton Method
   float singleton = 0;
   float sumy = 0;
   for(int i = 0; i < idx-1; ++i){
       singleton += poly[i].x*poly[i].y;
       sumy += poly[i].y;
   }
   return singleton/sumy;
}
// Defuzzify by Center of Area
float FuzzyController::CoA_Def(){
    //Center of Area
    //variables for area and centroid
    int i;
    float area;
    float centroid;
    area = centroid = 0;
    for(i = 0; i < idx-1; ++i){
        area += poly[i].x*poly[i+1].y-poly[i+1].x*poly[i].y;
        centroid += (poly[i].x*poly[i+1].y-poly[i+1].x*poly[i].y)*(poly[i].x+poly[i+1].x);
    }
    area /= 2.;
    centroid /= 6*area;
    //printf("area: %.6f\n",area);
    //printf("centroid: %.6f\n",centroid);
    return centroid/area;
}
// Get Response for the Fuzzy Controller
float FuzzyController::fuzzyControllerResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo, float *currentEK){
    SetValuesForLineEquationsInMF(controllerInfo);
    Fuzzify_and_Polyline(controllerInfo, currentEK);
    dU = Singleton_Def();
    controlSignal = adcInfo->uKFromADC + dU;
    controlSignal = max(controlSignal,0.0);  // Lower Saturation Limit
    controlSignal = min(5.0,controlSignal);  // Upper Satutaration Limit
    return controlSignal;
}
