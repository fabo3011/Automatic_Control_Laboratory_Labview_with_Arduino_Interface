/*
  FuzzyController.h - Library to model a Fuzzy Controller with 3 different defuzzification methods
  Created by fgg, November 15, 2017.
  Released into the public domain.
*/
#ifndef FuzzyController_h
#define FuzzyController_h

#include "Arduino.h"
#include "../../LabviewDataHandler/LabviewDataHandler.h"
#include "../../ADCDataHandler/ADCDataHandler.h"

//Struct Declaration
//Point Structure
typedef struct Point Point;
struct Point{
    float x, y;
};
//Line Equation Structure
typedef struct Line Line;
struct Line{
    float a, b, c;
};

class  FuzzyController{
  public:
    FuzzyController();
    float fuzzyControllerResponse(ControllerInfo *controllerInfo, ADCInfo *adcInfo, float *currentEK);
  private:
    float  ccw(const struct Point* a, const struct Point* b, const struct Point* c);
    int    intersect(const struct Point* a, const struct Point* b, const struct Point* c, const struct Point* d);
    struct Point intersection(const Line* l1, const Line* l2);
    void   Polygon_Conv(int* poly_size, int poly1, int poly2);
    void   Polygon(int* poly_size, int poly0);
    void   SetValuesForLineEquationsInMF(ControllerInfo *controllerInfo);
    void   Fuzzify_and_Polyline(ControllerInfo *controllerInfo, float *currentEK);
    float  Singleton_Def();
    float  CoA_Def();

    // --- Auxiliary Private Variables --- //
    float controlSignal;
    // Differential in the control signal obtained by the defuzzifier
    float dU;
    // Epsilon value defined for float type comparisions
    float eps;
    // Auxiliary array that stores the level of membership of the input variable
    float mf_level[3];
    // Auxiliary matrixes to store values of lines
    Line inp_rect[3][3];
    Line outp_rect[3][3];
    // Auxiliary value to store y values present in a trapezoidal shape
    float y_val[4];
    // Auxiliary structure to store x,y points of membership functions at output
    struct Point outp_points[3][4];
    // Auxiliary coordinates that describe the polyline
    struct Point poly[10];
    // Variable that stores the size of points in the polyline
    int idx;
};
#endif