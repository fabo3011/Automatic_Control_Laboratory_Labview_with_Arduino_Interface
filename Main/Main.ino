#include <Filters.h>
#include <string.h>
#include <math.h>
#define Y_SIGNAL A0
#define U_SIGNAL A1
#define pwm_pin 9
#define SAMPLE_SIGNAL 6



// -- Serial --
/*  data_H:       Header variable for validating serial protocol
 *  dataBuff:     Saves the values of each constant sent through the interface
 *  dataControl:  Saves type of controller
 *  temp:         used to pad floats when sending to serial
 */
char data_H;
float dataBuff[12];
float dataControl;
float temp, temp2;
float outCentroids[3];


// -- Filter --
/*
 * filterFrequency:   Sets the cutOff Frequency 
 * lowPassFilter:     Filter object for Y(k) and u(k)
 */
float filterFrequency = 5.0;
FilterTwoPole lowPassFilter;

// -- Control Signals --
/*
 * y_k:       Output signal
 * u_k:       Control signal
 * ref:       Calculated reference value
 * tmpRef:    Temporal value for calculating the reference
 * error:     ref - y_k
 */
float y_k, u_k;
float ref = 0.0;
float tmpRef;
float error;

//  Hystheresys Controller
/*
 * percentage:    window percentage value
 */
float percentage;

// P controller
/*
 * kP:  proportional constant value
 */
float kP = 4.3;

// PI Controller
/*
 * Kp:        P constant value
 * Ki:        I constant value
 * F:         Floating point conversion factor
 * u_kant:    uk previous value
 * errorant:  error previous value
 * eact:      current error
 * uact:      current u
 * T:         Sampling period
 * t1:        measures start of program loop
 * t2:        measures end of program loop
 * a,b,c:     PI controller approximation values
 */
float Kp =  10;
float Ki =  0.05;
float F=100000;
float u_kant=0, errorant=0, eact, uact;
float T=0.13;
unsigned long t1,t2;
float a,b,c;  

// Fuzzy controller
float uant;
int mf,rect;
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

//Function to Check Counter Clock Wise Turn using cross product
float ccw(const struct Point* a, const struct Point* b, const struct Point* c){
    return (c->y-a->y)*(b->x-a->x) > (b->y-a->y)*(c->x-a->x);
}
//Function that uses CCW function to test wether 2 line segments intersect AB with CD
int intersect(const struct Point* a, const struct Point* b, const struct Point* c, const struct Point* d){
    return ((ccw(a,c,d)!=ccw(b,c,d))&&(ccw(a,b,c)!=ccw(a,b,d)));
}
//Intersection of 2 rects given a,b,c for one line and e,d,f for other line
struct Point intersection(const Line* l1, const Line* l2){
    struct Point tmp;
    float det = -1/(l1->a*l2->b-l1->b*l2->a);
    tmp.x = det*(l1->c*l2->b-l1->b*l2->c);
    tmp.y = det*(l2->c*l1->a-l1->c*l2->a);
    return tmp;
}


//membership coordinates (num of shape, coordinate at point j)
//input and output

// OnOff
//float inp_mf[3][4] = {{-5, -5, -5, 0},{-5, 0, 0, 5},{0, 5, 5, 5}};
//float outp_mf[3][4] = {{-1, -1, -0.6, 0},{-0.01, 0, 0, 0.01},{0, 0.6, 1, 1}};

float inp_mf[3][4] =  {{-5, -5, -1, 0},{-5, 0, 0, 5},{0, 1, 5, 5}};
float outp_mf[3][4] = {{-0.01, -0.01, -0.008, 0},{-0.001, 0, 0, 0.001},{0, 0.008, 0.01, 0.01}};

//// OnOff Candidate
//float inp_mf[3][4] = {{-5, -5, -2, 0},{-0.15, 0, 0, 0.15},{0, 2, 5, 5}};
//float outp_mf[3][4] = {{-1, -0.02, -0.016, 0},{-0.004, 0, 0, 0.004},{0, 0.016, 0.02, 1}};

// Slow
// float inp_mf[3][4] = {{-5, -5, -5, 0},{-5, 0, 0, 5},{0, 5, 5, 5}};
// float outp_mf[3][4] = {{-0.01, -0.01, -0.008, 0},{-0.001, 0, 0, 0.001},{0, 0.008, 0.01, 0.01}};

// Slow
//float inp_mf[3][4] = {{-5, -5, -1, 0},{-5, 0, 0, 5},{0, 1, 5, 5}};
//float outp_mf[3][4] = {{-0.001, -0.001, -0.001, 0},{-0.001, 0, 0, 0.001},{0, 0.001, 0.001, 0.001}};

// Slow
//float inp_mf[3][4] = {{-5, -5, -5, 0},{-5, 0, 0, 5},{0, 5, 5, 5}};
//float outp_mf[3][4] = {{-0.002, -0.002, -0.0012, 0},{-0.0002, 0, 0, 0.0002},{0, 0.0012, 0.002, 0.002}};


// Overshoot 30%
//float inp_mf[3][4] = {{-5, -5, -5, 0},{-5, 0, 0, 5},{0, 5, 5, 5}};
//float outp_mf[3][4] = {{-0.2, -0.2, -0.12, 0},{-0.002, 0, 0, 0.002},{0, 0.12, 0.2, 0.2}};

// Error
// float inp_mf[3][4] = {{-5, -5, -4.75, 0.25},{-1.75, 0.25, 0.25, 2.25},{0.25, 5, 5, 5}};
// float outp_mf[3][4] = {{-0.01, -0.01, -0.008, 0},{-0.001, 0, 0, 0.001},{0, 0.008, 0.01, 0.01}};

// Fast
//float inp_mf[3][4] = {{-5, -5, -3, 2},{-5, 2, 2, 5},{2, 5, 5, 5}};
//float outp_mf[3][4] = {{-0.01, -0.01, -0.008, 0},{-0.001, 0, 0, 0.001},{0, 0.008, 0.01, 0.01}};


//epsilon value
float eps = 1e-6;

//auxiliar matrixes to store values of lines
Line inp_rect[3][3];
Line outp_rect[3][3];

//auxiliar value to store y values in a trapezoidal shape
float y_val[4] = {0, 1, 1, 0};

//auxiliar array with the level of membership at input
float mf_level[3];

//auxiliar to store x,y points of membership functions at output
struct Point outp_points[3][4];

//auxiliar coordinates for polyline
struct Point poly[10];
//variable that stores the size of points in the polyline
int idx;

//Calculates the combined points of 2 mf that intersect each other
void Polygon_Conv(int* poly_size, int poly1, int poly2){
    *poly_size = 0;
    poly[(*poly_size)++] = outp_points[poly1][0];
    poly[(*poly_size)++] = outp_points[poly1][1];
    //check intersection with rect 1 of poly1 and rect 0 of poly2
    if(intersect(&outp_points[poly1][1],&outp_points[poly1][2],&outp_points[poly2][0],&outp_points[poly2][1])){
        Line tmpLine;
        tmpLine.a = outp_points[poly1][2].x-outp_points[poly1][1].x;
        tmpLine.b = 0;
        tmpLine.c = -tmpLine.a*outp_points[poly1][1].y;
        poly[(*poly_size)++] = intersection(&tmpLine,&outp_rect[poly2][0]);
        poly[(*poly_size)++] = outp_points[poly2][1];
        poly[(*poly_size)++] = outp_points[poly2][2];
        poly[(*poly_size)++] = outp_points[poly2][3];
    }
    else{
        //check intersection with rect 2 of poly1 and rect 0 of poly2
        if(intersect(&outp_points[poly1][2],&outp_points[poly1][3],&outp_points[poly2][0],&outp_points[poly2][1])){
            poly[(*poly_size)++] = outp_points[poly1][2];
            poly[(*poly_size)++] = intersection(&outp_rect[poly1][2],&outp_rect[poly2][0]);
            poly[(*poly_size)++] = outp_points[poly2][1];
            poly[(*poly_size)++] = outp_points[poly2][2];
            poly[(*poly_size)++] = outp_points[poly2][3];
        }
        else{ //defualt intersection with rect 2 of poly1 and rect 1 of poly2
            poly[(*poly_size)++] = outp_points[poly1][2];
            Line tmpLine;
            tmpLine.a = outp_points[poly2][2].x-outp_points[poly2][1].x;
            tmpLine.b = 0;
            tmpLine.c = -tmpLine.a*outp_points[poly2][1].y;
            poly[(*poly_size)++] = intersection(&outp_rect[poly1][2],&tmpLine);
            poly[(*poly_size)++] = outp_points[poly2][2];
            poly[(*poly_size)++] = outp_points[poly2][3];
        }
    }
}
//Extract Points of a single Polygon
void Polygon(int* poly_size, int poly0){
    *poly_size = 4; int i;
    for(i = 0; i < *poly_size; ++i){
        poly[i] = outp_points[poly0][i];
    }
}

void Fuzzify_and_Polyline(float x_test){

    int mf,rect;
    //Fuzzification and PolyLine
    //Calculate lvl of membership by each input function using crisp variable,
     for(mf = 0; mf < 3; ++mf) {
        for(rect = 0; rect < 3; ++rect) {
            if (fabs(inp_rect[mf][rect].a) > eps) {//a != 0
                if (((x_test-inp_mf[mf][rect]) > -eps) && ((x_test-inp_mf[mf][rect+1]) < eps)) { //x_test is in range of a line
                    mf_level[mf] = (-inp_rect[mf][rect].c-inp_rect[mf][rect].b*x_test)/inp_rect[mf][rect].a;
                }
            }
        }
     }
    /*printf("Membership functions: \n");
    for(mf = 0; mf < 3; ++mf){
        printf("%.6f ",mf_level[mf]);
    }printf("\n");*/

    if(mf_level[0]>mf_level[2])
        mf_level[2] = 0.0;
    if(mf_level[2]>mf_level[0])
        mf_level[0] = 0.0;
     //Mf output points
     for(mf = 0; mf < 3; ++mf) {
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
    /*
    int i;
    printf("Size of polyline: %d\n",idx);
    printf("Polyline\n");
    for(i = 0; i < idx; ++i){
        printf("Point: %.6f %.6f\n",poly[i].x,poly[i].y);
    }*/
}

//Defuzzify by Singleton
float Singleton_Def(){
     //Singleton Method
    int i;
    float singleton = 0;
    float sumy = 0;
    for(i = 0; i < idx-1; ++i){
        singleton += poly[i].x*poly[i].y;
        sumy += poly[i].y;
    }
    return singleton/sumy;
}

//Defuzzify by Center of Area
float CoA_Def(){
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




void setup() {
  /*//P values for WA
  P[0] = 0.2*5;
  P[1] = 0.5*5;
  P[2] = 0.8*5;

  //temporal values for fuzzy
  inp[0] = -0.25;
  inp[1] = 0;
  inp[2] = 0;
  inp[3] = 0;
  inp[4] = 0.25;*/
  uant = 0;
  // Initialize serial communication with custom baudrate
  Serial.begin(921600);
  // Set filter pin as output (PWM)
  pinMode(pwm_pin,OUTPUT); 
  // Configure filter type for input signal
  lowPassFilter.setAsFilter(LOWPASS_BUTTERWORTH,filterFrequency); // Filtro digital de la entrada
  // Set sample singal pin as output
  pinMode(SAMPLE_SIGNAL,OUTPUT);
  
  
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
    }
  
}

void loop() {
  // Set t1 as time before starting program
  t1=micros();
  // Set sample_signal as high (used to obtain period)
  digitalWrite(SAMPLE_SIGNAL,HIGH);
  
  // -- Serial communication --
  // Example frames: 
  /*
   *    OnOff Ref 15                :  #1,15,
   *    OnOff Ref 10                :  #1,10,
   *    Hyste R: 10, W: 50          :  #2,10,50,
   *    Hyste R: 10, W: 10          :  #2,10,10,
   *    P     R: 10, kP:5           :  #3,10,5,
   *    P     R: 15, kP:8           :  #3,15,8,
   *    PI R: 15, kP:0.5, Ki: 3.3   :  #4,15,0.5,3.3,
   */
   
  /*
   * 1. If data is available at serial port, read
   * 2. If byte read is the header (#), read in order:
   *        Controller
   *        Reference
   *        Window  / Kp  
   *        Ki
   *        ...(Fuzzy)...
   */
/*
  if(millis()>5000){
    
    dataControl = 5;
    tmpRef = 2;
    //tmpRef = (tmpRef+1.0322)/4.5583;
    ref = tmpRef;
  }*/
  if(Serial.available() > 0){
    data_H =Serial.read();
    // If header found
    if(data_H == '#'){
      // Read Controller type
      dataControl = Serial.parseFloat();
      data_H = Serial.read();
      
      // Read controller configs
      switch((int)dataControl){
        case 1:     // On/Off
          tmpRef      = Serial.parseFloat();
          break;
        case 2:     // On/Off Hyst
          tmpRef      = Serial.parseFloat();
          data_H      = Serial.read();
          percentage  = Serial.parseFloat();
          break;
        case 3:      // P Controller
          tmpRef      = Serial.parseFloat();
          data_H      = Serial.read();
          kP          = Serial.parseFloat();
          break;
        case 4:     // PI Controller
          tmpRef  = Serial.parseFloat();
          data_H  = Serial.read();
          Kp      = Serial.parseFloat();
          data_H  = Serial.read();
          Ki      = Serial.parseFloat();
          break;
        case 5:     // Fuzzy Controller
        
        case 6:     // Open loop
          tmpRef =Serial.parseFloat();
          break;
      }
      
      // Convert reference to 5v logic
      //tmpRef = dataBuff[0];
     // tmpRef = (tmpRef+1.0322)/4.5583;
      ref = tmpRef;
     }
  }

  // Read convert and filter output signal
  y_k = analogRead(Y_SIGNAL)*5.0/1024.0;
  y_k = lowPassFilter.input(y_k);
  
  // Execute the selected controller
  switch((int)dataControl){
    case 1:       // OnOff
      error = ref-y_k;
      //eact = error;
      if(error > 0){
          analogWrite(pwm_pin,255);
          u_k = 5;
      
      }else if(error < 0){
          analogWrite(pwm_pin,0); 
          u_k = 0;
      }
          
      break;
    case 2:       // OnOff Hyst
      error = ref-y_k;
      eact = error;
      if(error > ref * (percentage/100.0)){        //Upper Limit Hysteresis
         analogWrite(pwm_pin,255);
         u_k = 5;
      }else if (error > -ref *(percentage/100.0)){  // Limit Hysteresis
         analogWrite(pwm_pin,0); //Lower Limit
         u_k = 0;
      }
          
      break;
    case 3:       // P Controller
      eact = (ref-y_k);
      error = eact * kP;
      if(error >= 5){                              //Upper Satutaration Limit
          analogWrite(pwm_pin,255);
      }else if(error <= 0){                         //Lower Saturation Limit
          analogWrite(pwm_pin,0);
      }else{
        analogWrite(pwm_pin,round((error/5.0)*255.0));
      }
      u_k = analogRead(U_SIGNAL)*5.0/1024.0;
      break;
    case 4:       // PI Controller
      a=1*F;
      b=(Kp+((Ki*T)/2))*F;
      c=(((Ki*T)/2)-Kp)*F;
      eact=ref-y_k;
      uact =((a*u_kant) + (b*eact) + (c*errorant))/F;
      uact = uact ;
      uact=min(uact,5.0);
      uact=max(uact,0.0);
      analogWrite(pwm_pin,round((uact/5.0)*255.0));
      u_k = analogRead(U_SIGNAL)*5.0/1024.0;
      u_kant=uact;
      errorant=eact;
     break;
    case 5:         // Fuzzy Controller
      
    
      error = (ref-y_k);
      eact = error;
      
      Fuzzify_and_Polyline(eact);
      uact = Singleton_Def();
      uant = uant + uact;
      uant=min(uant,5.0);
      uant=max(uant,0.0);
//      Serial.print(eact);
//      Serial.print(",");
//      Serial.println(uact);
      analogWrite(pwm_pin,round((uant/5.0)*255.0));
      //u_k = analogRead(U_SIGNAL)*5.0/1024.0;
      u_k = uant;
      
      break;
    case 6:
      analogWrite(pwm_pin,round((ref/5.0)*255.0));
      u_k = analogRead(U_SIGNAL)*5.0/1024.0;
      break;
    default:
      break;
  }
  // Read current output signal
  //u_k = analogRead(U_SIGNAL)*5.0/1024.0;
  
  //Serial.print(lowPassFilter.input(max(((u_k* 4.5583)-1.032),0)),2);
  // Send y(k) to serial and pad 0
  //temp =round(max(abs((y_k* 4.5583)-1.032),0)*100.0)/100.0;
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
