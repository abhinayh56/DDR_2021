#include "Math_functions.h"

float Math_functions::saturate(float x, float x_min, float x_max){
  if(x<x_min){
    return x_min;
  }
  else if(x>x_max){
    return x_max;
  }
  else{
    return x;
  }
}

float Math_functions::mod(float x){
  if(x<0){
    return -x;
  }
  else{
    return x;
  }
}

float Math_functions::rad_to_deg(float x){
  return x*57.295779513082320876798154814105; //x*180/PI
}

float Math_functions::deg_to_rad(float x){
  return x*0.01745329251994329576923690768489; //x*PI/180
}

float Math_functions::wrap(float x, float a, float b){
  if(x<=a){
    return x - a + b;
  }
  else if(x>b){
    return x + a - b;
  }
  else{
    return x;
  }
}

float Math_functions::apply_deadband(float x, float deadband){
  if(mod(x)<deadband*0.5){
    return 0;
  }
  else{
    return x;
  }
}

float Math_functions::apply_deadband2(float x, float deadband){
  if(mod(x)<=deadband*0.5){
    return 0;
  }
  else{
    if(x>deadband*0.5){
      return x - deadband*0.5;
    }
    else{
      return x + deadband*0.5;
    }
  }
}

float Math_functions::linear_map(float x, float x1, float x2, float y1, float y2){
  return y1 + ((y2-y1)/(x2-x1))*(x-x1);
}

float Math_functions::poly_map_deg1(float x, float a0, float a1){
  return a0 + a1*x;
}

float Math_functions::poly_map_deg2(float x, float a0, float a1, float a2){
  return a0 + a1*x + a2*x*x;
}

float Math_functions::poly_map_deg3(float x, float a0, float a1, float a2, float a3){
  return a0 + a1*x + a2*x*x + a3*x*x*x;
}

float Math_functions::poly_map_deg4(float x, float a0, float a1, float a2, float a3, float a4){
  return a0 + a1*x + a2*x*x + a3*x*x*x + a4*x*x*x*x;
}

float Math_functions::poly_map_deg5(float x, float a0, float a1, float a2, float a3, float a4, float a5){
  return a0 + a1*x + a2*x*x + a3*x*x*x + a4*x*x*x*x + a4*x*x*x*x*x;
}

float Math_functions::poly_map_deg6(float x, float a0, float a1, float a2, float a3, float a4, float a5, float a6){
  return a0 + a1*x + a2*x*x + a3*x*x*x + a4*x*x*x*x + a4*x*x*x*x*x + a4*x*x*x*x*x*x;
}

float Math_functions::poly_map_degn(float x, float a[], int n){
  if(n>=1){
    return 0;//poly_map_degn(float x, float a[], int n-1);
  }
  else{
    return a[0];
  }
}
