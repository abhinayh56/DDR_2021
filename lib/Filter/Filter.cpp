#include "Filter.h"

Filter_LP::Filter_LP(){
  
}

float Filter_LP::apply_LPF(float x){
  float y = alpha_LPF*y_prev + (1-alpha_LPF)*x;
  y_prev = y;
  return y;
}

void Filter_LP::set_alpha_LPF(float _alpha_LPF){
  alpha_LPF = _alpha_LPF;
}

float Filter_LP::get_alpha_LPF(){
  return alpha_LPF;
}

//////////////////////////////////////////////////////////////

Filter_HP::Filter_HP(){
  
}

float Filter_HP::apply_HPF(float x){
  float y = alpha_HPF*y_prev + alpha_HPF*(x - x_prev);
  x_prev = x;
  y_prev = y;
  return y;
}

void Filter_HP::set_alpha_HPF(float _alpha_HPF){
  alpha_HPF = _alpha_HPF;
}

float Filter_HP::get_alpha_HPF(){
  return alpha_HPF;
}
