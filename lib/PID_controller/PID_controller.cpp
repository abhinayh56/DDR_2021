#include "PID_controller.h"

PID_controller::PID_controller(){
  lpf_alpha = 0;
  lpf_filter.set_alpha_LPF(lpf_alpha);
  initialize();
}

void PID_controller::set_gains(float _Kp, float _Ki, float _Kd){
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

void PID_controller::set_Kp(float _Kp){
  Kp = _Kp;
}

float PID_controller::get_Kp(){
  return Kp;
}

void PID_controller::set_Ki(float _Ki){
  Ki = _Ki;
}

float PID_controller::get_Ki(){
  return Ki;
}

void PID_controller::set_Kd(float _Kd){
  Kd = _Kd;
}

float PID_controller::get_Kd(){
  return Kd;
}

void PID_controller::set_I_max(float _I_max){
  I_max = _I_max;
}

float PID_controller::get_I_max(){
  return I_max;
}

void PID_controller::set_output_max(float _pid_max){
  pid_max = _pid_max;
}

float PID_controller::get_output_max(){
  return pid_max;
}

void PID_controller::set_lpf_alpha(float _lpf_alpha){
  lpf_alpha = _lpf_alpha;
  lpf_filter.set_alpha_LPF(_lpf_alpha);
}

float PID_controller::get_lpf_alpha(){
  return lpf_alpha;
}

void PID_controller::reset(){
  de = 0;
  e_pre = 0;
  x_pre = 0;
  P = 0;
  I = 0;
  D = 0;
  pid = 0;
}

void PID_controller::initialize(){
  de = 0;
  e_pre = 0;
  x_pre = 0;
  P = 0;
  I = 0;
  D = 0;
  pid = 0;
}

float PID_controller::get_P(){
  return P;
}

float PID_controller::get_I(){
  return I;
}

float PID_controller::get_D(){
  return D;
}

float PID_controller::calculate_output1(float x_des, float x, float dt){
  // pid in parallel form
  // integral saturation
  // output saturation
  float e = x_des - x;
  
  P = Kp*e;

  I = I + Ki*e*dt;
  I = math_fun.saturate(I, -I_max, I_max);

  de = (e-e_pre)/dt;
  e_pre = e;
  D = Kd*de;

  pid = P + I + D;
  pid = math_fun.saturate(pid, -pid_max, pid_max);
  
  return pid;
}

float PID_controller::calculate_output2(float x_des, float x, float dt){
  // pid in parallel form
  // integral saturation
  // output saturation
  // derivative filter (both input and output rate)
  float e = x_des - x;

  P = Kp*e;
  
  I = I + Ki*e*dt;
  I = math_fun.saturate(I, -I_max, I_max);

  e = lpf_filter.apply_LPF(e);
  de = (e-e_pre)/dt;
  e_pre = e;
  D = Kd*de;

  pid = P + I + D;

  pid = math_fun.saturate(pid, -pid_max, pid_max);
  
  return pid;
}

float PID_controller::calculate_output3(float x_des, float x, float dt){
  // pid in parallel form
  // with integral
  // output saturation
  // derivative filter of angular acceleration
  // zero setpoint acceleration
  float e = x_des - x;
  
  P = Kp*e;
  
  I = I + Ki*e*dt;
  I = math_fun.saturate(I, -I_max, I_max);

  x = lpf_filter.apply_LPF(x);
  float dx = (x-x_pre)/dt;
  x_pre = x;
  D = Kd*(0 - dx);

  pid = P + I + D;
  pid = math_fun.saturate(pid, -pid_max, pid_max);
  
  return pid;
}
