#include "Estimate_angle.h"

Estimate_angle::Estimate_angle(){
  filt_ac_x.set_alpha_LPF(0.775);
  filt_ac_y.set_alpha_LPF(0.775);
  filt_ac_z.set_alpha_LPF(0.775);
  filt_mg_x.set_alpha_LPF(0.6);
  filt_mg_y.set_alpha_LPF(0.6);
  filt_mg_z.set_alpha_LPF(0.6);
}

void Estimate_angle::calibrate_gyro(int16_t GyX, int16_t GyY, int16_t GyZ){
  gy_out_to_phy_signal(GyX, GyY, GyZ);
  float alpha = 0.995;
  b_gy_x = alpha*b_gy_x + (1-alpha)*w_b_sen_x;
  b_gy_y = alpha*b_gy_y + (1-alpha)*w_b_sen_y;
  b_gy_z = alpha*b_gy_z + (1-alpha)*w_b_sen_z;
}

void Estimate_angle::calibrate_gyro2(int16_t GyX, int16_t GyY, int16_t GyZ){
  gy_out_to_phy_signal(GyX, GyY, GyZ);
  b_gy_x += w_b_sen_x;
  b_gy_y += w_b_sen_y;
  b_gy_z += w_b_sen_z;
}

float Estimate_angle::get_bias_gy_x(){
  return b_gy_x;
}

float Estimate_angle::get_bias_gy_y(){
  return b_gy_y;
}

float Estimate_angle::get_bias_gy_z(){
  return b_gy_z;
}

void Estimate_angle::get_bias_gy_xyz(float* _b_gy_x, float* _b_gy_y, float* _b_gy_z){
  *_b_gy_x = b_gy_x;
  *_b_gy_y = b_gy_y;
  *_b_gy_z = b_gy_z;
}

void Estimate_angle::set_bias_gy_x(float _b_gy_x){
  b_gy_x = _b_gy_x;
}

void Estimate_angle::set_bias_gy_y(float _b_gy_y){
  b_gy_y = _b_gy_y;
}

void Estimate_angle::set_bias_gy_z(float _b_gy_z){
  b_gy_z = _b_gy_z;
}

void Estimate_angle::set_bias_gy_xyz(float _b_gy_x, float _b_gy_y, float _b_gy_z){
  b_gy_x = _b_gy_x;
  b_gy_y = _b_gy_y;
  b_gy_z = _b_gy_z;
}

void Estimate_angle::set_fusion_alpha_gy_ac_cf(float _alpha_1){
  alpha_1 = _alpha_1;
}

float Estimate_angle::get_fusion_alpha_gy_ac_cf(){
  return alpha_1;
}

void Estimate_angle::set_fusion_alpha_gy_mg_cf(float _alpha_2){
  alpha_2 = _alpha_2;
}

float Estimate_angle::get_fusion_alpha_gy_mg_cf(){
  return alpha_2;
}

void Estimate_angle::set_sensitivity_ac(float _sensitivity_ac){
  sensitivity_ac = _sensitivity_ac;
}

float Estimate_angle::get_sensitivity_ac(){
  return sensitivity_ac;
}

void Estimate_angle::set_sensitivity_gy(float _sensitivity_gy){
  sensitivity_gy = _sensitivity_gy;
}

float Estimate_angle::get_sensitivity_gy(){
  return sensitivity_gy;
}

void Estimate_angle::set_sensitivity_mg(float _sensitivity_mg){
  sensitivity_mg = _sensitivity_mg;
}

float Estimate_angle::get_sensitivity_mg(){
  return sensitivity_mg;
}

float Estimate_angle::get_g_sen(){
  return g_sen;
}

void Estimate_angle::calibrate_acc(int16_t AcX, int16_t AcY, int16_t AcZ){
  float alpha_ac_calib = 0.995;
  b_ac_x = alpha_ac_calib*b_ac_x + (1-alpha_ac_calib)*(float)AcX;
  b_ac_y = alpha_ac_calib*b_ac_y + (1-alpha_ac_calib)*(float)AcY;
  b_ac_z = alpha_ac_calib*b_ac_z + (1-alpha_ac_calib)*(float)AcZ;
}

float Estimate_angle::get_bias_ac_x(){
  return b_ac_x;
}

float Estimate_angle::get_bias_ac_y(){
  return b_ac_y;
}

float Estimate_angle::get_bias_ac_z(){
  return b_ac_z;
}

void Estimate_angle::set_bias_ac_x(float _b_ac_x){
  b_ac_x = _b_ac_x;
}

void Estimate_angle::set_bias_ac_y(float _b_ac_y){
  b_ac_y = _b_ac_y;
}

void Estimate_angle::set_bias_ac_z(float _b_ac_z){
  b_ac_z = _b_ac_z;
}

void Estimate_angle::update_angles(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ, int16_t MgX, int16_t MgY, int16_t MgZ){
  sen_out_to_phy_signal(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ);
  filter_ac();
  angle_from_acc();
  filter_mg();
  angle_from_mag();
  angle_from_gyro();
  fuse_gy_ac_mg_angles();
  phi = phi_gy;
  th = th_gy;
  psi = psi_gy;
  //Serial.print(phi*180/PI); Serial.print('\t');
  //Serial.print(th*180/PI); Serial.print('\t');
  //Serial.print(psi*180/PI); Serial.print('\t');
  update_trig_ratios();
  estimate_body_acc_b();
  estimate_body_acc_F();
}

void Estimate_angle::update_angles(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ){
  sen_out_to_phy_signal(AcX, AcY, AcZ, GyX, GyY, GyZ);
  filter_ac();
  angle_from_acc();
  angle_from_gyro();
  fuse_gy_ac_angles();
  phi = phi_gy;
  th = th_gy;
  psi = psi_gy;
  //Serial.print(phi*180/PI); Serial.print('\t');
  //Serial.print(th*180/PI); Serial.print('\t');
  //Serial.print(psi*180/PI); Serial.print('\t');
  update_trig_ratios();
  estimate_body_acc_b();
  estimate_body_acc_F();
}

void Estimate_angle::sen_out_to_phy_signal(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ, int16_t MgX, int16_t MgY, int16_t MgZ){
  ac_out_to_phy_signal(AcX, AcY, AcZ);
  gy_out_to_phy_signal(GyX, GyY, GyZ);
  mg_out_to_phy_signal(MgX, MgY, MgZ);
}

void Estimate_angle::sen_out_to_phy_signal(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ){
  ac_out_to_phy_signal(AcX, AcY, AcZ);
  gy_out_to_phy_signal(GyX, GyY, GyZ);
}

void Estimate_angle::ac_out_to_phy_signal(int16_t AcX, int16_t AcY, int16_t AcZ){
  a_x_sen = (float)AcX/sf_ac_x; //in g
  a_y_sen = (float)AcY/sf_ac_y;
  a_z_sen = (float)AcZ/sf_ac_z;
}

void Estimate_angle::gy_out_to_phy_signal(int16_t GyX, int16_t GyY, int16_t GyZ){
  w_b_sen_x = (float)GyX/sensitivity_gy; // degree/sec
  w_b_sen_y = (float)GyY/sensitivity_gy;
  w_b_sen_z = (float)GyZ/sensitivity_gy;
}

void Estimate_angle::mg_out_to_phy_signal(int16_t MgX, int16_t MgY, int16_t MgZ){
  B_b_sen_x = (float)MgX/sensitivity_mg; // mGauss
  B_b_sen_y = (float)MgY/sensitivity_mg;
  B_b_sen_z = (float)MgZ/sensitivity_mg;
}

void Estimate_angle::filter_ac(){
  a_x_sen = filt_ac_x.apply_LPF(a_x_sen);
  a_y_sen = filt_ac_y.apply_LPF(a_y_sen);
  a_z_sen = filt_ac_z.apply_LPF(a_z_sen);
}

void Estimate_angle::angle_from_acc(){
  g_x_b = a_x_sen - b_ac_x; //m/sec^2
  g_y_b = a_y_sen - b_ac_y;
  g_z_b = a_z_sen - b_ac_z;
  g_sen = sqrt(g_x_b*g_x_b + g_y_b*g_y_b + g_z_b*g_z_b);
  phi_ac = atan2(g_y_b,g_z_b);
  th_ac = asin(-g_x_b/g_sen);
}

void Estimate_angle::filter_mg(){
  B_b_sen_x = filt_mg_x.apply_LPF(B_b_sen_x);
  B_b_sen_y = filt_mg_y.apply_LPF(B_b_sen_y);
  B_b_sen_z = filt_mg_z.apply_LPF(B_b_sen_z);
}

//10. angle_from_magnetometer (psi_mg)
void Estimate_angle::angle_from_mag(){
  /* https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
  Jodhpur India
  Latitude:  26° 16' 0" N
  Longitude:  73° 1' 49" E
  Model Used: WMM-2020
  Date: 2021-01-10
  Declination: 1°5'E ± 0°18' changing by 0°2'E per year
  */
  B_b_x = B_b_sen_x - V_x;
  B_b_y = B_b_sen_y - V_y;
  B_b_z = B_b_sen_z - V_z;
  
  psi_mg = atan2((B_b_z*s_phi - B_b_y*c_phi), (B_b_x*c_th + B_b_z*c_phi*s_th + B_b_y*s_phi*s_th));
  psi_mg = psi_mg + mg_declination;
  //Serial.print(psi_mg*180/PI); Serial.print('\t');
}

void Estimate_angle::angle_from_gyro(){
  if(gyro_start == 0){
    gyro_start = 1;
    
    phi_gy = phi_ac;
    th_gy  = th_ac;
    psi_gy = psi_mg;
  }
  else{
    // removing bias from gyroscope
    w_b_x = w_b_sen_x - b_gy_x;
    w_b_y = w_b_sen_y - b_gy_y;
    w_b_z = w_b_sen_z - b_gy_z;
    
    // converting angular velocity from degree/second to radians/second
    w_b_x = math.deg_to_rad(w_b_x);
    w_b_y = math.deg_to_rad(w_b_y);
    w_b_z = math.deg_to_rad(w_b_z);
    
    // mapping angular velocity in body frame to Euler angle rates
    dphi_dt =  w_b_x + (w_b_y*s_phi + w_b_z*c_phi)*t_th;
    dth_dt  =  w_b_y*c_phi - w_b_z*s_phi;
    dpsi_dt = (w_b_y*s_phi + w_b_z*c_phi)/c_th;
  
    // integrating Euler angle rates w.r.t. time to get Euler angle
    phi_gy = phi_gy + dphi_dt*dt;
    th_gy  = th_gy  + dth_dt*dt;
    psi_gy = psi_gy + dpsi_dt*dt;
  
    // conserving the range of angles within (-pi, pi]
    phi_gy = math.wrap(phi_gy,-PI,PI);
    th_gy  = math.wrap(th_gy,-PI,PI);
    psi_gy = math.wrap(psi_gy,-PI,PI);
  }
}

void Estimate_angle::fuse_gy_ac_mg_angles(){
  fuse_gy_ac_angles();
  fuse_gy_mg_angles();
}

void Estimate_angle::fuse_gy_ac_angles(){
  float err_phi_ac_gy = phi_ac - phi_gy;
  err_phi_ac_gy = math.wrap(err_phi_ac_gy,-PI,PI);
  phi_gy = phi_gy + alpha_1*err_phi_ac_gy;
  th_gy  = th_gy  + alpha_1*(th_ac - th_gy);
}

void Estimate_angle::fuse_gy_mg_angles(){
  float err_psi_mg_gy = psi_mg - psi_gy;
  err_psi_mg_gy = math.wrap(err_psi_mg_gy,-PI,PI);
  psi_gy = psi_gy + alpha_2*err_psi_mg_gy;
}

void Estimate_angle::estimate_body_acc_b(){
  g_x_b = -1*s_th;
  g_y_b =  1*c_th*s_phi;
  g_z_b =  1*c_phi*c_th;
  
  a_x_b = a_x_sen - b_ac_x - g_x_b;
  a_y_b = a_y_sen - b_ac_y - g_y_b;
  a_z_b = a_z_sen - b_ac_z - g_z_b;
  
  //Serial.print(a_x_b*9.80665,4); Serial.print('\t');
  //Serial.print(a_y_b*9.80665,4); Serial.print('\t');
  //Serial.print(a_z_b*9.80665,4);
}

void Estimate_angle::estimate_body_acc_F(){
  float r11 = c_psi*c_th;
  float r12 = c_psi*s_phi*s_th - c_phi*s_psi;
  float r13 = s_phi*s_psi + c_phi*c_psi*s_th;

  float r21 = c_th*s_psi;
  float r22 = c_phi*c_psi + s_phi*s_psi*s_th;
  float r23 = c_phi*s_psi*s_th - c_psi*s_phi;

  float r31 = -s_th;
  float r32 = c_th*s_phi;
  float r33 = c_phi*c_th;
  
  a_x = r11*a_x_b + r12*a_y_b + r13*a_z_b;
  a_y = r21*a_x_b + r22*a_y_b + r23*a_z_b;
  a_z = r31*a_x_b + r32*a_y_b + r33*a_z_b;

  //Serial.print(a_x*9.80665,4); Serial.print('\t');
  //Serial.print(a_y*9.80665,4); Serial.print('\t');
  //Serial.print(a_z*9.80665,4);
}

void Estimate_angle::update_trig_ratios(){
  s_phi = sin(phi);
  c_phi = cos(phi);
  t_phi = tan(phi);
  s_th = sin(th);
  c_th = cos(th);
  t_th = tan(th);
  s_psi = sin(psi);
  c_psi = cos(psi);
  t_psi = tan(psi);
}

void Estimate_angle::get_angles(float* _phi, float* _th, float* _psi){
  *_phi = phi;
  *_th = th;
  *_psi = psi;
}

float Estimate_angle::get_angle_phi(){
  return phi;
}

float Estimate_angle::get_angle_th(){
  return th;
}

float Estimate_angle::get_angle_psi(){
  return psi;
}

float Estimate_angle::get_angle_phi_ac(){
  return phi_ac;
}

float Estimate_angle::get_angle_th_ac(){
  return th_ac;
}

float Estimate_angle::get_angle_psi_mg(){
  return psi_mg;
}

float Estimate_angle::get_wx_b(){
  return w_b_x;
}

float Estimate_angle::get_wy_b(){
  return w_b_y;
}

float Estimate_angle::get_wz_b(){
  return w_b_z;
}

void Estimate_angle::get_wxyz_b(float* wx_b, float* wy_b, float* wz_b){
  *wx_b = w_b_x;
  *wy_b = w_b_y;
  *wz_b = w_b_z;
}

float Estimate_angle::get_ax_b(){
  return a_x_b;
}

float Estimate_angle::get_ay_b(){
  return a_y_b;
}

float Estimate_angle::get_az_b(){
  return a_z_b;
}

void Estimate_angle::get_axyz_b(float* ax_b, float* ay_b, float* az_b){
  *ax_b = a_x_b;
  *ay_b = a_y_b;
  *az_b = a_z_b;
}

float Estimate_angle::get_ax_F(){
  return a_x;
}

float Estimate_angle::get_ay_F(){
  return a_y;
}

float Estimate_angle::get_az_F(){
  return a_z;
}

void Estimate_angle::get_axyz_F(float* ax_F, float* ay_F, float* az_F){
  *ax_F = a_x;
  *ay_F = a_y;
  *az_F = a_z;
}
