#ifndef ESTIMATE_ANGLE
#define ESTIMATE_ANGLE

#include "Arduino.h"
#include "Math_functions.h"
#include "Filter.h"

class Estimate_angle{
  public:
    Estimate_angle();
    void calibrate_gyro(int16_t GyX, int16_t GyY, int16_t GyZ);
    void calibrate_gyro2(int16_t GyX, int16_t GyY, int16_t GyZ);
    float get_bias_gy_x();
    float get_bias_gy_y();
    float get_bias_gy_z();
    void calibrate_acc(int16_t AcX, int16_t AcY, int16_t AcZ);
    float get_bias_ac_x();
    float get_bias_ac_y();
    float get_bias_ac_z();
    void set_bias_ac_x(float _b_ac_x);
    void set_bias_ac_y(float _b_ac_y);
    void set_bias_ac_z(float _b_ac_z);
    void get_bias_gy_xyz(float* _b_gy_x, float* _b_gy_y, float* _b_gy_z);
    void set_bias_gy_x(float _b_gy_x);
    void set_bias_gy_y(float _b_gy_y);
    void set_bias_gy_z(float _b_gy_z);
    void set_bias_gy_xyz(float _b_gy_x, float _b_gy_y, float _b_gy_z);
    void set_fusion_alpha_gy_ac_cf(float _alpha_1);
    float get_fusion_alpha_gy_ac_cf();
    void set_fusion_alpha_gy_mg_cf(float _alpha_2);
    float get_fusion_alpha_gy_mg_cf();
    void set_sensitivity_ac(float _sensitivity_ac);
    float get_sensitivity_ac();
    void set_sensitivity_gy(float _sensitivity_gy);
    float get_sensitivity_gy();
    void set_sensitivity_mg(float _sensitivity_mg);
    float get_sensitivity_mg();
    float get_g_sen();
    
    void update_angles(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ, int16_t MgX, int16_t MgY, int16_t MgZ);
    void update_angles(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ);
    void get_angles(float* _phi, float* _th, float* _psi);
    float get_angle_phi();
    float get_angle_th();
    float get_angle_psi();
    float get_angle_phi_ac();
    float get_angle_th_ac();
    float get_angle_psi_mg();

    // temporary functions
    float get_wx_b();
    float get_wy_b();
    float get_wz_b();
    void get_wxyz_b(float* wx_b, float* wy_b, float* wz_b);

    float get_ax_b();
    float get_ay_b();
    float get_az_b();
    void get_axyz_b(float* ax_b, float* ay_b, float* az_b);

    float get_ax_F();
    float get_ay_F();
    float get_az_F();
    void get_axyz_F(float* ax_F, float* ay_F, float* az_F);
    
  private:
    Math_functions math;
    Filter_LP filt_ac_x;
    Filter_LP filt_ac_y;
    Filter_LP filt_ac_z;

    Filter_LP filt_mg_x;
    Filter_LP filt_mg_y;
    Filter_LP filt_mg_z;

    float dt = 0.0025;
    
    //int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
    //int16_t MgX, MgY, MgZ;

    // sensitivity of sensors depending upon the selected full scale
    float sensitivity_ac = 8192.0; // LSB/g
    float sensitivity_gy = 65.5; // LSB/deg/sec
    float sensitivity_mg = 1090.0; // LSB/Gauss

    // accelerometer variables starts
    // output of accelerometer sensor in body frame
    float a_x_sen;
    float a_y_sen;
    float a_z_sen;

    // sensitivity of accelerometer along each axis (after calibration), sensitivity_ac is acc to datasheet & mayn't be true
    float sf_ac_x = 8316.5; //8192.0
    float sf_ac_y = 8247.0;
    float sf_ac_z = 8215.0;

    // true acceleration in body frame
    float a_x_b = 0;
    float a_y_b = 0;
    float a_z_b = 0;

    // acceleration due to gravity in body frame
    float g_x_b;
    float g_y_b;
    float g_z_b;
    
    // bias in accelerometer reading (in g)
    float b_ac_x = -0.09372933;
    float b_ac_y = 0.0309203347;
    float b_ac_z = 0.0205721242;

    // true acceleration in ground frame
    float a_x = 0;
    float a_y = 0;
    float a_z = 0;

    float g_sen;
    float phi_ac, th_ac;
    
    // gyroscope variables starts
    float w_b_sen_x;
    float w_b_sen_y;
    float w_b_sen_z;

    float B_b_sen_x;
    float B_b_sen_y;
    float B_b_sen_z;

    float B_b_x;
    float B_b_y;
    float B_b_z;

    /*float V_x = 71.1432/sensitivity_mg;
    float V_y = -107.2400/sensitivity_mg;
    float V_z = -34.1834/sensitivity_mg;*/

    float V_x = 60.5866/sensitivity_mg;
    float V_y = -121.6686/sensitivity_mg;
    float V_z = -40.4693/sensitivity_mg;
    
    float s_phi = 0;
    float c_phi = 0;
    float t_phi = 0;
    float s_th = 0;
    float c_th = 0;
    float t_th = 0;
    float s_psi = 0;
    float c_psi = 0;
    float t_psi = 0;

    float mg_declination = math.deg_to_rad(1 + 5.0/60.0);
    float psi_mg;

    bool gyro_start = 0;
    float b_gy_x = -0.5852;
    float b_gy_y = 0.3389;
    float b_gy_z = -1.2118;

    float w_b_x, w_b_y, w_b_z;
    float dphi_dt, dth_dt, dpsi_dt;
    float phi_gy, th_gy, psi_gy;
    float phi, th, psi;

    float alpha_1 = 0.005; // 0.015
    float alpha_2 = 0.005;
    
    void sen_out_to_phy_signal(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ, int16_t MgX, int16_t MgY, int16_t MgZ);
    void sen_out_to_phy_signal(int16_t AcX, int16_t AcY, int16_t AcZ, int16_t GyX, int16_t GyY, int16_t GyZ);
    void ac_out_to_phy_signal(int16_t AcX, int16_t AcY, int16_t AcZ);
    void gy_out_to_phy_signal(int16_t GyX, int16_t GyY, int16_t GyZ);
    void mg_out_to_phy_signal(int16_t MgX, int16_t MgY, int16_t MgZ);
    void filter_ac();
    void angle_from_acc();
    void filter_mg();
    void angle_from_mag();
    void angle_from_gyro();
    void fuse_gy_ac_mg_angles();
    void fuse_gy_ac_angles();
    void fuse_gy_mg_angles();
    void update_trig_ratios();
    void estimate_body_acc_b();
    void estimate_body_acc_F();
};

#endif
