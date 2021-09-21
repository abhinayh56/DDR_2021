#include <Arduino.h>
#include "I2Cdev.h"
#include "Filter.h"
#include "Math_functions.h"
#include "MPU6050_driver.h"
#include "Estimate_angle.h"
#include "QuadEncoder.h"
#include "PID_controller.h"

// time variable starts ------------------------------------------------------------------------------------
float SampleFrequency = 400;
float dt = 1.0/SampleFrequency;
float loop_timer = 1000000.0*dt;
float t = 0;
// time variables ends -------------------------------------------------------------------------------------

// I2C driver starts ---------------------------------------------------------------------------------------
I2Cdev i2c_master;
// I2C driver ends -----------------------------------------------------------------------------------------

// MPU6050 variables  starts -------------------------------------------------------------------------------
MPU6050_driver mpu6050(MPU6050_ADDR_DEFAULT);
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float ax_b, ay_b, az_b, temp_imu, wx_b, wy_b, wz_b; // assuming body frame and sensor frame is same
// MPU6050 variables variables starts ----------------------------------------------------------------------

// Orientation estimation variables starts -----------------------------------------------------------------
Estimate_angle angle_state;
float phi, th, psi;
// Orientation estimation variables ends -------------------------------------------------------------------

// Robot parameters starts ---------------------------------------------------------------------------------
double L = 0.265; // wheel base in m
double r = 0.125/2.0; // wheel radius in m
unsigned long N_encoder = 133600.0;
// Robot parameters ends -----------------------------------------------------------------------------------

// Wheel odometry variables starts -------------------------------------------------------------------------
QuadEncoder enc_left(1, 2, 3);
QuadEncoder enc_right(2, 4, 5);
// encoder ticks
long count_L = 0;
long count_R = 0;

long count_L_pre = 0;
long count_R_pre = 0;

long N_L_pre = 0;
long N_R_pre = 0;

// wheel rpm
double w_L = 0;
double w_R = 0;

// center of mass velocity
double Vc = 0;
double Wc = 0;

// position, velocity, angle
double x   = 0;
double y   = 0;
double yaw_angle = 0;
double vx  = 0;
double vy  = 0;
double yaw_rate  = 0;

unsigned int odom_counter_50Hz = 1;
Filter_LP lpf_w_L;
Filter_LP lpf_w_R;
// Wheel odometry variables ends ---------------------------------------------------------------------------

// Controller variables starts -----------------------------------------------------------------------------
uint16_t velocity_controller_50Hz_counter = 1;
PID_controller velocity_controller;
PID_controller turning_controller;
PID_controller angle_controller;
PID_controller heading_controller;
float Vc0 = 0;
float th_ref = 0;
// Controller variables ends -------------------------------------------------------------------------------

// Motor driving variables starts --------------------------------------------------------------------------
#define DIR_PIN_1 31
#define DIR_PIN_2 30
#define PWM_PIN_1 29
#define PWM_PIN_2 28
// Motor driving variables ends ----------------------------------------------------------------------------

// Xtras variables starts ----------------------------------------------------------------------------------
Math_functions math;
// Xtras variables ends ------------------------------------------------------------------------------------

// Function prototypes -------------------------------------------------------------------------------------
void wait();
void print_raw_mpu6050_data();
void print_mpu6050_physical_data();
void calibrate_gyro();
void calibrate_gyro2();
void print_angles();
void print_angular_velocity();
void calibrate_acc_out();
void wheel_odometry_ddr(long N_L, long N_R, double L, double r, unsigned long N, double dt);
void init_motors();
void drive_motors(int16_t pwm_1=0, int16_t pwm_2=0);

struct States{
  float x = 0;
  float y = 0;
  float z = 0;
  float vx = 0;
  float vy = 0;
  float vz = 0;
  float ax = 0;
  float ay = 0;
  float az = 0;
  float phi = 0;
  float th = 0;
  float psi = 0;
  float wx = 0;
  float wy = 0;
  float wz = 0;
};

struct States_des{
  float x0 = 0;
  float y0 = 0;
  float z0 = 0;
  float vx0 = 0;
  float vy0 = 0;
  float vz0 = 0;
  float ax0 = 0;
  float ay0 = 0;
  float az0 = 0;
  float phi0 = 0;
  float th0 = 0;
  float psi0 = 0;
  float wx0 = 0;
  float wy0 = 0;
  float wz0 = 0;
};

struct States state;
struct States_des state_des;

float V_bat = 11.1;

float control_rpm_L(float w_L0);
float control_rpm_R(float w_R0);

void setup(){
  // Initialize motors -------------------------------------------------------------------------------------
  init_motors();

  // Initialize encoders -----------------------------------------------------------------------------------
  enc_left.setInitConfig();
  enc_left.init();
  enc_right.setInitConfig();
  enc_right.init();

  // Initialize serial communication -----------------------------------------------------------------------
  Serial.begin(115200); // for usb comunication

  // initialize I2C communication on master arduino --------------------------------------------------------
  i2c_master.initialize();

  // initialize and configure MPU6050 (accelerometer + gyroscope) ------------------------------------------
  mpu6050.initialize(); //imu.set_SLEEP(SET_0);
  mpu6050.set_DLPF_CFG(DLPF_CFG_1);
  mpu6050.set_SMPLRT_DIV(SMPLRT_DIV);
  mpu6050.set_FS_SEL(FS_SEL_1);
  mpu6050.set_AFS_SEL(AFS_SEL_1);
  
  // orientation estimation starts -------------------------------------------------------------------------
  //angle_state.set_sensitivity_ac(8192.0); // comment this to use calibrated scale factor
  angle_state.set_sensitivity_gy(65.5);
  //calibrate_gyro(); // using exponential filter
  //calibrate_gyro2(); // using average
  //calibrate_acc_out(); // it just gives the range of values, manually calculate bias and sensitivity using it
  
  float alpha_w_wheel = 0.6825;
  lpf_w_L.set_alpha_LPF(alpha_w_wheel);
  lpf_w_R.set_alpha_LPF(alpha_w_wheel);

  // controller starts
  //velocity_controller
  velocity_controller.initialize();
  velocity_controller.set_gains(40.0,0,0);
  velocity_controller.set_I_max(18.0);
  velocity_controller.set_output_max(30.0);
  velocity_controller.set_lpf_alpha(0);

  //turning_controller
  turning_controller.initialize();
  turning_controller.set_gains(0,0,0);
  turning_controller.set_I_max(70.0);
  turning_controller.set_output_max(70.0);
  turning_controller.set_lpf_alpha(0);

  //heading_controller
  heading_controller.initialize();
  heading_controller.set_gains(0,0,0);
  heading_controller.set_I_max(25.0);
  heading_controller.set_output_max(25.0);
  heading_controller.set_lpf_alpha(0);
  
  //angle_controller
  angle_controller.initialize();
  angle_controller.set_gains(20.5,135.0,0.32);
  angle_controller.set_I_max(250.0);
  angle_controller.set_output_max(250.0);
  angle_controller.set_lpf_alpha(0);

  Vc0 = 0;

  delay(5000);
  t = micros();
}

////////////////////////
float e_kn   = 0;
float e_k_1n = 0;
float e_k_2n = 0;
float u_k_1n = 0;
float u_kn   = 0;

float calc_del_un(float x0, float x);

float calc_del_un(float x0, float x){
  float Kp    = 16.2;
  float tau_d = 0.046497333375;
  float tau_i = 0.1859893335;
  float T_0   = 1.0/50.0;

  float q0 =  Kp*(1.0 + tau_d/T_0);
  float q1 = -Kp*(1.0 + 2.0*tau_d/T_0 - T_0/tau_i);
  float q2 = Kp*tau_d/T_0;

  e_kn = x0 - x;
  float delta_u = q0*e_kn + q1*e_k_1n + q2*e_k_2n;
  e_k_2n = e_k_1n;
  e_k_1n = e_kn;

  return delta_u;
}
////////////////////////
float u1 = 0;
float u2 = 0;

void loop(){
  // SECTION 1: GET DESIRED STATES -------------------------------------------------------------------------
  state_des.x0   = 0;
  state_des.y0   = 0;

  state_des.vx0  = 0;
  state_des.vy0  = 0;

  state_des.ax0  = 0;
  state_des.ay0  = 0;

  state_des.th0  = 0;
  state_des.psi0 = 0;

  state_des.wy0  = 0;
  state_des.wz0  = 0;

  // SECTION 2: READ IMU DATA ------------------------------------------------------------------------------
  mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  int16_t tempAcX = AcX;
  int16_t tempAcY = AcY;
  int16_t tempAcZ = AcZ;
  AcX = tempAcZ;
  AcY = -tempAcY;
  AcZ = tempAcX;
  
  int16_t tempGyX = GyX;
  int16_t tempGyY = GyY;
  int16_t tempGyZ = GyZ;
  GyX = tempGyZ;
  GyY = -tempGyY;
  GyZ = tempGyX;
  //print_raw_mpu6050_data();
  //print_mpu6050_physical_data();

  // SECTION 3: READ BATTERY VOLTAGE -----------------------------------------------------------------------
  V_bat = 11.5;
  
  // SECTION 4: ESTIMATE ORIENTATION -----------------------------------------------------------------------
  angle_state.update_angles(AcX,AcY,AcZ,GyX,GyY,GyZ);

  angle_state.get_angles(&state.phi, &state.th, &state.psi);
  angle_state.get_wxyz_b(&state.wx, &state.wy, &state.wz);
  angle_state.get_axyz_F(&state.ax,&state.ay,&state.az);
  //print_angles();
  //print_angular_velocity();

  // SECTION 5: ESTIMATE POSITION --------------------------------------------------------------------------
  count_L = -enc_left.read();
  count_R = enc_right.read();
  wheel_odometry_ddr(count_L, count_R, L, r, N_encoder, dt);
  state.x   = x; //cm
  state.y   = y; //cm
  state.vx  = vx; //cm/sec
  state.vy  = vy; //cm/sec
  state.psi = yaw_angle;
  state.wz  = yaw_rate;
  Vc        = Vc;
  Wc        = Wc;
  w_L       = w_L;
  w_R       = w_R;
  

  //Serial.print(count_L); Serial.print('\t');
  //Serial.print(count_R); Serial.print('\t');
  //Serial.print(w_L*60.0/(2.0*PI)); Serial.print('\t');
  //Serial.print(w_R*60.0/(2.0*PI)); Serial.print('\t');
  //Serial.print(Vc); Serial.print('\t');
  //Serial.print(Wc*180.0/PI); Serial.print('\t');
  //Serial.print(state.x); Serial.print('\t');
  //Serial.print(state.y); Serial.print('\t');

  // SECTION 10: WAY POINT PLANNER -------------------------------------------------------------------------
  // SECTION 11: TRAJECTORY PLANNER ------------------------------------------------------------------------
  // SECTION 12: POSITION CONTROLLER -----------------------------------------------------------------------
  // SECTION 13: VELOCITY CONTROLLER -----------------------------------------------------------------------
  // SECTION 14: ATTITUDE CONTROLLER -----------------------------------------------------------------------

  if(velocity_controller_50Hz_counter==1){
    //velocity_controller.set_gains(25.0,10.0,0.3);
    velocity_controller.set_gains(12.0,8.5,0.15*0);
    th_ref = velocity_controller.calculate_output2(Vc0,Vc,dt*20.0);

    float psi0 = 0;
    /*if(millis()<10000){
      psi0 = 0;
      Vc0 = 0;
    }
    else{
      if(millis()<18000){
        psi0 = 150;
        Vc0 = 0;
      }
      else{
        psi0 = -90;
        if(millis()>25000){
          Vc0 = -0.25;
          if(millis()>30000){
            Vc0 = 0;
          }
        }
      }
    }*/
    float Wz0 = 0;
    heading_controller.set_gains(2,0.2,0);
    Wz0 = 0.4*math.wrap(psi0-state.psi*180.0/PI,-180.0,180.0);
    Wz0 = math.saturate(Wz0,-30,30);
    turning_controller.set_gains(0.8,5.0,0);
    u2 = turning_controller.calculate_output3(Wz0,state.wz*180.0/PI,dt*20.0);

    //Serial.print(Vc0*100.0); Serial.print('\t');
    //Serial.print(Vc*100.0); Serial.println('\t');
  }
  velocity_controller_50Hz_counter++;
  if(velocity_controller_50Hz_counter==21){
    velocity_controller_50Hz_counter = 1;
  }
  //Serial.print(th_ref*10.0); Serial.print('\t');

  /*heading_controller.set_gains(2,0.2,0);
  float u2 = heading_controller.calculate_output3(0,state.psi*180.0/PI,dt);*/

  float th_neutral = - 3.75;
  float th0 = th_ref + th_neutral;
  angle_controller.set_gains(19.0,150.0,0.32);
  float u1 = angle_controller.calculate_output2(th0,state.th*180.0/PI,dt);

    //th_ref = 27.0*(Vc0 - Vc);
    /*float delta_un = calc_del_un(Vc0,Vc);
    u_kn = u_k_1n + delta_un;
    u_k_1n = u_kn;
    th_ref = u_kn;*/

  Serial.print(th0); Serial.print('\t');
  Serial.print(state.th*180.0/PI); Serial.println('\t');

  // SECTION 16: FORCES & MOMENTS TO ROTOR RPM -------------------------------------------------------------
  // SECTION 18: DRIVE MOTOR -------------------------------------------------------------------------------
  if(abs(state.th)*180.0/PI>70.0){
    u1 = 0;
    //u2 = 0;
  }
  drive_motors(-u1+u2,-u1-u2); // right, left

  //Serial.print(1000000.0/(micros()-t)); Serial.print("\t");
  wait();
  //Serial.println();
}

void wait(){
  while(micros()-t<loop_timer){}
  t = micros();
}

void print_raw_mpu6050_data(){
  Serial.print(AcX); Serial.print('\t');
  Serial.print(AcY); Serial.print('\t');
  Serial.print(AcZ); Serial.print('\t');
  Serial.print(Tmp); Serial.print('\t');
  Serial.print(GyX); Serial.print('\t');
  Serial.print(GyY); Serial.print('\t');
  Serial.print(GyZ); Serial.print('\t');
}

void print_mpu6050_physical_data(){
  ax_b = (float)AcX/8192;
  ay_b = (float)AcY/8192;
  az_b = (float)AcZ/8192;
  temp_imu = (float)Tmp/340.0 + 36.53; // in degree C
  wx_b = (float)GyX/65.5;
  wy_b = (float)GyY/65.5;
  wz_b = (float)GyZ/65.5;

  Serial.print(ax_b); Serial.print('\t');
  Serial.print(ay_b); Serial.print('\t');
  Serial.print(az_b); Serial.print('\t');
  Serial.print(temp_imu); Serial.print('\t');
  Serial.print(wx_b); Serial.print('\t');
  Serial.print(wy_b); Serial.print('\t');
  Serial.print(wz_b); Serial.print('\t');
}

void calibrate_gyro(){
  for(int i=0;i<15000;i++){
    mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
    int16_t tempGyX = GyX;
    int16_t tempGyY = GyY;
    int16_t tempGyZ = GyZ;
    GyX = tempGyZ;
    GyY = -tempGyY;
    GyZ = tempGyX;
    angle_state.calibrate_gyro(GyX, GyY, GyZ);
    //Serial.print(millis()); Serial.print('\t');
    if(i%10==0){
      Serial.print(angle_state.get_bias_gy_x(),4); Serial.print('\t');
      Serial.print(angle_state.get_bias_gy_y(),4); Serial.print('\t');
      Serial.println(angle_state.get_bias_gy_z(),4);
    }
    wait();
  }
}

void calibrate_gyro2(){
  uint32_t N = 5000;
  for(uint32_t i=0;i<N;i++){
    mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
    int16_t tempGyX = GyX;
    int16_t tempGyY = GyY;
    int16_t tempGyZ = GyZ;
    GyX = tempGyZ;
    GyY = -tempGyY;
    GyZ = tempGyX;
    angle_state.calibrate_gyro2(GyX, GyY, GyZ);
    //Serial.print("-"); Serial.print('\t');
    if(i%10==0){
      Serial.print(angle_state.get_bias_gy_x(),4); Serial.print('\t');
      Serial.print(angle_state.get_bias_gy_y(),4); Serial.print('\t');
      Serial.println(angle_state.get_bias_gy_z(),4);
    }
    wait();
  }
  angle_state.set_bias_gy_x(angle_state.get_bias_gy_x()/(float(N)));
  angle_state.set_bias_gy_y(angle_state.get_bias_gy_y()/(float(N)));
  angle_state.set_bias_gy_z(angle_state.get_bias_gy_z()/(float(N)));

  Serial.print(angle_state.get_bias_gy_x(),4); Serial.print('\t');
  Serial.print(angle_state.get_bias_gy_y(),4); Serial.print('\t');
  Serial.println(angle_state.get_bias_gy_z(),4);
  wait();
}

void calibrate_acc_out(){
  uint32_t N = 10000;
  for(uint32_t i=0;i<N;i++){
    mpu6050.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
    int16_t tempAcX = AcX;
    int16_t tempAcY = AcY;
    int16_t tempAcZ = AcZ;
    AcX = tempAcZ;
    AcY = -tempAcY;
    AcZ = tempAcX;
    angle_state.calibrate_acc(AcX, AcY, AcZ);
    //Serial.print(millis()); Serial.print('\t');
    if(i%10==0){
      Serial.print(angle_state.get_bias_ac_x(),6); Serial.print('\t');
      Serial.print(angle_state.get_bias_ac_y(),6); Serial.print('\t');
      Serial.println(angle_state.get_bias_ac_z(),6);
    }
    wait();
  }
}

void print_angles(){
  Serial.print(state.phi*180/PI); Serial.print('\t');
  Serial.print(state.th*180/PI); Serial.print('\t');
  Serial.print(state.psi*180/PI); Serial.print('\t');
}

void print_angular_velocity(){
  Serial.print(state.wx*180/PI); Serial.print('\t');
  Serial.print(state.wy*180/PI); Serial.print('\t');
  Serial.print(state.wz*180/PI); Serial.print('\t');
}

void wheel_odometry_ddr(long N_L, long N_R, double L, double r, unsigned long N, double dt){
  // calculate yaw angle
  double d_yaw_angle = ( (double)((N_R-N_R_pre)-(N_L-N_L_pre))*2.0*PI*r ) / (double(N)*L);
  yaw_angle += d_yaw_angle;
  yaw_angle = math.wrap(yaw_angle,-PI,PI);

  // calculate x,y position
  double d_L = ( (double)((N_L-N_L_pre)+(N_R-N_R_pre))*PI*r ) / (double(N));
  x += d_L*cos(yaw_angle);
  y += d_L*sin(yaw_angle);

  N_R_pre = N_R;
  N_L_pre = N_L;

  // wheel speed
  if(odom_counter_50Hz==1){
    w_L = (double)(count_L - count_L_pre) *2.0*PI / (((double)N)*dt*10.0);
    w_R = (double)(count_R - count_R_pre) *2.0*PI / (((double)N)*dt*10.0);
    count_L_pre = count_L;
    count_R_pre = count_R;

    //Serial.print(w_L*180.0/PI); Serial.print('\t');
    //Serial.print(w_R*180.0/PI); Serial.print('\t');
    w_L = lpf_w_L.apply_LPF(w_L);
    w_R = lpf_w_R.apply_LPF(w_R);
    //Serial.print(w_L*180.0/PI); Serial.print('\t');
    //Serial.print(w_R*180.0/PI); Serial.print('\t');

    Vc = (w_R+w_L)*r/2.0;
    Wc = (w_R-w_L)*r/L;

    vx = Vc*cos(yaw_rate);
    vy = Vc*sin(yaw_rate);
    yaw_rate = Wc;
  }
  odom_counter_50Hz++;
  if(odom_counter_50Hz==6/*11*/){
    odom_counter_50Hz = 1;
  }
}

void init_motors(){
  pinMode(DIR_PIN_1,OUTPUT);
  pinMode(DIR_PIN_2,OUTPUT);

  digitalWrite(DIR_PIN_1,LOW);
  digitalWrite(DIR_PIN_2,LOW);
  
  pinMode(PWM_PIN_1,OUTPUT);
  pinMode(PWM_PIN_2,OUTPUT);

  /*
  analogWriteResolution(12); // 0 to 4095, or 4096 for high
  analogWriteResolution(12);
  
  analogWriteFrequency(PWM_PIN_1,0);
  analogWriteFrequency(PWM_PIN_2,0);
  */
  
  analogWrite(PWM_PIN_1,0);
  analogWrite(PWM_PIN_2,0);
}

void drive_motors(int16_t pwm_1, int16_t pwm_2){
  if(pwm_1<0){
    digitalWrite(DIR_PIN_1,LOW);
    analogWrite(PWM_PIN_1,-pwm_1);
  }
  else{
    digitalWrite(DIR_PIN_1,HIGH);
    analogWrite(PWM_PIN_1,pwm_1);
  }

  if(pwm_2<0){
    digitalWrite(DIR_PIN_2,LOW);
    analogWrite(PWM_PIN_2,-pwm_2);
  }
  else{
    digitalWrite(DIR_PIN_2,HIGH);
    analogWrite(PWM_PIN_2,pwm_2);
  }
}
