#include "I2Cdev.h"
#include "MPU6050_driver.h"
#include "HMC5883L_driver.h"
#include "Estimate_angle.h"

I2Cdev i2c_master;
MPU6050_driver imu(MPU6050_ADDR_DEFAULT);
HMC5883L_driver mag;
Estimate_angle angle_state;

float SampleFrequency = 100, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t MgX, MgY, MgZ;

float phi, th, psi;

void setup(){
  Serial.begin(115200);
  
  i2c_master.initialize();
  
  imu.initialize(); //imu.set_SLEEP(SET_0);
  imu.set_DLPF_CFG(DLPF_CFG_1);
  imu.set_SMPLRT_DIV(SMPLRT_DIV);
  imu.set_FS_SEL(FS_SEL_2);
  imu.set_AFS_SEL(AFS_SEL_2);

  mag.initialize();
  mag.set_MD(0); // 0 (continuous meas.) to 1 (single meas. mode)
  mag.set_MA(0); // 0,1,2,3:1,2,4,8
  mag.set_DO(6); // 0 to 6 75 Hz
  mag.set_MS(0); // 0 to 3
  mag.set_GN(1); // 0 to 8
  
  t = micros();

  /*for(int i=0;i<2000;i++){
    imu.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
    angle_state.calibrate_gyro(GyX, GyY, GyZ);
    //Serial.print(millis()); Serial.print('\t');
    if(i%10==0){
      Serial.print(angle_state.get_bias_gy_x(),4); Serial.print('\t');
      Serial.print(angle_state.get_bias_gy_y(),4); Serial.print('\t');
      Serial.println(angle_state.get_bias_gy_z(),4);
    }
    wait();
  }*/
}

void loop(){
  imu.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  mag.get_MAG_OUT(&MgX, &MgY, &MgZ);
  MgX = MgX;
  MgY = -MgY;
  MgZ = -MgZ;
  
  //print_raw_sensor_data();
  //print_raw_mag_data();

  angle_state.update_angles(AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ);
  angle_state.get_angles(&phi, &th, &psi);
  /*Serial.print(angle_state.get_angle_phi_ac()*180/PI); Serial.print('\t');
  Serial.print(angle_state.get_angle_th_ac()*180/PI); Serial.print('\t');
  Serial.print(angle_state.get_angle_psi_mg()*180/PI); Serial.print('\t');
  print_angles();*/
  
  wait();
}

void print_raw_sensor_data(){
  Serial.print(AcX); Serial.print('\t');
  Serial.print(AcY); Serial.print('\t');
  Serial.print(AcZ); Serial.print('\t');
  Serial.print(Tmp); Serial.print('\t');
  Serial.print(GyX); Serial.print('\t');
  Serial.print(GyY); Serial.print('\t');
  Serial.print(GyZ); Serial.print('\t');
}

void print_raw_mag_data(){
  Serial.print(MgX); Serial.print('\t');
  Serial.print(MgY); Serial.print('\t');
  Serial.println(MgZ);
}

void print_angles(){
  Serial.print(phi*180/PI); Serial.print('\t');
  Serial.print(th*180/PI); Serial.print('\t');
  Serial.println(psi*180/PI);
}

void wait(){
  //Serial.println(1000000.0/(micros()-t));
  while(micros()-t<loop_timer){}
  t = micros();
}