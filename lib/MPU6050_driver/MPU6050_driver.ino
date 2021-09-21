#include "I2Cdev.h"
#include "MPU6050_driver.h"

I2Cdev i2c_master;
MPU6050_driver imu(MPU6050_ADDR_DEFAULT);

float SampleFrequency = 100, dt = 1/SampleFrequency, loop_timer = 1000000*dt, t;
float tn;

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float sensitivity_ac = 4096;
float sensitivity_gy = 32.8;
float ax_b, ay_b, az_b, temp_imu, wx_b, wy_b, wz_b; // assuming body frame and sensor frame is same

void setup(){
  Serial.begin(115200);
  
  i2c_master.initialize();
  
  imu.initialize(); //imu.set_SLEEP(SET_0);
  imu.set_DLPF_CFG(DLPF_CFG_1);
  imu.set_SMPLRT_DIV(SMPLRT_DIV);
  imu.set_FS_SEL(FS_SEL_2);
  imu.set_AFS_SEL(AFS_SEL_2);
  
  Serial.println("---------------------------");
  Serial.println("Initializing test!");
  
  /*Serial.println(TWBR);

  Serial.println("MD:");
  Serial.println(mag.get_MD());
  mag.set_MD(0); // 0 to 1
  Serial.println(mag.get_MD());
  
  Serial.println("MA:");
  Serial.println(mag.get_MA());
  //mag.set_MA(0); //0,1,2,3:1,2,4,8
  Serial.println(mag.get_MA());

  Serial.println("DO:");
  Serial.println(mag.get_DO());
  mag.set_DO(6); //0 to 6 75 Hz
  Serial.println(mag.get_DO());

  Serial.println("MS:");
  Serial.println(mag.get_MS());
  mag.set_MS(0); // 0 to 3
  Serial.println(mag.get_MS());

  Serial.println("GN:");
  Serial.println(mag.get_GN());
  //mag.set_GN(1); // 0 to 8
  Serial.println(mag.get_GN());*/
  
  /*Serial.println(TWBR);

  Serial.println("sleep: ");
  Serial.println(imu.get_SLEEP());
  imu.set_SLEEP(SET_1);
  Serial.println(imu.get_SLEEP());

  // gyro data have delay from sensor as compared to accelerometer
  Serial.println("dlpf: ");
  Serial.println(imu.get_DLPF_CFG());
  imu.set_DLPF_CFG(DLPF_CFG_1); //see register map for specification
  Serial.println(imu.get_DLPF_CFG());
  
  Serial.println("smplrt_divider: ");
  Serial.println(imu.get_SMPLRT_DIV());
  imu.set_SMPLRT_DIV(SMPLRT_DIV);
  Serial.println(imu.get_SMPLRT_DIV());

  Serial.println("gy_fs: ");
  Serial.println(imu.get_FS_SEL());
  imu.set_FS_SEL(FS_SEL_2);
  Serial.println(imu.get_FS_SEL());

  Serial.println("ac_fs: ");
  Serial.println(imu.get_AFS_SEL());
  imu.set_AFS_SEL(AFS_SEL_2);
  Serial.println(imu.get_AFS_SEL());*/

  t = micros();
}

void loop(){
  imu.get_MPU6050_OUT(&AcX, &AcY, &AcZ, &Tmp, &GyX, &GyY, &GyZ);
  print_raw_sensor_data();

  /*ax_b = (float)AcX/sensitivity_ac;
  ay_b = (float)AcY/sensitivity_ac;
  az_b = (float)AcZ/sensitivity_ac;
  temp_imu = (float)Tmp/340.0 + 36.53; // in degree C
  wx_b = (float)GyX/sensitivity_gy;
  wy_b = (float)GyY/sensitivity_gy;
  wz_b = (float)GyZ/sensitivity_gy;
  print_raw_physical_data();*/

  //mag.get_MAG_OUT(&MagX, &MagY, &MagZ);
  //MagX = MagX;
  //MagY = -MagY;
  //MagZ = -MagZ;
  
  //print_raw_mag_data();
  //print_calib_mag_data();

  //tn = t;
  //Serial.print(1000000.0/(micros()-tn)); Serial.print("\t");
  wait();
  //Serial.print(1000000.0/(micros()-tn)); Serial.print("\t");
}

void print_raw_sensor_data()
{
  Serial.print(AcX); Serial.print('\t');
  Serial.print(AcY); Serial.print('\t');
  Serial.print(AcZ); Serial.print('\t');
  Serial.print(Tmp); Serial.print('\t');
  Serial.print(GyX); Serial.print('\t');
  Serial.print(GyY); Serial.print('\t');
  Serial.print(GyZ); Serial.print('\t');
}

void print_raw_physical_data()
{
  Serial.print(ax_b); Serial.print('\t');
  Serial.print(ay_b); Serial.print('\t');
  Serial.print(az_b); Serial.print('\t');
  Serial.print(temp_imu); Serial.print('\t');
  Serial.print(wx_b); Serial.print('\t');
  Serial.print(wy_b); Serial.print('\t');
  Serial.print(wz_b); Serial.print('\t');
}

void wait(){
  while(micros()-t<loop_timer){}
  t = micros();
}
