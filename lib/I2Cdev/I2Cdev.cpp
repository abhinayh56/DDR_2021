#include "I2Cdev.h"
#include "Wire.h"

I2Cdev::I2Cdev(){
  //Wire.begin();
  //TWBR = 12;
}

void I2Cdev::initialize(){
  Wire.begin();
  //TWBR = 12;
  Wire.setClock(400000);
}

uint8_t I2Cdev::read_register(uint8_t i2c_dev_addr, uint8_t reg_addr){
  Wire.beginTransmission(i2c_dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(i2c_dev_addr,1,true);
  while(Wire.available() < 1);
  return Wire.read();
}

void I2Cdev::read_registers(uint8_t i2c_dev_addr, uint8_t reg_addr, uint8_t n_bytes, uint8_t buff_[], uint8_t buff_i){
  Wire.beginTransmission(i2c_dev_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  Wire.requestFrom(i2c_dev_addr,n_bytes);
  while(Wire.available() < n_bytes);
  for(int i=buff_i;i<buff_i+n_bytes;i++){
    buff_[i] = Wire.read();
  }
}

void I2Cdev::write_register(uint8_t i2c_dev_addr, uint8_t reg_addr, uint8_t register_val){
  Wire.beginTransmission(i2c_dev_addr);
  Wire.write(reg_addr);
  Wire.write(register_val);
  Wire.endTransmission();
}

void I2Cdev::sendCommand(uint8_t i2c_dev_addr, uint8_t cmd)
{
  Wire.beginTransmission(i2c_dev_addr);
  Wire.write(cmd);
  Wire.endTransmission();
}
