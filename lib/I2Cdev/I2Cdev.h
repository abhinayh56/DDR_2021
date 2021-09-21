#ifndef I2CDEV_H
#define I2CDEV_H

#include  "Arduino.h"

class I2Cdev{
  public:
    I2Cdev();
    void initialize();
    uint8_t read_register(uint8_t i2c_dev_addr, uint8_t reg_addr);
    void read_registers(uint8_t i2c_dev_addr, uint8_t reg_addr, uint8_t num_bytes, uint8_t buff_[], uint8_t buff_i);
    void write_register(uint8_t i2c_dev_addr, uint8_t reg_addr, uint8_t register_val);
    void sendCommand(uint8_t i2c_dev_addr, uint8_t cmd);
};

#endif
