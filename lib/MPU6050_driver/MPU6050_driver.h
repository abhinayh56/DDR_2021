#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "I2Cdev.h"

// device address
#define MPU6050_ADDR_AD0_LOW  0x68
#define MPU6050_ADDR_AD0_HIGH 0x69
#define MPU6050_ADDR_DEFAULT  MPU6050_ADDR_AD0_LOW // default address

// registers address
/*
  The reset value is 0x00 for all registers other than the registers below.
  Register 107(0x6B): 0x40
  Register 117(0x75): 0x68
*/
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define SMPRT_DIV        0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CT 0x67
#define SIGNAL_PATH_RES  0x68
#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I         0x75

/*  8-bit mask for data access and manipulation of registers
    mask for data bit location for a functionality
    it contains locations of bits inside a register for particular
    feature / function / behaviour / mode etc.
    A functionality can be activated / deactivated using a register
    and some bits of registers needs to be set (i.e. assign 1) or
    reset / clear (i.e. assign 0) depending upon the guidelines of
    the datasheet. Number of bits to be manipulated may be 0, 1, ... , 8.
    So the length of bits can be from zero to 8.
    The below variables contains mask to extract those bits for particular
    functionality
*/
// mask / address of data of one unit length
#define BITM_0 0x01 //0b00000001 (1)
#define BITM_1 0x02 //0b00000010 (2)
#define BITM_2 0x04 //0b00000100 (4)
#define BITM_3 0x08 //0b00001000 (8)
#define BITM_4 0x10 //0b00010000 (16)
#define BITM_5 0x20 //0b00100000 (32)
#define BITM_6 0x40 //0b01000000 (64)
#define BITM_7 0x80 //0b10000000 (128)

// mask / address of data of two unit length
#define BITM_01 (BITM_0 | BITM_1)
#define BITM_12 (BITM_1 | BITM_2)
#define BITM_23 (BITM_2 | BITM_3)
#define BITM_34 (BITM_3 | BITM_4)
#define BITM_45 (BITM_4 | BITM_5)
#define BITM_56 (BITM_5 | BITM_6)
#define BITM_67 (BITM_6 | BITM_7)

// mask / address of data of three unit length
#define BITM_02 (BITM_01 | BITM_2)
#define BITM_13 (BITM_12 | BITM_3)
#define BITM_24 (BITM_23 | BITM_4)
#define BITM_35 (BITM_34 | BITM_5)
#define BITM_46 (BITM_45 | BITM_6)
#define BITM_57 (BITM_56 | BITM_7)

// mask / address of data of fours unit length
#define BITM_03 (BITM_02 | BITM_3)
#define BITM_14 (BITM_13 | BITM_4)
#define BITM_25 (BITM_24 | BITM_5)
#define BITM_36 (BITM_35 | BITM_6)
#define BITM_47 (BITM_46 | BITM_7)

// mask / address of data of five unit length
#define BITM_04 (BITM_03 | BITM_4)
#define BITM_15 (BITM_14 | BITM_5)
#define BITM_26 (BITM_25 | BITM_6)
#define BITM_37 (BITM_36 | BITM_7)

// mask / address of data of six unit length
#define BITM_05 (BITM_04 | BITM_5)
#define BITM_16 (BITM_15 | BITM_6)
#define BITM_27 (BITM_26 | BITM_7)

// mask / address of data of seven unit length
#define BITM_06 (BITM_05 | BITM_6)
#define BITM_17 (BITM_16 | BITM_7)

// mask / address of data of eight unit length
#define BITM_07 (BITM_06 | BITM_7)

// values at bit location
#define SET_0 0x00
#define SET_1 0x01

// number of bits (data length) (bytes) OR location of bit in register
#define ZERO  0x00
#define ONE   0x01
#define TWO   0x02
#define THREE 0x03
#define FOUR  0x04
#define FIVE  0x05
#define SIX   0x06
#define SEVEN 0x07
#define EIGHT 0x08

// Register Name: SMPRT_DIV (Sample Rate Divider)
// location information of EXT_SMPLRT_DIV function
#define SMPLRT_DIV_LCN_1      BITM_07 // data bit location mask indicated by 1
#define SMPLRT_DIV_LCN_0     ~BITM_07 // data bit location mask indicated by 0
#define SMPLRT_DIV_LEN        EIGHT
#define SMPLRT_DIV_LCN_START  ZERO
#define SMPLRT_DIV_LCN_END    SEVEN

// values to be set in EXT_SMPLRT_DIV bit location
#define SMPLRT_DIV 0 // [0,255]


// Register Name: CONFIG
// location information of EXT_FSYNC function
#define EXT_SYNC_SET_LCN_1      BITM_35 // data bit location mask indicated by 1
#define EXT_SYNC_SET_LCN_0     ~BITM_35 // data bit location mask indicated by 0
#define EXT_SYNC_SET_LEN        THREE
#define EXT_SYNC_SET_LCN_START  THREE
#define EXT_SYNC_SET_LCN_END    FIVE

// values to be set in EXT_FSYNC bit location
#define EXT_SYNC_SET_0 0
#define EXT_SYNC_SET_1 1
#define EXT_SYNC_SET_2 2
#define EXT_SYNC_SET_3 3
#define EXT_SYNC_SET_4 4
#define EXT_SYNC_SET_5 5
#define EXT_SYNC_SET_6 6
#define EXT_SYNC_SET_7 7

// location information of DLPF_CFG function
#define DLPF_CFG_LCN_1      BITM_02 // data bit location mask indicated by 1
#define DLPF_CFG_LCN_0     ~BITM_02 // data bit location mask indicated by 0
#define DLPF_CFG_LEN        THREE
#define DLPF_CFG_LCN_START  ZERO
#define DLPF_CFG_LCN_END    TWO

// values to be set in DLPF_CFG bit location
#define DLPF_CFG_0 0
#define DLPF_CFG_1 1
#define DLPF_CFG_2 2
#define DLPF_CFG_3 3
#define DLPF_CFG_4 4
#define DLPF_CFG_5 5
#define DLPF_CFG_6 6
#define DLPF_CFG_7 7 // reserved

// Register Name: GYRO_CONFIG (Gyroscope Configuration)
// location information of XG_ST function
#define XG_ST_LCN_1      BITM_7 // data bit location mask indicated by 1
#define XG_ST_LCN_0     ~BITM_7 // data bit location mask indicated by 0
#define XG_ST_LEN        ONE
#define XG_ST_LCN_START  SEVEN
#define XG_ST_LCN_END    SEVEN

// values to be set in XG_ST bit location
#define XG_ST_0 0
#define XG_ST_1 1

// location information of YG_ST function
#define YG_ST_LCN_1      BITM_6 // data bit location mask indicated by 1
#define YG_ST_LCN_0     ~BITM_6 // data bit location mask indicated by 0
#define YG_ST_LEN        ONE
#define YG_ST_LCN_START  SIX
#define YG_ST_LCN_END    SIX

// values to be set in YG_ST bit location
#define YG_ST_0 0
#define YG_ST_1 1

// location information of ZG_ST function
#define ZG_ST_LCN_1      BITM_5 // data bit location mask indicated by 1
#define ZG_ST_LCN_0     ~BITM_5 // data bit location mask indicated by 0
#define ZG_ST_LEN        ONE
#define ZG_ST_LCN_START  FIVE
#define ZG_ST_LCN_END    FIVE

// values to be set in YG_ST bit location
#define ZG_ST_0 0
#define ZG_ST_1 1

// location information of FS_SEL function
#define FS_SEL_LCN_1      BITM_34 // data bit location mask indicated by 1
#define FS_SEL_LCN_0     ~BITM_34 // data bit location mask indicated by 0
#define FS_SEL_LEN        TWO
#define FS_SEL_LCN_START  THREE
#define FS_SEL_LCN_END    FOUR

// values to be set in FS_SEL bit location
#define FS_SEL_0 0
#define FS_SEL_1 1
#define FS_SEL_2 2
#define FS_SEL_3 3

// Register Name: ACCEL_CONFIG (Accelerometer Configuration)
// location information of XA_ST function
#define XA_ST_LCN_1      BITM_7 // data bit location mask indicated by 1
#define XA_ST_LCN_0     ~BITM_7 // data bit location mask indicated by 0
#define XA_ST_LEN        ONE
#define XA_ST_LCN_START  SEVEN
#define XA_ST_LCN_END    SEVEN

// values to be set in XA_ST bit location
#define XA_ST_0 0
#define XA_ST_1 1

// location information of YA_ST function
#define YA_ST_LCN_1      BITM_6 // data bit location mask indicated by 1
#define YA_ST_LCN_0     ~BITM_6 // data bit location mask indicated by 0
#define YA_ST_LEN        ONE
#define YA_ST_LCN_START  SIX
#define YA_ST_LCN_END    SIX

// values to be set in YA_ST bit location
#define YA_ST_0 0
#define YA_ST_1 1

// location information of ZA_ST function
#define ZA_ST_LCN_1      BITM_5 // data bit location mask indicated by 1
#define ZA_ST_LCN_0     ~BITM_5 // data bit location mask indicated by 0
#define ZA_ST_LEN        ONE
#define ZA_ST_LCN_START  FIVE
#define ZA_ST_LCN_END    FIVE

// values to be set in ZA_ST bit location
#define ZA_ST_0 0
#define ZA_ST_1 1

// location information of AFS_SEL function
#define AFS_SEL_LCN_1      BITM_34 // data bit location mask indicated by 1
#define AFS_SEL_LCN_0     ~BITM_34 // data bit location mask indicated by 0
#define AFS_SEL_LEN        TWO
#define AFS_SEL_LCN_START  THREE
#define AFS_SEL_LCN_END    FOUR

// values to be set in AFS_SEL bit location
#define AFS_SEL_0 0
#define AFS_SEL_1 1
#define AFS_SEL_2 2
#define AFS_SEL_3 3

// Register Name: PWR_MGMT_1 (Power Management 1)
// location information of DEVICE_RESET function
#define DEVICE_RESET_LCN_1      BITM_7 // data bit location mask indicated by 1
#define DEVICE_RESET_LCN_0     ~BITM_7 // data bit location mask indicated by 0
#define DEVICE_RESET_LEN        ONE
#define DEVICE_RESET_LCN_START  SEVEN
#define DEVICE_RESET_LCN_END    SEVEN

// values to be set in DEVICE_RESET bit location
#define DEVICE_RESET_0 0
#define DEVICE_RESET_1 1

// location information of SLEEP function
#define SLEEP_LCN_1      BITM_6 // data bit location mask indicated by 1
#define SLEEP_LCN_0     ~BITM_6 // data bit location mask indicated by 0
#define SLEEP_LEN        ONE
#define SLEEP_LCN_START  SIX
#define SLEEP_LCN_END    SIX

// values to be set in SLEEP bit location
#define SLEEP_0 0
#define SLEEP_1 1

// location information of CYCLE function
#define CYCLE_LCN_1      BITM_5 // data bit location mask indicated by 1
#define CYCLE_LCN_0     ~BITM_5 // data bit location mask indicated by 0
#define CYCLE_LEN        ONE
#define CYCLE_LCN_START  FIVE
#define CYCLE_LCN_END    FIVE

// values to be set in CYCLE bit location
#define CYCLE_0 0
#define CYCLE_1 1

// location information of TEMP_DIS function
#define TEMP_DIS_LCN_1      BITM_3 // data bit location mask indicated by 1
#define TEMP_DIS_LCN_0     ~BITM_3 // data bit location mask indicated by 0
#define TEMP_DIS_LEN        ONE
#define TEMP_DIS_LCN_START  THREE
#define TEMP_DIS_LCN_END    THREE

// values to be set in TEMP_DIS bit location
#define TEMP_DIS_0 0
#define TEMP_DIS_1 1

// location information of CLKSEL function
#define CLKSEL_LCN_1      BITM_02 // data bit location mask indicated by 1
#define CLKSEL_LCN_0     ~BITM_02 // data bit location mask indicated by 0
#define CLKSEL_LEN        THREE
#define CLKSEL_LCN_START  ZERO
#define CLKSEL_LCN_END    TWO

// values to be set in CLKSEL bit location
#define CLKSEL_0 0
#define CLKSEL_1 1
#define CLKSEL_2 2
#define CLKSEL_3 3
#define CLKSEL_4 4
#define CLKSEL_5 5
#define CLKSEL_6 6
#define CLKSEL_7 7

// Register Name: PWR_MGMT_2 (Power Management 2)
// location information of LP_WAKE_CTRL function
#define LP_WAKE_CTRL_LCN_1      BITM_67 // data bit location mask indicated by 1
#define LP_WAKE_CTRL_LCN_0     ~BITM_67 // data bit location mask indicated by 0
#define LP_WAKE_CTRL_LEN        TWO
#define LP_WAKE_CTRL_LCN_START  SIX
#define LP_WAKE_CTRL_LCN_END    SEVEN

// values to be set in LP_WAKE_CTRL bit location
#define LP_WAKE_CTRL_0 0
#define LP_WAKE_CTRL_1 1
#define LP_WAKE_CTRL_2 2
#define LP_WAKE_CTRL_3 3

// location information of STBY_XA function
#define STBY_XA_LCN_1      BITM_5 // data bit location mask indicated by 1
#define STBY_XA_LCN_0     ~BITM_5 // data bit location mask indicated by 0
#define STBY_XA_LEN        ONE
#define STBY_XA_LCN_START  FIVE
#define STBY_XA_LCN_END    FIVE

// values to be set in STBY_XA bit location
#define STBY_XA_0 0
#define STBY_XA_1 1

// location information of STBY_YA function
#define STBY_YA_LCN_1      BITM_4 // data bit location mask indicated by 1
#define STBY_YA_LCN_0     ~BITM_4 // data bit location mask indicated by 0
#define STBY_YA_LEN        ONE
#define STBY_YA_LCN_START  FOUR
#define STBY_YA_LCN_END    FOUR

// values to be set in STBY_YA bit location
#define STBY_YA_0 0
#define STBY_YA_1 1

// location information of STBY_ZA function
#define STBY_ZA_LCN_1      BITM_3 // data bit location mask indicated by 1
#define STBY_ZA_LCN_0     ~BITM_3 // data bit location mask indicated by 0
#define STBY_ZA_LEN        ONE
#define STBY_ZA_LCN_START  THREE
#define STBY_ZA_LCN_END    THREE

// values to be set in STBY_ZA bit location
#define STBY_ZA_0 0
#define STBY_ZA_1 1

// location information of STBY_XG function
#define STBY_XG_LCN_1      BITM_2 // data bit location mask indicated by 1
#define STBY_XG_LCN_0     ~BITM_2 // data bit location mask indicated by 0
#define STBY_XG_LEN        ONE
#define STBY_XG_LCN_START  TWO
#define STBY_XG_LCN_END    TWO

// values to be set in STBY_XG bit location
#define STBY_XG_0 0
#define STBY_XG_1 1

// location information of STBY_YG function
#define STBY_YG_LCN_1      BITM_1 // data bit location mask indicated by 1
#define STBY_YG_LCN_0     ~BITM_1 // data bit location mask indicated by 0
#define STBY_YG_LEN        ONE
#define STBY_YG_LCN_START  ONE
#define STBY_YG_LCN_END    ONE

// values to be set in STBY_YG bit location
#define STBY_YG_0 0
#define STBY_YG_1 1

// location information of STBY_ZG function
#define STBY_ZG_LCN_1      BITM_0 // data bit location mask indicated by 1
#define STBY_ZG_LCN_0     ~BITM_0 // data bit location mask indicated by 0
#define STBY_ZG_LEN        ONE
#define STBY_ZG_LCN_START  ZERO
#define STBY_ZG_LCN_END    ZERO

// values to be set in STBY_ZG bit location
#define STBY_ZG_0 0
#define STBY_ZG_1 1

// Register Name: WHO_AM_I (Who Am I)
// location information of WHO_AM_I function
#define WHO_AM_I_LCN_1      BITM_16 // data bit location mask indicated by 1
#define WHO_AM_I_LCN_0     ~BITM_16 // data bit location mask indicated by 0
#define WHO_AM_I_LEN        SIX
#define WHO_AM_I_LCN_START  ONE
#define WHO_AM_I_LCN_END    SIX

class MPU6050_driver {
  public:
    MPU6050_driver();
    MPU6050_driver(uint8_t address = MPU6050_ADDR_DEFAULT);

    void initialize();
    bool testConnection();

    // Register Name: SMPRT_DIV (Sample Rate Divider)
    uint8_t get_SMPLRT_DIV();
    void set_SMPLRT_DIV(uint8_t val);

    // Register Name: CONFIG
    uint8_t get_ext_FSYNC();
    void set_ext_FSYNC(uint8_t sync);
    uint8_t get_DLPF_CFG();
    void set_DLPF_CFG(uint8_t val);

    // Register Name: GYRO_CONFIG (Gyroscope Configuration)
    uint8_t get_XG_ST();
    void set_XG_ST(uint8_t val);
    uint8_t get_YG_ST();
    void set_YG_ST(uint8_t val);
    uint8_t get_ZG_ST();
    void set_ZG_ST(uint8_t val);
    uint8_t get_FS_SEL();
    void set_FS_SEL(uint8_t val);

    // Register Name: ACCEL_CONFIG (Accelerometer Configuration)
    uint8_t get_XA_ST();
    void set_XA_ST(uint8_t val);
    uint8_t get_YA_ST();
    void set_YA_ST(uint8_t val);
    uint8_t get_ZA_ST();
    void set_ZA_ST(uint8_t val);
    uint8_t get_AFS_SEL();
    void set_AFS_SEL(uint8_t val);

    // Register Name: PWR_MGMT_1 (Power Management 1)
    uint8_t get_DEVICE_RESET();
    void set_DEVICE_RESET(uint8_t val);
    uint8_t get_SLEEP();
    void set_SLEEP(uint8_t val);
    uint8_t get_CYCLE();
    void set_CYCLE(uint8_t val);
    uint8_t get_TEMP_DIS();
    void set_TEMP_DIS(uint8_t val);
    uint8_t get_CLKSEL();
    void set_CLKSEL(uint8_t val);

    // Register Name: PWR_MGMT_2 (Power Management 2)
    uint8_t get_LP_WAKE_CTRL();
    void set_LP_WAKE_CTRL(uint8_t val);
    uint8_t get_STBY_XA();
    void set_STBY_XA(uint8_t val);
    uint8_t get_STBY_YA();
    void set_STBY_YA(uint8_t val);
    uint8_t get_STBY_ZA();
    void set_STBY_ZA(uint8_t val);
    uint8_t get_STBY_XG();
    void set_STBY_XG(uint8_t val);
    uint8_t get_STBY_YG();
    void set_STBY_YG(uint8_t val);
    uint8_t get_STBY_ZG();
    void set_STBY_ZG(uint8_t val);

    // Register Name: WHO_AM_I (Who Am I)
    uint8_t get_WHO_AM_I();

    // Reading sensor data register
    uint8_t get_ACCEL_XOUT_H();
    uint8_t get_ACCEL_XOUT_L();
    uint8_t get_ACCEL_YOUT_H();
    uint8_t get_ACCEL_YOUT_L();
    uint8_t get_ACCEL_ZOUT_H();
    uint8_t get_ACCEL_ZOUT_L();
    int16_t get_ACCEL_XOUT();
    int16_t get_ACCEL_YOUT();
    int16_t get_ACCEL_ZOUT();
    uint8_t get_TEMP_OUT_H();
    uint8_t get_TEMP_OUT_L();
    int16_t get_TEMP_OUT();
    uint8_t get_GYRO_XOUT_H();
    uint8_t get_GYRO_XOUT_L();
    uint8_t get_GYRO_YOUT_H();
    uint8_t get_GYRO_YOUT_L();
    uint8_t get_GYRO_ZOUT_H();
    uint8_t get_GYRO_ZOUT_L();
    int16_t get_GYRO_XOUT();
    int16_t get_GYRO_YOUT();
    int16_t get_GYRO_ZOUT();
    void get_ACCEL_OUT(int16_t* AC_X, int16_t* AC_Y, int16_t* AC_Z);
    void get_GYRO_OUT(int16_t* GY_X, int16_t* GY_Y, int16_t* GY_Z);
    void get_MPU6050_OUT(int16_t* AC_X, int16_t* AC_Y, int16_t* AC_Z, int16_t* TEMP, int16_t* GY_X, int16_t* GY_Y, int16_t* GY_Z);

  private:
    I2Cdev i2cdev1;

    uint8_t mpu6050_addr;
    uint8_t register_val;
    uint8_t buff[14];
    /*
      buff[0]  = ACCEL_XOUT_H
      buff[1]  = ACCEL_XOUT_L
      buff[2]  = ACCEL_YOUT_H
      buff[3]  = ACCEL_YOUT_L
      buff[4]  = ACCEL_ZOUT_H
      buff[5]  = ACCEL_ZOUT_L
      buff[6]  =   TEMP_OUT_H
      buff[7]  =   TEMP_OUT_L
      buff[8]  =  GYRO_XOUT_H
      buff[9]  =  GYRO_XOUT_L
      buff[10] =  GYRO_YOUT_H
      buff[11] =  GYRO_YOUT_L
      buff[12] =  GYRO_ZOUT_H
      buff[13] =  GYRO_ZOUT_L
    */
    // index number of register values in buff array
#define buff_ACCEL_XOUT_H 0
#define buff_ACCEL_XOUT_L 1
#define buff_ACCEL_YOUT_H 2
#define buff_ACCEL_YOUT_L 3
#define buff_ACCEL_ZOUT_H 4
#define buff_ACCEL_ZOUT_L 5
#define buff_TEMP_OUT_H   6
#define buff_TEMP_OUT_L   7
#define buff_GYRO_XOUT_H  8
#define buff_GYRO_XOUT_L  9
#define buff_GYRO_YOUT_H  10
#define buff_GYRO_YOUT_L  11
#define buff_GYRO_ZOUT_H  12
#define buff_GYRO_ZOUT_L  13
};

#endif
