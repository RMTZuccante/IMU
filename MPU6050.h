#ifndef MPU6050_h
#define MPU6050_h

#include <Wire.h>
#include <math.h>
#include "Arduino.h"

#define MPU6050_ADDRESS 0x68

#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C


#define MPU6050_ACCEL 0x3B
#define MPU6050_TEMP 0x41
#define MPU6050_GYRO 0x43

#define MPU6050_PWR_MGMT 0x6B
#define MPU6050_WHO_AM_I 0x75

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector {
  float x;
  float y;
  float z;
};
#endif

typedef enum {
  MPU6050_CLOCK_KEEP_RESET      = 0b111,
  MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
  MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
  MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
  MPU6050_CLOCK_PLL_YGYRO       = 0b010,
  MPU6050_CLOCK_PLL_XGYRO       = 0b001,
  MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

typedef enum {
  MPU6050_SCALE_2000DPS         = 0b11,
  MPU6050_SCALE_1000DPS         = 0b10,
  MPU6050_SCALE_500DPS          = 0b01,
  MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum {
  MPU6050_RANGE_16G             = 0b11,
  MPU6050_RANGE_8G              = 0b10,
  MPU6050_RANGE_4G              = 0b01,
  MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

typedef enum {
  MPU6050_DELAY_3MS             = 0b11,
  MPU6050_DELAY_2MS             = 0b10,
  MPU6050_DELAY_1MS             = 0b01,
  MPU6050_NO_DELAY              = 0b00,
} mpu6050_onDelay_t;

typedef enum {
  MPU6050_DHPF_HOLD             = 0b111,
  MPU6050_DHPF_0_63HZ           = 0b100,
  MPU6050_DHPF_1_25HZ           = 0b011,
  MPU6050_DHPF_2_5HZ            = 0b010,
  MPU6050_DHPF_5HZ              = 0b001,
  MPU6050_DHPF_RESET            = 0b000,
} mpu6050_dhpf_t;

typedef enum {
  MPU6050_DLPF_6                = 0b110,
  MPU6050_DLPF_5                = 0b101,
  MPU6050_DLPF_4                = 0b100,
  MPU6050_DLPF_3                = 0b011,
  MPU6050_DLPF_2                = 0b010,
  MPU6050_DLPF_1                = 0b001,
  MPU6050_DLPF_0                = 0b000,
} mpu6050_dlpf_t;

class MPU6050 {
  public:
    MPU6050();
    bool begin(mpu6050_dps_t scale = MPU6050_SCALE_2000DPS, mpu6050_range_t range = MPU6050_RANGE_2G);

    void setScale(mpu6050_dps_t scale);
    void setRange(mpu6050_range_t range);
    mpu6050_dps_t getScale();
    mpu6050_range_t getRange();
    void setDHPFMode(mpu6050_dhpf_t dhpf);
    void setDLPFMode(mpu6050_dlpf_t dlpf);
    bool getSleepEnabled();
    void setSleepEnabled(bool state);

    mpu6050_clockSource_t getClockSource();
    void setClockSource(mpu6050_clockSource_t source);

    float readTemperature();

    void calibrateGyro(uint8_t samples = 50);
    void setThreshold(uint8_t multiple = 1);
    uint8_t getThreshold();

    void readData();

    Vector  readRawGyro();
    Vector  readNormalizeGyro();

    Vector  readRawAccel();
    Vector  readNormalizeAccel();

  private:
    Vector ra, rg; // Raw vectors
    Vector na, ng; // Normalized vectors
    Vector tg, dg; // Threshold and Delta for Gyro
    Vector th;     // Threshold

    float dpsPerDigit, rangePerDigit;
    float actualThreshold;
    bool useCalibrate;

    uint8_t fastRegister8(uint8_t reg);

    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

    int16_t readRegister16(uint8_t reg);
    void writeRegister16(uint8_t reg, int16_t value);

    bool readRegisterBit(uint8_t reg, uint8_t pos);
    void writeRegisterBit(uint8_t reg, uint8_t pos, bool state);

};

#endif
