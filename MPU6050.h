#ifndef MPU6050_h
#define MPU6050_h

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

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

class MPU6050 {
  public:
    MPU6050();
    void begin(mpu6050_dps_t scale = MPU6050_SCALE_2000DPS, mpu6050_range_t range = MPU6050_RANGE_2G);
    bool check();

    void setScale(mpu6050_dps_t scale);
    void setRange(mpu6050_range_t range);

    float getTemperature();

    void calibrateGyro(uint8_t samples = 50);
    void setThreshold(uint8_t multiple = 1);

    Vector  readRawGyro();
    Vector  readNormalizeGyro();

    Vector  readRawAccel();
    Vector  readNormalizeAccel();

  private:
    Vector ra, rg;
    Vector na, ng;
    Vector tg, dg;
    Vector th;

    float dpsPerDigit, rangePerDigit;
    float actualThreshold;
    bool useCalibrate;

    void readData();

    uint8_t readRegister8(uint8_t reg);
    void writeRegister8(uint8_t reg, uint8_t value);

};

#endif
