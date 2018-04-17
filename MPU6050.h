#ifndef MPU6050_h
#define MPU6050_h

#include <Arduino.h>
#include <math.h>
#include "I2C.h"

#define FORTHREE for (uint8_t i = 0; i < 3; i++)

#define MPU6050_ADDRESS 0x68

#define MPU6050_PWR_MGMT 0x6B
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL 0x3B
#define MPU6050_TEMP 0x41
#define MPU6050_GYRO 0x43


class MPU6050 {
  public:
    MPU6050();
    void begin();
    bool check();

    void calibrateGyro(uint8_t samples = 50);
    void setThreshold(uint8_t multiple = 1);

    void  readGyro();
    void  readAccel();
    float getTemperature();

    float a[3];
    float g[3];
  private:
    float threshold[3], drift[3], th[3];

    float dpsPerDigit, rangePerDigit;
    float actualThreshold;
    bool useCalibrate;

    void readData(uint8_t reg, int16_t *arr);
    void writeRegister(uint8_t reg, uint8_t value);
};

#endif
