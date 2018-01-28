#ifndef IMU_h
#define IMU_h

#include "MadgwickAHRS.h"
#include "MPU6050.h"

#define REFRESH 50

class IMU {
  public:
    IMU();
    void begin();
    void start();
    float yaw();
    float pitch();
    float roll();
    void calibrate();
  private:
    void read();
    void update();
    MPU6050 imu;
    Vector acc;
    Vector gyro;
    Madgwick filter;
    unsigned long microsPerReading;
    unsigned long microsPrevious;
};

#endif
