#ifndef IMU_h
#define IMU_h

#include "MadgwickAHRS.h"
#include "MPU6050.h"

#define REFRESH 50

class IMU {
  public:
    IMU();
    void begin();
    bool check();
    void start();
    float yaw();
    float pitch();
    float roll();
    float getTemperature();
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
