#ifndef IMU_h
#define IMU_h

#include "MadgwickAHRS.h"
#include "MPU6050.h"

class IMU {
  public:
    IMU() : microsPerReading(0) {}
    void begin();
    bool check();
    void start(unsigned long refresh);
    float yaw();
    float pitch();
    float roll();
    float inclination();
    float getTemperature();
    void calibrate();
    void update();
  private:
    void read();
    MPU6050 imu;
    Madgwick filter;
    unsigned long microsPerReading;
    unsigned long microsPrevious;
};

#endif
