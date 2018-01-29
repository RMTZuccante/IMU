#include "IMU.h"

IMU::IMU() {
  microsPerReading = 1000000 / REFRESH;
}

void IMU::begin() {
  imu.begin();
}

bool IMU::check() {
  return imu.check();
}

void IMU::start() {
  filter.begin(REFRESH);
  microsPrevious = micros();
}

void IMU::calibrate() {
  imu.calibrateGyro();
}

float IMU::yaw() {
  update();
  return filter.getYaw();
}

float IMU::pitch() {
  update();
  return filter.getPitch();
}

float IMU::roll() {
  update();
  return filter.getRoll();
}

float IMU::getTemperature() {
  imu.getTemperature();
}

void IMU::read() {
  imu.readAccel();
  imu.readGyro();
}

void IMU::update() {
  unsigned long microsNow;
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    read();
    filter.update( imu.g[0], imu.g[1], imu.g[2], imu.a[0], imu.a[1], imu.a[2]);
    microsPrevious+=microsPerReading;
  }
}
