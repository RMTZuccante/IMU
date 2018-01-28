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
  acc = imu.readNormalizeAccel();
  gyro = imu.readNormalizeGyro();
}

void IMU::update() {
  unsigned long microsNow;
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    read();
    filter.update(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
    microsPrevious+=microsPerReading;
  }
}
