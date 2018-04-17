#include "IMU.h"

/**
 * Makes the IMU ready to be read.
 */
void IMU::begin() {
  imu.begin();
}

/**
 * Checks if the IMU is connected and working.
 * @return TRUE if the IMU works.
 */
bool IMU::check() {
  bool ok = imu.check();
  return imu.check();
}

/**
 * Starts the filter and the IMU with the given refresh.
 * @param refresh Update refresh in Hertz.
 */
void IMU::start(unsigned long refresh) {
  microsPerReading = 1000000 / refresh;
  filter.start(refresh);
  microsPrevious = micros();
}

/**
 * Calibrates the IMU.
 */
void IMU::calibrate() {
  imu.calibrateGyro();
}

/**
 * Reads the yaw.
 * @return The already corrected yaw value.
 */
float IMU::yaw() {
  update();
  return filter.getYaw();
}

/**
 * Reads the pitch.
 * @return The already corrected pitch value.
 */
float IMU::pitch() {
  update();
  return filter.getPitch();
}

/**
 * Reads the roll.
 * @return The already corrected roll value.
 */
float IMU::roll() {
  update();
  return filter.getRoll();
}

/**
 * Reads the inclination, without using the filter.
 * @return The already corrected inclination value.
 */
float IMU::inclination() {
  imu.readAccel();
  return atan2(imu.a[0], imu.a[2])*180./PI;
}

/**
 * Reads the IMU internal temperature.
 * @return The temperature already corrected.
 */
float IMU::getTemperature() {
  imu.getTemperature();
}

/**
 * Reads the data from the accelerometer and the gyroscope.
 */
void IMU::read() {
  imu.readAccel();
  imu.readGyro();
}

/**
 * Updates the IMU and the filter.
 */
void IMU::update() {
  unsigned long microsNow;
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    read();
    filter.update(imu.g[0], imu.g[1], imu.g[2], imu.a[0], imu.a[1], imu.a[2]);
    microsPrevious += microsPerReading;
  }
}
