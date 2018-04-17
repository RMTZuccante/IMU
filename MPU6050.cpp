#include "MPU6050.h"

/**
 * Sets the sensor default values.
 */
MPU6050::MPU6050() {
  FORTHREE drift[i] = 0;
  FORTHREE threshold[i] = 0;
  useCalibrate = false;
  actualThreshold = 0;
}

/**
 * Makes the sensor ready to be read from, using a default configuration.
 * The gyroscope scale is 250DPS.
 * The accelerometer range is 2G
 */
void MPU6050::begin() {
  dpsPerDigit = .007633f; //gyro scale at 250DPS
  rangePerDigit = .000061f; //accel range at 2G
  writeRegister(MPU6050_GYRO_CONFIG, 0); //set gyro scale to 250DPS
  writeRegister(MPU6050_ACCEL_CONFIG, 0); //set accel range to 2G
  writeRegister(MPU6050_PWR_MGMT, 1); //set the clock to XGyro and turn on
}

/**
 * Checks if the sensor is working.
 * @return TRUE if the sensor works.
 */
bool MPU6050::check() {
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  if (I2C_2.endTransmission()) return false;

  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.write(MPU6050_WHO_AM_I);
  I2C_2.endTransmission();
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.requestFrom(MPU6050_ADDRESS, 1);
  uint8_t value = I2C_2.read();
  I2C_2.endTransmission();
  return value == MPU6050_ADDRESS;
}

/**
 * Reads and corrects the accelerometer.
 */
void MPU6050::readAccel() {
  int16_t raw[3];
  readData(MPU6050_ACCEL, raw);
  FORTHREE a[i] = 9.80665f * rangePerDigit * raw[i];
}

/**
 * Reads and corrects the gyroscope.
 */
void MPU6050::readGyro() {
  int16_t raw[3];
  readData(MPU6050_GYRO, raw);
  FORTHREE g[i] = raw[i];
  if (useCalibrate) FORTHREE g[i] -= drift[i];
  FORTHREE g[i] *= dpsPerDigit;
  if (actualThreshold) FORTHREE if (abs(g[i]) < threshold[i]) g[i] = 0;
}

/**
 * Reads the temperature sensor inside the MPU6050.
 * @return The already corrected temperature.
 */
float MPU6050::getTemperature() {
  int16_t temp;
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.write(MPU6050_TEMP);
  I2C_2.endTransmission();
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.requestFrom(MPU6050_ADDRESS, 2);
  while (!I2C_2.available());
  temp = I2C_2.read() << 8 | I2C_2.read();
  I2C_2.endTransmission();
  return (float)temp / 340.0 + 36.53;
}

/**
 * Calibrates the gyroscope by using a given number of samples.
 * More samples means more precision, but needs more time.
 * @param samples Number of samples to use for the calibration.
 */
void MPU6050::calibrateGyro(uint8_t samples) {
  useCalibrate = true;
  float sum[3];
  float sigma[3];
  int16_t raw[3];
  FORTHREE sigma[i] = sum[i] = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    readData(MPU6050_GYRO, raw);
    for (uint8_t i = 0; i < 3; i++) {
      sum[i] += raw[i];
      sigma[i] += raw[i] * raw[i];
    }
    delay(5);
  }

  FORTHREE {
    drift[i] = sum[i] / samples;
    th[i] = sqrt((sigma[i] / 50) - (drift[i] * drift[i]));
  }

  if (actualThreshold > 0) setThreshold(actualThreshold);
}

/**
 * Sets the gyroscope threshold value.
 * @param multiple
 */
void MPU6050::setThreshold(uint8_t multiple) {
  if (multiple > 0) {
    if (!useCalibrate) calibrateGyro();
    FORTHREE threshold[i] *= multiple;
  } else FORTHREE threshold[i] = 0;
  actualThreshold = multiple;
}

/**
 * Writes a single byte to the given register.
 * @param reg The register to write to.
 * @param value Byte to write.
 */
void MPU6050::writeRegister(uint8_t reg, uint8_t value) {
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.write(reg);
  I2C_2.write(value);
  I2C_2.endTransmission();
}

/**
 * Reads three words from the given register.
 * @param reg The register to read from.
 * @param arr The array containing the words.
 */
void MPU6050::readData(uint8_t reg, int16_t *arr) {
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.write(reg);
  I2C_2.endTransmission();
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.requestFrom(MPU6050_ADDRESS, 6);
  while (I2C_2.available() < 6);
  FORTHREE arr[i] = I2C_2.read() << 8 | I2C_2.read();
}
