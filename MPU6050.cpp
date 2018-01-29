#include "MPU6050.h"

MPU6050::MPU6050() {
  FORTHREE drift[i] = 0;
  FORTHREE threshold[i] = 0;
  useCalibrate = false;
  actualThreshold = 0;
}

void MPU6050::begin(mpu6050_dps_t scale, mpu6050_range_t range) {
  Wire.begin();
  writeRegister8(MPU6050_PWR_MGMT, 1); //set the clock to XGyro and turn on
  setScale(scale);
  setRange(range);
}

bool MPU6050::check() {
  Wire.begin();

  Wire.beginTransmission(MPU6050_ADDRESS);
  if (Wire.endTransmission()) return false;

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.requestFrom(MPU6050_ADDRESS, 1);
  uint8_t value = Wire.read();
  Wire.endTransmission();
  return value == MPU6050_ADDRESS;
}

void MPU6050::setScale(mpu6050_dps_t scale) {
  uint8_t value;
  switch (scale) {
    case MPU6050_SCALE_250DPS:
      dpsPerDigit = .007633f;
      break;
    case MPU6050_SCALE_500DPS:
      dpsPerDigit = .015267f;
      break;
    case MPU6050_SCALE_1000DPS:
      dpsPerDigit = .030487f;
      break;
    case MPU6050_SCALE_2000DPS:
      dpsPerDigit = .060975f;
      break;
  }
  value = readRegister8(MPU6050_GYRO_CONFIG);
  value &= 0b11100111;
  value |= (scale << 3);
  writeRegister8(MPU6050_GYRO_CONFIG, value);
}

void MPU6050::setRange(mpu6050_range_t range) {
  uint8_t value;
  switch (range) {
    case MPU6050_RANGE_2G:
      rangePerDigit = .000061f;
      break;
    case MPU6050_RANGE_4G:
      rangePerDigit = .000122f;
      break;
    case MPU6050_RANGE_8G:
      rangePerDigit = .000244f;
      break;
    case MPU6050_RANGE_16G:
      rangePerDigit = .0004882f;
      break;
  }
  value = readRegister8(MPU6050_ACCEL_CONFIG);
  value &= 0b11100111;
  value |= (range << 3);
  writeRegister8(MPU6050_ACCEL_CONFIG, value);
}

void MPU6050::readAccel() {
  int16_t raw[3];
  readData(MPU6050_ACCEL, raw);
  FORTHREE a[i] = 9.80665f * rangePerDigit * raw[i];
}

void MPU6050::readGyro() {
  int16_t raw[3];
  readData(MPU6050_GYRO, raw);
  FORTHREE g[i] = raw[i];
  if (useCalibrate) FORTHREE g[i]-=drift[i];
  FORTHREE g[i]*=dpsPerDigit;
  if (actualThreshold) FORTHREE if (abs(g[i]) < threshold[i]) g[i] = 0;
}

float MPU6050::getTemperature() {
  int16_t temp;
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_TEMP);
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.requestFrom(MPU6050_ADDRESS, 2);
  while (!Wire.available());
  temp = Wire.read() << 8 | Wire.read();
  Wire.endTransmission();
  return (float)temp / 340.0 + 36.53;
}

void MPU6050::calibrateGyro(uint8_t samples) {
  useCalibrate = true;
  float sum[3];
  float sigma[3];
  int16_t raw[3];
  FORTHREE sigma[i]=sum[i]=0;
  for (uint8_t i = 0; i < samples; ++i) {
    readData(MPU6050_GYRO, raw);
    for (uint8_t i = 0; i < 3; i++) {
      sum[i]+=raw[i];
      sigma[i]+=raw[i]*raw[i];
    }
    delay(5);
  }

  FORTHREE {
    drift[i] = sum[i] / samples;
    th[i] = sqrt((sigma[i] / 50) - (drift[i] * drift[i]));
  }

  if (actualThreshold > 0) setThreshold(actualThreshold);
}

void MPU6050::setThreshold(uint8_t multiple) {
  if (multiple > 0) {
    if (!useCalibrate) calibrateGyro();
    FORTHREE threshold[i] *= multiple;
  } else FORTHREE threshold[i] = 0;
  actualThreshold = multiple;
}

uint8_t MPU6050::readRegister8(uint8_t reg) {
  uint8_t value;
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.requestFrom(MPU6050_ADDRESS, 1);
  while (!Wire.available());
  value = Wire.read();
  Wire.endTransmission();
  return value;
}

void MPU6050::writeRegister8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MPU6050::readData(uint8_t reg, int16_t *arr) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.requestFrom(MPU6050_ADDRESS, 6);
  while (Wire.available() < 6);
  FORTHREE arr[i] = Wire.read() << 8 | Wire.read();
}
