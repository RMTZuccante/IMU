#include "MPU6050.h"

MPU6050::MPU6050() {
  dg.x = 0;
  dg.y = 0;
  dg.z = 0;
  useCalibrate = false;
  tg.x = 0;
  tg.y = 0;
  tg.z = 0;
  actualThreshold = 0;
}

void MPU6050::begin(mpu6050_dps_t scale, mpu6050_range_t range) {
  Wire.begin();
  writeRegister8(MPU6050_PWR_MGMT, 1); //set the clock to XGyro

  setScale(scale);
  setRange(range);

  writeRegisterBit(MPU6050_PWR_MGMT, 6, false); //turn on the sensor
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
    default:
      break;
  }
  value = readRegister8(MPU6050_ACCEL_CONFIG);
  value &= 0b11100111;
  value |= (range << 3);
  writeRegister8(MPU6050_ACCEL_CONFIG, value);
}

Vector MPU6050::readRawAccel() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_ACCEL);
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.requestFrom(MPU6050_ADDRESS, 6);
  while (Wire.available() < 6);
  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  ra.x = x;
  ra.y = y;
  ra.z = z;
  return ra;
}

Vector MPU6050::readNormalizeAccel() {
  readRawAccel();
  na.x = ra.x * rangePerDigit * 9.80665f;
  na.y = ra.y * rangePerDigit * 9.80665f;
  na.z = ra.z * rangePerDigit * 9.80665f;
  return na;
}

Vector MPU6050::readRawGyro() {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_GYRO);
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.requestFrom(MPU6050_ADDRESS, 6);
  while (Wire.available() < 6);
  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  rg.x = x;
  rg.y = y;
  rg.z = z;
  return rg;
}

void MPU6050::readData() {

}

Vector MPU6050::readNormalizeGyro() {
  readRawGyro();
  if (useCalibrate) {
    ng.x = (rg.x - dg.x) * dpsPerDigit;
    ng.y = (rg.y - dg.y) * dpsPerDigit;
    ng.z = (rg.z - dg.z) * dpsPerDigit;
  } else {
    ng.x = rg.x * dpsPerDigit;
    ng.y = rg.y * dpsPerDigit;
    ng.z = rg.z * dpsPerDigit;
  }
  if (actualThreshold) {
    if (abs(ng.x) < tg.x) ng.x = 0;
    if (abs(ng.y) < tg.y) ng.y = 0;
    if (abs(ng.z) < tg.z) ng.z = 0;
  }
  return ng;
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

  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  float sigmaX = 0;
  float sigmaY = 0;
  float sigmaZ = 0;

  for (uint8_t i = 0; i < samples; ++i) {
    readRawGyro();
    sumX += rg.x;
    sumY += rg.y;
    sumZ += rg.z;
    sigmaX += rg.x * rg.x;
    sigmaY += rg.y * rg.y;
    sigmaZ += rg.z * rg.z;
    delay(5);
  }

  dg.x = sumX / samples;
  dg.y = sumY / samples;
  dg.z = sumZ / samples;

  th.x = sqrt((sigmaX / 50) - (dg.x * dg.x));
  th.y = sqrt((sigmaY / 50) - (dg.y * dg.y));
  th.z = sqrt((sigmaZ / 50) - (dg.z * dg.z));

  if (actualThreshold > 0) setThreshold(actualThreshold);
}

void MPU6050::setThreshold(uint8_t multiple) {
  if (multiple > 0) {
    if (!useCalibrate) calibrateGyro();

    tg.x = th.x * multiple;
    tg.y = th.y * multiple;
    tg.z = th.z * multiple;
  } else {
    tg.x = 0;
    tg.y = 0;
    tg.z = 0;
  }
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

void MPU6050::writeRegisterBit(uint8_t reg, uint8_t pos, bool state) {
  uint8_t value;
  value = readRegister8(reg);
  if (state) value |= (1 << pos);
  else value &= ~(1 << pos);
  writeRegister8(reg, value);
}
