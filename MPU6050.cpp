#include "MPU6050.h"

MPU6050::MPU6050() {
  FORTHREE drift[i] = 0;
  FORTHREE threshold[i] = 0;
  useCalibrate = false;
  actualThreshold = 0;
}

void MPU6050::begin() {
  Wire2.begin();
  dpsPerDigit = .007633f; //gyro scale at 250DPS
  rangePerDigit = .000061f; //accel ramge at 2G
  writeRegister(MPU6050_GYRO_CONFIG,0); //set gyro scale to 250DPS
  writeRegister(MPU6050_ACCEL_CONFIG,0); //set accel range to 2G
  writeRegister(MPU6050_PWR_MGMT, 1); //set the clock to XGyro and turn on
}

bool MPU6050::check() {
  Wire2.begin();

  Wire2.beginTransmission(MPU6050_ADDRESS);
  if (Wire2.endTransmission()) return false;

  Wire2.beginTransmission(MPU6050_ADDRESS);
  Wire2.write(MPU6050_WHO_AM_I);
  Wire2.endTransmission();

  Wire2.beginTransmission(MPU6050_ADDRESS);
  Wire2.requestFrom(MPU6050_ADDRESS, 1);
  uint8_t value = Wire2.read();
  Wire2.endTransmission();
  return value == MPU6050_ADDRESS;
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
  Wire2.beginTransmission(MPU6050_ADDRESS);
  Wire2.write(MPU6050_TEMP);
  Wire2.endTransmission();
  Wire2.beginTransmission(MPU6050_ADDRESS);
  Wire2.requestFrom(MPU6050_ADDRESS, 2);
  while (!Wire2.available());
  temp = Wire2.read() << 8 | Wire2.read();
  Wire2.endTransmission();
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

void MPU6050::writeRegister(uint8_t reg, uint8_t value) {
  Wire2.beginTransmission(MPU6050_ADDRESS);
  Wire2.write(reg);
  Wire2.write(value);
  Wire2.endTransmission();
}

void MPU6050::readData(uint8_t reg, int16_t *arr) {
  Wire2.beginTransmission(MPU6050_ADDRESS);
  Wire2.write(reg);
  Wire2.endTransmission();
  Wire2.beginTransmission(MPU6050_ADDRESS);
  Wire2.requestFrom(MPU6050_ADDRESS, 6);
  while (Wire2.available() < 6);
  FORTHREE arr[i] = Wire2.read() << 8 | Wire2.read();
}
