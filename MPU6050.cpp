#include "MPU6050.h"

MPU6050::MPU6050() {
  FORTHREE drift[i] = 0;
  FORTHREE threshold[i] = 0;
  useCalibrate = false;
  actualThreshold = 0;
}

void MPU6050::begin() {
  dpsPerDigit = .007633f; //gyro scale at 250DPS
  rangePerDigit = .000061f; //accel ramge at 2G
  writeRegister(MPU6050_GYRO_CONFIG,0); //set gyro scale to 250DPS
  writeRegister(MPU6050_ACCEL_CONFIG,0); //set accel range to 2G
  writeRegister(MPU6050_PWR_MGMT, 1); //set the clock to XGyro and turn on
}

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
  Serial.println("whoami done");
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
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.write(reg);
  I2C_2.write(value);
  I2C_2.endTransmission();
}

void MPU6050::readData(uint8_t reg, int16_t *arr) {
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.write(reg);
  I2C_2.endTransmission();
  I2C_2.beginTransmission(MPU6050_ADDRESS);
  I2C_2.requestFrom(MPU6050_ADDRESS, 6);
  while (I2C_2.available() < 6);
  FORTHREE arr[i] = I2C_2.read() << 8 | I2C_2.read();
}
