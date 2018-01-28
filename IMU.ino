#include "IMU.h"

IMU orientation;

void setup() {
  Serial.begin(115200);
  orientation.begin();
  delay(100);
  orientation.calibrate();
  delay(100);
  orientation.start();
  
}

void loop() {
  Serial.print("Yaw: ");
  Serial.print(orientation.yaw());
  Serial.print("\tPitch: ");
  Serial.print(orientation.pitch());
  Serial.print("\tRoll: ");
  Serial.println(orientation.roll());

  
  
}
