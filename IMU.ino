#include "IMU.h"

IMU orientation;

void setup() {
  Serial.begin(115200);
  if (!orientation.check()) while(1) Serial.println("IMU not found!");
  orientation.begin();
  delay(100);
  orientation.calibrate();
  delay(100);
  orientation.start();
  Serial.println("All ok! Starting...");
}

void loop() {
  Serial.print("Yaw: ");
  Serial.print(orientation.yaw());
  Serial.print("\tPitch: ");
  Serial.print(orientation.pitch());
  Serial.print("\tRoll: ");
  Serial.println(orientation.roll());



}
