#include <Arduino.h>
#include "sensors.hpp"

void setup() {
  Serial.begin(9600);
}

void loop() {
  Vector3 gyro = readGyro();         // in rad/s
  Vector3 mag = readMagnetometer();  // in µT
   
  // Print gyro data
  Serial.print("Gyro: ");
  Serial.print("X: "); Serial.print(gyro.x, 4);
  Serial.print(" Y: "); Serial.print(gyro.y, 4);
  Serial.print(" Z: "); Serial.println(gyro.z, 4);

  // Print magnetometer data
  Serial.print("Magnetometer: ");
  Serial.print("X: "); Serial.print(mag.x, 4);
  Serial.print(" Y: "); Serial.print(mag.y, 4);
  Serial.print(" Z: "); Serial.println(mag.z, 4);

  delay(1000); // Delay for readability
}