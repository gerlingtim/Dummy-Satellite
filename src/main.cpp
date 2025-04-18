#include <Arduino.h>
#include "sensors.hpp"

void setup() {
  Serial.begin(9600);
  initSensors(); // Initialize sensors
}

void loop() {
  GyroData gyroData = readGyro();         // in rad/s
  MagData magData = readMagnetometer();  // in µT
   
  // Print gyro data
  Serial.print("Gyro: ");
  Serial.print("gX: "); Serial.print(gyroData.gyroX, 4);
  Serial.print(" gY: "); Serial.print(gyroData.gyroY, 4);
  Serial.print(" gZ: "); Serial.println(gyroData.gyroZ, 4);
  Serial.print("aX: "); Serial.print(gyroData.acclX, 4);
  Serial.print(" aY: "); Serial.print(gyroData.acclY, 4);
  Serial.print(" aZ: "); Serial.println(gyroData.acclZ, 4);

  // Print magnetometer data
  Serial.print("Magnetometer: ");
  Serial.print("mX: "); Serial.print(magData.magX, 4);
  Serial.print(" mY: "); Serial.print(magData.magY, 4);
  Serial.print(" mZ: "); Serial.println(magData.magZ, 4);

  delay(1000); // Delay for readability
}