#include <Arduino.h>
#include "sensors.hpp"

void printCSVWithTimestamp(float* values, size_t size){
  Serial.print(millis()); // Print timestamp
  Serial.print(",");
  for (size_t i = 0; i < size; ++i) {
    Serial.print(values[i], 3);
    if (i < size - 1) Serial.print(",");
  }
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  initSensors(); // Initialize sensors
  calibrateSensors(); // Calibrate sensors
}

void loop() {
  GyroData gyroData = readGyro();         // in °/s
  MagData magData = readMagnetometer();  // raw 
   
  // Print gyro data
  //Serial.print("Gyro: ");
  //Serial.print("gX: "); Serial.print(gyroData.gyroX);
  //Serial.print(" gY: "); Serial.print(gyroData.gyroY);
  //Serial.print(" gZ: "); Serial.println(gyroData.gyroZ);
//
  //Serial.print("aX: "); Serial.print(gyroData.acclX, 4);
  //Serial.print(" aY: "); Serial.print(gyroData.acclY, 4);
  //Serial.print(" aZ: "); Serial.println(gyroData.acclZ, 4);
//
  //Serial.print("Temperature: "); Serial.println(gyroData.temperature, 4);

  // Print magnetometer data
  //Serial.print("Magnetometer: ");
  //Serial.print("mX: "); Serial.print(magData.magX, 4);
  //Serial.print(" mY: "); Serial.print(magData.magY, 4);
  //Serial.print(" mZ: "); Serial.println(magData.magZ, 4);
  //Serial.print("Azimuth: "); Serial.println(magData.azimuth, 4);
//
  float transmissionData[] = {
    gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ,
    gyroData.acclX, gyroData.acclY, gyroData.acclZ,
    gyroData.temperature,
    magData.magX, magData.magY, magData.magZ,
    magData.azimuth
  };
  printCSVWithTimestamp(transmissionData, 11); // Print data with timestamp
  delay(100); // Delay for readability
}