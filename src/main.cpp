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
   
  float transmissionData[] = {
    gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ,
    gyroData.acclX, gyroData.acclY, gyroData.acclZ,
    gyroData.temperature,
    magData.magX, magData.magY, magData.magZ,
    magData.azimuth
  };
  printCSVWithTimestamp(transmissionData, 11); // Print data with timestamp
  delay(50); // Delay for readability
}