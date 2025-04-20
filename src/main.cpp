#include <Arduino.h>
#include "sensors.hpp"
#include "utils.hpp"

#define LED_PIN 8
#define TEMP_THRESHOLD_HIGH 28.0 // Temperature threshold for LED indication
#define TEMP_THRESHOLD_LOW 27.9  

bool ledState = false; // LED state

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);

  initSensors(); // Initialize sensors
  calibrateSensors(); // Calibrate sensors
}

void loop() {
  GyroData gyroData = readGyro();         // in °/s
  MagData magData = readMagnetometer();  // raw 

  float smoothedTemp = getSmoothedTemperature(gyroData.temperature);

  if (!ledState && smoothedTemp > TEMP_THRESHOLD_HIGH) {
    digitalWrite(LED_PIN, HIGH);
    ledState = true;
  } else if (ledState && smoothedTemp < TEMP_THRESHOLD_LOW) {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  };
   
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