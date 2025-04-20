#include <Arduino.h>
#include <DallasTemperature.h>
#include "sensors.hpp"
#include "utils.hpp"

#define LED_PIN_GYRO 8
#define LED_PIN_TEMP 2
#define TEMP_THRESHOLD_HIGH 28.0 // Temperature threshold for LED indication
#define TEMP_THRESHOLD_LOW 27.9

bool ledStateGyro = false; // LED state
bool ledStateTemp = false; // LED state

void setup()
{
  Serial.begin(9600);
  pinMode(LED_PIN_GYRO, OUTPUT);
  pinMode(LED_PIN_TEMP, OUTPUT);

  initSensors();      // Initialize sensors
  calibrateSensors(); // Calibrate sensors
}

void loop()
{
  GyroData gyroData = readGyro();       // in °/s
  MagData magData = readMagnetometer(); // raw

  float temperature = readTemperatureSensor(); // in °C

  if (temperature != DEVICE_DISCONNECTED_C)
  {
    //float smoothedTemp = getSmoothedTemperature(temperature);
    // Check if Temperature is overheating
    if (!ledStateTemp && temperature > TEMP_THRESHOLD_HIGH)
    {
      digitalWrite(LED_PIN_TEMP, HIGH);
      ledStateTemp = true;
    }
    else if (ledStateTemp && temperature < TEMP_THRESHOLD_LOW)
    {
      digitalWrite(LED_PIN_TEMP, LOW);
      ledStateTemp = false;
    };
  }
  else
  {
    Serial.println("\n[!] Temperature Sensor: Device disconnected!");
  }

  float smoothedTempGyro = getSmoothedTemperature(gyroData.temperature);

  // Check if Gyro is overheating
  if (!ledStateGyro && smoothedTempGyro > TEMP_THRESHOLD_HIGH)
  {
    digitalWrite(LED_PIN_GYRO, HIGH);
    ledStateGyro = true;
  }
  else if (ledStateGyro && smoothedTempGyro < TEMP_THRESHOLD_LOW)
  {
    digitalWrite(LED_PIN_GYRO, LOW);
    ledStateGyro = false;
  };

  float transmissionData[] = {
      gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ,
      gyroData.acclX, gyroData.acclY, gyroData.acclZ,
      gyroData.temperature,
      magData.magX, magData.magY, magData.magZ,
      magData.azimuth,
      temperature};
  // printCSVWithTimestamp(transmissionData, 11); // Print data with timestamp
  delay(50); // Delay for readability
}