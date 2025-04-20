#pragma once

struct GyroData {
    float gyroX, gyroY, gyroZ;
    float acclX, acclY, acclZ;
    float temperature;
  };

struct MagData {
    float magX, magY, magZ;
    float azimuth;
  };
  
void initSensors();
void calibrateSensors();
GyroData readGyro();  
MagData readMagnetometer();
float readTemperatureSensor();