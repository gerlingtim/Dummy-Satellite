#pragma once

struct GyroData {
    float gyroX, gyroY, gyroZ;
    float acclX, acclY, acclZ;
    float temperature;
  };

struct MagData {
    float magX, magY, magZ;
  };
  
void initSensors();
GyroData readGyro();  
MagData readMagnetometer();