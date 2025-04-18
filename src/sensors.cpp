#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include "sensors.hpp"

MPU6050 gyro;
QMC5883LCompass compass;


void initSensors() {
    Wire.begin(); 

    // MPU6050 Init
    gyro.initialize();
    if (!gyro.testConnection()) {
        Serial.println("MPU6050 connection failed!");
    } else {
        Serial.println("MPU6050 connected.");
    }

    // QMC5883LCompass Init
    compass.init();
    compass.setCalibration(-1227, 328, -2994, 546, -1500, 600); // Werte ggf. anpassen
    Serial.println("QMC5883L initialized.");
}

GyroData readGyro(){
    GyroData data;

    // Gyro data (raw, convert to deg/s if nötig)
    int16_t gx, gy, gz;

    // Accelerometer data (raw, convert to g if nötig)
    int16_t ax, ay, az;

    // Temperature data (raw, convert to °C if nötig)
    int16_t temp;

    gyro.getRotation(&gx, &gy, &gz);
    data.gyroX = gx / 131.0;
    data.gyroY = gy / 131.0;
    data.gyroZ = gz / 131.0;

    gyro.getAcceleration(&ax, &ay, &az);
    data.acclX = ax / 16384.0; // 16384 LSB/g
    data.acclY = ay / 16384.0; // 16384 LSB/g
    data.acclZ = az / 16384.0; // 16384 LSB/g

    data.temperature = gyro.getTemperature() / 340.0 + 36.53; // °C

    return data;
}

MagData readMagnetometer(){
    MagData data;

    compass.read();

    data.magX = compass.getX();
    data.magY = compass.getY();
    data.magZ = compass.getZ();
   
    return data;
}
