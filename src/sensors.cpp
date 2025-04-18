#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include "sensors.hpp"

MPU6050 gyro;
QMC5883LCompass compass;


void initSensors() {
    Wire.begin(); 

    // Gyro (MPU6050) Init
    gyro.initialize();
    if (!gyro.testConnection()) {
        Serial.println("MPU6050 connection failed!");
    } else {
        Serial.println("MPU6050 initialized.");
    }

    // Magnetometer (QMC5883L) Init
    compass.init();
    Serial.println("QMC5883L initialized.");

}

void calibrateSensors() {
    // Gyro calibration
    Serial.println("\n[!] Do not move sensor - calibrating Gyro...");
    delay(2000); // Wait for 2 seconds
    gyro.CalibrateGyro(6); // Calibrate gyro
    gyro.CalibrateAccel(6); // Calibrate accelerometer
    Serial.println("\n[!] Gyro calibration complete.");

    // Magnetometer calibration
    Serial.println("\n[!] Move sensor in a figure 8 pattern - calibrating Magnetometer...");
    compass.calibrate(); // Calibrate magnetometer
    Serial.println("\n[!] Magnetometer calibration complete.");
}

GyroData readGyro(){
    GyroData data;

    // Gyro data (raw)
    int16_t gx, gy, gz;

    // Accelerometer data (raw)
    int16_t ax, ay, az;

    // Temperature data (raw)
    int16_t temp;

    gyro.getRotation(&gx, &gy, &gz);
    data.gyroX = gx / 131.0; // 131 LSB/(°/s)
    data.gyroY = gy / 131.0; // 131 LSB/(°/s)  
    data.gyroZ = gz / 131.0; // 131 LSB/(°/s)

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

    float scale =  (8.0 / 32767.0) * 100.0; // G/LSB * 100

    data.magX = compass.getX() * scale; // µT
    data.magY = compass.getY() * scale; // µT
    data.magZ = compass.getZ() * scale; // µT
   
    return data;
}
