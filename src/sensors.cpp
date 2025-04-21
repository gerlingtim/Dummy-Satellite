#include "sensors.hpp"

#define ONE_WIRE_BUS 4

MPU6050 gyro;
QMC5883LCompass compass;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);


void initSensors() {
    Wire.begin(); 

    tempSensor.begin(); // Initialize temperature sensor

    // Gyro (MPU6050) Init
    gyro.initialize();
    if (!gyro.testConnection()) {
        Serial.println("MPU6050 connection failed!");
    } else {
        Serial.println("MPU6050 initialized.");
    }

    // Magnetometer (QMC5883L) Init
    compass.init();
    compass.setMode(0x01,0x0C,0x10,0X00); // Set mode, ODR, RNG, OSR
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
    //compass.calibrate(); // Calibrate magnetometer
    
    // Alternative: set calibration values manually
    compass.setCalibrationOffsets(-891.0, -710.0, -272.0); // Set offsets (x, y, z)
    compass.setCalibrationScales(1.1, 0.83, 1.13); // Set scale factors (x, y, z)
    compass.setMagneticDeclination(3, 10); // Set magnetic declination (degrees, minutes) Bonn: +3° 10'
    Serial.println("\n[!] Magnetometer calibration complete.");
}

GyroData readGyro(){
    GyroData data;

    // Gyro data (raw)
    int16_t gx, gy, gz;

    // Accelerometer data (raw)
    int16_t ax, ay, az;

    gyro.getRotation(&gx, &gy, &gz);
    data.gyroX = gx / 131.0; // 131 LSB/(°/s)
    data.gyroY = gy / 131.0; // 131 LSB/(°/s)  
    data.gyroZ = gz / 131.0; // 131 LSB/(°/s)

    gyro.getAcceleration(&ax, &ay, &az);
    data.acclX = ax; 
    data.acclY = ay; 
    data.acclZ = az; 

    data.temperature = gyro.getTemperature() / 340.0 + 35.53; // °C

    return data;
}

MagData readMagnetometer(){
    MagData data;

    compass.read();

    float scale =  (8.0 / 32767.0) * 100.0; // G/LSB * 100

    data.magX = compass.getX() * scale; // µT
    data.magY = compass.getY() * scale; // µT
    data.magZ = compass.getZ() * scale; // µT

    data.azimuth = compass.getAzimuth(); // degrees
   
    return data;
}

float readTemperatureSensor() {
    tempSensor.requestTemperatures(); // Request temperature conversion
    return tempSensor.getTempCByIndex(0); // Get temperature in Celsius
}
