#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include "sensors.hpp"

MPU6050 gyro;
QMC5883LCompass compass;


void initSensors() {
    Wire.begin(); 

    
}

Vector3 readGyro(){
    Vector3 g;

    g.x = 10.0;
    g.y = 20.0;
    g.z = 30.0;

    return g;
}

Vector3 readMagnetometer(){
    Vector3 m;

    m.x = 40.0;
    m.y = 50.0;
    m.z = 60.0;

    return m;
}
