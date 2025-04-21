# Dummy Satellite Project

A hobby project to showcase programming skills and concepts related to satellite systems. This project demonstrates sensor fusion, attitude monitoring, and embedded systems programming in a fun and educational way.

---

## Features

- **Sensor Fusion**: Combines gyroscope, accelerometer, and magnetometer data using a Kalman filter.
- **3D Visualization**: Displays satellite orientation in real-time using Python and Matplotlib.
- **Embedded Programming**: Arduino-based sensor data acquisition and processing.
- **Heat Warning**: Optical warning system with LEDs if threshold tempreture is exceeded.

---

## Project Structure

Dummy Satellite/ \
├── include/ \
│   ├── sensors.hpp          *# Header for sensors.cpp*    
│   ├── utils.hpp            *# Header for utils.cpp*\
├── src/\
│   ├── main.cpp               *# Arduino code for processing and communication*\
│   ├── sensors.cpp            *# Sensor functions*\
│   ├── utils.hpp              *# Utility functions*\
├── attitude_monitoring.py     *# Python visualization of attitude with Kalman filter*\
├── README.md                  *# Documentation*

---

## How It Works

1. **Microcontroller**: Arduino collects sensor data and sends it via serial communication.
2. **Python Visualization**: `attitude_monitoring.py` applies a Kalman filter and visualizes the satellite's orientation in 3D.

---

## Getting Started

### Prerequisites

- **Hardware**: Arduino-compatible board, Gyroscope and Accelerometer (MPU6050), Magnetometer (QMC5883L), Temperature sensor (DS18B20), LEDs, Wires, Resistors.
- **Software**: I used [PlatformIO](https://platformio.org/), C++-Libraries: `MPU6050`, `OneWire`, `DallasTemperature`, `QMC5883LCOMPASS`, Python 3.x with `numpy`, `matplotlib`, `scipy`, `pyserial`.

### Steps

1. Clone the repository:
   ```bash
   git clone https://github.com/gerlingtim/dummy-satellite.git
   cd dummy-satellite
2. Upload the Arduino code using PlatformIO.

3. Run the Python visualization:
    ```bash 
    python attitude_monitoring.py
## Future Improvements

- Communication via CAN-bus.
- Sending telecommands. 
- Implement an Extended Kalman Filter (EKF).
- Using Quaternions for calculations.
- Add real-time data logging.

## License 

This project is licensed under the MIT License. Feel free to use and modify it!