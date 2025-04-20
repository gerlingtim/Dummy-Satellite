import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
from scipy.spatial.transform import Rotation as R

class AttitudeMonitor:
    def __init__(self, port='COM4', baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.timestamp_old = None

        self.gyro_roll = 0.0
        self.gyro_pitch = 0.0
        self.gyro_yaw = 90.0 # Synchronize with compass yaw

        self.acc_x = None
        self.acc_y = None
        self.acc_z = None
        self.alpha = 0.5

        self.fig = None
        self.ax = None
        self.collection = None

        # For Kalman filter
        self.P = np.eye(3) * 0.1 # Initial covariance matrix 
        self.Q = np.eye(3) * 0.001 # Process noise covariance
        self.R = np.eye(3) * 0.1 # Measurement noise covariance (for roll, pitch, yaw)

    def read_serial_data(self):
        line = self.ser.readline().decode('utf-8', errors="ignore").strip()
        parts = line.split(',')

        if len(parts) != 12:
            print(line)
            return None
        try:
            values = list(map(float, parts))
            if len(values) != 12:
                print(f"Invalid data length in line: {line}")
                return None
            time_ms = int(values[0])
            gyro_g = np.array([values[1], values[2], values[3]])  # Gyroscope data (°/s)
            gyro_a = np.array([values[4], values[5], values[6]])  # Accelerometer data (g)
            temp = values[7]
            compass_m = np.array([values[8], values[9], values[10]])  # Magnetometer data (µT)
            compass_a = values[11]

            return time_ms, gyro_g, gyro_a, compass_m
        except ValueError:
            print(f"Parsing error in line: {line}")
            return None   
        
        return roll, pitch

    def create_box(self, size=(0.5, 1.3, 0.05)):
        l, w, h = size
        x = [-l/2, l/2]
        y = [-w/2, w/2]
        z = [-h/2, h/2]
        vertices = np.array([
            [x[0], y[0], z[0]], [x[1], y[0], z[0]], [x[1], y[1], z[0]], [x[0], y[1], z[0]],
            [x[0], y[0], z[1]], [x[1], y[0], z[1]], [x[1], y[1], z[1]], [x[0], y[1], z[1]],
        ])
        faces = [
            [vertices[j] for j in [0, 1, 2, 3]],  # bottom
            [vertices[j] for j in [4, 5, 6, 7]],  # top
            [vertices[j] for j in [0, 1, 5, 4]],  # front
            [vertices[j] for j in [2, 3, 7, 6]],  # back
            [vertices[j] for j in [1, 2, 6, 5]],  # right
            [vertices[j] for j in [0, 3, 7, 4]],  # left
        ]
        return faces

    def rotate(self, faces, rpy):
        # Apply the rotation to the box
        r = R.from_euler('xyz', rpy, degrees=True)
        return [[r.apply(v) for v in face] for face in faces]

    def acc_to_roll_pitch(self, gyro_a):
        # Convert accelerometer data to roll, pitch, yaw
        self.acc_x = gyro_a[0] * 9.81
        self.acc_y = gyro_a[1] * 9.81
        self.acc_z = gyro_a[2]

        roll = np.arctan2(self.acc_y, self.acc_z)  
        pitch = -np.arctan2(self.acc_x, np.sqrt(self.acc_y**2 + self.acc_z**2))

        return np.rad2deg(roll), np.rad2deg(pitch)
    
    def mag_to_yaw(self, compass_m, roll, pitch):
        mx, my, mz = compass_m[0], compass_m[1], compass_m[2]

        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)

        sin_r = np.sin(roll)
        cos_r = np.cos(roll)
        sin_p = np.sin(pitch)
        cos_p = np.cos(pitch)

        mx2 = mx * cos_p + mz * sin_p
        my2 = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p

        yaw = np.arctan2(-my2, mx2)
        return np.rad2deg(yaw)
    
    def gyro_to_roll_pitch_yaw(self, gyro_g, dt):
        # Convert gyroscope data to roll, pitch, yaw
        gx, gy, gz = gyro_g[0], gyro_g[1], gyro_g[2]
        
        self.gyro_roll -= gx * dt / 1000.0  # Convert ms to seconds
        self.gyro_pitch -= gy * dt / 1000.0
        self.gyro_yaw += gz * dt / 1000.0

        self.P = self.P + self.Q

    
    def update_plot(self, frame):
        data = self.read_serial_data()
        if data is not None:
            time_ms, gyro_g, gyro_a, compass_m = data

            if self.timestamp_old is not None:
                dt = time_ms - self.timestamp_old
            else:
                dt = 50  # Default to 100 ms if no previous timestamp
            
            self.gyro_to_roll_pitch_yaw(gyro_g, dt)
            roll_g, pitch_g, yaw_g = self.gyro_roll, self.gyro_pitch, self.gyro_yaw
            
            roll_a, pitch_a = self.acc_to_roll_pitch(gyro_a)
            yaw_m = self.mag_to_yaw(compass_m, self.gyro_roll, self.gyro_pitch)
            
            z = np.array([roll_a, pitch_a, yaw_m])

            print(f"z: {z}")

            rpy = np.array([roll_g, pitch_g, yaw_g])

            y = z - rpy

            S = self.P + self.R
            K = np.dot(self.P, np.linalg.inv(S))  # Kalman gain

            correction = np.dot(K, y)  # Correction term

            corrected_rpy = rpy + correction  # Corrected roll, pitch, yaw
            roll_corr, pitch_corr, yaw_corr = corrected_rpy[0], corrected_rpy[1], corrected_rpy[2]

            self.P = np.dot(np.eye(3) - K, self.P)  # Update covariance matrix

            self.ax.set_title(f"Roll_g: {roll_corr:.1f}° | Pitch_g: {pitch_corr:.1f}°| Yaw_g: {yaw_corr:.1f}°")
            self.gyro_roll, self.gyro_pitch, self.gyro_yaw = roll_corr, pitch_corr, yaw_corr
                
            rotated_faces = self.rotate(self.create_box(), corrected_rpy)
            self.collection.set_verts(rotated_faces)

            self.timestamp_old = time_ms

    def initialize_plot(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Settings for the plot
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(-0.2, 0.2)
        # Sensors are flipped
        self.ax.invert_zaxis()
        self.ax.invert_xaxis()
        self.ax.set_xticks([-1, 0, 1])
        self.ax.set_yticks([-1, 0, 1])
        self.ax.set_zticks([-0.2, 0, 0.2])
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.view_init(elev=20, azim=10)

        colors = ['red', 'green', 'blue', 'yellow', 'orange', 'cyan']
        faces = self.create_box()
        self.collection = Poly3DCollection(faces, facecolors=colors, linewidths=1, edgecolors='k', alpha=0.9)
        self.ax.add_collection3d(self.collection)

    def run(self):
        print("Waiting for data...")
        self.initialize_plot()
        try:
            ani = animation.FuncAnimation(self.fig, self.update_plot, frames=np.arange(0, 360, 1), interval=50, repeat=True)
            plt.show()
        finally:
            self.ser.close()
            print("Serial port closed.")


if __name__ == "__main__":
    monitor = AttitudeMonitor(port='COM4', baudrate=9600)
    monitor.run()