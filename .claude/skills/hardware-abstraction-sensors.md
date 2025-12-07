name: hardware-abstraction-sensors
description: Expert knowledge of hardware abstraction layers, sensor integration, signal processing, and device drivers for robotics
license: MIT

---

# Hardware Abstraction & Sensors Intelligence

You are an expert in hardware abstraction layers (HAL), sensor integration, signal processing, and device driver development for robotics systems.

## Core HAL Concepts

### 1. Hardware Abstraction Layer Design

**Layered Architecture**
```
┌─────────────────────────────────────────────────┐
│  APPLICATION LAYER (ROS2 Nodes, Controllers)    │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────┴────────────────────────────────┐
│  HAL INTERFACE (Abstract base classes)          │
│  - IMU Interface                                │
│  - Camera Interface                             │
│  - Motor Controller Interface                   │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────┴────────────────────────────────┐
│  HAL IMPLEMENTATIONS (Device-specific)          │
│  - Xsens IMU Driver                             │
│  - Realsense Camera Driver                      │
│  - Dynamixel Motor Driver                       │
└────────────────┬────────────────────────────────┘
                 │
┌────────────────┴────────────────────────────────┐
│  HARDWARE LAYER (Physical devices)              │
│  - I2C/SPI/UART buses                           │
│  - USB interfaces                               │
│  - Ethernet                                     │
└─────────────────────────────────────────────────┘
```

**Abstract Interface Example**
```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple
import numpy as np

@dataclass
class IMUData:
    timestamp: float
    acceleration: np.ndarray  # (3,) in m/s²
    angular_velocity: np.ndarray  # (3,) in rad/s
    orientation: np.ndarray  # (4,) quaternion [w, x, y, z]
    temperature: float  # °C

class IMUInterface(ABC):
    """Abstract interface for IMU sensors"""

    @abstractmethod
    def initialize(self, config: dict) -> bool:
        """Initialize sensor with configuration"""
        pass

    @abstractmethod
    def read(self) -> IMUData:
        """Read current IMU data"""
        pass

    @abstractmethod
    def calibrate(self) -> bool:
        """Perform sensor calibration"""
        pass

    @abstractmethod
    def get_health_status(self) -> dict:
        """Get sensor health diagnostics"""
        pass

    @abstractmethod
    def shutdown(self):
        """Gracefully shutdown sensor"""
        pass

class XsensIMU(IMUInterface):
    """Concrete implementation for Xsens IMU"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.device = None

    def initialize(self, config: dict) -> bool:
        import serial
        try:
            self.device = serial.Serial(self.port, self.baudrate, timeout=1.0)
            # Send initialization commands
            self.device.write(b'GoToConfig\n')
            # Configure output data format
            self.device.write(b'SetOutputConfiguration\n')
            return True
        except Exception as e:
            print(f"Initialization failed: {e}")
            return False

    def read(self) -> IMUData:
        # Parse binary data from sensor
        raw_data = self.device.read(64)  # Read expected packet size
        parsed = self._parse_packet(raw_data)

        return IMUData(
            timestamp=time.time(),
            acceleration=np.array(parsed['accel']),
            angular_velocity=np.array(parsed['gyro']),
            orientation=np.array(parsed['quat']),
            temperature=parsed['temp']
        )

    def _parse_packet(self, raw_data: bytes) -> dict:
        # Device-specific parsing logic
        pass

    def calibrate(self) -> bool:
        # Calibration procedure
        pass

    def get_health_status(self) -> dict:
        return {
            'connected': self.device.is_open,
            'temperature': self._read_temperature(),
            'error_count': self._get_error_count()
        }

    def shutdown(self):
        if self.device and self.device.is_open:
            self.device.close()
```

### 2. Sensor Types and Integration

**IMU (Inertial Measurement Unit)**
- **Sensors**: 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer
- **Interfaces**: I2C, SPI, UART
- **Common Chips**: MPU-6050, BMI088, ICM-20948, Xsens MTi
- **Challenges**: Bias drift, temperature sensitivity, magnetometer interference
- **Calibration**: Gyro bias, accel scale, magnetometer hard/soft iron

**Example: MPU-6050 via I2C**
```python
import smbus2
import time

class MPU6050:
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def __init__(self, bus=1, address=0x68):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        # Wake up MPU-6050
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)

    def read_raw_data(self, register):
        # Read two bytes and combine
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) | low
        # Convert to signed
        if value > 32768:
            value -= 65536
        return value

    def get_accel(self):
        ax = self.read_raw_data(self.ACCEL_XOUT_H) / 16384.0  # ±2g range
        ay = self.read_raw_data(self.ACCEL_XOUT_H + 2) / 16384.0
        az = self.read_raw_data(self.ACCEL_XOUT_H + 4) / 16384.0
        return np.array([ax, ay, az]) * 9.81  # Convert to m/s²

    def get_gyro(self):
        gx = self.read_raw_data(self.GYRO_XOUT_H) / 131.0  # ±250°/s range
        gy = self.read_raw_data(self.GYRO_XOUT_H + 2) / 131.0
        gz = self.read_raw_data(self.GYRO_XOUT_H + 4) / 131.0
        return np.array([gx, gy, gz]) * np.pi / 180.0  # Convert to rad/s
```

**LiDAR**
- **Types**: Rotating (Velodyne), solid-state (Ouster), single-line (RPLIDAR)
- **Interfaces**: Ethernet, USB, UART
- **Output**: Point clouds (x, y, z, intensity)
- **Calibration**: Intrinsic (beam angles, ranges), extrinsic (mount position)

**Example: RPLIDAR A1**
```python
from rplidar import RPLidar

class RPLidarDriver:
    def __init__(self, port='/dev/ttyUSB0'):
        self.lidar = RPLidar(port)

    def start_scan(self):
        self.lidar.start_motor()
        for scan in self.lidar.iter_scans():
            # Each scan is a full 360° rotation
            # scan = [(quality, angle, distance), ...]
            yield self._process_scan(scan)

    def _process_scan(self, scan):
        points = []
        for quality, angle, distance in scan:
            if quality > 10 and distance > 0:  # Filter low-quality points
                # Convert to Cartesian coordinates
                angle_rad = np.deg2rad(angle)
                x = distance * np.cos(angle_rad) / 1000.0  # mm to m
                y = distance * np.sin(angle_rad) / 1000.0
                points.append([x, y])
        return np.array(points)

    def stop(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
```

**Cameras**
- **Types**: RGB, stereo, depth (ToF, structured light), thermal
- **Interfaces**: USB (UVC), CSI, Ethernet (GigE Vision)
- **Common**: RealSense D435, ZED 2, Basler, FLIR
- **Calibration**: Intrinsic (focal length, distortion), extrinsic (pose in world)

**Example: RealSense D435**
```python
import pyrealsense2 as rs
import numpy as np

class RealSenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Align depth to color
        self.align = rs.align(rs.stream.color)

    def start(self):
        self.pipeline.start(self.config)

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        return color_image, depth_image

    def get_point_cloud(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        return vertices

    def stop(self):
        self.pipeline.stop()
```

**Force/Torque Sensors**
- **Measurement**: 3-axis force, 3-axis torque (6-DOF)
- **Interfaces**: Analog (ADC), CAN, EtherCAT
- **Common**: ATI Mini40, Robotiq FT300
- **Calibration**: Zero offset, temperature compensation

**Example: ATI Force/Torque Sensor**
```python
import socket
import struct

class ATIForceTorque:
    def __init__(self, ip='192.168.1.1', port=49152):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(1.0)
        self.address = (ip, port)

        # Calibration matrix (from sensor datasheet)
        self.calibration_matrix = np.array([...])  # 6x6 matrix
        self.bias = np.zeros(6)

    def set_bias(self):
        """Set current reading as zero"""
        readings = []
        for _ in range(100):
            readings.append(self.read_raw())
        self.bias = np.mean(readings, axis=0)

    def read_raw(self):
        """Read raw ADC counts"""
        # Request reading (RDT command)
        self.socket.sendto(b'\x12\x34\x00\x02', self.address)

        # Receive response
        data, _ = self.socket.recvfrom(1024)

        # Parse binary data (device-specific)
        counts = struct.unpack('>6h', data[12:24])  # 6 signed shorts
        return np.array(counts)

    def read(self):
        """Read calibrated force/torque"""
        raw = self.read_raw() - self.bias
        ft = self.calibration_matrix @ raw
        return {
            'force': ft[:3],  # Fx, Fy, Fz in N
            'torque': ft[3:]  # Tx, Ty, Tz in Nm
        }
```

### 3. Signal Processing and Filtering

**Low-Pass Filter (Butterworth)**
```python
from scipy import signal

class LowPassFilter:
    def __init__(self, cutoff_freq, sample_rate, order=4):
        nyquist = sample_rate / 2
        normal_cutoff = cutoff_freq / nyquist
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low')
        self.zi = signal.lfilter_zi(self.b, self.a)

    def filter(self, data):
        filtered, self.zi = signal.lfilter(self.b, self.a, [data], zi=self.zi)
        return filtered[0]
```

**Complementary Filter (IMU Fusion)**
```python
class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha  # Weight for gyro vs accel
        self.angle = 0.0

    def update(self, accel, gyro, dt):
        # Integrate gyroscope
        gyro_angle = self.angle + gyro * dt

        # Calculate angle from accelerometer
        accel_angle = np.arctan2(accel[1], accel[2])

        # Complementary filter
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle

        return self.angle
```

**Kalman Filter (State Estimation)**
```python
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance  # Q
        self.measurement_variance = measurement_variance  # R
        self.estimate = 0.0
        self.estimate_error = 1.0

    def update(self, measurement):
        # Prediction
        prediction_error = self.estimate_error + self.process_variance

        # Update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error

        return self.estimate
```

**Moving Average Filter**
```python
from collections import deque

class MovingAverageFilter:
    def __init__(self, window_size=10):
        self.window = deque(maxlen=window_size)

    def filter(self, value):
        self.window.append(value)
        return sum(self.window) / len(self.window)
```

### 4. Motor Controllers and Actuators

**Dynamixel Servo**
```python
from dynamixel_sdk import *

class DynamixelMotor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=1000000, motor_id=1):
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id

        # Control table addresses (XM430)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_VELOCITY = 128
        self.ADDR_PRESENT_CURRENT = 126

        # Initialize port
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(2.0)  # Protocol version 2.0

        if not self.port_handler.openPort():
            raise Exception("Failed to open port")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise Exception("Failed to set baudrate")

    def enable_torque(self):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1
        )

    def set_position(self, position):
        """Position in encoder ticks (0-4095)"""
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_GOAL_POSITION, position
        )

    def get_position(self):
        result, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
        )
        return result

    def get_velocity(self):
        result, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_VELOCITY
        )
        return result

    def get_current(self):
        result, _, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_CURRENT
        )
        # Convert to mA
        return result * 2.69

    def disable_torque(self):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 0
        )

    def close(self):
        self.disable_torque()
        self.port_handler.closePort()
```

**CAN Bus Motor Controller**
```python
import can

class CANMotorController:
    def __init__(self, channel='can0', bustype='socketcan'):
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)

    def send_position_command(self, motor_id, position):
        # CAN message format (device-specific)
        data = struct.pack('<Bf', 0x01, position)  # Command ID + float position
        msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def read_feedback(self, motor_id, timeout=0.1):
        msg = self.bus.recv(timeout=timeout)
        if msg and msg.arbitration_id == motor_id + 0x100:  # Feedback ID offset
            # Parse feedback (device-specific)
            position, velocity, torque = struct.unpack('<fff', msg.data)
            return {'position': position, 'velocity': velocity, 'torque': torque}
        return None

    def close(self):
        self.bus.shutdown()
```

### 5. Sensor Fusion

**Multi-Sensor Integration**
```python
from scipy.spatial.transform import Rotation

class SensorFusion:
    def __init__(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = Rotation.identity()

    def update_imu(self, accel, gyro, dt):
        # Integrate acceleration (remove gravity)
        gravity = self.orientation.apply([0, 0, 9.81])
        accel_world = self.orientation.apply(accel) - gravity
        self.velocity += accel_world * dt
        self.position += self.velocity * dt

        # Integrate gyroscope for orientation
        rotation_increment = Rotation.from_rotvec(gyro * dt)
        self.orientation = self.orientation * rotation_increment

    def update_lidar(self, position_estimate):
        # Fuse LiDAR localization with IMU dead-reckoning
        alpha = 0.7  # Trust LiDAR more than IMU drift
        self.position = alpha * position_estimate + (1 - alpha) * self.position

    def update_gps(self, lat, lon, alt):
        # Convert GPS to local coordinates and fuse
        local_position = self.gps_to_local(lat, lon, alt)
        self.position = 0.5 * local_position + 0.5 * self.position

    def get_state(self):
        return {
            'position': self.position,
            'velocity': self.velocity,
            'orientation': self.orientation.as_quat()
        }
```

### 6. Real-Time Considerations

**Deterministic Timing**
```python
import time

class RealtimeLoop:
    def __init__(self, frequency=100):
        self.period = 1.0 / frequency
        self.last_time = time.perf_counter()

    def sleep(self):
        current_time = time.perf_counter()
        elapsed = current_time - self.last_time
        sleep_time = self.period - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print(f"Warning: Loop overrun by {-sleep_time*1000:.2f} ms")

        self.last_time = time.perf_counter()

# Usage
loop = RealtimeLoop(frequency=1000)  # 1 kHz
while True:
    # Read sensors
    imu_data = imu.read()

    # Process
    filtered = filter.update(imu_data)

    # Send commands
    motor.set_torque(control_output)

    # Maintain timing
    loop.sleep()
```

**Priority Scheduling (Linux)**
```python
import os
import sched

def set_realtime_priority():
    # Set FIFO scheduling with high priority
    param = os.sched_param(50)  # Priority 1-99
    os.sched_setscheduler(0, os.SCHED_FIFO, param)

# Call at start of real-time thread
set_realtime_priority()
```

### 7. Error Handling and Diagnostics

**Health Monitoring**
```python
class SensorHealthMonitor:
    def __init__(self, sensor, timeout=1.0):
        self.sensor = sensor
        self.timeout = timeout
        self.last_valid_time = time.time()
        self.error_count = 0

    def check_health(self):
        try:
            data = self.sensor.read()

            # Validate data
            if self._is_valid(data):
                self.last_valid_time = time.time()
                self.error_count = 0
                return {'status': 'OK', 'data': data}
            else:
                self.error_count += 1
                return {'status': 'INVALID_DATA', 'error_count': self.error_count}

        except TimeoutError:
            if time.time() - self.last_valid_time > self.timeout:
                return {'status': 'TIMEOUT', 'duration': time.time() - self.last_valid_time}

        except Exception as e:
            self.error_count += 1
            return {'status': 'ERROR', 'exception': str(e), 'error_count': self.error_count}

    def _is_valid(self, data):
        # Check for NaN, out-of-range values, etc.
        if np.any(np.isnan(data.acceleration)):
            return False
        if np.linalg.norm(data.acceleration) > 50:  # Unrealistic accel
            return False
        return True
```

## Best Practices

- [ ] Abstract interfaces for hardware independence
- [ ] Calibration procedures documented and automated
- [ ] Signal filtering matched to application (cutoff frequencies)
- [ ] Error handling for sensor failures
- [ ] Health monitoring and diagnostics
- [ ] Real-time guarantees for control loops
- [ ] Thread-safe access to shared sensor data
- [ ] Graceful degradation when sensors fail
- [ ] Version control for device configurations
- [ ] Unit tests with mock hardware

This skill empowers Claude with expert hardware abstraction and sensor integration knowledge for building robust robotics systems.
