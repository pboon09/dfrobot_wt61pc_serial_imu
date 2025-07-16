# DFRobot WT61PC Serial IMU ROS2 Package

This package provides a ROS2 Node for the [DFRobot Serial 6-Axis Accelerometer/Gyroscope (SKU: SEN0386)](https://wiki.dfrobot.com/Serial_6_Axis_Accelerometer_SKU_SEN0386). It publishes standard ROS2 IMU messages with automatic bias calibration, gravity compensation, and real-time covariance calculation.

## Features
- Direct serial communication with DFRobot WT61PC IMU sensor
- ROS2 publisher for standard `sensor_msgs/Imu` messages
- Automatic bias detection and calibration for stationary robot
- Optional gravity compensation using sensor orientation
- Real-time covariance calculation or configurable fixed values
- Configurable data rates up to 200Hz
- Motion threshold filtering for X/Y acceleration
- Robust error handling and automatic reconnection
- Compatible with navigation and SLAM packages

## Hardware Requirements
- DFRobot Serial 6-Axis Accelerometer/Gyroscope (WT61PC)
- USB to TTL converter module (recommended: CH340 CH340G USB to TTL Converter)
- Computer with USB port running ROS2

## Dependencies
- ROS2 (tested on Humble and Iron)
- Python 3.8+
- pyserial
- tf_transformations
- sensor_msgs
- geometry_msgs
- std_msgs

## Hardware Setup and Wiring

### Required Hardware
1. **DFRobot WT61PC IMU Sensor**
   - [Product Page](https://wiki.dfrobot.com/Serial_6_Axis_Accelerometer_SKU_SEN0386)
   - [Datasheet](https://wiki.dfrobot.com/Serial_6_Axis_Accelerometer_SKU_SEN0386)

2. **CH340 CH340G USB to TTL Converter Module** (Recommended)
   - Provides reliable USB to serial communication
   - 3.3V/5V compatible
   - Automatic driver installation on most systems

### Wiring Connections
Connect the IMU sensor to the USB-TTL converter as follows:

| WT61PC Pin | CH340 TTL Pin | Wire Color (typical) |
|------------|---------------|---------------------|
| VCC        | 3.3V or 5V    | Red                 |
| GND        | GND           | Black               |
| TX         | RX            | Yellow/White        |
| RX         | TX            | Green/Blue          |

**Important Notes:**
- The WT61PC operates at both 3.3V and 5V - check your specific module
- Ensure TX/RX are crossed (TX → RX, RX → TX)
- Double-check power supply voltage to avoid damaging the sensor

### Sensor Configuration
The WT61PC sensor supports various output frequencies and can be configured through software commands. The default settings work well for most applications:
- Default baud rate: 9600
- Default output rate: 10Hz
- Data format: Binary protocol with acceleration, gyroscope, and angle data

## Installation

### Installing Python Dependencies
```bash
# Install required Python packages
pip3 install pyserial

# For ROS2 tf_transformations (if not already installed)
sudo apt install ros-$ROS_DISTRO-tf-transformations
```


### Building the Package
```bash
# Create workspace
mkdir -p ~/imu_ws/src
cd ~/imu_ws/src

# Clone the repository (replace with your actual repository)
git clone https://github.com/yourusername/dfrobot_wt61pc_serial_imu.git

# Build the package
cd ~/imu_ws
colcon build --symlink-install --packages-select dfrobot_wt61pc_serial_imu

# Source the workspace
source ~/imu_ws/install/setup.bash
```

## Usage

### Basic Usage
To start the IMU publisher node with default settings:
```bash
ros2 run dfrobot_wt61pc_serial_imu dfrobot_imu_publisher.py --ros-args -p port:=/dev/ttyUSB0
```

This will publish IMU data to the `/imu` topic as `sensor_msgs/Imu` messages.

### Finding Your Serial Port
```bash
# List available serial devices
ls /dev/ttyUSB* /dev/ttyACM*
```

### Parameters
The `imu_publisher` node supports the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | String | `/dev/ttyUSB0` | Serial port of the IMU device |
| `baudrate` | Integer | `9600` | Serial communication baud rate |
| `frame_id` | String | `imu_link` | TF frame ID for IMU measurements |
| `publish_rate` | Double | `10.0` | ROS2 publishing rate in Hz |
| `sensor_frequency` | Integer | `0x06` | IMU sensor output frequency (see frequency constants) |
| `remove_gravity` | Boolean | `False` | Remove gravity from acceleration measurements |
| `auto_detect_bias` | Boolean | `True` | Automatically detect and correct acceleration bias |
| `bias_calibration_samples` | Integer | `100` | Number of samples for bias calibration |
| `bias_calibration_timeout` | Double | `15.0` | Maximum time for bias calibration (seconds) |
| `xy_motion_threshold` | Double | `0.2` | Motion threshold for X/Y acceleration (m/s²) |
| `auto_calculate_covariance` | Boolean | `True` | Auto-calculate covariance from sensor noise |
| `covariance_window_size` | Integer | `100` | Window size for covariance calculation |

#### Frequency Constants
Available sensor frequency settings:
- `0x01`: 0.1Hz
- `0x02`: 0.5Hz  
- `0x03`: 1Hz
- `0x04`: 2Hz
- `0x05`: 5Hz
- `0x06`: 10Hz (default)
- `0x07`: 20Hz
- `0x08`: 50Hz
- `0x09`: 100Hz
- `0x0A`: 125Hz
- `0x0B`: 200Hz

### Advanced Usage Examples

#### High-Rate IMU for SLAM
```bash
ros2 run dfrobot_wt61pc_serial_imu dfrobot_imu_publisher.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p sensor_frequency:=0x08 \
  -p publish_rate:=50.0 \
  -p remove_gravity:=True \
  -p auto_detect_bias:=True
```

#### Custom Frame and Fixed Covariance
```bash
ros2 run dfrobot_wt61pc_serial_imu dfrobot_imu_publisher.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p frame_id:=base_imu \
  -p auto_calculate_covariance:=False \
  -p orientation_covariance:="[0.005, 0.005, 0.01]" \
  -p angular_velocity_covariance:="[0.001, 0.001, 0.002]" \
  -p linear_acceleration_covariance:="[0.05, 0.05, 0.1]"
```

### Launch File Usage
Run the launch file:
```bash
ros2 launch dfrobot_wt61pc_serial_imu dfrobot_imu_launch.launch.py
```

## Calibration and Setup

### Automatic Bias Calibration
The node performs automatic bias calibration when started:

1. **Keep the robot stationary** during the first 10-15 seconds
2. The node collects samples and calculates bias values
3. Calibration status is logged to the console
4. After calibration, the IMU will provide bias-corrected acceleration data

**Calibration Output Example:**
```
[INFO] Starting bias calibration. Keep robot stationary...
[INFO] Bias calibration progress: 20/100
[INFO] Bias calibration progress: 40/100
...
[INFO] ==================================================
[INFO] BIAS CALIBRATION COMPLETE
[INFO] Samples used: 100
[INFO] Detected bias - X: 0.0234, Y: -0.0456, Z: 0.0123
[INFO] Standard dev - X: 0.0123, Y: 0.0098, Z: 0.0145
[INFO] Calibration quality: GOOD
[INFO] ==================================================
```

### Manual Calibration Disable
To disable automatic bias calibration:
```bash
ros2 run dfrobot_wt61pc_serial_imu dfrobot_imu_publisher.py --ros-args \
  -p auto_detect_bias:=False
```
## Troubleshooting

### Common Issues

#### Serial Port Not Found
```bash
# Check if device is connected
lsusb | grep CH340

# Check dmesg for connection messages
dmesg | tail -20

# Verify permissions
ls -la /dev/ttyUSB0
```

#### Permission Denied
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or change permissions temporarily
sudo chmod 666 /dev/ttyUSB0
```

#### No Data Received
1. Check wiring connections (TX/RX crossed)
2. Verify power supply voltage
3. Try different baud rates: 9600, 38400, 115200
4. Check if another process is using the serial port

#### Poor Calibration Quality
- Ensure robot is completely stationary during calibration
- Check for vibrations or movement
- Increase `bias_calibration_samples` parameter
- Verify sensor mounting is secure

#### High Noise in Data
- Check covariance values in published messages
- Ensure good electrical connections
- Consider electromagnetic interference
- Adjust `xy_motion_threshold` parameter

### Diagnostic Commands
```bash
# Monitor IMU messages
ros2 topic echo /imu

# Check publishing rate
ros2 topic hz /imu

# View TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# Plot IMU data
ros2 run rqt_plot rqt_plot /imu/linear_acceleration/x:y:z
```

## Acknowledgments
- Based on the DFRobot WT61PC C++ library
- Compatible with standard ROS2 navigation packages
- 
## Feedback
If you have any feedback, please create an issue and I will answer your questions there.