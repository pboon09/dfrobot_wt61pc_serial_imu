#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from typing import Optional
import time
from collections import deque

# ROS2 message types
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler

from dfrobot_wt61pc_serial_imu.dfrobot_wt61pc_serial_imu import DFRobotWT61PC, FrequencyConstants

class IMUPublisherNode(Node): 
    def __init__(self):
        super().__init__('imu_publisher_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('sensor_frequency', FrequencyConstants.FREQUENCY_10HZ)
        
        # Gravity compensation parameter
        self.declare_parameter('remove_gravity', False)
        
        # Bias detection parameters
        self.declare_parameter('auto_detect_bias', True)
        self.declare_parameter('bias_calibration_samples', 100)  # Number of samples for bias calculation
        self.declare_parameter('bias_calibration_timeout', 15.0)  # Max time to wait for calibration
        self.declare_parameter('xy_motion_threshold', 0.2)  # Threshold for motion detection
        
        # Covariance parameters - fixed or auto-calculate
        self.declare_parameter('auto_calculate_covariance', True)
        self.declare_parameter('covariance_window_size', 100)
        
        # Fixed covariance values (used when auto_calculate_covariance is False)
        self.declare_parameter('orientation_covariance', [0.01, 0.01, 0.01])
        self.declare_parameter('angular_velocity_covariance', [0.001, 0.001, 0.001])
        self.declare_parameter('linear_acceleration_covariance', [0.1, 0.1, 0.1])
        
        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.sensor_freq = self.get_parameter('sensor_frequency').get_parameter_value().integer_value
        
        self.remove_gravity = self.get_parameter('remove_gravity').get_parameter_value().bool_value
        
        # Bias detection parameters
        self.auto_detect_bias = self.get_parameter('auto_detect_bias').get_parameter_value().bool_value
        self.bias_calibration_samples = self.get_parameter('bias_calibration_samples').get_parameter_value().integer_value
        self.bias_calibration_timeout = self.get_parameter('bias_calibration_timeout').get_parameter_value().double_value
        self.xy_motion_threshold = self.get_parameter('xy_motion_threshold').get_parameter_value().double_value
        
        self.auto_calculate_covariance = self.get_parameter('auto_calculate_covariance').get_parameter_value().bool_value
        self.covariance_window_size = self.get_parameter('covariance_window_size').get_parameter_value().integer_value
        
        # Fixed covariance values
        self.fixed_orientation_cov = self.get_parameter('orientation_covariance').get_parameter_value().double_array_value
        self.fixed_angular_velocity_cov = self.get_parameter('angular_velocity_covariance').get_parameter_value().double_array_value
        self.fixed_linear_acceleration_cov = self.get_parameter('linear_acceleration_covariance').get_parameter_value().double_array_value
        
        # Bias detection state
        self.bias_calibrated = False
        self.bias_calibration_buffer = []
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0
        self.calibration_start_time = None
        
        # Publisher - changed to /imu to match Gazebo
        self.imu_publisher = self.create_publisher(
            Imu, 
            '/imu', 
            10
        )
        
        # Sensor initialization
        self.sensor: Optional[DFRobotWT61PC] = None
        self.init_sensor()
        
        # Timer setup
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Error handling
        self.consecutive_failures = 0
        self.max_failures = 10
        
        # Auto-covariance calculation setup
        if self.auto_calculate_covariance:
            self.orientation_buffer = deque(maxlen=self.covariance_window_size)
            self.angular_velocity_buffer = deque(maxlen=self.covariance_window_size)
            self.linear_acceleration_buffer = deque(maxlen=self.covariance_window_size)
            
            # Initialize with reasonable values
            self.calculated_orientation_cov = [0.01, 0.01, 0.01]
            self.calculated_angular_velocity_cov = [0.001, 0.001, 0.001]
            self.calculated_linear_acceleration_cov = [0.1, 0.1, 0.1]
        
        # Gravity vector (assuming IMU Z-axis points up when robot is level)
        self.gravity_vector = np.array([0.0, 0.0, 9.8])
        
        # Log initialization
        self.get_logger().info(f"IMU Publisher Node started")
        self.get_logger().info(f"Port: {self.port}, Baudrate: {self.baudrate}")
        self.get_logger().info(f"Frame ID: {self.frame_id}, Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Remove gravity: {self.remove_gravity}")
        self.get_logger().info(f"Auto-calculate covariance: {self.auto_calculate_covariance}")
        self.get_logger().info(f"Auto detect bias: {self.auto_detect_bias}")
        if self.auto_detect_bias:
            self.get_logger().info(f"Bias calibration: {self.bias_calibration_samples} samples, {self.bias_calibration_timeout}s timeout")
        self.get_logger().info(f"Publishing to /imu topic")
    
    def init_sensor(self) -> bool:
        try:
            self.sensor = DFRobotWT61PC(port=self.port, baudrate=self.baudrate, timeout=1.0)
            
            if self.sensor.connect():
                self.get_logger().info("Connected to WT61PC sensor")
                
                self.sensor.modify_frequency(self.sensor_freq)
                self.get_logger().info(f"Set sensor frequency to: 0x{self.sensor_freq:02X}")

                time.sleep(1)
                
                # Start bias calibration if enabled
                if self.auto_detect_bias and not self.bias_calibrated:
                    self.start_bias_calibration()
                
                return True
            else:
                self.get_logger().error("Failed to connect to WT61PC sensor")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error initializing sensor: {e}")
            return False
    
    def start_bias_calibration(self):
        """Start the bias calibration process"""
        self.get_logger().info("Starting bias calibration. Keep robot stationary...")
        self.bias_calibration_buffer = []
        self.calibration_start_time = time.time()
        self.bias_calibrated = False
    
    def update_bias_calibration(self, acc, gyro):
        """Update bias calibration with new sample"""
        if self.bias_calibrated or not self.auto_detect_bias:
            return
        
        # Check for timeout
        if time.time() - self.calibration_start_time > self.bias_calibration_timeout:
            self.get_logger().error("Bias calibration timeout! Using zero bias.")
            self.bias_calibrated = True
            return
        
        # Check if we have enough samples
        if len(self.bias_calibration_buffer) >= self.bias_calibration_samples:
            self.finalize_bias_calibration()
            return
        
        # Add sample to calibration buffer
        self.bias_calibration_buffer.append({
            'acc_x': acc.X,
            'acc_y': acc.Y,
            'acc_z': acc.Z,
            'gyro_x': gyro.X,
            'gyro_y': gyro.Y,
            'gyro_z': gyro.Z
        })
        
        # Log progress every 20 samples
        if len(self.bias_calibration_buffer) % 20 == 0:
            self.get_logger().info(f"Bias calibration progress: {len(self.bias_calibration_buffer)}/{self.bias_calibration_samples}")
    
    def finalize_bias_calibration(self):
        """Calculate and set the bias values"""
        if len(self.bias_calibration_buffer) < 10:
            self.get_logger().error("Not enough samples for bias calibration!")
            self.bias_calibrated = True
            return
        
        # Calculate mean values (bias)
        acc_x_samples = [sample['acc_x'] for sample in self.bias_calibration_buffer]
        acc_y_samples = [sample['acc_y'] for sample in self.bias_calibration_buffer]
        acc_z_samples = [sample['acc_z'] for sample in self.bias_calibration_buffer]
        
        self.bias_x = np.mean(acc_x_samples)
        self.bias_y = np.mean(acc_y_samples)
        self.bias_z = np.mean(acc_z_samples) - 9.8  # Remove expected gravity
        
        # Calculate standard deviations for quality check
        std_x = np.std(acc_x_samples)
        std_y = np.std(acc_y_samples)
        std_z = np.std(acc_z_samples)
        
        self.bias_calibrated = True
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("BIAS CALIBRATION COMPLETE")
        self.get_logger().info(f"Samples used: {len(self.bias_calibration_buffer)}")
        self.get_logger().info(f"Detected bias - X: {self.bias_x:.4f}, Y: {self.bias_y:.4f}, Z: {self.bias_z:.4f}")
        self.get_logger().info(f"Standard dev - X: {std_x:.4f}, Y: {std_y:.4f}, Z: {std_z:.4f}")
        
        # Quality check
        if std_x > 0.1 or std_y > 0.1 or std_z > 0.1:
            self.