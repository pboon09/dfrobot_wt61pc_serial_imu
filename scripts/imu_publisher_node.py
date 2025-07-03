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
        self.auto_calculate_covariance = self.get_parameter('auto_calculate_covariance').get_parameter_value().bool_value
        self.covariance_window_size = self.get_parameter('covariance_window_size').get_parameter_value().integer_value
        
        # Fixed covariance values
        self.fixed_orientation_cov = self.get_parameter('orientation_covariance').get_parameter_value().double_array_value
        self.fixed_angular_velocity_cov = self.get_parameter('angular_velocity_covariance').get_parameter_value().double_array_value
        self.fixed_linear_acceleration_cov = self.get_parameter('linear_acceleration_covariance').get_parameter_value().double_array_value
        
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
        self.get_logger().info(f"Publishing to /imu topic")
    
    def init_sensor(self) -> bool:
        try:
            self.sensor = DFRobotWT61PC(port=self.port, baudrate=self.baudrate, timeout=1.0)
            
            if self.sensor.connect():
                self.get_logger().info("Connected to WT61PC sensor")
                
                self.sensor.modify_frequency(self.sensor_freq)
                self.get_logger().info(f"Set sensor frequency to: 0x{self.sensor_freq:02X}")

                time.sleep(1)
                
                return True
            else:
                self.get_logger().error("Failed to connect to WT61PC sensor")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error initializing sensor: {e}")
            return False
    
    def _calculate_covariance(self, data_buffer) -> list:
        """Calculate covariance from buffered data"""
        if len(data_buffer) < 10:  # Need minimum samples
            return [0.01, 0.01, 0.01]  # Default values
        
        # Convert to numpy array
        data_array = np.array(list(data_buffer))
        
        # Calculate variance for each axis
        variances = np.var(data_array, axis=0)
        
        # Ensure minimum variance to avoid numerical issues
        variances = np.maximum(variances, 1e-6)
        
        return variances.tolist()
    
    def _update_covariance_buffers(self, acc, gyro, angle):
        """Update covariance calculation buffers"""
        if self.auto_calculate_covariance:
            # Add new data to buffers
            self.orientation_buffer.append([angle.X, angle.Y, angle.Z])
            self.angular_velocity_buffer.append([gyro.X, gyro.Y, gyro.Z])
            self.linear_acceleration_buffer.append([acc.X, acc.Y, acc.Z])
            
            # Calculate new covariances every 10 samples
            if len(self.orientation_buffer) % 10 == 0:
                self.calculated_orientation_cov = self._calculate_covariance(self.orientation_buffer)
                self.calculated_angular_velocity_cov = self._calculate_covariance(self.angular_velocity_buffer)
                self.calculated_linear_acceleration_cov = self._calculate_covariance(self.linear_acceleration_buffer)
    
    def _remove_gravity_from_acceleration(self, acc, angle):
        """Remove gravity from acceleration using current orientation"""
        if not self.remove_gravity:
            return acc
        
        # Convert angles to radians
        roll_rad = math.radians(angle.X)
        pitch_rad = math.radians(angle.Y)
        yaw_rad = math.radians(angle.Z)
        
        # Create rotation matrix to transform gravity vector to IMU frame
        # This is a simplified approach - assumes roll, pitch, yaw rotation order
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)
        cos_pitch = math.cos(pitch_rad)
        sin_pitch = math.sin(pitch_rad)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        
        # Rotation matrix (simplified for gravity removal)
        # This rotates the gravity vector into the IMU frame
        gravity_in_imu_frame = np.array([
            -sin_pitch * self.gravity_vector[2],
            sin_roll * cos_pitch * self.gravity_vector[2], 
            cos_roll * cos_pitch * self.gravity_vector[2]
        ])
        
        # Remove gravity from acceleration
        acc_corrected = type(acc)()
        acc_corrected.X = acc.X - gravity_in_imu_frame[0]
        acc_corrected.Y = acc.Y - gravity_in_imu_frame[1]
        acc_corrected.Z = acc.Z - gravity_in_imu_frame[2]
        
        return acc_corrected
       
    def create_imu_message(self, timestamp) -> Optional[Imu]:
        if not self.sensor:
            return None
            
        if not self.sensor.available():
            return None
            
        acc, gyro, angle = self.sensor.get_all_data()
        
        # Update covariance buffers
        self._update_covariance_buffers(acc, gyro, angle)
        
        # Remove gravity if requested
        acc_processed = self._remove_gravity_from_acceleration(acc, angle)
        
        # Create IMU message
        imu_msg = Imu()
        
        # Header
        imu_msg.header = Header()
        imu_msg.header.stamp = timestamp
        imu_msg.header.frame_id = self.frame_id
        
        # Linear acceleration (gravity removed if requested)
        imu_msg.linear_acceleration.x = acc_processed.X
        imu_msg.linear_acceleration.y = acc_processed.Y
        imu_msg.linear_acceleration.z = acc_processed.Z
        
        # Angular velocity (convert to rad/s)
        imu_msg.angular_velocity.x = math.radians(gyro.X)
        imu_msg.angular_velocity.y = math.radians(gyro.Y)
        imu_msg.angular_velocity.z = math.radians(gyro.Z)
        
        # Orientation (convert to quaternion)
        roll_rad = math.radians(angle.X)
        pitch_rad = math.radians(angle.Y) 
        yaw_rad = math.radians(angle.Z)
        
        quat_array = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        
        imu_msg.orientation.x = quat_array[0]
        imu_msg.orientation.y = quat_array[1]
        imu_msg.orientation.z = quat_array[2]
        imu_msg.orientation.w = quat_array[3]

        # Covariance matrices
        if self.auto_calculate_covariance:
            # Use calculated covariances
            orientation_cov = self.calculated_orientation_cov
            angular_velocity_cov = self.calculated_angular_velocity_cov
            linear_acceleration_cov = self.calculated_linear_acceleration_cov
        else:
            # Use fixed covariances
            orientation_cov = self.fixed_orientation_cov
            angular_velocity_cov = self.fixed_angular_velocity_cov
            linear_acceleration_cov = self.fixed_linear_acceleration_cov
        
        # 3x3 covariance matrices (flattened to 9 elements)
        imu_msg.orientation_covariance = [
            orientation_cov[0], 0.0, 0.0,
            0.0, orientation_cov[1], 0.0,
            0.0, 0.0, orientation_cov[2]
        ]
        
        imu_msg.angular_velocity_covariance = [
            angular_velocity_cov[0], 0.0, 0.0,
            0.0, angular_velocity_cov[1], 0.0,
            0.0, 0.0, angular_velocity_cov[2]
        ]
        
        imu_msg.linear_acceleration_covariance = [
            linear_acceleration_cov[0], 0.0, 0.0,
            0.0, linear_acceleration_cov[1], 0.0,
            0.0, 0.0, linear_acceleration_cov[2]
        ]
        
        return imu_msg
    
    def timer_callback(self):
        try:
            timestamp = self.get_clock().now().to_msg()
            
            imu_msg = self.create_imu_message(timestamp)
            
            if imu_msg is not None:
                self.imu_publisher.publish(imu_msg)
                
                self.consecutive_failures = 0
                
                # Optional: Log data periodically
                # if self.get_clock().now().nanoseconds % 1000000000 < 50000000:
                #     acc, gyro, angle = self.sensor.get_all_data()
                #     self.get_logger().info(
                #         f"IMU Data - Acc: ({acc.X:.2f}, {acc.Y:.2f}, {acc.Z:.2f}), "
                #         f"Gyro: ({gyro.X:.2f}, {gyro.Y:.2f}, {gyro.Z:.2f}), "
                #         f"Angle: ({angle.X:.2f}, {angle.Y:.2f}, {angle.Z:.2f})"
                #     )
            else:
                self.consecutive_failures += 1
                if self.consecutive_failures % 50 == 0:
                    self.get_logger().warn(f"No IMU data available ({self.consecutive_failures} failures)")
                
                if self.consecutive_failures >= self.max_failures:
                    self.get_logger().error("Too many consecutive failures, attempting to reconnect...")
                    self.reconnect_sensor()
                    
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {e}")
            self.consecutive_failures += 1
    
    def reconnect_sensor(self):
        try:
            if self.sensor:
                self.sensor.disconnect()
            
            self.get_logger().info("Attempting to reconnect to sensor...")
            if self.init_sensor():
                self.consecutive_failures = 0
                self.get_logger().info("Successfully reconnected to sensor")
            else:
                self.get_logger().error("Failed to reconnect to sensor")
                
        except Exception as e:
            self.get_logger().error(f"Error during reconnection: {e}")
    
    def destroy_node(self):
        if self.sensor:
            self.sensor.disconnect()
            self.get_logger().info("Disconnected from WT61PC sensor")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        imu_node = IMUPublisherNode()
        rclpy.spin(imu_node)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")

    except Exception as e:
        print(f"Error in main: {e}")
        
    finally:
        if 'imu_node' in locals():
            imu_node.destroy_node()
        
        rclpy.shutdown()
        print("IMU Publisher Node shutdown complete")

if __name__ == '__main__':
    main()