#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from typing import Optional

# ROS2 message types
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class IMUDataCheckerNode(Node):
    def __init__(self):
        super().__init__('imu_data_checker')
        
        # Parameters
        self.declare_parameter('display_rate', 10.0)  # Hz - how often to print data
        self.declare_parameter('precision', 3)  # decimal places
        
        self.display_rate = self.get_parameter('display_rate').get_parameter_value().double_value
        self.precision = self.get_parameter('precision').get_parameter_value().integer_value
        
        # Subscribe to IMU topic
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Timer for periodic display
        self.display_timer = self.create_timer(
            1.0 / self.display_rate,
            self.display_callback
        )
        
        # Store latest IMU data
        self.latest_imu_data: Optional[Imu] = None
        self.data_received = False
        
        # Statistics
        self.message_count = 0
        self.last_display_time = self.get_clock().now()
        
        self.get_logger().info("IMU Data Checker Node started")
        self.get_logger().info(f"Subscribing to /imu topic")
        self.get_logger().info(f"Display rate: {self.display_rate} Hz")
        self.get_logger().info(f"Precision: {self.precision} decimal places")
        self.get_logger().info("=" * 80)
    
    def imu_callback(self, msg: Imu):
        """Store the latest IMU message"""
        self.latest_imu_data = msg
        self.data_received = True
        self.message_count += 1
    
    def extract_covariance_diagonal(self, cov_matrix: list) -> list:
        """Extract diagonal elements from 3x3 covariance matrix (flattened)"""
        if len(cov_matrix) == 9:
            return [cov_matrix[0], cov_matrix[4], cov_matrix[8]]  # xx, yy, zz
        return [0.0, 0.0, 0.0]
    
    def display_callback(self):
        """Display IMU data periodically"""
        if not self.data_received or self.latest_imu_data is None:
            self.get_logger().warn("No IMU data received yet...")
            return
        
        msg = self.latest_imu_data
        
        # Calculate data rate
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_display_time).nanoseconds / 1e9
        data_rate = self.message_count / time_diff if time_diff > 0 else 0.0
        
        # Extract quaternion and convert to Euler angles
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        
        try:
            roll, pitch, yaw = euler_from_quaternion(quat)
            # Convert to degrees
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
        except Exception as e:
            self.get_logger().error(f"Error converting quaternion to euler: {e}")
            roll_deg = pitch_deg = yaw_deg = 0.0
        
        # Extract angular velocity (convert from rad/s to deg/s)
        angular_vel_deg = [
            math.degrees(msg.angular_velocity.x),
            math.degrees(msg.angular_velocity.y),
            math.degrees(msg.angular_velocity.z)
        ]
        
        # Extract linear acceleration
        linear_accel = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        
        # Extract covariance diagonal elements
        orientation_cov = self.extract_covariance_diagonal(msg.orientation_covariance)
        angular_vel_cov = self.extract_covariance_diagonal(msg.angular_velocity_covariance)
        linear_accel_cov = self.extract_covariance_diagonal(msg.linear_acceleration_covariance)
        
        # Calculate magnitude of acceleration (useful for checking gravity removal)
        accel_magnitude = math.sqrt(sum(x**2 for x in linear_accel))
        
        # Format numbers with specified precision
        p = self.precision
        
        # Display header
        print("\n" + "=" * 80)
        print(f"IMU DATA CHECK - Messages: {self.message_count}, Rate: {data_rate:.1f} Hz")
        print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        print(f"Frame ID: {msg.header.frame_id}")
        print("-" * 80)
        
        # Angular Velocity
        print(f"ANGULAR VELOCITY (deg/s):")
        print(f"  Roll Rate (X):  {angular_vel_deg[0]:+{p+7}.{p}f}")
        print(f"  Pitch Rate (Y): {angular_vel_deg[1]:+{p+7}.{p}f}")
        print(f"  Yaw Rate (Z):   {angular_vel_deg[2]:+{p+7}.{p}f}")
        print(f"  Covariance:     [{angular_vel_cov[0]:.{p}f}, {angular_vel_cov[1]:.{p}f}, {angular_vel_cov[2]:.{p}f}]")
        
        print()
        
        # Linear Acceleration
        print(f"LINEAR ACCELERATION (m/s²):")
        print(f"  X: {linear_accel[0]:+{p+7}.{p}f}")
        print(f"  Y: {linear_accel[1]:+{p+7}.{p}f}")
        print(f"  Z: {linear_accel[2]:+{p+7}.{p}f}")
        print(f"  Magnitude: {accel_magnitude:{p+7}.{p}f}")
        print(f"  Covariance: [{linear_accel_cov[0]:.{p}f}, {linear_accel_cov[1]:.{p}f}, {linear_accel_cov[2]:.{p}f}]")
        
        print()
        
        # Orientation (Euler Angles)
        print(f"ORIENTATION (degrees):")
        print(f"  Roll (X):  {roll_deg:+{p+7}.{p}f}")
        print(f"  Pitch (Y): {pitch_deg:+{p+7}.{p}f}")
        print(f"  Yaw (Z):   {yaw_deg:+{p+7}.{p}f}")
        print(f"  Covariance: [{orientation_cov[0]:.{p}f}, {orientation_cov[1]:.{p}f}, {orientation_cov[2]:.{p}f}]")
        
        print()
        
        # Quaternion (for reference)
        print(f"QUATERNION:")
        print(f"  x: {msg.orientation.x:+{p+7}.{p}f}")
        print(f"  y: {msg.orientation.y:+{p+7}.{p}f}")
        print(f"  z: {msg.orientation.z:+{p+7}.{p}f}")
        print(f"  w: {msg.orientation.w:+{p+7}.{p}f}")
        
        print()
        
        # Status indicators
        print(f"STATUS INDICATORS:")
        if accel_magnitude < 1.0:
            print(f"  ✓ Gravity appears to be REMOVED (magnitude < 1.0 m/s²)")
        elif 8.0 < accel_magnitude < 11.0:
            print(f"  ✓ Gravity appears to be PRESENT (magnitude ≈ 9.8 m/s²)")
        else:
            print(f"  ⚠ Unusual acceleration magnitude - check sensor")
        
        # Check if robot is moving (gravity-aware)
        angular_motion = max(abs(x) for x in angular_vel_deg)
        
        # Smart linear motion detection based on gravity presence
        if accel_magnitude < 1.0:
            # Gravity appears to be removed - check all components
            linear_motion = max(abs(x) for x in linear_accel)
            linear_threshold = 0.5  # m/s²
            gravity_status = "removed"
        elif 8.0 < accel_magnitude < 11.0:
            # Gravity appears to be present - check X,Y components only
            linear_motion = max(abs(linear_accel[0]), abs(linear_accel[1]))
            linear_threshold = 0.5  # m/s²
            gravity_status = "present"
        else:
            # Unusual acceleration - use magnitude deviation from expected
            expected_gravity = 9.8
            linear_motion = abs(accel_magnitude - expected_gravity)
            linear_threshold = 1.0  # m/s²
            gravity_status = "unusual"
        
        # Motion detection thresholds
        angular_threshold = 1.0  # deg/s
        
        # Determine motion status
        is_rotating = angular_motion > angular_threshold
        is_accelerating = linear_motion > linear_threshold
        
        if not is_rotating and not is_accelerating:
            print(f"  ✓ Robot appears to be STATIONARY")
        else:
            motion_types = []
            if is_rotating:
                motion_types.append(f"rotating ({angular_motion:.1f}°/s)")
            if is_accelerating:
                motion_types.append(f"accelerating ({linear_motion:.1f}m/s²)")
            print(f"  ➤ Robot appears to be MOVING: {', '.join(motion_types)}")
        
        print(f"  ℹ Motion detection: gravity {gravity_status}, thresholds: {angular_threshold}°/s, {linear_threshold}m/s²")
        
        print("=" * 80)
        
        # Reset counters
        self.message_count = 0
        self.last_display_time = current_time
    
    def destroy_node(self):
        self.get_logger().info("IMU Data Checker Node shutting down")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        imu_check_node = IMUDataCheckerNode()
        rclpy.spin(imu_check_node)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")

    except Exception as e:
        print(f"Error in main: {e}")
        
    finally:
        if 'imu_check_node' in locals():
            imu_check_node.destroy_node()
        
        rclpy.shutdown()
        print("IMU Data Checker Node shutdown complete")

if __name__ == '__main__':
    main()