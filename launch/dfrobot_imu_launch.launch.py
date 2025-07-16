from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dfrobot_wt61pc_serial_imu',
            executable='imu_publisher',
            name='imu_publisher',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600,
                'frame_id': 'imu_link',
                'publish_rate': 20.0,
                'sensor_frequency': 0x07,  # 20Hz
                'remove_gravity': True,
                'auto_detect_bias': True,
                'bias_calibration_samples': 150,
                'xy_motion_threshold': 0.15
            }],
            remappings=[
                ('/imu', '/robot/imu')
            ]
        )
    ])