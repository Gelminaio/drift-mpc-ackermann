from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                # the A1 spins at 6.8 Hz on the USB adapter (no motor PWM); the driver sizes
                # its angle bins from this, it does not set the motor speed (issue #49)
                'scan_frequency': 6.8,
            }],
        ),
    ])
