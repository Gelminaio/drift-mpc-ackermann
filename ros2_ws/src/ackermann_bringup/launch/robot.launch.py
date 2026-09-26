from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('ackermann_bringup')
    description = FindPackageShare('ackermann_description')
    params = PathJoinSubstitution([description, 'config', 'vehicle_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_odometry', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([description, 'launch', 'description.launch.py'])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, 'launch', 'lidar.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_lidar')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, 'launch', 'camera.launch.py'])),
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),

        # the EKF owns odom -> base_footprint, so the odometry node does not publish it
        Node(
            package='ackermann_odometry',
            executable='odometry_node',
            name='ackermann_odometry',
            output='screen',
            parameters=[params, {'publish_tf': False}],
            condition=IfCondition(LaunchConfiguration('use_odometry')),
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[PathJoinSubstitution([bringup, 'config', 'ekf.yaml'])],
            condition=IfCondition(LaunchConfiguration('use_odometry')),
        ),
    ])