from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('ackermann_bringup')
    description = FindPackageShare('ackermann_description')

    return LaunchDescription([
        DeclareLaunchArgument('speed_scale', default_value='1.0'),
        DeclareLaunchArgument('laps', default_value='3'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),   # true in the sim
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        Node(
            package='ackermann_control',
            executable='pure_pursuit_node',
            name='pure_pursuit',
            output='screen',
            parameters=[
                PathJoinSubstitution([description, 'config', 'vehicle_params.yaml']),
                {'track_file': PathJoinSubstitution([bringup, 'maps', 'room_track.csv']),
                 'speed_scale': ParameterValue(LaunchConfiguration('speed_scale'), value_type=float),
                 'laps': ParameterValue(LaunchConfiguration('laps'), value_type=int)},
            ],
        ),
    ])
