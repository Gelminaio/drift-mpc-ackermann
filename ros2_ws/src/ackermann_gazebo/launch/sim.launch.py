from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = FindPackageShare('ackermann_gazebo')
    world = PathJoinSubstitution([gazebo, 'worlds', 'room.sdf'])
    xacro_file = PathJoinSubstitution([FindPackageShare('ackermann_description'), 'urdf', 'ackermann.xacro'])
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),

        # headless server, or server + GUI
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r -s --headless-rendering ', world]}.items(),
                                 condition=UnlessCondition(LaunchConfiguration('gui'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r ', world]}.items(),
                                 condition=IfCondition(LaunchConfiguration('gui'))),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim:=true']), value_type=str),
                         'use_sim_time': True}],
            remappings=[('joint_states', 'unused/joint_states')],
        ),

        # on the start mark of the real room (docs/localization.md)
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-topic', 'robot_description', '-name', 'car',
                       '-x', '3.010', '-y', '0.350', '-z', '0.0', '-Y', '-0.801'],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'config_file': PathJoinSubstitution([gazebo, 'config', 'bridge.yaml']),
                         'use_sim_time': True}],
        ),
    ])
