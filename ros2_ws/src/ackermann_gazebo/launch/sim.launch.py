from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = FindPackageShare('ackermann_gazebo')
    world = PathJoinSubstitution([gazebo, 'worlds', [LaunchConfiguration('world'), '.sdf']])
    xacro_file = PathJoinSubstitution([FindPackageShare('ackermann_description'), 'urdf', 'ackermann.xacro'])
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('world', default_value='room'),      # worlds/<world>.sdf
        DeclareLaunchArgument('x', default_value='3.010'),         # the start mark
        DeclareLaunchArgument('y', default_value='0.350'),
        DeclareLaunchArgument('yaw', default_value='-0.801'),

        # headless server, or server + GUI
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r -s --headless-rendering ', world]}.items(),
                                 condition=UnlessCondition(LaunchConfiguration('gui'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': ['-r ', world]}.items(),
                                 condition=IfCondition(LaunchConfiguration('gui'))),

        # on the start mark (default: the real room's, docs/localization.md). robot_state_publisher
        # runs in robot.launch.py, as on the Pi
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=['-string', Command(['xacro ', xacro_file, ' sim:=true']), '-name', 'car',
                       '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.0',
                       '-Y', LaunchConfiguration('yaw')],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{'config_file': PathJoinSubstitution([gazebo, 'config', 'bridge.yaml']),
                         'use_sim_time': True}],
        ),
    ])
