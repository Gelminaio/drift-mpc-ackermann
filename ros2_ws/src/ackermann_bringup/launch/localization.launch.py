from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('ackermann_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),   # true to replay a bag
        DeclareLaunchArgument('map', default_value='room'),             # maps/<map>.yaml
        DeclareLaunchArgument('x', default_value='3.010'),              # the start mark, AMCL's initial pose
        DeclareLaunchArgument('y', default_value='0.350'),
        DeclareLaunchArgument('yaw', default_value='-0.801'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': PathJoinSubstitution([bringup, 'maps', [LaunchConfiguration('map'), '.yaml']])}],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[PathJoinSubstitution([bringup, 'config', 'amcl.yaml']),
                        {f'initial_pose.{k}': ParameterValue(LaunchConfiguration(k), value_type=float) for k in ('x', 'y', 'yaw')}],
        ),

        # map_server and amcl are lifecycle nodes: configure and activate them
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}],
        ),
    ])
