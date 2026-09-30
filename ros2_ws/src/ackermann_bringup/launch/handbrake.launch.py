from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = FindPackageShare('ackermann_description')
    handbrake = Node(
        package='ackermann_nmpc',
        executable='handbrake_node',
        name='handbrake',
        output='screen',
        parameters=[
            PathJoinSubstitution([description, 'config', 'vehicle_params.yaml']),
            {name: ParameterValue(LaunchConfiguration(name), value_type=float)
             for name in ['straight', 'brake_at', 'target', 'locked_friction', 'turn_at', 'turn_steer', 'turn_speed',
                          'brake_vx', 'brake_vy', 'side_gap']},
            {'box': ParameterValue(LaunchConfiguration('box'), value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('straight', default_value='0.6'),         # s at speed before the turn
        DeclareLaunchArgument('brake_at', default_value='150.0'),       # deg
        DeclareLaunchArgument('target', default_value='180.0'),         # deg, where it should stop
        DeclareLaunchArgument('locked_friction', default_value='1.5'),  # rear friction the node assumes
        DeclareLaunchArgument('box', default_value='false'),            # turn from the box on the left
        DeclareLaunchArgument('turn_at', default_value='0.17'),         # m, its far corner from the rear axle
        DeclareLaunchArgument('turn_steer', default_value='0.45'),      # steering command in the turn, without box B
        DeclareLaunchArgument('turn_speed', default_value='1.0'),       # m/s command in the turn
        DeclareLaunchArgument('brake_vx', default_value='0.95'),        # m/s, rear axle at the brake, assumed
        DeclareLaunchArgument('brake_vy', default_value='0.0'),
        DeclareLaunchArgument('side_gap', default_value='0.10'),        # m, car to box B at the stop
        DeclareLaunchArgument('use_sim_time', default_value='false'),   # true in the sim
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        handbrake,
        # the run ends with the node
        RegisterEventHandler(OnProcessExit(target_action=handbrake, on_exit=[EmitEvent(event=Shutdown())])),
    ])
