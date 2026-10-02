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
    gap = Node(
        package='ackermann_nmpc',
        executable='gap_node',
        name='gap',
        output='screen',
        parameters=[
            PathJoinSubstitution([description, 'config', 'vehicle_params.yaml']),
            {name: ParameterValue(LaunchConfiguration(name), value_type=float)
             for name in ['target', 'kick_ahead', 'kick_left', 'side_in', 'sweep', 'slide_ref', 'slide_turn']},
            {'slide_bisect': ParameterValue(LaunchConfiguration('slide_bisect'), value_type=bool)},
            {'donut_turns': ParameterValue(LaunchConfiguration('donut_turns'), value_type=int),
             'donut_exit': ParameterValue(LaunchConfiguration('donut_exit'), value_type=float)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('target', default_value='180.0'),        # deg, where it should stop
        DeclareLaunchArgument('kick_ahead', default_value='0.30'),    # m, stopped car ahead of the kick
        DeclareLaunchArgument('kick_left', default_value='0.42'),      # m, and left of it
        DeclareLaunchArgument('side_in', default_value='0.02'),        # m, car side inside the boxes
        DeclareLaunchArgument('sweep', default_value='0.12'),          # m, tail swung past where it stops
        DeclareLaunchArgument('slide_ref', default_value='0.52'),      # slide steering the brake assumes
        DeclareLaunchArgument('slide_bisect', default_value='false'),  # steering by bisection in the slide
        DeclareLaunchArgument('slide_turn', default_value='36.4'),     # deg turned in the slide, 0: the model
        DeclareLaunchArgument('donut_turns', default_value='0'),       # donut before the parking: turns
        DeclareLaunchArgument('donut_exit', default_value='180.0'),    # deg, its exit against the start
        DeclareLaunchArgument('use_sim_time', default_value='false'),  # true in the sim
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        gap,
        # the run ends with the node
        RegisterEventHandler(OnProcessExit(target_action=gap, on_exit=[EmitEvent(event=Shutdown())])),
    ])
