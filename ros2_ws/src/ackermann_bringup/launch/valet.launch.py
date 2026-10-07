from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# park anywhere: from rest, the boxes found and the approach planned (valet_node), the NMPC on it, the parking from
# where it stopped (straight away when valet_node exits 2: the car already there). Each step only when the one before
# reached its end (exit code 0). The car armed before (ros2 topic pub -w 1 -t 10 /arm ...), localization running
def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('ackermann_description'), 'config', 'vehicle_params.yaml'])
    path_file = '/tmp/valet_path.csv'
    kick = {name: ParameterValue(LaunchConfiguration(name), value_type=float) for name in ['kick_ahead', 'kick_left', 'sweep']}

    plan = Node(
        package='ackermann_nmpc', executable='valet_node.py', name='valet', output='screen',
        parameters=[{'map': PathJoinSubstitution([FindPackageShare('ackermann_bringup'), 'maps', 'room.yaml']),
                     'path_file': path_file}, kick])
    approach = Node(
        package='ackermann_nmpc', executable='nmpc_node', name='nmpc', output='screen',
        parameters=[params, {'track_file': path_file, 'open_path': True}])
    park = Node(
        package='ackermann_nmpc', executable='gap_node', name='gap', output='screen',
        parameters=[params, kick, {'slide_turn': ParameterValue(LaunchConfiguration('slide_turn'), value_type=float)}])

    def then(action, there=None):
        def on_exit(event, context):
            if event.returncode == 0:
                return [action]
            if event.returncode == 2 and there is not None:
                return [there]
            return [EmitEvent(event=Shutdown(reason='stopped short'))]
        return on_exit

    return LaunchDescription([
        DeclareLaunchArgument('kick_ahead', default_value='0.30'),
        DeclareLaunchArgument('kick_left', default_value='0.42'),
        DeclareLaunchArgument('sweep', default_value='0.15'),
        DeclareLaunchArgument('slide_turn', default_value='36.4'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),  # true in the sim
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        plan,
        RegisterEventHandler(OnProcessExit(target_action=plan, on_exit=then(approach, park))),
        RegisterEventHandler(OnProcessExit(target_action=approach, on_exit=then(park))),
        RegisterEventHandler(OnProcessExit(target_action=park, on_exit=[EmitEvent(event=Shutdown())])),
    ])
