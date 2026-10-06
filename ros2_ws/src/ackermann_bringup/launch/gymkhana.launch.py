from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# the gymkhana: the NMPC from the start mark round the ring with the slalom to the corner after it, the donut there,
# the NMPC back to the start mark, the parking. Each step only when the one before reached its end (exit code 0).
# The car armed before (ros2 topic pub -w 1 -t 10 /arm ...), localization running.
def generate_launch_description():
    bringup = FindPackageShare('ackermann_bringup')
    params = PathJoinSubstitution([FindPackageShare('ackermann_description'), 'config', 'vehicle_params.yaml'])

    def nmpc(path):
        return Node(
            package='ackermann_nmpc', executable='nmpc_node', name='nmpc', output='screen',
            parameters=[params,
                        {'track_file': PathJoinSubstitution([bringup, 'maps', f'room_{path}.csv']), 'open_path': True,
                         'speed_scale': ParameterValue(LaunchConfiguration('speed_scale'), value_type=float),
                         'mu_scale': ParameterValue(LaunchConfiguration('mu_scale'), value_type=float)}])

    def gap(donut):
        return Node(
            package='ackermann_nmpc', executable='gap_node', name='gap', output='screen',
            parameters=[params,
                        {name: ParameterValue(LaunchConfiguration(name), value_type=float)
                         for name in ['kick_ahead', 'kick_left', 'sweep', 'slide_turn']},
                        {'donut_turns': ParameterValue(LaunchConfiguration('donut_turns'), value_type=int) if donut else 0,
                         'donut_exit': 0.0, 'donut_only': donut}])

    out, donut, back, park = nmpc('gymkhana'), gap(True), nmpc('gymkhana_back'), gap(False)

    def then(action):
        def on_exit(event, context):
            return [action] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='stopped short'))]
        return on_exit

    return LaunchDescription([
        DeclareLaunchArgument('speed_scale', default_value='1.0'),
        DeclareLaunchArgument('mu_scale', default_value='1.0'),        # floor friction the NMPC assumes
        DeclareLaunchArgument('donut_turns', default_value='1'),
        DeclareLaunchArgument('kick_ahead', default_value='0.30'),
        DeclareLaunchArgument('kick_left', default_value='0.42'),
        DeclareLaunchArgument('sweep', default_value='0.15'),          # the tail swept nearer box B on this floor (gymk_15-23)
        DeclareLaunchArgument('slide_turn', default_value='36.4'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),  # true in the sim
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        out,
        RegisterEventHandler(OnProcessExit(target_action=out, on_exit=then(donut))),
        RegisterEventHandler(OnProcessExit(target_action=donut, on_exit=then(back))),
        RegisterEventHandler(OnProcessExit(target_action=back, on_exit=then(park))),
        RegisterEventHandler(OnProcessExit(target_action=park, on_exit=[EmitEvent(event=Shutdown())])),
    ])
