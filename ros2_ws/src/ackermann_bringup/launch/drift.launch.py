import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from ackermann_nmpc import drift


def drift_node(context):
    # the drift equilibrium for the steering and wheel speed asked, and its circle (drift.py);
    # the friction from the drift
    params_file = os.path.join(get_package_share_directory('ackermann_description'), 'config', 'vehicle_params.yaml')
    p = yaml.safe_load(open(params_file))['/**']['ros__parameters']
    d = np.radians(float(LaunchConfiguration('steering').perform(context)))
    u = float(LaunchConfiguration('wheel_speed').perform(context))
    mu = float(LaunchConfiguration('mu_scale').perform(context))
    vx, vy, r = drift.equilibrium(p, d, u, mu)
    radius, heading = drift.circle(p, vx, vy, r)
    return [Node(
        package='ackermann_nmpc',
        executable='drift_node',
        name='drift',
        output='screen',
        parameters=[params_file, {
            'cone_x': float(LaunchConfiguration('cone_x').perform(context)),
            'cone_y': float(LaunchConfiguration('cone_y').perform(context)),
            'launch_speed': float(LaunchConfiguration('launch_speed').perform(context)),
            'hold': float(LaunchConfiguration('hold').perform(context)),
            'mu_scale': mu,
            'drift_radius': float(radius), 'drift_heading': float(heading),
            'drift_vx': float(vx), 'drift_vy': float(vy), 'drift_r': float(r),
            'drift_steering': float(d), 'drift_wheel_speed': u}],
    ), Node(
        package='ackermann_nmpc',
        executable='friction_node.py',
        name='friction',
        output='screen',
        parameters=[{'drift_radius': float(radius), 'mu_scale': mu}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('steering', default_value='13.0'),     # deg, the drift of drift.ipynb
        DeclareLaunchArgument('wheel_speed', default_value='1.12'),  # m/s
        DeclareLaunchArgument('mu_scale', default_value='1.0'),      # floor friction the NMPC assumes
        # 0.8 m ahead of the start mark and 0.29 m to its left: the run-up is the circle's tangent
        DeclareLaunchArgument('cone_x', default_value='3.774'),
        DeclareLaunchArgument('cone_y', default_value='-0.023'),
        DeclareLaunchArgument('launch_speed', default_value='0.8'),
        DeclareLaunchArgument('hold', default_value='10.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),   # true in the sim
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        OpaqueFunction(function=drift_node),
    ])
