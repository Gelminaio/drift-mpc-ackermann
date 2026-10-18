# Map of the test room

slam_toolbox, online async, on the desktop (`ackermann_bringup/config/slam.yaml`); the Pi
runs `robot.launch.py` (lidar, odometry, robot_localization).

    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<share>/ackermann_bringup/config/slam.yaml
    ros2 run nav2_map_server map_saver_cli -f maps/room
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: ...}"

Driven by hand with teleop_twist_keyboard at ~0.3 m/s, one loop of the room, 15.4 m in
171 s, back to the start (bag `mapping`).

## Numbers

Map 7.1 x 14.9 m at 5 cm, 28 m² free; the room itself ~5 x 4 m.
Back at the start (robot left 5 cm short): the map pose is 7.6 cm from the start pose.
slam moved the EKF pose by 12.5 cm over the loop.

Lidar: RPLIDAR A1 at 6.3-6.9 Hz on the USB adapter (no motor PWM); the driver's
`scan_frequency` only sizes its angle bins, now 6.8 (1442 points per turn, was 1080).

The pose graph (`room.posegraph`, `room.data`, 12 MB) stays out of git, in `data/maps/`.

## The maps of the results

The room changed after this map (things moved, a 1.8 m surface 14-17 cm off) and AMCL ran 12.6 cm off in one
corner. `scripts/map_from_scans.py` builds the map again, same grid and frame, from the lidar paths of driven laps
(`scripts/lidar_path.py`): a cell is a wall where at least 3 beams, and 30% of those reaching it, end in it.
`maps/room` from the twelve gymkhana takes, their boxes and cones masked as floor: the corner 5.6 cm off.
`maps/room2`, the sofa moved for the video, from one lap driven by hand (`remap_1`, 34 m) from the X on the floor
at (3.040, 0.745), 222.6 deg; from `scripts/`:

    python3 lidar_path.py remap_1 48.0 210.1
    python3 map_from_scans.py room2 remap_1 --start 3.040 0.745 222.6
