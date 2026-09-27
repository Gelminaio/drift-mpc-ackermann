# Map of the test room

slam_toolbox, online async, on the desktop (`ackermann_bringup/config/slam.yaml`); the Pi
runs `robot.launch.py use_camera:=false` (lidar, odometry, robot_localization).

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
