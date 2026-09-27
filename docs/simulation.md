# Simulation

Gazebo Harmonic (gz-sim 8) through ros_gz. `ros2 launch ackermann_gazebo sim.launch.py`,
`gui:=true` for the window. The car spawns on the start mark of the room.

World: `scripts/map_to_world.py` turns `maps/room` into `worlds/room.sdf`, 505 boxes 0.5 m
high. Lidar: gpu_lidar as the A1, 6.8 Hz, 1442 points, 1 cm noise.

## sim_car

A Gazebo system plugin in place of the ESP32 and the car, same topics as the firmware:
`/drive` and `/arm` in, `/joint_states`, `/imu/data_raw` and `/steering_angle` out at 50 Hz,
stamped with sim time. Kept from the firmware: soft stop at 2 m/s² after 0.5 s without
`/drive`, arming zeroes the setpoint, duty 0 when disarmed, 45 ms EMA on the wheel speed, the IMU
covariances. `/ground_truth`: the true pose and velocities of base_footprint in the map.

Dynamics: the Phase 5 model (`scripts/vehicle_model.py`), parameters read from
`vehicle_params.yaml`, RK4 at the 1 ms world step, pose written to the model every step.
Wheel speed follows the setpoint with the measured lags (15-45 ms). At duty 0 it decays
with 78 ms: the BTS7960 brakes the motors, fitted on 3 Pure Pursuit stops. Below 0.1 m/s
the tire model is singular, so it switches to the kinematic bicycle. The car has no
collisions.

IMU noise as on tiles in the Pure Pursuit runs: gyro 0.0014 rad/s at rest and 0.063 v moving,
accelerometer 0.07 m/s² at rest and 3.0 v moving (vibration).

Floor friction: parameter `mu_scale` (0.1-2.0, default 1.0 = the tiles), also while driving:
`ros2 param set /sim_car mu_scale 0.5`. The peak tire force scales with it, the stiffness at
small slip does not (brush model): mu -> mu_scale mu, B -> B / mu_scale.

## Open loop

The /drive of the sysid runs replayed (`scripts/sim_replay.py`), yaw rate against the real run:

| run | yaw rms [rad/s] | NRMSE | Phase 5 fit, measured v_x |
|---|---|---|---|
| id_steps | 0.094 | 0.13 | 0.088 |
| val_drift | 0.48 | 0.17 | 0.79 |

The plugin matches the Python model to 0.016 rad/s rms, the simulated gyro noise. In steady
turns the model gives 6-9% less yaw rate than the car. Real time factor 1.0 on the desktop.

Drift, steady donut at 1.3 m/s, sideslip from /ground_truth against the video:

| | yaw rate [rad/s] | beta CG [deg] | beta rear [deg] | wheel speed [m/s] |
|---|---|---|---|---|
| real, id / val | 3.56 / 3.54 | -21 / -18 | -44 / -41 | 1.12 |
| sim | 4.07 | -24 | -50 | 1.20 |

Same donut, deeper: the sim wheels spin at the 1.20 m/s free speed, the real ones at 1.12 under
the load of the slide. The model has no motor torque limit.

## Laps

The Phase 7 session with the same nodes and parameters, started in the order used on the Pi:

    ros2 launch ackermann_gazebo sim.launch.py
    ros2 launch ackermann_bringup robot.launch.py use_sim_time:=true use_lidar:=false use_camera:=false
    ros2 launch ackermann_bringup localization.launch.py use_sim_time:=true
    ros2 topic pub -w 1 -t 10 -r 10 /arm std_msgs/msg/Bool "{data: true}"
    ros2 launch ackermann_bringup pure_pursuit.launch.py use_sim_time:=true speed_scale:=0.6

`notebooks/simulation.ipynb`, 3 laps each, lateral error measured against the localization as
on the real runs, and in the sim against the truth:

| | speed | laps [s] | error rms [cm] | true error rms [cm] | localization rms [cm] |
|---|---|---|---|---|---|
| real | 0.6 | 17.1, 17.2 | 1.8 | | |
| sim | 0.6 | 16.9, 16.7 | 3.6 | 4.1 | 6.2 |
| real | 1.0 | 10.2, 10.1 | 2.7 | | |
| sim | 1.0 | 10.1, 10.2 | 4.4 | 4.7 | 5.7 |

Lap times, AMCL (6.8 Hz, same spread) and the stop past the line (9-21 cm, real 10-15) match.
The sim runs 3-5 cm wide in the corners, where it turns 4-6% less at the same command: the
open loop deficit above.

## Friction

val_drift replayed at other `mu_scale`, the steady donut at 1.3 m/s:

| mu_scale | 1.15 | 1.00 | 0.85 | 0.70 |
|---|---|---|---|---|
| beta CG [deg] | -18 | -24 | -30 | -37 |
| beta rear [deg] | -44 | -50 | -55 | -60 |

At 0.70 the car already lets go at 0.9 m/s. Pure Pursuit at full speed, the profile of the tiles:

| mu_scale | 1.00 | 0.60 | 0.45 | 0.35 | 0.25 | 0.20 |
|---|---|---|---|---|---|---|
| true error rms [cm] | 4.7 | 4.8 | 6.0 | 7.6 | spins, lap 1 | spins, lap 1 |
| rear slip max [deg] | 3.5 | 6.1 | 8.4 | 13 | 62 | 62 |
