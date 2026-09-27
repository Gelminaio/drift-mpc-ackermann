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
covariances.

Dynamics: the Phase 5 model (`scripts/vehicle_model.py`), parameters read from
`vehicle_params.yaml`, RK4 at the 1 ms world step, pose written to the model every step.
Wheel speed follows the setpoint with the measured lags (15-45 ms). At duty 0 it decays
with 78 ms: the BTS7960 brakes the motors, fitted on 3 Pure Pursuit stops. Below 0.1 m/s
the tire model is singular, so it switches to the kinematic bicycle. The car has no
collisions.

## Check

The /drive of the sysid runs replayed open loop (`scripts/sim_replay.py`), yaw rate against
the real run:

| run | yaw rms [rad/s] | NRMSE | Phase 5 fit, measured v_x |
|---|---|---|---|
| id_steps | 0.099 | 0.14 | 0.088 |
| val_drift | 0.47 | 0.17 | 0.79 |

The plugin matches the Python model to 0.033 rad/s rms, the simulated gyro noise after a 5
sample mean. In val_drift the sim goes into the donut 1 s early and turns at 4.0 rad/s,
real 3.3-3.9. Real time factor 1.0 on the desktop.
