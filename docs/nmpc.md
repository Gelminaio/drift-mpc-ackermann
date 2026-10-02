# NMPC

Package `ackermann_nmpc`. Dynamic NMPC on the Phase 5 model with
[acados](https://docs.acados.org/).

## Setup

acados v0.6.0 from source, in the home directory (no root). On the Pi: cmake with
`-DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 -DHPIPM_TARGET=GENERIC`, the t_renderer `linux-arm64`, and
scipy from apt (`python3-scipy`): pip's would bring numpy 2 over the numpy of ROS.

    git clone https://github.com/acados/acados.git ~/acados
    cd ~/acados && git checkout v0.6.0 && git submodule update --recursive --init
    mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make install -j16
    pip install --user --break-system-packages -e ~/acados/interfaces/acados_template
    pip install --user --break-system-packages casadi==3.7.2
    mkdir -p ~/acados/bin && curl -L -o ~/acados/bin/t_renderer \
        https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64
    chmod +x ~/acados/bin/t_renderer

and in the shell that runs it:

    export ACADOS_SOURCE_DIR=$HOME/acados
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib

pip adds only casadi (3.7, the version acados 0.6 is tested with) and Cython to the system
numpy 1.26 and scipy 1.11.

## Model

`ackermann_nmpc/model.py`: `scripts/vehicle_model.py` in CasADi. Path coordinates of the rear
axle (base_footprint): when the tires grip it moves along the heading, so e_psi = 0 on the line.
The body dynamics stay at the CG.

| states | s, n, e_psi | progress, lateral offset (+ left), heading error to the path |
|---|---|---|
| | vx, vy, r | body velocities and yaw rate at the CG |
| | d, u | steering angle (0.169 s lag), rear wheel speed (30 ms lag) |
| | d_cmd, u_cmd | the /drive commands: wheel angle, wheel speed |
| inputs | dd_cmd, du_cmd | rates of the commands |
| parameters | mu_scale, kappa | friction against the tiles (brush model, as sim_car), path curvature |

Against the numpy model (`notebooks/nmpc.ipynb`): forces within 1.4e-7 N, 16 s of val_drift
through the acados integrator within 1e-7 m, on a straight path and on a circle.

## OCP

`ackermann_nmpc/ocp.py`: 1 s ahead in 25 stages, 10 of 20 ms then 15 of 53 ms, solved every 20 ms.
`ocp` follows a line, `drift_ocp` holds a drift on a circle (`notebooks/drift.ipynb`).

| cost | n 5 cm, e_psi 0.1 rad, speed against the profile 0.3 m/s, command rates 0.5 |
|---|---|
| hard | d_cmd within the steering map, u_cmd 0-1.2 m/s, rates 4.36 rad/s and 5 m/s² |
| soft | \|n\| < 0.3 m, vx > 0.3 m/s (the tire model is singular at rest) |
| integrator | implicit Gauss-Legendre, 1 step a stage: the front tire is stiff at low speed (1.4 ms at 0.3 m/s), explicit RK4 needs < 3.8 ms steps there |
| solver | 1 Gauss-Newton SQP iteration per control step with a merit line search, HPIPM, warm start the previous solution |

No bound on the slip: the tires may saturate. Tracking the CG with e_psi = 0 instead made the
NMPC lock the rear wheels to rotate the car (the CG has 5-7 deg of sideslip in a corner).

## Closed loop in Python

The Phase 5 car with the true state, no delay, against the Phase 7 Pure Pursuit. Speed profile
recomputed for the floor at a fraction f of its grip (0.276 g times mu_scale). Rear axle off the
line, rms (max), cm:

| mu_scale | f | Pure Pursuit | NMPC |
|---|---|---|---|
| 1 (tiles) | Phase 7 profile | 3.2 (6.8) | 1.6 (3.1) |
| 0.35 | 0.6 / 1.0 / 1.2 | 2.2 / 5.0 / 5.8 (15) | 1.3 / 1.3 / 1.7 (3.5) |
| 0.25 | 0.6 / 1.0 / 1.2 | 1.7 / 5.4 / spins | 1.1 / 1.2 / 1.8 (4.5) |

At 120% of the grip the NMPC gives up 11% of the lap time and lets the rear slip 14-17 deg. With
the plain real time iteration (full step, no line search) at f 1.0: steering chatter on 25-38% of
the steps, 7-9 cm at worst against 3. Solve time 0.5 ms mean, 1.3 ms max, desktop.

## Node

`src/nmpc_node.cpp`, one solve per first stage (20 ms), publishes /drive and `nmpc/solve_time`. CMake runs
`scripts/generate.py` at configure time: acados generates the solver in C from `ocp.py` with the
numbers of `vehicle_params.yaml` built in, so a change there rebuilds it. RPATH to the acados
libraries, no LD_LIBRARY_PATH at run time; ACADOS_SOURCE_DIR is needed to build.

Measured: rear axle pose (the newest AMCL correction on the EKF odometry), vx and yaw rate (/odometry/filtered), wheel
speed (/joint_states). Lateral velocity and steering angle come from the previous solve, one step
ahead. Stops after the laps, 0.5 m off the line, odometry older than 0.2 s or an AMCL correction older
than 0.5 s.

    ros2 launch ackermann_bringup nmpc.launch.py mu_scale:=0.35 [use_sim_time:=true]

## Drift

`src/drift_node.cpp`, the drift of `drift_ocp` around a cone (`notebooks/drift.ipynb`): straight
from the start mark up to 0.8 m/s on the localization, the drift from where the cone is abeam,
zero commands after `hold` s. In the drift the reference is the cone in the scan: at 3.3 rad/s
AMCL is 12 cm rms off in Gazebo. The launch file computes the equilibrium and its circle
(`drift.py`) from the steering and wheel speed.

    ros2 launch ackermann_bringup drift.launch.py steering:=13 wheel_speed:=1.12 [use_sim_time:=true]

Gazebo, the NMPC told mu_scale 1, rear axle from the cone, target 0.289 m:

| reference | floor mu_scale | from the cone [m] | closest [m] | rear slip [deg] | steering [deg] |
|---|---|---|---|---|---|
| localization | 1.00 | 0.238 +- 0.042 | 0.116 | -19.4 | 28.4 |
| cone | 1.00 | 0.288 +- 0.001 | 0.273 | -25.4 | 12.8 |
| cone | 0.95 | 0.282 +- 0.002 | 0.266 | -32.2 | 6.8 |
| cone | 1.05 | 0.300 +- 0.003 | 0.281 | -17.4 | 20.1 |

`scripts/friction_node.py`, launched with it: the floor friction from the drift. On the circle
the state is known (cone: radius and heading error, which is minus the rear slip; gyro, encoders,
steering command), and the model has one steady state there per friction: `drift.friction`
solves the force balances for mu_scale on the mean state from 3 s after the entry, once a second
on `drift/mu`. Gazebo, the NMPC told 1.00: floors 0.90 / 0.95 / 1.00 / 1.05 / 1.10 give
0.897 / 0.950 / 1.001 / 1.049 / 1.094. With /scan held back 0.15 s (`scripts/scan_delay.py`, the
latency of the A1) 1.000 and 0.951, the circle within 1.1 cm.

On the car (step 10.6, runs `drift_car1`-`9`) the node reaches the cone on the line, with the cone
tracked in the rear axle frame from the start, but the drift never held more than 0.5 s. The rear
lets go only on a torque step (full lock, full throttle); open loop the car keeps sliding only at
16 deg and more, on a tight circle (0.19 m of CG radius at 16 deg), and grips again 0.25-0.5 s after
any correction, less steering or less wheel speed. The drift of the table (13-14 deg, 0.27-0.28 m)
is the unstable middle state of the model: the NMPC holds it in Gazebo, not on the car.

## Handbrake

`src/handbrake_node.cpp` (`notebooks/handbrake.ipynb`, `notebooks/parking.ipynb`). From the start mark:
straight on the line of the start (pure pursuit, the offset from the wheels and the gyro heading), a
turn, and at `brake_at` (gyro heading) speed 0: the motor driver locks the rear wheels in 0.35-0.4 s and
the car slides. In the slide, every 20 ms, the Phase 5 model is run to the stop for a steering held
constant, and bisection finds the one that stops at `target`. The locked rear slides with 1.5 times the
friction of a spinning one (0.41 against 0.28); the steering stays between full lock and -0.15
(countersteered further the car swings back 5-12 deg); the slide is aimed 2.6 deg short, since the car
turns the last degrees as it stops (182.6 +- 1.5 deg over 11 turns aimed at 180).

The turn is in grip, steering 0.35-0.45 at 1.0 m/s: at full lock and full throttle the rear lets go
when the floor decides and the stops spread 20 cm sideways.

With `box:=true` the car parks between box A, ahead on the left, and box B beyond it. At rest the side
of A is the nearest line of points left of the path; in the straight the scans, turned by the gyro
heading, follow its far corner, and the turn starts when it is `turn_at` (0.17 m) ahead of the rear
axle, not before 1.6 s from the start (turning while the car still speeds up, the turn is shorter and
less repeatable). Past A, the side of B is the nearest line further out, and the steering of the turn
comes from how far out it is (`stop_y`, `stop_steer`: 0.35 to 0.45 moves the stop 20 cm) to stop
`side_gap` (0.10 m) from it.

    ros2 launch ackermann_bringup handbrake.launch.py box:=true [turn_at:=0.17 side_gap:=0.10] [use_sim_time:=true]

On the tiles, target 180 deg, the stops by the lidar:

| | runs | stop [deg] | stops across [cm] |
|---|---|---|---|
| open loop, full lock, brake at 136 deg | 6 | 177-197 | |
| full lock, turning from box A | 4 | 180.7-186.7 | 20 |
| grip, on the line of the start | 3 | 180.9-183.2 | 2.3 |
| grip, from A and B, the same boxes | 3 | 175.9-181.0 | 1.3 |

With B 10 cm further out the car stopped 9.6 cm further out; the nose 6-10 cm from A, the side 9-15 cm
from B.

## Gap parking

`src/gap_node.cpp` (`notebooks/sideways.ipynb`, `notebooks/identification.ipynb`). Sideways into the gap
between two boxes on the left, from rest. At rest the lidar gives the gap; the car holds the line of the
start at 0.4 m/s and kicks where it will stop with as much room at its nose as beyond its swept tail:
full lock, full throttle, the rear steps out and the car drifts round at 3.4-4.0 rad/s. It brakes so
that the heading where the brake acts plus the turn of the slide at full lock (`slide_turn`, 36.4 deg)
is 180, the command sent when due between the 20 ms steps. The heading now is the gyro's, `gyro_lag`
old, carried on by the refit model.

    ros2 launch ackermann_bringup gap.launch.py [kick_ahead:=0.30 sweep:=0.12] [use_sim_time:=true]

On the tiles, full lock, closest approach of the body to box A (nose side) and B:

| gap | runs | stop [deg] | closest A / B [cm] |
|---|---|---|---|
| 0.60 m | 6 | 171.4-183.8 | 7.1-14.2 / 4.0-9.2 |
| 0.54 m | 5 | 172.2-183.3 | 2.7-10.9 / 6.4-11.6 |
| 0.50-0.52 m | 7 | 176.5-185.5 | 1.9-6.7 / 2.9-8.1 |
| 0.47 m | 9 | 173.2-185.7 | 2.5-6.1 / 1.7-6.8, one touch |

The first runs braked on the model, the later ones on the measured slide. The ESP32 took `/drive`
every 20 ms, one message per pass: the brake acted 10-41 ms after the command, a delay set at each
start. Taking it every 5 ms, 10 +- 3 ms.

What is left is the slide: 29.5 deg on freshly wiped tiles (the touch), 35 or 42 on used ones, the two
modes parting 100 ms after the brake. From the yaw rate the rest of the turn is known to 1.1 deg only
150 ms in, too late for the servo. Held straight the slide turns 27.1 +- 1.8 deg but the body sweeps
5 cm further. One run needs 0.40 +- 0.02 m; ~0.50 m keeps 2-7 cm at each end.

## Gazebo

The session of `docs/simulation.md`, 3 laps on the speed profile of the tiles, the NMPC told the
friction. Against /ground_truth, rms (max), cm:

| mu_scale | profile / grip | Pure Pursuit | NMPC | localization, NMPC |
|---|---|---|---|---|
| 1.00 | ~40% | 4.7 (11) | 5.6 (12) | 6.0 |
| 0.35 | ~105% | 7.6 (28) | 3.3 (8) | 6.4 |
| 0.25 | ~150% | spins, lap 1 | 6.7 (18), 10% slower | 18 |
| tight line, 1.00 | ~100% | 8.0 (21) | 4.8 (12), 15% slower | 4.8 |

On the tiles both follow a pose ~6 cm off the truth: localization, not the controller, sets the
error. Solve time 0.6-0.7 ms mean, 2.2 max after the first call (10 iterations).

## On the Pi

Raspberry Pi 4, the closed loop of `nmpc.ipynb` timed by acados, with the lidar, odometry, EKF
and AMCL running:

| horizon | solve mean / 99% / max [ms] | Gazebo, mu_scale 0.25, cm rms, 3 runs |
|---|---|---|
| 50 x 20 ms, 50 Hz | 10.5 / 15.5 / 16.7 (idle) | 6.2, 8.1, 6.2 |
| 25 x 40 ms, 25 Hz | 5.5 / 8.1 / 9.1 | 14.6, 8.9, 8.7 |
| 10 x 20 + 15 x 53 ms, 50 Hz | 5.6 / 8.5 / 9.6 | 8.3, 7.2, 7.1 |

The NMPC runs on the Pi with the last one: 10 ms at worst in a 20 ms loop. The drift problem
takes the same, 5.5 / 6.3 / 6.4 ms.

## On the car

2026-09-28, localization and control on the Pi, 3 laps at full speed on the tiles
(`nmpc.ipynb`). Rear axle off the line against the localization, rms (max), cm:

| line | Pure Pursuit | NMPC | lap, PP / NMPC | steering rate, PP / NMPC |
|---|---|---|---|---|
| Phase 7 | 3.4 (8.5) | 3.5 (7.1) | 10.1-10.3 / 9.7 s | 0.25 / 0.45 rad/s |
| tight (step 9.4) | 7.1 (19.4) | 2.7 (7.4) | 8.0 / 9.7 s | 0.36 / 0.67 rad/s |

Solve time 8 ms mean, 17 at the 99th percentile, over the 20 ms step once in 1548. Measured with
a tape at the stops, the localization was 8-11 cm behind the car along the line and 2-4 cm to the
side.

AMCL answers 0.15-0.19 s after the start of the scan and dates its correction 0.2 s ahead: the
pose map -> base_footprint ran 0.1-0.2 s old at speed and a slow update stopped a run after 21 s.
The node now takes the newest correction on the newest EKF odometry.

On the tiles the error is the localization's, ~6 cm rms in Gazebo, and the NMPC chases it: it
steers twice as much as Pure Pursuit for the same error. Two fixes tried in Gazebo, both left
out: blending each AMCL correction in over 0.3 s changed nothing (5.5-6.0 cm on the tiles); a
steering rate weight of 0.2 instead of 0.5 steers 20% less but loses the car at mu_scale 0.25,
where the NMPC is worth having.
