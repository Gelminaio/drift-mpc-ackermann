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

Measured: rear axle pose (map -> base_footprint), vx and yaw rate (/odometry/filtered), wheel
speed (/joint_states). Lateral velocity and steering angle come from the previous solve, one step
ahead. Stops like the Pure Pursuit node: after the laps, 0.5 m off the line, pose older than 0.2 s.

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
