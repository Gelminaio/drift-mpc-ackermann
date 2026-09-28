# NMPC

Package `ackermann_nmpc`. Dynamic NMPC on the Phase 5 model with
[acados](https://docs.acados.org/).

## Setup

acados v0.6.0 from source, in the home directory (no root):

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

`ackermann_nmpc/ocp.py`: 50 stages of 20 ms, 1 s ahead.

| cost | n 5 cm, e_psi 0.1 rad, speed against the profile 0.3 m/s, command rates 0.5 |
|---|---|
| hard | d_cmd within the steering map, u_cmd 0-1.2 m/s, rates 4.36 rad/s and 5 m/s² |
| soft | \|n\| < 0.3 m, vx > 0.3 m/s (the tire model is singular at rest) |
| integrator | implicit Gauss-Legendre, 1 step a stage: the front tire is stiff at low speed (1.4 ms at 0.3 m/s), explicit RK4 needs < 3.8 ms steps there |
| solver | 1 Gauss-Newton SQP iteration per control step with a merit line search, HPIPM, warm start shifted by a stage |

No bound on the slip: the tires may saturate. Tracking the CG with e_psi = 0 instead made the
NMPC lock the rear wheels to rotate the car (the CG has 5-7 deg of sideslip in a corner).

## Closed loop in Python

The Phase 5 car with the true state, no delay, against the Phase 7 Pure Pursuit. Speed profile
recomputed for the floor at a fraction f of its grip (0.276 g times mu_scale). Rear axle off the
line, rms (max), cm:

| mu_scale | f | Pure Pursuit | NMPC |
|---|---|---|---|
| 1 (tiles) | Phase 7 profile | 3.2 (6.8) | 1.4 (2.7) |
| 0.35 | 0.6 / 1.0 / 1.2 | 2.2 / 5.0 / 5.8 (15) | 1.2 / 1.0 / 1.3 (3.5) |
| 0.25 | 0.6 / 1.0 / 1.2 | 1.7 / 5.4 / spins | 1.1 / 0.8 / 1.1 (2.6) |

At 120% of the grip the NMPC gives up 7% of the lap time and lets the rear slip 12 deg. With
the plain real time iteration (full step, no line search) at f 1.0: steering chatter on 63% of
the steps, 28-38 deg of rear slip, 5-7 cm rms. Solve time 1.0-1.1 ms mean, 2.5 ms max, desktop.

## Node

`src/nmpc_node.cpp`, 50 Hz, publishes /drive and `nmpc/solve_time`. CMake runs
`scripts/generate.py` at configure time: acados generates the solver in C from `ocp.py` with the
numbers of `vehicle_params.yaml` built in, so a change there rebuilds it. RPATH to the acados
libraries, no LD_LIBRARY_PATH at run time; ACADOS_SOURCE_DIR is needed to build.

Measured: rear axle pose (map -> base_footprint), vx and yaw rate (/odometry/filtered), wheel
speed (/joint_states). Lateral velocity and steering angle come from the previous solve, one step
ahead. Stops like the Pure Pursuit node: after the laps, 0.5 m off the line, pose older than 0.2 s.

    ros2 launch ackermann_bringup nmpc.launch.py mu_scale:=0.35 [use_sim_time:=true]

## Gazebo

The session of `docs/simulation.md`, 3 laps on the speed profile of the tiles, the NMPC told the
friction. Against /ground_truth, rms (max), cm:

| mu_scale | profile / grip | Pure Pursuit | NMPC | localization, NMPC |
|---|---|---|---|---|
| 1.00 | ~40% | 4.7 (11) | 5.2 (12) | 5.6 |
| 0.35 | ~105% | 7.6 (28) | 4.4 (11) | 7.2 |
| 0.25 | ~150% | spins, lap 1 | 5.6 (14), 8% slower | 13 |

On the tiles both follow a pose 5.6 cm off the truth: localization, not the controller, sets the
error. Solve time 1.2-1.3 ms mean, 3.3 max after the first call (10 iterations, 15-21 ms).
