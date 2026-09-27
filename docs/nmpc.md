# NMPC

Package `ackermann_nmpc`. Dynamic NMPC on the Phase 5 model with
[acados](https://docs.acados.org/).

## Setup

acados v0.6.0 from source, in the home directory (no root):

    git clone https://github.com/acados/acados.git ~/acados
    cd ~/acados && git checkout v0.6.0 && git submodule update --recursive --init
    mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make install -j16
    pip install --user --break-system-packages -e ~/acados/interfaces/acados_template
    mkdir -p ~/acados/bin && curl -L -o ~/acados/bin/t_renderer \
        https://github.com/acados/tera_renderer/releases/download/v0.2.0/t_renderer-v0.2.0-linux-amd64
    chmod +x ~/acados/bin/t_renderer

and in the shell that runs it:

    export ACADOS_SOURCE_DIR=$HOME/acados
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib

pip adds only casadi 3.8.1 and Cython to the system numpy 1.26 and scipy 1.11.

## Model

`ackermann_nmpc/model.py`: `scripts/vehicle_model.py` in CasADi, in path coordinates at the CG.

| states | s, n, e_psi | progress, lateral offset (+ left), heading error to the path |
|---|---|---|
| | vx, vy, r | body velocities and yaw rate |
| | d, u | steering angle (0.169 s lag), rear wheel speed (30 ms lag) |
| | d_cmd, u_cmd | the /drive commands: wheel angle, wheel speed |
| inputs | dd_cmd, du_cmd | rates of the commands |
| parameters | mu_scale, kappa | friction against the tiles (brush model, as sim_car), path curvature |

Against the numpy model (`notebooks/nmpc.ipynb`): forces within 1.4e-7 N, 16 s of val_drift
through the acados integrator within 1e-7 m, on a straight path and on a circle.
