import os

import numpy as np
import yaml

# the car of Phase 5: parameters from vehicle_params.yaml, forces as in system_identification.ipynb
_p = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src',
                                      'ackermann_description', 'config', 'vehicle_params.yaml')))
p = _p['/**']['ros__parameters']
M, IZ, LF, LR, T, G = p['mass'], p['iz'], p['lf'], p['lr'], p['track'], 9.81
L = LF + LR
NF, NR = M * G * LR / L, M * G * LF / L
CMD, DEL = np.array(p['steer_cmd']), np.array(p['steer_angle'])
MU_F, B_F, MU_R, B_R, C, TAU = (p['tire_mu_f'], p['tire_b_f'], p['tire_mu_r'], p['tire_b_r'],
                                p['tire_c'], p['steer_lag'])


def rear_wheel(vx, vy, u, k=1.0):
    # one rear wheel at surface speed u: force against its sliding direction
    sx, sy = (vx - u) / max(u, 0.1), vy / max(u, 0.1)
    s = np.hypot(sx, sy) + 1e-9
    f = k * MU_R * NR / 2 * np.sin(C * np.arctan(B_R / k * s))
    return -f * sx / s, -f * sy / s


def forces(x, u, k=1.0, rear=1.0):
    # body forces and yaw moment at the CG; x = vx, vy, r, steering angle; u = rear wheel speed.
    # Floor friction k times the tiles, as sim_car: the peak scales, the stiffness does not.
    # rear: the rear alone scaled on top (a locked rear slides with more, handbrake.ipynb)
    vx, vy, r, d = x
    vx = max(vx, 0.05)
    fyf = k * MU_F * NF * np.sin(C * np.arctan(B_F / k * (d - np.arctan((vy + LF * r) / vx))))
    fxl, fyl = rear_wheel(vx - T / 2 * r, vy - LR * r, u, k * rear)
    fxr, fyr = rear_wheel(vx + T / 2 * r, vy - LR * r, u, k * rear)
    fx = fxl + fxr - fyf * np.sin(d)
    fy = fyf * np.cos(d) + fyl + fyr
    mz = LF * fyf * np.cos(d) - LR * (fyl + fyr) + T / 2 * (fxr - fxl)
    return fx, fy, mz
