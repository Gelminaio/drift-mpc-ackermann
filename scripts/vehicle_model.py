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


# the refit on the lidar paths (identification.ipynb, fit_* in vehicle_params.yaml)
H_CG = p['h_cg']
FIT = dict(mu_f=p['fit_mu_f'], b_f=p['fit_b_f'], mu_r=p['fit_mu_r'], b_r=p['fit_b_r'], lam=p['fit_lambda'],
           locked=p['fit_locked'], locked_y=p['fit_locked_y'], lt_x=p['fit_lt_x'], lt_y=p['fit_lt_y'],
           tau=p['fit_steer_lag'])
STEER_RATE, STEER_DEAD = p['steer_rate_max'], p['steer_dead']


def forces_fit(vx, vy, r, d, u, braked, ax, ay, q=FIT, k=1.0):
    # body forces and yaw moment at the CG, arrays or scalars. ax, ay: the acceleration of the last step for
    # the load transfer. Rear slip ((v_x,i - u) / lambda, v_y,i) / u; braked, the rear friction x locked along
    # the wheel and x locked_y across it. Floor friction k as in forces()
    vx = np.maximum(vx, 0.05)
    dn_x = q['lt_x'] * M * ax * H_CG / L
    dn = q['lt_y'] * M * ay * H_CG / T
    fyf = k * q['mu_f'] * (NF - dn_x) * np.sin(C * np.arctan(q['b_f'] / k * (d - np.arctan((vy + LF * r) / vx))))
    kr = k * np.where(braked, q['locked'], 1.0)
    ky = np.where(braked, q['locked_y'] / q['locked'], 1.0)
    fx = fy = mz = 0.0
    for side, n in ((-1, (NR + dn_x) / 2 - dn), (1, (NR + dn_x) / 2 + dn)):    # left, right
        sx = (vx + side * T / 2 * r - u) / np.maximum(u, 0.1) / q['lam']
        sy = (vy - LR * r) / np.maximum(u, 0.1)
        s = np.hypot(sx, sy) + 1e-9
        f = kr * q['mu_r'] * np.maximum(n, 0) * np.sin(C * np.arctan(q['b_r'] / kr * s))
        fxi, fyi = -f * sx / s, -ky * f * sy / s
        fx, fy, mz = fx + fxi, fy + fyi, mz + side * T / 2 * fxi - LR * fyi
    return fx - fyf * np.sin(d), fy + fyf * np.cos(d), mz + LF * fyf * np.cos(d)


def wheel_accel(u, command):
    # rear wheel speed from the speed command: the speed loop within the motor line; command 0, the shorted motor
    return np.where(command < 0.02, -u / p['wheel_brake'],
                    np.clip(p['wheel_k'] * (np.minimum(command, p['wheel_u']) - u), -p['wheel_d'],
                            p['wheel_a'] * (1 - u / p['wheel_u'])))
