import os

import numpy as np
from acados_template import AcadosOcp

from ackermann_nmpc.model import path_model

# Two problems on the Phase 5 model, same constraints and solver: follow a line at the speed of a
# profile (ocp), or hold a drift on a circle (drift_ocp). No bound on the slip: the tires may
# saturate.
# 1 s ahead in 25 stages, 20 ms near and 53 ms far: 50 stages of 20 ms take 10 ms a solve on the
# Pi, too close to the 20 ms loop; 25 of 40 ms held the line worse at the edge of the grip
TIME_STEPS = np.r_[np.full(10, 0.02), np.full(15, 0.8 / 15)]
N = len(TIME_STEPS)
DT = TIME_STEPS[0]      # s, the control loop (50 Hz)
N_MAX = 0.3     # m off the line, soft
VX_MIN = 0.3    # m/s, soft: below it the tire model is singular
DD_MAX = 4.36   # rad/s, servo rate (vehicle_params steer_rate_max)
DU_MAX = 5.0    # m/s2, wheel speed command

# y = n, e_psi, vx, dd_cmd, du_cmd, against yref = 0, 0, v_ref, 0, 0
W = np.diag([1 / 0.05 ** 2, 1 / 0.1 ** 2, 1 / 0.3 ** 2, 1 / 0.5 ** 2, 1 / 0.5 ** 2])
# y = n, e_psi, vx, vy, r, d_cmd, u_cmd, dd_cmd, du_cmd, against the drift equilibrium
W_DRIFT = np.diag([1 / 0.05 ** 2, 1 / 0.1 ** 2, 1 / 0.1 ** 2, 1 / 0.1 ** 2, 1 / 0.3 ** 2, 1 / 0.2 ** 2,
                   1 / 0.2 ** 2, 1 / 0.5 ** 2, 1 / 0.5 ** 2])


def car(p, code_dir):
    # model, constraints and solver of both problems
    o = AcadosOcp()
    o.model = path_model(p)
    o.code_gen_options.code_export_directory = code_dir
    o.code_gen_options.json_file = os.path.join(code_dir, 'ocp.json')

    # the commands within what the car can do, offset and speed soft
    o.constraints.idxbx = np.array([1, 3, 8, 9])
    o.constraints.lbx = np.array([-N_MAX, VX_MIN, min(p['steer_angle']), 0.0])
    o.constraints.ubx = np.array([N_MAX, 2.0, max(p['steer_angle']), p['v_max']])
    o.constraints.idxsbx = np.array([0, 1])
    o.cost.zl = o.cost.zu = np.array([100.0, 100.0])
    o.cost.Zl = o.cost.Zu = np.array([1000.0, 1000.0])
    o.constraints.idxbu = np.array([0, 1])
    o.constraints.lbu = np.array([-DD_MAX, -DU_MAX])
    o.constraints.ubu = np.array([DD_MAX, DU_MAX])
    o.constraints.x0 = np.zeros(10)
    o.parameter_values = np.array([1.0, 0.0])

    o.solver_options.N_horizon = N
    o.solver_options.tf = TIME_STEPS.sum()
    o.solver_options.time_steps = TIME_STEPS
    # implicit: the front tire makes the model stiff at low speed (1.4 ms at 0.3 m/s)
    o.solver_options.integrator_type = 'IRK'
    o.solver_options.sim_method_num_stages = 2
    o.solver_options.sim_method_num_steps = 1
    # one Gauss-Newton iteration per step, damped by a line search: the full step overshoots
    # where the tires saturate and the commands chatter. Warm start: the previous solution
    o.solver_options.nlp_solver_type = 'SQP'
    o.solver_options.nlp_solver_max_iter = 1
    o.solver_options.globalization = 'MERIT_BACKTRACKING'
    o.solver_options.hessian_approx = 'GAUSS_NEWTON'
    o.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    return o


def ocp(p, code_dir):
    # follow the line (n, e_psi) at the speed of the profile, smooth commands
    o = car(p, code_dir)
    o.cost.cost_type = 'LINEAR_LS'
    o.cost.cost_type_e = 'LINEAR_LS'
    o.cost.W = W
    o.cost.W_e = W[:3, :3] * 10
    o.cost.Vx = np.zeros((5, 10))
    o.cost.Vx[0, 1] = o.cost.Vx[1, 2] = o.cost.Vx[2, 3] = 1
    o.cost.Vu = np.zeros((5, 2))
    o.cost.Vu[3, 0] = o.cost.Vu[4, 1] = 1
    o.cost.Vx_e = o.cost.Vx[:3]
    o.cost.yref = np.zeros(5)
    o.cost.yref_e = np.zeros(3)
    return o


def drift_ocp(p, code_dir):
    # hold a drift on a circle: no offset, the heading error, body velocities and commands of
    # the drift equilibrium (drift.py), smooth commands. yref set by the caller
    o = car(p, code_dir)
    o.cost.cost_type = 'LINEAR_LS'
    o.cost.cost_type_e = 'LINEAR_LS'
    o.cost.W = W_DRIFT
    o.cost.W_e = W_DRIFT[:5, :5] * 10
    o.cost.Vx = np.zeros((9, 10))
    for row, col in enumerate([1, 2, 3, 4, 5, 8, 9]):
        o.cost.Vx[row, col] = 1
    o.cost.Vu = np.zeros((9, 2))
    o.cost.Vu[7, 0] = o.cost.Vu[8, 1] = 1
    o.cost.Vx_e = o.cost.Vx[:5]
    o.cost.yref = np.zeros(9)
    o.cost.yref_e = np.zeros(5)
    return o
