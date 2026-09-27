import os

import numpy as np
from acados_template import AcadosOcp

from ackermann_nmpc.model import path_model

# Path tracking on the Phase 5 model: follow the line (n, e_psi) at the speed of the profile,
# smooth commands. No bound on the slip: the tires may saturate.
N = 50          # stages
DT = 0.02       # s, as the control loop: horizon 1.0 s
N_MAX = 0.3     # m off the line, soft
VX_MIN = 0.3    # m/s, soft: below it the tire model is singular
DD_MAX = 4.36   # rad/s, servo rate (vehicle_params steer_rate_max)
DU_MAX = 5.0    # m/s2, wheel speed command

# y = n, e_psi, vx, dd_cmd, du_cmd, against yref = 0, 0, v_ref, 0, 0
W = np.diag([1 / 0.05 ** 2, 1 / 0.1 ** 2, 1 / 0.3 ** 2, 1 / 0.5 ** 2, 1 / 0.5 ** 2])


def ocp(p, code_dir):
    o = AcadosOcp()
    o.model = path_model(p)
    o.code_gen_options.code_export_directory = code_dir
    o.code_gen_options.json_file = os.path.join(code_dir, 'ocp.json')
    nx, nu = 10, 2

    o.cost.cost_type = 'LINEAR_LS'
    o.cost.cost_type_e = 'LINEAR_LS'
    o.cost.W = W
    o.cost.W_e = W[:3, :3] * 10
    o.cost.Vx = np.zeros((5, nx))
    o.cost.Vx[0, 1] = o.cost.Vx[1, 2] = o.cost.Vx[2, 3] = 1
    o.cost.Vu = np.zeros((5, nu))
    o.cost.Vu[3, 0] = o.cost.Vu[4, 1] = 1
    o.cost.Vx_e = o.cost.Vx[:3]
    o.cost.yref = np.zeros(5)
    o.cost.yref_e = np.zeros(3)

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
    o.constraints.x0 = np.zeros(nx)
    o.parameter_values = np.array([1.0, 0.0])

    o.solver_options.N_horizon = N
    o.solver_options.tf = N * DT
    # implicit: the front tire makes the model stiff at low speed (1.4 ms at 0.3 m/s)
    o.solver_options.integrator_type = 'IRK'
    o.solver_options.sim_method_num_stages = 2
    o.solver_options.sim_method_num_steps = 1
    # one Gauss-Newton iteration per step, damped by a line search: the full step overshoots
    # where the tires saturate and the commands chatter
    o.solver_options.nlp_solver_type = 'SQP'
    o.solver_options.nlp_solver_max_iter = 1
    o.solver_options.globalization = 'MERIT_BACKTRACKING'
    o.solver_options.hessian_approx = 'GAUSS_NEWTON'
    o.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    return o
