# The NMPC solver of the car (ackermann_nmpc/ocp.py), generated and compiled with acados exactly as the
# nmpc node uses it, run in closed loop on the same model. Needs acados built (ACADOS_SOURCE_DIR set,
# libacados on LD_LIBRARY_PATH): see docs/nmpc.md. Skipped otherwise.
import os

import numpy as np
import pytest

pytest.importorskip('casadi')
acados = pytest.importorskip('acados_template')
if not os.environ.get('ACADOS_SOURCE_DIR'):
    pytest.skip('acados is not installed (ACADOS_SOURCE_DIR not set)', allow_module_level=True)

from ackermann_nmpc.model import path_model  # noqa: E402
from ackermann_nmpc.ocp import DT, N, ocp  # noqa: E402
import vehicle_model as vm  # noqa: E402


def test_the_nmpc_brings_the_car_back_to_the_line_within_the_loop_period(tmp_path):
    solver = acados.AcadosOcpSolver(ocp(vm.p, str(tmp_path / 'nmpc')), verbose=False)

    # the plant: the same model, integrated over one control period
    sim = acados.AcadosSim()
    sim.model = path_model(vm.p)
    sim.solver_options.T = DT
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 4
    sim.parameter_values = np.array([1.0, 0.0])     # tiles, straight line
    sim.code_gen_options.code_export_directory = str(tmp_path / 'sim')
    plant = acados.AcadosSimSolver(sim, verbose=False)

    # follow a straight line at 0.8 m/s, starting 10 cm to its left
    v = 0.8
    for k in range(N):
        solver.set(k, 'yref', np.array([0.0, 0.0, v, 0.0, 0.0]))
    solver.set(N, 'yref', np.array([0.0, 0.0, v]))
    x = np.array([0.0, 0.10, 0.0, v, 0.0, 0.0, 0.0, v, 0.0, v])

    solve_times = []
    for _ in range(100):    # 2 s at 50 Hz
        solver.set(0, 'lbx', x)
        solver.set(0, 'ubx', x)
        status = solver.solve()
        # one SQP iteration per control step (as on the car): 2 = iteration limit reached, the normal case
        assert status in (0, 2)
        solve_times.append(solver.get_stats('time_tot'))
        x = plant.simulate(x=x, u=solver.get(0, 'u'))

    assert abs(x[1]) < 0.02            # back within 2 cm of the line
    assert abs(x[2]) < 0.05            # and parallel to it
    assert abs(x[3] - v) < 0.05        # at the reference speed
    assert np.median(solve_times) < DT  # a solve fits in the 20 ms loop (on this machine, faster than the Pi)
