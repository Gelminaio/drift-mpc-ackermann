# The model inside the NMPC (ackermann_nmpc/model.py, CasADi) and the drift tools built on it
# (ackermann_nmpc/drift.py). Needs casadi, scipy and the acados Python interface.
import numpy as np
import pytest

ca = pytest.importorskip('casadi')
pytest.importorskip('acados_template')

from ackermann_nmpc import drift  # noqa: E402
from ackermann_nmpc import model as nmpc_model  # noqa: E402
import vehicle_model as vm  # noqa: E402

P = vm.p
D_EQ, U_EQ = 0.20, 1.20     # rad, m/s: steering into a left turn with the rear wheels spinning faster


def residual(vx, vy, r, d, u, mu=1.0):
    # body accelerations of a state with d and u held: zero in a steady state
    fx, fy, mz = vm.forces((vx, vy, r, d), u, k=mu)
    return np.abs([fx / P['mass'] + vy * r, fy / P['mass'] - vx * r, mz / P['iz']]).max()


def test_the_controller_uses_the_identified_car():
    # the NMPC optimises with model.forces (CasADi); identification, notebooks and simulator use
    # vehicle_model.forces (numpy). Same parameters, same formulas: they must give the same forces
    z, mu = ca.SX.sym('z', 5), ca.SX.sym('mu')
    f = ca.Function('f', [z, mu], [ca.vertcat(*nmpc_model.forces(P, z[0], z[1], z[2], z[3], z[4], mu))])
    rng = np.random.default_rng(1)
    for _ in range(300):
        vx, vy, r, d, u = rng.uniform([0.1, -0.6, -6.0, -0.45, 0.1], [1.5, 0.6, 6.0, 0.45, 1.5])
        k = rng.uniform(0.5, 1.2)
        got = np.array(f([vx, vy, r, d, u], k)).ravel()
        np.testing.assert_allclose(got, vm.forces((vx, vy, r, d), u, k=k), rtol=1e-4, atol=1e-4)


def path_derivative(x, kappa=0.0, mu=1.0):
    m = nmpc_model.path_model(P)
    f = ca.Function('f', [m.x, m.u, m.p], [m.f_expl_expr])
    return np.array(f(x, [0.0, 0.0], [mu, kappa])).ravel()


def test_driving_straight_down_the_line_is_an_equilibrium():
    # s, n, e_psi, vx, vy, r, d, u, d_cmd, u_cmd: on the line, at speed, commands held. Only s moves
    v = 0.8
    xdot = path_derivative([0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v, 0.0, v])
    np.testing.assert_allclose(xdot, [v, 0, 0, 0, 0, 0, 0, 0, 0, 0], atol=1e-9)


def test_path_coordinates_follow_a_car_turning_with_the_path():
    # on a circle of curvature kappa, the rear axle moving along it (no lateral speed at the rear axle,
    # yaw rate v kappa): progress at v, no drift of the offset nor of the heading error
    v, kappa = 0.8, 2.0
    r = v * kappa
    xdot = path_derivative([0.0, 0.0, 0.0, v, P['lr'] * r, r, 0.0, v, 0.0, v], kappa=kappa)
    np.testing.assert_allclose(xdot[:3], [v, 0.0, 0.0], atol=1e-9)


def test_the_drift_equilibrium_is_a_steady_drift():
    vx, vy, r = drift.equilibrium(P, D_EQ, U_EQ)
    # a real steady state of the model: fsolve can stop elsewhere without saying so
    assert residual(vx, vy, r, D_EQ, U_EQ) < 1e-6
    # turning left and sliding: a drift, not a grip turn
    assert r > 0
    assert np.degrees(np.arctan2(-vy, vx)) > 15


@pytest.mark.parametrize('mu', [0.7, 0.85, 1.0])
def test_the_friction_estimate_reads_back_the_floor(mu):
    # friction_node reads the floor friction from a steady drift on a known circle. Make that drift on a
    # floor of known friction, then ask drift.friction for it, starting as the node does: from mu = 1 and
    # the measured slip and yaw rate, here 5 % off
    for guess in [(0.55, -0.2, 3.2), (0.8, -0.15, 3.0), (0.5, -0.25, 3.5)]:
        vx, vy, r = drift.equilibrium(P, D_EQ, U_EQ, mu_scale=mu, guess=guess)
        if residual(vx, vy, r, D_EQ, U_EQ, mu) < 1e-6:
            break
    else:
        pytest.fail('no steady drift found')
    radius, e_psi = drift.circle(P, vx, vy, r)
    z, ok = drift.friction(P, D_EQ, U_EQ, radius, (-e_psi * 0.95, r * 1.05, 1.0))
    assert ok and z[1] >= 1.0      # friction_node discards answers with a yaw rate under 1 rad/s
    assert z[2] == pytest.approx(mu, rel=1e-3)
