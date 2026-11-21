# The Phase 5 model (scripts/vehicle_model.py): physical properties that any version of the identified
# parameters must keep. Needs numpy and PyYAML only.
import numpy as np
import pytest

import vehicle_model as vm

RNG = np.random.default_rng(0)


def random_states(n, rng=RNG):
    # vx, vy, r, steering angle, rear wheel speed: the envelope of the runs and some way beyond it
    return rng.uniform([0.1, -0.6, -6.0, -0.45, 0.1], [1.5, 0.6, 6.0, 0.45, 1.5], size=(n, 5))


def test_rolling_straight_without_slip_needs_no_force():
    # wheels turning at the ground speed, no sideslip, no yaw rate: the tires carry nothing
    fx, fy, mz = vm.forces((0.8, 0.0, 0.0, 0.0), 0.8)
    assert abs(fx) < 1e-9 and abs(fy) < 1e-9 and abs(mz) < 1e-9


@pytest.mark.parametrize('vx, vy, r, d, u', random_states(20))
def test_left_and_right_turns_are_mirror_images(vx, vy, r, d, u):
    # the car is symmetric: mirroring the state (vy, r, steering) mirrors the lateral force and the yaw moment
    fx, fy, mz = vm.forces((vx, vy, r, d), u)
    fx_m, fy_m, mz_m = vm.forces((vx, -vy, -r, -d), u)
    assert fx_m == pytest.approx(fx, abs=1e-9)
    assert fy_m == pytest.approx(-fy, abs=1e-9)
    assert mz_m == pytest.approx(-mz, abs=1e-9)


def test_a_tire_never_pushes_harder_than_the_friction_limit():
    # whatever the slip, a rear wheel gives at most mu_r times its load (the magic formula peaks at sin = 1)
    for vx, vy, r, d, u in random_states(2000):
        for side in (-1, 1):
            fxi, fyi = vm.rear_wheel(vx + side * vm.T / 2 * r, vy - vm.LR * r, u)
            assert np.hypot(fxi, fyi) <= vm.MU_R * vm.NR / 2 + 1e-9


def test_lower_friction_lowers_the_peak_but_not_the_stiffness():
    # brush model, as in sim_car: on a floor with k times the friction the force at full slide is k times
    # lower, while the slope at small slip stays the same
    small = [vm.rear_wheel(1.0, 0.001, 1.0, k=k)[1] for k in (1.0, 0.5)]
    assert small[1] == pytest.approx(small[0], rel=2e-3)
    full = [vm.rear_wheel(1.0, 3.0, 1.0, k=k)[1] for k in (1.0, 0.5)]
    assert full[1] / full[0] == pytest.approx(0.5, rel=0.02)


def test_a_slower_wheel_brakes_and_a_faster_one_drives():
    fx_brake, _ = vm.rear_wheel(1.0, 0.0, 0.6)    # wheel slower than the ground: braking force
    fx_drive, _ = vm.rear_wheel(0.6, 0.0, 1.0)    # wheelspin: traction
    assert fx_brake < 0 < fx_drive


def test_the_refit_model_reduces_to_the_base_model():
    # forces_fit (identification.ipynb) adds load transfer, a slip scale and locked-wheel friction to
    # forces(); with those terms neutral and the base tire parameters both must give the same car
    q = dict(mu_f=vm.MU_F, b_f=vm.B_F, mu_r=vm.MU_R, b_r=vm.B_R, lam=1.0, locked=1.0, locked_y=1.0,
             lt_x=0.0, lt_y=0.0, tau=vm.TAU)
    for vx, vy, r, d, u in random_states(200):
        base = vm.forces((vx, vy, r, d), u)
        fit = vm.forces_fit(vx, vy, r, d, u, False, 0.0, 0.0, q=q)
        np.testing.assert_allclose(fit, base, rtol=1e-9, atol=1e-12)


def test_wheel_speed_model():
    p = vm.p
    # command 0: the shorted motor brakes the wheel with the time constant wheel_brake
    assert vm.wheel_accel(1.0, 0.0) == pytest.approx(-1.0 / p['wheel_brake'])
    # from rest towards a command: positive, at most wheel_a
    assert 0 < vm.wheel_accel(0.0, 1.0) <= p['wheel_a']
    # at the loaded free speed the motor has nothing left
    assert vm.wheel_accel(p['wheel_u'], 2.0) == pytest.approx(0.0, abs=1e-9)
    # slowing down to a lower command: at most wheel_d
    assert vm.wheel_accel(1.0, 0.2) == pytest.approx(-p['wheel_d'])
