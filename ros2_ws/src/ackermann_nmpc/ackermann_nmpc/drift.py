import casadi as ca
import numpy as np
from scipy.optimize import fsolve

from ackermann_nmpc.model import forces


def equilibrium(p, d, u, mu_scale=1.0, guess=(0.8, -0.15, 3.0)):
    # steady vx, vy, r with the steering angle d and the wheel speed u held; which of the
    # steady states (grip, saddle, drift) depends on the guess
    z = ca.SX.sym('z', 3)
    fx, fy, mz = forces(p, z[0], z[1], z[2], d, u, mu_scale)
    f = ca.Function('f', [z], [ca.vertcat(fx / p['mass'] + z[1] * z[2], fy / p['mass'] - z[0] * z[2], mz / p['iz'])])
    return fsolve(lambda z: np.array(f(z)).ravel(), guess)


def circle(p, vx, vy, r):
    # a steady state turning left: radius of the rear axle circle and heading error to it
    V, beta = np.hypot(vx, vy), np.arctan2(vy, vx)
    heading = np.pi / 2 - beta      # CG at (V / r, 0) going +y
    rear = np.array([V / r - p['lr'] * np.cos(heading), -p['lr'] * np.sin(heading)])
    return np.hypot(*rear), np.angle(np.exp(1j * (heading - np.arctan2(rear[1], rear[0]) - np.pi / 2)))
