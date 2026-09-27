import casadi as ca
from acados_template import AcadosModel

# The car of Phase 5 (scripts/vehicle_model.py) in CasADi, in path coordinates at the CG.
# p: the ros__parameters of vehicle_params.yaml.
G = 9.81
TAU_U = 0.03    # s, wheel speed loop, middle of the measured 15-45 ms


def forces(p, vx, vy, r, d, u, mu_scale):
    # body forces and yaw moment at the CG, as vehicle_model.forces. Friction mu_scale times the
    # tiles: the peak scales with it, the stiffness at small slip does not (brush model)
    L = p['lf'] + p['lr']
    nf, nr = p['mass'] * G * p['lr'] / L, p['mass'] * G * p['lf'] / L
    mu_f, b_f = mu_scale * p['tire_mu_f'], p['tire_b_f'] / mu_scale
    mu_r, b_r = mu_scale * p['tire_mu_r'], p['tire_b_r'] / mu_scale
    c = p['tire_c']
    vx = ca.fmax(vx, 0.05)
    fyf = mu_f * nf * ca.sin(c * ca.atan(b_f * (d - ca.atan((vy + p['lf'] * r) / vx))))

    def rear_wheel(vxi, vyi):
        sx, sy = (vxi - u) / ca.fmax(u, 0.1), vyi / ca.fmax(u, 0.1)
        s = ca.sqrt(sx ** 2 + sy ** 2 + 1e-9)    # smooth at zero slip
        f = mu_r * nr / 2 * ca.sin(c * ca.atan(b_r * s))
        return -f * sx / s, -f * sy / s

    fxl, fyl = rear_wheel(vx - p['track'] / 2 * r, vy - p['lr'] * r)
    fxr, fyr = rear_wheel(vx + p['track'] / 2 * r, vy - p['lr'] * r)
    fx = fxl + fxr - fyf * ca.sin(d)
    fy = fyf * ca.cos(d) + fyl + fyr
    mz = p['lf'] * fyf * ca.cos(d) - p['lr'] * (fyl + fyr) + p['track'] / 2 * (fxr - fxl)
    return fx, fy, mz


def path_model(p):
    # states: progress s, lateral offset n (+ left) and heading error e_psi to the path, the
    # Phase 5 states vx, vy, r, steering angle d, wheel speed u, and the two /drive commands
    # (wheel angle, wheel speed). Inputs: the rates of the commands. Parameters: mu_scale and
    # the path curvature kappa at the stage
    s, n, e_psi, vx, vy, r, d, u, d_cmd, u_cmd = [ca.SX.sym(name) for name in [
        's', 'n', 'e_psi', 'vx', 'vy', 'r', 'd', 'u', 'd_cmd', 'u_cmd']]
    dd_cmd, du_cmd = ca.SX.sym('dd_cmd'), ca.SX.sym('du_cmd')
    mu_scale, kappa = ca.SX.sym('mu_scale'), ca.SX.sym('kappa')

    fx, fy, mz = forces(p, vx, vy, r, d, u, mu_scale)
    s_dot = (vx * ca.cos(e_psi) - vy * ca.sin(e_psi)) / (1 - n * kappa)

    model = AcadosModel()
    model.name = 'car'
    model.x = ca.vertcat(s, n, e_psi, vx, vy, r, d, u, d_cmd, u_cmd)
    model.u = ca.vertcat(dd_cmd, du_cmd)
    model.p = ca.vertcat(mu_scale, kappa)
    model.f_expl_expr = ca.vertcat(
        s_dot,
        vx * ca.sin(e_psi) + vy * ca.cos(e_psi),
        r - kappa * s_dot,
        fx / p['mass'] + vy * r,
        fy / p['mass'] - vx * r,
        mz / p['iz'],
        (d_cmd - d) / p['steer_lag'],
        (u_cmd - u) / TAU_U,
        dd_cmd,
        du_cmd)
    model.xdot = ca.SX.sym('xdot', 10)
    model.f_impl_expr = model.xdot - model.f_expl_expr
    return model
