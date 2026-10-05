import sys

import numpy as np
from scipy.interpolate import CubicSpline

# the gymkhana's two open paths on the racing line of maps/room (room_track.csv): from rest on the start mark, past
# the boxes, round the ring with a slalom round CONES cones on the far straight, to a stop in the corner after it
# (DONUT: the donut there); then from there to where the parking starts, past the start mark. Speed profile from the
# lateral and the longitudinal grip, from rest to rest. Run from the repo root.
# make_gymkhana.py [cones spacing amplitude a_lat] -> maps/room_gymkhana.csv, room_gymkhana_back.csv, room_gymkhana_cones.csv
MAPS = 'ros2_ws/src/ackermann_bringup/maps'
CONES, SPACING, AMPLITUDE = 2, 0.9, 0.2     # m between the cones, m the line passes beside them
FIRST = -1       # the first cone passed on the right (-1): the swing then turns against the corner before the straight
A_LAT = 2.0      # m/s2, the slalom near the limit of the tiles (~0.26 g); the NMPC holds the car there
A_LONG = 1.5     # m/s2
V_MAX = 1.1      # m/s, the wheels reach 1.16
START = 0.05     # m, s of the ring where the car starts at rest: the start mark
TAPE = (3.010, 0.350, np.radians(-45.9))    # the start mark on the map (AMCL's initial pose): the boxes are set from it
DONUT = 6.65     # m, s of the ring where the first path stops: the corner after the far straight, ~0.6 m clear
R_LAST = 0.45    # m, the way back takes the last corner tighter than the ring (0.6) to be on the parking's line sooner
PAST = 0.15      # m past the start mark where the way back stops
LEFT = 0.06      # m left of the start mark's line, on the parking's: the boxes 0.36 m left of it, kick_left 0.42
V_FINAL, FINAL = 0.3, 0.3    # m/s over the last m of the lap: the NMPC does not follow a hard braking to the end
V_BACK, FINAL_BACK = 0.35, 1.0    # and of the way back, the end of its corner too: the parking wants the car settled
V_END = 0.25     # m/s at the ends, then braked
DS = 0.02
if len(sys.argv) > 1:
    CONES, SPACING, AMPLITUDE, A_LAT = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])

ring = np.loadtxt(f'{MAPS}/room_track.csv', delimiter=',', skiprows=1)
s_r, x_r, y_r, yaw_r = ring[:, 0], ring[:, 1], ring[:, 2], ring[:, 3]
length = s_r[-1] + np.hypot(x_r[0] - x_r[-1], y_r[0] - y_r[-1])


def base(s):
    # the ring at s (past the end of a lap: the next lap), heading unwrapped
    yaw = np.unwrap(np.r_[yaw_r, yaw_r[0] + 2 * np.pi])
    ss = np.r_[s_r, length]
    xs, ys = np.r_[x_r, x_r[0]], np.r_[y_r, y_r[0]]
    sm = np.mod(s, length)
    return np.interp(sm, ss, xs), np.interp(sm, ss, ys), np.interp(sm, ss, yaw)


# the slalom: the cones on the far straight, centred on it; the line beside each, alternately left and right,
# on the racing line at the ends of the straight (its corners keep their curvature alone)
straight_from, straight_to = 4.38, 6.43
mid = (straight_from + straight_to) / 2
s_cones = mid + (np.arange(CONES) - (CONES - 1) / 2) * SPACING
knots = np.r_[straight_from, s_cones, straight_to]
offsets = np.r_[0.0, FIRST * AMPLITUDE * (-1.0) ** np.arange(CONES), 0.0]
lateral = CubicSpline(knots, offsets, bc_type='clamped')

def line(s_from, s_to):
    # a straight of the ring: a point on it and its heading, fitted on its points
    m = (s_r >= s_from) & (s_r <= s_to)
    return np.array([x_r[m].mean(), y_r[m].mean()]), np.arctan2(np.sin(yaw_r[m]).mean(), np.cos(yaw_r[m]).mean())


SHORT = 7.6      # m, s on the short straight before the last corner where the way back leaves the ring


def back_end():
    # the end of the way back: from the ring's short straight on to the vertex V of its last corner with the parking's
    # line (along the start mark's: the ring's near straight is 3.4 deg off it), the arc of R_LAST between them, to PAST
    p1, h1 = line(7.45, 7.75)
    p0, h0 = np.array(TAPE[:2]), TAPE[2]
    d1, d0 = np.array([np.cos(h1), np.sin(h1)]), np.array([np.cos(h0), np.sin(h0)])
    p0 = p0 + LEFT * np.array([-d0[1], d0[0]])
    k = np.linalg.solve(np.c_[d1, -d0], p0 - p1)
    v = p1 + k[0] * d1                                                     # the vertex
    turn = np.remainder(h0 - h1, 2 * np.pi)
    t = R_LAST * np.tan(turn / 2)                                          # from V to where the arc meets each line
    bx, by, _ = base(SHORT)
    a0 = np.dot([bx, by] - v, d1)                                          # where the way back leaves the ring
    a = np.linspace(a0, -t, max(2, round((-t - a0) / DS) + 1))[1:]
    c = v - t * d1 + R_LAST * np.array([-d1[1], d1[0]])
    th = np.linspace(0, turn, round(R_LAST * turn / DS) + 1)[1:]
    end = np.dot(p0 - v, d0) + PAST
    b = np.linspace(t, end, max(2, round((end - t) / DS) + 1))[1:]
    return np.vstack([np.c_[v[0] + a * d1[0], v[1] + a * d1[1], np.full(len(a), h1)],
                      np.c_[c[0] + R_LAST * np.cos(h1 - np.pi / 2 + th), c[1] + R_LAST * np.sin(h1 - np.pi / 2 + th), h1 + th],
                      np.c_[v[0] + b * d0[0], v[1] + b * d0[1], np.full(len(b), h1 + turn)]]).T


def path(s_from, s_to, name, final, v_final):
    s = np.arange(s_from, s_to, DS)
    bx, by, byaw = base(s)
    if s_to > length:
        s = np.arange(s_from, SHORT + DS / 2, DS)
        bx, by, byaw = [np.r_[p, q] for p, q in zip(base(s), back_end())]
        s = np.r_[s[0], s[0] + np.cumsum(np.hypot(np.diff(bx), np.diff(by)))]
    n = np.where((s > knots[0]) & (s < knots[-1]), lateral(np.clip(s, knots[0], knots[-1])), 0.0)
    x, y = bx - np.sin(byaw) * n, by + np.cos(byaw) * n
    d = np.gradient(np.c_[x, y], axis=0)
    dd = np.gradient(d, axis=0)
    yaw = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    kappa = (d[:, 0] * dd[:, 1] - d[:, 1] * dd[:, 0]) / np.hypot(d[:, 0], d[:, 1]) ** 3
    step = np.hypot(np.diff(x), np.diff(y))
    s_path = np.r_[0, np.cumsum(step)]
    # speed: lateral limit, from rest, to V_END at the end
    v = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(np.abs(kappa), 1e-9)))
    v[s_path > s_path[-1] - final] = np.minimum(v[s_path > s_path[-1] - final], v_final)
    v[0], v[-1] = 0.0, V_END
    for i in range(1, len(v)):
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * A_LONG * step[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * A_LONG * step[i]))
    np.savetxt(f'{MAPS}/room_{name}.csv', np.c_[s_path, x, y, yaw, kappa, v], delimiter=',', fmt='%.4f',
               header='s,x,y,yaw,kappa,v', comments='')
    print(f'{name}: {s_path[-1]:.2f} m, radius min {1 / np.abs(kappa).max():.2f} m, speed max {v.max():.2f}, '
          f'{np.sum(step / np.maximum(v[1:], 0.05)):.1f} s; from ({x[0]:.2f}, {y[0]:.2f}) {np.degrees(yaw[0]):.1f} deg '
          f'to ({x[-1]:.2f}, {y[-1]:.2f}) {np.degrees(yaw[-1]):.1f} deg')


print(f'{CONES} cones {SPACING} m apart, {AMPLITUDE} m beside them')
path(START, DONUT, 'gymkhana', FINAL, V_FINAL)
path(DONUT, length + START, 'gymkhana_back', FINAL_BACK, V_BACK)
cx, cy, _ = base(s_cones)
np.savetxt(f'{MAPS}/room_gymkhana_cones.csv', np.c_[cx, cy], delimiter=',', fmt='%.3f', header='x,y', comments='')
for i, (a, b) in enumerate(zip(cx, cy)):
    print(f'cone {i + 1}: ({a:.3f}, {b:.3f})')
