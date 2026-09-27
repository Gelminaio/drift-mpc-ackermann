import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

# racing line on maps/room: the largest rounded rectangle that keeps CLEAR from every
# obstacle, counterclockwise, with a speed profile. Run from the repo root.
MAP = 'ros2_ws/src/ackermann_bringup/maps/room'
R = 0.6         # m, corner radius, twice the car's minimum (0.31 m)
CLEAR = 0.5     # m from any occupied or unknown cell
DS = 0.05       # m between points
V_MAX = 1.0     # m/s, baseline speed cap (motors: 1.2)
A_LAT = 1.0     # m/s2, ~40% of the rear grip (0.276 g): no drift
A_LONG = 0.8    # m/s2, speeding up and braking


def rounded_rect(lx, ly):
    # points and curvature, counterclockwise from the start of the bottom straight
    a, b = lx / 2 - R, ly / 2 - R
    straights = [((-a, -b - R), (a, -b - R)), ((a + R, -b), (a + R, b)),
                 ((a, b + R), (-a, b + R)), ((-a - R, b), (-a - R, -b))]
    corners = [(a, -b, -np.pi / 2), (a, b, 0.0), (-a, b, np.pi / 2), (-a, -b, np.pi)]
    pts, kappa = [], []
    for (p, q), (cx, cy, a0) in zip(straights, corners):
        n = max(1, round(np.hypot(q[0] - p[0], q[1] - p[1]) / DS))
        t = np.arange(n) / n
        pts.append(np.c_[p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])])
        kappa.append(np.zeros(n))
        n = round(R * np.pi / 2 / DS)
        th = a0 + np.arange(n) / n * np.pi / 2
        pts.append(np.c_[cx + R * np.cos(th), cy + R * np.sin(th)])
        kappa.append(np.full(n, 1 / R))
    return np.vstack(pts), np.concatenate(kappa)


def place(P, deg, cx, cy):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return P @ np.array([[c, s], [-s, c]]) + [cx, cy]


meta = yaml.safe_load(open(MAP + '.yaml'))
img = np.array(Image.open(MAP + '.pgm'))
res, (x0, y0) = meta['resolution'], meta['origin'][:2]
h, w = img.shape
clearance_map = ndimage.distance_transform_edt(img > 250) * res     # m to the nearest non-free cell


def clearance(P):
    col = ((P[:, 0] - x0) / res).astype(int)
    row = (h - 1 - (P[:, 1] - y0) / res).astype(int)
    if col.min() < 0 or row.min() < 0 or col.max() >= w or row.max() >= h:
        return 0.0
    return clearance_map[row, col].min()


def first_fit(P):
    # orientation and center, on a grid around the middle of the room
    for deg in np.arange(-45, 45.1, 2.5):
        for cx in np.arange(3.3, 4.61, 0.05):
            for cy in np.arange(-0.5, 0.81, 0.05):
                if clearance(place(P, deg, cx, cy)) >= CLEAR:
                    return deg, cx, cy
    return None


# every size that could be longer than the best so far
best = None
for lx in np.arange(1.8, 4.01, 0.1):
    for ly in np.arange(1.4, lx + 0.01, 0.1):
        if best and lx + ly <= best[0] + best[1]:
            continue
        fit = first_fit(rounded_rect(lx, ly)[0])
        if fit:
            best = (lx, ly, *fit)

lx, ly, deg, cx, cy = best
P, kappa = rounded_rect(lx, ly)
P = place(P, deg, cx, cy)
d = np.diff(np.vstack([P, P[:1]]), axis=0)
yaw = np.arctan2(d[:, 1], d[:, 0])
s = np.r_[0, np.cumsum(np.hypot(d[:, 0], d[:, 1]))[:-1]]

# speed: lateral limit, then acceleration and braking limits, twice around the closed loop
v = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(np.abs(kappa), 1e-9)))
n = len(v)
for _ in range(2):
    for i in range(1, 2 * n):
        v[i % n] = min(v[i % n], np.sqrt(v[(i - 1) % n]**2 + 2 * A_LONG * DS))
    for i in range(2 * n - 2, -1, -1):
        v[i % n] = min(v[i % n], np.sqrt(v[(i + 1) % n]**2 + 2 * A_LONG * DS))

np.savetxt(MAP + '_track.csv', np.c_[s, P, yaw, kappa, v], delimiter=',', fmt='%.4f',
           header='s,x,y,yaw,kappa,v', comments='')
print(f'{lx:.1f} x {ly:.1f} m at {deg:.1f} deg, center ({cx:.2f}, {cy:.2f}); length {s[-1] + DS:.2f} m, '
      f'clearance {clearance(P):.2f} m, speed {v.min():.2f}-{v.max():.2f} m/s, '
      f'lap {np.sum(DS / v):.1f} s at the profile')
