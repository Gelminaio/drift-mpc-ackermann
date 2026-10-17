import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

# Park anywhere: the two boxes among the lidar points the map does not explain, the line their sides share and the
# gap along it; where the parking starts (on its line, the boxes on the left); the approach there as the shortest
# arc-straight-arc path clear of the walls and the boxes, with the gymkhana's speed profile, or to the line further
# along when that start cannot be reached driving forward, the car then backing up. Poses (x, y, yaw) on the map,
# the rear axle.
NEW = 0.10           # m from the map's walls: a point of something new
LINK = 0.06          # m between two points of one object
SIZE = (0.15, 0.70)  # m, the extent of a box
GAP = (0.40, 0.75)   # m between the boxes
BAND = 0.015         # m, points on a face
STEP = np.radians(0.5)
FRONT, REAR, HALF = 0.2165, -0.0435, 0.10    # car outline from the rear axle
MARGIN = 0.05        # m between the car's outline and anything on the approach
R = 0.5              # m, the approach's turns
DS = 0.02
V_MAX, A_LAT, A_LONG, V_FINAL, FINAL, V_END = 0.8, 1.5, 1.0, 0.3, 0.3, 0.25
REVERSE_COST = 2.0   # a m backed up (0.3 m/s, no lidar on the boxes behind) counts as 2 m forward


def new_points(points, wall_dist):
    # points farther than NEW from the map's walls; wall_dist(points) gives the distance of each
    return points[wall_dist(points) > NEW]


def objects(points):
    # groups of points closer than LINK, as arrays
    pairs = np.array(list(cKDTree(points).query_pairs(LINK)))
    if len(pairs) == 0:
        return []
    n = len(points)
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    k, label = connected_components(graph, directed=False)
    return [points[label == i] for i in range(k) if (label == i).sum() >= 15]


def find_gap(points, car):
    # two boxes whose sides towards the car lie on one line with a gap between them. Returns the line's frame
    # (origin, direction with the boxes on its left) and the gap's ends along it, or None
    boxes = [o for o in objects(points) if SIZE[0] < np.ptp(o, axis=0).max() < SIZE[1] * 1.5]
    best = None
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            a, b = boxes[i], boxes[j]
            if not GAP[0] < np.linalg.norm(a.mean(axis=0) - b.mean(axis=0)) < SIZE[1] + GAP[1]:
                continue
            both = np.vstack([a, b])
            for th in np.arange(0, np.pi, STEP):
                n = np.array([-np.sin(th), np.cos(th)])
                if np.dot(car[:2] - both.mean(axis=0), n) < 0:
                    n = -n            # the normal towards the car
                fa, fb = np.percentile(a @ n, 90), np.percentile(b @ n, 90)
                if abs(fa - fb) > 2 * BAND:
                    continue
                face = (fa + fb) / 2
                on = np.abs(both @ n - face) < BAND
                if best is None or on.sum() > best[0]:
                    best = (on.sum(), n, face, a, b, both[on])
    if best is None:
        return None
    _, n, face, a, b, on = best
    # the line through the face points; its normal towards the car, so going along d the boxes are on the left
    c = on.mean(axis=0)
    n = np.linalg.svd(on - c)[2][1]
    if np.dot(car[:2] - c, n) < 0:
        n = -n
    d = np.array([-n[1], n[0]])
    a_along, b_along = (a - c) @ d, (b - c) @ d
    if a_along.mean() > b_along.mean():
        a_along, b_along = b_along, a_along
    g_from, g_to = a_along.max(), b_along.min()
    if not GAP[0] < g_to - g_from < GAP[1]:
        return None
    return c, d, g_from, g_to


def parking_start(c, d, g_from, g_to, kick_ahead, kick_left, sweep, side_in, runup):
    # where the parking starts at rest, as gap_node plans from there: on its line, runup before the kick
    left = np.array([-d[1], d[0]])
    mid = (g_from + g_to - sweep) / 2
    p = c + (mid - kick_ahead - runup) * d + (HALF + side_in - kick_left) * left
    return np.array([p[0], p[1], np.arctan2(d[1], d[0])])


def words(start, goal):
    # the four arc-straight-arc paths of radius R from start to goal (Dubins): segment curvatures and lengths
    dx, dy = goal[:2] - start[:2]
    dist = np.hypot(dx, dy) / R
    th = np.arctan2(dy, dx)
    a, b = np.mod(start[2] - th, 2 * np.pi), np.mod(goal[2] - th, 2 * np.pi)
    sa, sb, ca, cb = np.sin(a), np.sin(b), np.cos(a), np.cos(b)
    mod = lambda x: np.mod(x, 2 * np.pi)
    out = []
    p2 = 2 + dist ** 2 - 2 * np.cos(a - b) + 2 * dist * (sa - sb)        # LSL
    if p2 >= 0:
        t1 = np.arctan2(cb - ca, dist + sa - sb)
        out.append(((1, 0, 1), (mod(-a + t1), np.sqrt(p2), mod(b - t1))))
    p2 = 2 + dist ** 2 - 2 * np.cos(a - b) + 2 * dist * (sb - sa)        # RSR
    if p2 >= 0:
        t1 = np.arctan2(ca - cb, dist - sa + sb)
        out.append(((-1, 0, -1), (mod(a - t1), np.sqrt(p2), mod(-b + t1))))
    p2 = -2 + dist ** 2 + 2 * np.cos(a - b) + 2 * dist * (sa + sb)       # LSR
    if p2 >= 0:
        p = np.sqrt(p2)
        t2 = np.arctan2(-ca - cb, dist + sa + sb) - np.arctan2(-2, p)
        out.append(((1, 0, -1), (mod(-a + t2), p, mod(-b + t2))))
    p2 = -2 + dist ** 2 + 2 * np.cos(a - b) - 2 * dist * (sa + sb)       # RSL
    if p2 >= 0:
        p = np.sqrt(p2)
        t2 = np.arctan2(ca + cb, dist - sa - sb) - np.arctan2(2, p)
        out.append(((-1, 0, 1), (mod(a - t2), p, mod(b - t2))))
    return out


def sample(start, turns, lengths):
    # the path along the segments (turn +1 left, -1 right, 0 straight; lengths in units of R): x, y, yaw, kappa
    x, y, h = start
    pts = [(x, y, h, turns[0] / R)]
    for turn, length in zip(turns, lengths):
        n = max(1, int(np.ceil(length * R / DS)))
        ds = length * R / n
        k = turn / R
        for _ in range(n):
            if turn == 0:
                x, y = x + np.cos(h) * ds, y + np.sin(h) * ds
            else:
                x, y = x + (np.sin(h + k * ds) - np.sin(h)) / k, y - (np.cos(h + k * ds) - np.cos(h)) / k
                h += k * ds
            pts.append((x, y, h, k))
    return np.array(pts)


def clear(path, obstacles):
    # the car's outline plus MARGIN at every pose of the path against the obstacle points
    tree = cKDTree(obstacles)
    mid, half_len = (FRONT + REAR) / 2, (FRONT - REAR) / 2 + MARGIN
    for x, y, h, _ in path[::2]:
        cx, cy = x + np.cos(h) * mid, y + np.sin(h) * mid
        near = obstacles[tree.query_ball_point((cx, cy), np.hypot(half_len, HALF + MARGIN))]
        if len(near):
            lx = np.cos(h) * (near[:, 0] - cx) + np.sin(h) * (near[:, 1] - cy)
            ly = -np.sin(h) * (near[:, 0] - cx) + np.cos(h) * (near[:, 1] - cy)
            if np.any((np.abs(lx) < half_len) & (np.abs(ly) < HALF + MARGIN)):
                return False
    return True


def profile(path):
    # speed: lateral limit, from rest, the last FINAL m at V_FINAL, V_END at the end (make_gymkhana.py)
    step = np.hypot(*np.diff(path[:, :2], axis=0).T)
    s = np.r_[0, np.cumsum(step)]
    v = np.minimum(V_MAX, np.sqrt(A_LAT / np.maximum(np.abs(path[:, 3]), 1e-9)))
    v[s > s[-1] - FINAL] = np.minimum(v[s > s[-1] - FINAL], V_FINAL)
    v[0], v[-1] = 0.0, V_END
    for i in range(1, len(v)):
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * A_LONG * step[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * A_LONG * step[i]))
    return s, v


def plan(car, goal, obstacles, straight=0.3, reverse_max=3.0, reverse_cost=REVERSE_COST):
    # the clear path from the car to goal, its last straight m on the parking's line, or to a point further along
    # the line from where gap_node backs up straight to goal (the leg clear as well): the one with the least length
    # plus reverse_cost per m backed up. Returns the path (s, x, y, yaw, kappa, v) and the m to back up, or None, 0
    along = np.array([np.cos(goal[2]), np.sin(goal[2]), 0.0])
    best, best_cost = None, np.inf
    for back in np.r_[0.0, np.arange(0.3, reverse_max + 1e-9, 0.2)]:
        end = goal + back * along
        if back > 0 and not clear(np.array([[*(goal + a * along)[:2], goal[2], 0.0] for a in np.arange(0, back + DS / 2, DS)]), obstacles):
            continue
        for turns, lengths in words(car, end - straight * along):
            path = sample(car, turns + (0,), lengths + (straight / R,))
            cost = len(path) * DS + reverse_cost * back
            if cost < best_cost and np.hypot(*(path[-1, :2] - end[:2])) < 0.01 and clear(path, obstacles):
                best, best_cost = (path, back), cost
    if best is None:
        return None, 0.0
    path, back = best
    s, v = profile(path)
    return np.c_[s, path[:, 0], path[:, 1], np.unwrap(path[:, 2]), path[:, 3], v], back
