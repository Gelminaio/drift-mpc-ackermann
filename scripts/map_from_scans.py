import sys

import cv2
import numpy as np
import pandas as pd

import lidar_path

# The room's map from lidar paths: each scan of a lap from the start mark laid at the rear axle's pose
# (lidar_path.py), each beam from where the lidar was when it was taken. A cell is a wall where the beams that
# reach it end in it often enough, floor where they pass through. Same grid and frame as maps/room.yaml, so the
# start mark and the paths keep their coordinates. What moves between sessions (the boxes, the cones) is floor.
# Run from scripts/: map_from_scans.py <out name> <run>...  -> <out name>.pgm/.yaml (the bags' laps start at rest
# on the start mark)
MAPS = '../ros2_ws/src/ackermann_bringup/maps'
TAPE = (3.010, 0.350, np.radians(-45.9))    # the start mark on the map (AMCL's initial pose)
R_MAX = 0.5          # rad/s, scans taken while turning faster are left out: the de-skew smears them
SHARE_MIN = 0.6      # scans laid on fewer of the reference points are left out
HITS_MIN = 3         # a wall: at least this many beams ended in the cell
HIT_SHARE = 0.3      # and this share of the beams that reached it
# moving objects of the gymkhana, from the start mark (m ahead, m left): the two boxes, the two cones
BOXES = (0.55, 2.05, 0.30, 0.80)
CONES = ((1.39, 1.62), (0.51, 1.72))
CONE_R = 0.12


def read(run, name):
    return pd.read_parquet(f'{lidar_path.B}/{run}_parquet/{name}.parquet')


def lap(run):
    # the first stretch from rest to rest: from the start mark to where the car stops
    j = read(run, 'joint_states')
    moving = ((j['wl'] + j['wr']).abs() > 0.05).to_numpy()
    t = j['t'].to_numpy()
    start = t[1:][moving[1:] & ~moving[:-1]][0]
    stops = t[1:][~moving[1:] & moving[:-1]]
    return start, stops[stops > start + 0.5][0] + 0.3


def beams_on_map(run):
    # every kept beam: where the lidar was and where the beam ended, on the map
    t0, t1 = lap(run)
    L, G = lidar_path.path(run, t0, t1)
    L = L[L['share'] > SHARE_MIN]
    sc = read(run, 'scan')
    scan_time = np.median(np.diff(sc['stamp']))
    sc = sc.assign(stamp=sc['t'] - scan_time)
    c, s = np.cos(TAPE[2]), np.sin(TAPE[2])
    origins, ends = [], []
    for tm in L['t']:
        p, t = lidar_path.beams(sc.iloc[np.argmin(np.abs(sc['stamp'] + scan_time / 2 - tm))], scan_time)
        if np.abs(np.interp(t, G['t'], G['r'])).max() > R_MAX:
            continue
        h = np.interp(t, G['t'], G['psi']) + np.interp(t, L['t'], L['dyaw'])
        x, y = np.interp(t, L['t'], L['x']), np.interp(t, L['t'], L['y'])
        lx, ly = x + np.cos(h) * lidar_path.LIDAR_X, y + np.sin(h) * lidar_path.LIDAR_X
        ex, ey = x + np.cos(h) * p[:, 0] - np.sin(h) * p[:, 1], y + np.sin(h) * p[:, 0] + np.cos(h) * p[:, 1]
        # from the start mark's frame to the map's
        origins.append(np.c_[TAPE[0] + c * lx - s * ly, TAPE[1] + s * lx + c * ly])
        ends.append(np.c_[TAPE[0] + c * ex - s * ey, TAPE[1] + s * ex + c * ey])
    return np.vstack(origins), np.vstack(ends)


def main(out, runs):
    meta = dict(l.split(': ', 1) for l in open(f'{MAPS}/room.yaml').read().splitlines() if ': ' in l)
    res = float(meta['resolution'])
    ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    nr, nc = cv2.imread(f'{MAPS}/room.pgm', -1).shape

    def cells(x, y):
        return nr - 1 - np.floor((y - oy) / res).astype(int), np.floor((x - ox) / res).astype(int)

    hits, passes = np.zeros(nr * nc, int), np.zeros(nr * nc, int)
    for run in runs:
        o, e = beams_on_map(run)
        for i in range(0, len(o), 5000):
            oo, ee = o[i:i + 5000], e[i:i + 5000]
            # the cells a beam passes through: points every half cell up to half a cell before its end, one pass per
            # beam and cell
            d = np.hypot(*(ee - oo).T)
            n = np.maximum(np.floor((d - res / 2) / (res / 2)).astype(int), 0)
            k = np.repeat(np.arange(len(oo)), n)
            f = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)) * (res / 2) / np.repeat(d, n)
            r, c = cells(oo[k, 0] + f * (ee[k, 0] - oo[k, 0]), oo[k, 1] + f * (ee[k, 1] - oo[k, 1]))
            inside = (r >= 0) & (r < nr) & (c >= 0) & (c < nc)
            passes += np.bincount(np.unique(k[inside] * (nr * nc) + r[inside] * nc + c[inside]) % (nr * nc), minlength=nr * nc)
        r, c = cells(e[:, 0], e[:, 1])
        inside = (r >= 0) & (r < nr) & (c >= 0) & (c < nc)
        hits += np.bincount(r[inside] * nc + c[inside], minlength=nr * nc)
        print(f'{run}: {len(o)} beams')
    reached = hits + passes
    img = np.full(nr * nc, 205, np.uint8)
    img[passes > 0] = 254
    img[(hits >= HITS_MIN) & (hits >= HIT_SHARE * reached)] = 0
    img = img.reshape(nr, nc)
    # the moving objects as floor
    rows, cols = np.mgrid[0:nr, 0:nc]
    x, y = ox + (cols + 0.5) * res - TAPE[0], oy + (nr - rows - 0.5) * res - TAPE[1]
    ahead, left = np.cos(TAPE[2]) * x + np.sin(TAPE[2]) * y, -np.sin(TAPE[2]) * x + np.cos(TAPE[2]) * y
    moving = (ahead > BOXES[0]) & (ahead < BOXES[1]) & (left > BOXES[2]) & (left < BOXES[3])
    for cx, cy in CONES:
        moving |= np.hypot(ahead - cx, left - cy) < CONE_R
    img[moving] = 254
    cv2.imwrite(f'{out}.pgm', img)
    with open(f'{out}.yaml', 'w') as f:
        f.write(f'image: {out.split("/")[-1]}.pgm\nmode: trinary\nresolution: {res:.3f}\norigin: [{ox:.3f}, {oy:.3f}, 0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print(f'{out}: {np.sum(img == 0)} wall cells, {np.sum(img == 254)} floor, {np.sum(img == 205)} unknown')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
