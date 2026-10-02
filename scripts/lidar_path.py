import sys

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

# The path of the rear axle from the lidar, in the frame of the car at rest before the run: every scan laid
# by ICP on the scans at rest, each beam first moved to where the car was when it was taken (heading from
# the gyro, position between the scans). Starts from the wheels and the gyro, then iterates.
# Scan time: receive time less one turn (rplidar_ros stamps its call for the scan, which slips against
# the rotation); each beam at stamp + scan_time (pi - a) / 2pi, the turn begins behind and goes clockwise.
# Times in the output are the middle of each scan on that base: the lidar lag (vehicle_params.yaml) later
# than the car was there. The gyro is read GYRO_EARLY before, as it runs ahead of the lidar.
# usage: lidar_path.py <run> <from [s after the bag start]> <to [s]>  -> data/bags/<run>_parquet/lidar_path.parquet
B = '../data/bags'
LIDAR_X = 0.09225       # lidar ahead of the rear axle (URDF)
R_WHEEL = 0.0299
GYRO_SCALE = 1.0054     # the gyro reads low (identification.ipynb)
GYRO_EARLY = 0.012      # s, the IMU message against the lidar scan (identification.ipynb)
H = 0.002


def beams(m, scan_time):
    # one scan: points from the rear axle and the time of each
    r = np.array(m['ranges'], dtype=float)
    a = m['angle_min'] + np.arange(len(r)) * m['angle_increment']
    ok = np.isfinite(r) & (r > 0.15) & (r < 5.0)
    return np.c_[r[ok] * np.cos(a[ok]) + LIDAR_X, r[ok] * np.sin(a[ok])], m['stamp'] + scan_time * (np.pi - a[ok]) / (2 * np.pi)


def rigid(src, tree, dst, center, gate, iters=30):
    # small rigid motion (dx, dy, dyaw) about center laying src on dst, point to point; rms, share of pairs
    x = y = yaw = 0.0
    src, dst = src - center, dst - center
    for k in range(iters):
        c, s = np.cos(yaw), np.sin(yaw)
        moved = src @ np.array([[c, s], [-s, c]]) + [x, y]
        d, i = tree.query(moved + center)
        keep = d < (gate if k < 10 else 0.05)
        if keep.sum() < 30:
            return None
        p, q = moved[keep], dst[i[keep]]
        pm, qm = p.mean(axis=0), q.mean(axis=0)
        U, _, Vt = np.linalg.svd((p - pm).T @ (q - qm))
        R = (U @ Vt).T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = (U @ Vt).T
        yaw += np.arctan2(R[1, 0], R[0, 0])
        x, y = R @ np.array([x, y]) + qm - pm @ R.T
    return x, y, yaw, np.sqrt(np.mean(d[keep] ** 2)), keep.mean()


def path(run, t_from, t_to, iters=4):
    # t_from, t_to: bag time; the car at rest before t_from
    read = lambda name: pd.read_parquet(f'{B}/{run}_parquet/{name}.parquet')
    im, j, sc = read('imu_data_raw'), read('joint_states'), read('scan')
    scan_time = np.median(np.diff(sc['stamp']))
    sc = sc.assign(stamp=sc['t'] - scan_time)
    ref = np.vstack([beams(m, scan_time)[0] for _, m in sc[sc['stamp'] < t_from - 0.3].iloc[-8:].iterrows()])
    tree = cKDTree(ref)
    bias = im.loc[(im['t'] > t_from - 1.5) & (im['t'] < t_from - 0.3), 'gz'].mean()
    tt = np.arange(t_from - 0.3, t_to, H)
    r = np.interp(tt - GYRO_EARLY, im['t'], im['gz'] - bias) * GYRO_SCALE
    psi = np.cumsum(r) * H
    u = np.interp(tt, j['t'], (j['wl'] + j['wr']) / 2 * R_WHEEL)
    X, Y = np.cumsum(u * np.cos(psi)) * H, np.cumsum(u * np.sin(psi)) * H
    moving = sc[(sc['stamp'] > t_from - 0.3) & (sc['stamp'] + scan_time < t_to)]
    for it in range(iters):
        out = []
        for _, m in moving.iterrows():
            p, t = beams(m, scan_time)
            h = np.interp(t, tt, psi)
            c, s = np.cos(h), np.sin(h)
            world = np.c_[np.interp(t, tt, X) + c * p[:, 0] - s * p[:, 1], np.interp(t, tt, Y) + s * p[:, 0] + c * p[:, 1]]
            tm = m['stamp'] + scan_time / 2
            center = np.array([np.interp(tm, tt, X), np.interp(tm, tt, Y)])
            best = None
            for gate in (0.15, 0.4):
                res = rigid(world, tree, ref, center, gate)
                if res is not None and (best is None or res[4] > best[4] + 0.05):
                    best = res
                if best is not None and best[4] > 0.6:
                    break
            if best is not None:
                out.append((tm, center[0] + best[0], center[1] + best[1], best[2], best[3], best[4]))
        o = np.array(out)
        good = o[:, 5] > 0.5
        # the wheel/gyro path shifted through the laid scans, linear in time between them
        X = X + np.interp(tt, o[good, 0], o[good, 1] - np.interp(o[good, 0], tt, X))
        Y = Y + np.interp(tt, o[good, 0], o[good, 2] - np.interp(o[good, 0], tt, Y))
    return (pd.DataFrame(o, columns=['t', 'x', 'y', 'dyaw', 'rms', 'share']),
            pd.DataFrame({'t': tt, 'psi': psi, 'r': r}))


if __name__ == '__main__':
    run, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    t0 = pd.read_parquet(f'{B}/{run}_parquet/joint_states.parquet')['t'].iloc[0]
    scans, gyro = path(run, t0 + a, t0 + b)
    scans.to_parquet(f'{B}/{run}_parquet/lidar_path.parquet')
    gyro.to_parquet(f'{B}/{run}_parquet/gyro_path.parquet')
    print(f'{run}: {len(scans)} scans, rms {100 * scans["rms"].median():.1f} cm median, share {scans["share"].min():.2f} min')
