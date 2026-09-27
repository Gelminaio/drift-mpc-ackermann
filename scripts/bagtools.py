import os
import numpy as np
import pandas as pd


def load(parquet_dir):
    """Load all topic parquets from a bag directory into a dict of DataFrames."""
    out = {}
    for f in os.listdir(parquet_dir):
        if f.endswith('.parquet'):
            key = f[:-8]  # strip .parquet
            out[key] = pd.read_parquet(os.path.join(parquet_dir, f))
    return out


def zero_time(dfs):
    """Shift all timestamps so the earliest sample across topics is t=0."""
    t0 = min(df['t'].iloc[0] for df in dfs.values() if len(df))
    for df in dfs.values():
        df['t'] = df['t'] - t0
    return dfs


def merge_asof(dfs, base='imu_data_raw', tol=0.03):
    """Align all topics onto the timestamps of `base` using nearest-time join."""
    base_df = dfs[base].sort_values('t').reset_index(drop=True)
    merged = base_df.copy()
    for key, df in dfs.items():
        if key == base or not len(df):
            continue
        right = df.sort_values('t').reset_index(drop=True)
        merged = pd.merge_asof(merged, right, on='t',
                               direction='nearest', tolerance=tol)
    return merged


def window(df, t0, t1):
    """Slice a time window [t0, t1]."""
    return df[(df['t'] >= t0) & (df['t'] <= t1)].reset_index(drop=True)


IMU_YAW = -1.5708   # from urdf imu_yaw; MUST match the URDF value


def imu_to_vehicle(df):
    """Rotate IMU accel/gyro from sensor frame to vehicle (base_link) frame.
    Rotation is imu_yaw about Z, so only x,y mix; z is unchanged.
    With yaw = -pi/2:  ax_v = -ay_imu,  ay_v = +ax_imu."""
    c, s = np.cos(IMU_YAW), np.sin(IMU_YAW)
    out = df.copy()
    out['ax_v'] = c * df['ax'] - s * df['ay']
    out['ay_v'] = s * df['ax'] + c * df['ay']
    out['gz_v'] = df['gz']
    return out

def map_pose(d):
    """Rear axle in the map at the EKF rate, as the controller sees it: the last AMCL
    correction applied to the EKF pose. Returns t, x, y."""
    a, o = d['amcl_pose'], d['odometry_filtered']
    oyaw = np.unwrap(o['yaw'].to_numpy())
    # map -> odom at each AMCL update
    ox, oy = np.interp(a['stamp'], o['stamp'], o['x']), np.interp(a['stamp'], o['stamp'], o['y'])
    dyaw = a['yaw'].to_numpy() - np.interp(a['stamp'], o['stamp'], oyaw)
    tx = a['x'].to_numpy() - (np.cos(dyaw) * ox - np.sin(dyaw) * oy)
    ty = a['y'].to_numpy() - (np.sin(dyaw) * ox + np.cos(dyaw) * oy)
    k = np.searchsorted(a['stamp'].to_numpy(), o['stamp'].to_numpy()) - 1
    ok = k >= 0
    k, x, y = k[ok], o['x'].to_numpy()[ok], o['y'].to_numpy()[ok]
    X = tx[k] + np.cos(dyaw[k]) * x - np.sin(dyaw[k]) * y
    Y = ty[k] + np.sin(dyaw[k]) * x + np.cos(dyaw[k]) * y
    return o['t'].to_numpy()[ok], X, Y
