import numpy as np
import pandas as pd
import bagtools as bt

# overhead video, see handling.ipynb and transients.ipynb
D_MARK = 0.267        # m between marker centers
PINK_BEHIND = 0.047   # m, pink marker behind the rear axle
LR = 0.0744           # m, CG ahead of the rear axle
R_WHEEL = 0.0299


def load_track(path):
    m = pd.read_parquet(path)
    x = (m['pink_x'] + m['green_x']) / 2
    y = (m['pink_y'] + m['green_y']) / 2
    d = np.hypot(m['green_x'] - m['pink_x'], m['green_y'] - m['pink_y'])
    A = np.c_[np.ones(len(m)), x, y, x**2, y**2, x * y]
    ok = d.notna() & ((d - d.median()).abs() < 0.2 * d.median())
    for _ in range(3):
        coef = np.linalg.lstsq(A[ok], d[ok], rcond=None)[0]
        d_fit = A @ coef
        ok = d.notna() & ((d - d_fit).abs() < 0.05 * d_fit)
    m['valid'] = ok
    m['scale'] = D_MARK / d_fit                                    # m per px at the car

    ux, uy = (m['green_x'] - m['pink_x']) / d, (m['green_y'] - m['pink_y']) / d
    m['psi'] = np.arctan2(-uy, ux)                                 # image y points down
    k_rear, k_cg = PINK_BEHIND / D_MARK, (PINK_BEHIND + LR) / D_MARK
    dt = m['t'].diff().median()
    k = round(0.1 / dt)
    m['ok'] = ok & ok.shift(k, fill_value=False) & ok.shift(-k, fill_value=False)
    for p, f in [('rear', k_rear), ('cg', k_cg)]:
        px = m['pink_x'] + f * (m['green_x'] - m['pink_x'])
        py = m['pink_y'] + f * (m['green_y'] - m['pink_y'])
        vx = (px.shift(-k) - px.shift(k)) / (2 * k * dt)
        vy = (py.shift(-k) - py.shift(k)) / (2 * k * dt)
        m['v_' + p] = np.hypot(vx, vy) * m['scale']
        m['beta_' + p] = np.degrees(np.angle(np.exp(1j * (np.arctan2(-vy, vx) - m['psi']))))
    return m


def load_run(name, data='../data'):
    # aligned table, video track, and video time -> table time offset
    d = bt.zero_time(bt.load(f'{data}/{name}_parquet'))
    js, drive, imu = d['joint_states'], d['drive'], d['imu_data_raw']
    m = load_track(f'{data}/video/{name}_markers.parquet')
    t_cmd = drive['t'].iloc[0]
    bias = bt.window(imu, 0, t_cmd - 0.5)['gz'].median()

    # video time -> bag time: first motion in each for a rough value, then the
    # offset that makes the heading rate from the video match the gyro
    first = m.index[m['valid']][0]
    moved = m['valid'] & (np.hypot(m['pink_x'] - m.at[first, 'pink_x'], m['pink_y'] - m.at[first, 'pink_y'])
                          * m['scale'] > 0.005)
    rough = js.loc[(js['wl'] + js['wr']) / 2 > 0.3, 't'].iloc[0] - m.loc[moved, 't'].iloc[0]
    good = m[m['ok']]
    r_vid = np.gradient(np.unwrap(good['psi'].to_numpy()), good['t'].to_numpy())
    offsets = np.arange(rough - 2.0, rough + 2.0, 1 / 120)
    err = [np.mean((r_vid - np.interp(good['t'] + o, imu['t'], imu['gz'] - bias))**2) for o in offsets]
    off = offsets[np.argmin(err)]

    # one table on the IMU time base, t = 0 at the first command
    out = imu[['t']].copy()
    out['r'] = imu['gz'] - bias
    out = pd.merge_asof(out, drive[['t', 'cmd_speed', 'cmd_steer']], on='t', direction='backward')
    out = pd.merge_asof(out, js[['t', 'wl', 'wr']], on='t', direction='nearest', tolerance=0.03)
    out['v_wheels'] = R_WHEEL * (out['wl'] + out['wr']) / 2
    good = m[m['ok']]
    for c in ['v_cg', 'beta_cg', 'beta_rear']:
        out[c] = np.interp(out['t'], good['t'] + off, good[c], left=np.nan, right=np.nan)
    out.loc[out['v_cg'] < 0.1, ['beta_cg', 'beta_rear']] = np.nan      # no direction when nearly still
    gap = np.interp(out['t'], m['t'] + off, (~m['ok']).astype(float), left=1, right=1)
    out.loc[gap > 0, ['v_cg', 'beta_cg', 'beta_rear']] = np.nan
    out['t'] -= t_cmd
    out = out[(out['t'] > -1) & (out['t'] < drive['t'].iloc[-1] - t_cmd + 1)]
    return out.drop(columns=['wl', 'wr']).reset_index(drop=True), m, off - t_cmd
