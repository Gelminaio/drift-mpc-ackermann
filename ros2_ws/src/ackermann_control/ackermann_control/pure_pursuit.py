import numpy as np


def load_track(path):
    # columns s, x, y, yaw, kappa, v every ds metres, closed loop (make_track.py)
    return np.genfromtxt(path, delimiter=',', names=True)


def nearest(track, x, y, around=None, window=40):
    # index of the closest track point; with `around`, only a window of points near it
    n = len(track)
    idx = np.arange(n) if around is None else (around + np.arange(-window, window + 1)) % n
    d2 = (track['x'][idx] - x) ** 2 + (track['y'][idx] - y) ** 2
    return int(idx[np.argmin(d2)])


def lateral_error(track, i, x, y):
    # signed distance from the track at point i, positive to the left of it
    yaw = track['yaw'][i]
    return -np.sin(yaw) * (x - track['x'][i]) + np.cos(yaw) * (y - track['y'][i])


def steering(track, i, x, y, yaw, lookahead, wheelbase):
    # pure pursuit about the rear axle: the arc through the point `lookahead` metres
    # further along the track, curvature 2 y / d^2 with the point at (x, y) in the car frame
    ds = track['s'][1] - track['s'][0]
    j = (i + int(round(lookahead / ds))) % len(track)
    dx, dy = track['x'][j] - x, track['y'][j] - y
    x_car = np.cos(yaw) * dx + np.sin(yaw) * dy
    y_car = -np.sin(yaw) * dx + np.cos(yaw) * dy
    return np.arctan(wheelbase * 2 * y_car / (x_car ** 2 + y_car ** 2))
