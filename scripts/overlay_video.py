import sys
import os
import cv2
import numpy as np
import pandas as pd

D_MARK = 0.267        # m between marker centers
CG_AHEAD = 0.1214     # m, CG ahead of the pink marker (47 + 74.4 mm)
TRAIL = 1.5           # s of CG path drawn behind the car


def overlay(video_path):
    m = pd.read_parquet(os.path.splitext(video_path)[0] + '_markers.parquet')
    pink = m[['pink_x', 'pink_y']].to_numpy()
    green = m[['green_x', 'green_y']].to_numpy()
    d = np.hypot(*(green - pink).T)
    scale = D_MARK / np.nanmedian(d)                    # m per px
    valid = np.abs(d - np.nanmedian(d)) < 0.05 * np.nanmedian(d)
    u = (green - pink) / d[:, None]
    cg = pink + (CG_AHEAD / scale) * u

    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    k = round(0.1 * fps)
    v = np.full_like(cg, np.nan)
    v[k:-k] = (cg[2 * k:] - cg[:-2 * k]) * fps / (2 * k)       # px/s
    speed = np.hypot(*v.T) * scale
    # image y points down: flip both angles so positive sideslip means velocity left of heading
    beta = np.degrees(np.angle(np.exp(1j * (np.arctan2(-v[:, 1], v[:, 0]) - np.arctan2(-u[:, 1], u[:, 0])))))

    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out_path = os.path.splitext(video_path)[0] + '_overlay.mp4'
    out = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    n_trail = round(TRAIL * fps)
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        trail = cg[max(0, i - n_trail):i + 1][valid[max(0, i - n_trail):i + 1]]
        if len(trail) > 1:
            cv2.polylines(frame, [trail.astype(np.int32)], False, (0, 200, 255), 3)
        if valid[i]:
            c = tuple(cg[i].astype(int))
            cv2.arrowedLine(frame, c, tuple((cg[i] + 0.15 / scale * u[i]).astype(int)), (255, 255, 255), 3, tipLength=0.25)
            if np.isfinite(v[i]).all() and speed[i] > 0.1:
                cv2.arrowedLine(frame, c, tuple((cg[i] + 0.3 * v[i]).astype(int)), (0, 140, 255), 3, tipLength=0.25)
                text = f"v {speed[i]:.2f} m/s   sideslip {beta[i]:+.0f} deg"
                cv2.putText(frame, text, (30, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 0), 6)
                cv2.putText(frame, text, (30, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 255), 2)
        out.write(frame)
        i += 1
    out.release()
    print(f"{i} frames -> {out_path}")


if __name__ == '__main__':
    overlay(sys.argv[1])
