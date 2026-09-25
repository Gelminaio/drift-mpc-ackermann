import sys
import os
import subprocess
import cv2
import numpy as np
import pandas as pd

# HSV ranges of the two paper markers on the car (OpenCV hue is 0-180)
MARKERS = {
    'pink': ((140, 90, 90), (175, 255, 255)),     # rear
    'green': ((45, 80, 70), (85, 255, 255)),      # front
}
MIN_AREA = 20   # px, smaller blobs are noise


def frame_times(video_path):
    # phone videos are not exactly constant rate, and OpenCV loses the last timestamps
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                          '-show_entries', 'frame=pts_time', '-of', 'csv=p=0', video_path],
                         capture_output=True, text=True, check=True).stdout
    t = np.array([float(line.split(',')[0]) for line in out.split()])
    return t - t[0]


def track(video_path):
    times = frame_times(video_path)
    cap = cv2.VideoCapture(video_path)
    rows = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        row = {'t': times[i]}
        for name, (lo, hi) in MARKERS.items():
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            n, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
            areas = stats[1:, cv2.CC_STAT_AREA]
            if n > 1 and areas.max() >= MIN_AREA:
                k = 1 + np.argmax(areas)
                row[name + '_x'], row[name + '_y'] = centroids[k]
                row[name + '_area'] = areas.max()
            else:
                row[name + '_x'] = row[name + '_y'] = np.nan
                row[name + '_area'] = 0
        rows.append(row)
        i += 1

    df = pd.DataFrame(rows)
    out = os.path.splitext(video_path)[0] + '_markers.parquet'
    df.to_parquet(out)
    print(f"{len(df)} frames, {len(df) / df['t'].iloc[-1]:.2f} fps -> {out}")


if __name__ == '__main__':
    track(sys.argv[1])
