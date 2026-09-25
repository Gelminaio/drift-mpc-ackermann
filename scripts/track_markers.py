import sys
import os
import cv2
import numpy as np
import pandas as pd

# HSV ranges of the two paper markers on the car (OpenCV hue is 0-180)
MARKERS = {
    'pink': ((140, 90, 90), (175, 255, 255)),     # rear
    'green': ((45, 80, 70), (85, 255, 255)),      # front
}
MIN_AREA = 20   # px, smaller blobs are noise


def track(video_path):
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    rows = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        row = {'t': i / fps}
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
    print(f"{len(df)} frames at {fps:.2f} fps -> {out}")


if __name__ == '__main__':
    track(sys.argv[1])
