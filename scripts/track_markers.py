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
MIN_AREA = 20        # px, smaller blobs are noise
STATIC_FRAC = 0.8    # a pixel in a marker color this often is part of the room, not the car
SAMPLE_EVERY = 10    # frames sampled to find those pixels


def color_masks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return {name: cv2.inRange(hsv, np.array(lo), np.array(hi)) > 0 for name, (lo, hi) in MARKERS.items()}


def static_masks(video_path):
    # the markers move, objects of the same color in the room do not
    cap = cv2.VideoCapture(video_path)
    counts, n, i = None, 0, 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % SAMPLE_EVERY == 0:
            masks = color_masks(frame)
            if counts is None:
                counts = {name: np.zeros(m.shape, np.int32) for name, m in masks.items()}
            for name, m in masks.items():
                counts[name] += m
            n += 1
        i += 1
    kernel = np.ones((15, 15), np.uint8)
    return {name: cv2.dilate((c > STATIC_FRAC * n).astype(np.uint8), kernel) > 0 for name, c in counts.items()}


def frame_times(video_path):
    # phone videos are not exactly constant rate, and OpenCV loses the last timestamps
    out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                          '-show_entries', 'frame=pts_time', '-of', 'csv=p=0', video_path],
                         capture_output=True, text=True, check=True).stdout
    t = np.array([float(line.split(',')[0]) for line in out.split()])
    return t - t[0]


def track(video_path):
    times = frame_times(video_path)
    static = static_masks(video_path)
    cap = cv2.VideoCapture(video_path)
    rows = []
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        row = {'t': times[i]}
        for name, mask in color_masks(frame).items():
            mask = (mask & ~static[name]).astype(np.uint8)
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
