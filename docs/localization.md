# Localization

nav2 AMCL on `maps/room` (docs/mapping.md), run on the desktop:

    ros2 launch ackermann_bringup localization.launch.py

It starts from the start mark: tape at the rear wheels and the front left wheel, on the first
straight of the track, (3.010, 0.350, -45.9 deg). The configured initial pose has no spread
(all particles on one point), so the robot has to sit on the mark: the lidar alone, standing
still, does not pull a wrong start back. Placed ~30 cm off by eye, AMCL took ~4 s of driving
to correct it (bag `pp_run1`). The mark was set where AMCL put the robot after 3 laps.
Odometry is the robot_localization EKF (wheel speed + gyro).

## Numbers

Live lap by hand, 14.4 m in 352 s (bag `loc_lap`), default odometry noise: correction of
the map pose at each update 2.9 / 10.5 / 17.7 cm (median / 95% / max), 0.4 / 1.2 / 2.1 deg.
Back on the mark, within a few cm. Over the lap AMCL took out ~30 cm of odometry drift.

Same lap replayed on the desktop (odometry and EKF recomputed, `use_sim_time:=true`, 4x),
deterministic run to run:

| setting | correction cm (median / 95% / max) | heading deg (median / 95%) | sd at the end |
|---|---|---|---|
| default, alpha 0.2 | 1.6 / 6.3 / 9.9 | 0.36 / 1.28 | 9 cm, 5.5 deg |
| alpha 0.1 | 1.4 / 4.1 / 7.5 | 0.29 / 0.99 | 7 cm, 5.9 deg |
| **alpha 0.05** | **1.1 / 3.6 / 5.9** | **0.29 / 0.92** | **6 cm, 3.6 deg** |
| 180 beams | 1.8 / 6.5 / 10.5 | 0.40 / 1.35 | 10 cm, 6.2 deg |

With alpha 0.05 the end pose is 2.5 cm from the mark (by eye: ~2 cm).

## On the Pi

Localization and the controller run on the Pi (`localization.launch.py`, `pure_pursuit.launch.py`
in tmux): with AMCL and RViz on the desktop, the Pi streamed ~250 kB/s over wifi, the ping
went to ~1 s and robot_localization missed its 50 Hz. With nothing pulled to the desktop,
36 kB/s and 3 ms.

Before a session, `fastdds shm clean`. Nodes killed without a clean shutdown leave Fast DDS
shared memory ports behind, and a publish then blocked up to 0.32 s every ~30-60 s (one
0.35 s stall stopped the controller on "pose 0.39 s old"). Cleaned: 0 blocks in 3000
publishes, max 1.8 ms. Stop launches with Ctrl-C in their tmux window, not by killing it.
