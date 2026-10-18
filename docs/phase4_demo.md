# Phase 4 demo

Vehicle on battery, bringup over ssh (tmux), driven from the desktop over wifi.
RViz shows model, scan, tf and odometry live.

Rates: /odom 49 Hz, /scan 7.1 Hz.

Wifi cut while driving at 0.5 m/s: the vehicle stops by itself in 0.34 m, 0.9 s
after the last command. That is the 500 ms timeout plus the soft-stop ramp.
Pi and bag keep running, and control resumes without re-arming.

Bag not in the repo (1.9 GB).
