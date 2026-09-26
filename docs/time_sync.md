# ESP32 time sync

The ESP32 stamps /joint_states and /imu/data_raw on the agent (Pi) clock:
`rmw_uros_sync_session` right after the session is created, then every 10 s.
/odom takes the /joint_states stamp, and dt from consecutive stamps.

## Numbers

Arrival on the Pi minus stamp, /imu/data_raw, 60 s at 50 Hz: median 7.0-7.5 ms
per 5 s window, min 5.4, max 8.7. The first sync after a boot gave 4 ms: the
offset is known to a few ms (serial link, NTP-style exchange), then stays put.

Agent killed (`docker kill micro-ros-agent`): session back in 6.5 s, stamps
synced again from the first message.

/odom: delay 6 ms, 49.995 Hz, period sd 0.84 ms.
