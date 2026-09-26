# Wheel odometry

Forward Ackermann from /joint_states + /steering_angle, midpoint integration.
Publishes /odom and odom -> base_footprint, stamped with /joint_states (docs/time_sync.md).

## Numbers

Straight 1.49 m -> odom 1.515 m (+1.5%).
Circle R=0.53 m -> odom 1.00 rad/s vs gyro 0.744 (+34%).

Rear axle is a spool, both wheels at the same setpoint. At that radius the
outer wheel would need +39% distance, so it slips. Encoders see the wheels,
not the ground. Fine straight, bad in tight turns. Gyro is right -> EKF in
Phase 6.
