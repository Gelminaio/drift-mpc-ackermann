import sys
import threading
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

# handbrake turn: straight up to speed, full lock at speed, rear wheels locked (speed 0, the motor
# driver brakes) once the heading from the gyro reaches brake_at, then stopped.
# usage: handbrake.py <speed [m/s]> <straight [s]> <brake_at [deg]> [steer, default 0.52] [steer braked, default steer]
#        [speed at full lock, default speed]
SPEED, STRAIGHT, BRAKE_AT = float(sys.argv[1]), float(sys.argv[2]), np.radians(float(sys.argv[3]))
STEER = float(sys.argv[4]) if len(sys.argv) > 4 else 0.52
STEER_BRAKED = float(sys.argv[5]) if len(sys.argv) > 5 else STEER
SPEED_TURN = float(sys.argv[6]) if len(sys.argv) > 6 else SPEED
T_RAMP = 1.0    # s from rest to SPEED
T_BRAKE = 1.0   # s braked before disarming
T_TURN = 2.5    # s at full lock at most (on the stand the heading does not grow)

rclpy.init()
node = rclpy.create_node('handbrake')
pub_drive = node.create_publisher(AckermannDrive, '/drive', 10)
pub_arm = node.create_publisher(Bool, '/arm', 10)
gz = []
node.create_subscription(Imu, '/imu/data_raw', lambda m: gz.append((time.time(), m.angular_velocity.z)), 50)
# callbacks in their own thread: every gyro sample counted as it comes
executor = SingleThreadedExecutor()
executor.add_node(node)
spinner = threading.Thread(target=executor.spin)
spinner.start()


# arming zeroes the setpoint in the firmware: only at rest (with every command it made the wheel speed
# ripple, 0.99-1.19 m/s at full throttle in the runs of 2026-09-29)
def send(speed, steer):
    pub_drive.publish(AckermannDrive(speed=float(speed), steering_angle=float(steer)))
    time.sleep(0.02)


while pub_arm.get_subscription_count() == 0 or pub_drive.get_subscription_count() == 0 or len(gz) < 50:
    time.sleep(0.1)
bias = np.median([w for _, w in gz[-50:]])

t0 = time.time()
while time.time() - t0 < 0.5:                      # armed at rest
    pub_arm.publish(Bool(data=True))
    send(0.0, 0.0)
t0 = time.time()
while time.time() - t0 < T_RAMP + STRAIGHT:        # up to speed, straight
    send(SPEED * min(1.0, (time.time() - t0) / T_RAMP), 0.0)
n0, heading = len(gz), 0.0
t0 = time.time()
while heading < BRAKE_AT and time.time() - t0 < T_TURN:   # full lock at speed
    send(SPEED_TURN, STEER)
    new = gz[n0 - 1:]
    heading = sum((b[0] - a[0]) * (b[1] - bias) for a, b in zip(new, new[1:]))
at_brake = heading
t0 = time.time()
while time.time() - t0 < T_BRAKE:                  # rear locked
    send(0.0, STEER_BRAKED)
new = gz[n0 - 1:]
heading = sum((b[0] - a[0]) * (b[1] - bias) for a, b in zip(new, new[1:]))
for _ in range(25):
    pub_arm.publish(Bool(data=False))
    send(0.0, 0.0)
print(f'braked at {np.degrees(at_brake):.0f} deg, stopped at {np.degrees(heading):.0f} deg')
rclpy.shutdown()     # ends the spin
spinner.join()
