import sys

import numpy as np
import pandas as pd
import rclpy
from ackermann_msgs.msg import AckermannDrive
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

# /drive of a sysid run replayed open loop in the sim; gyro z and wheel speed recorded.
# usage, from the repo root with the sim running: python3 scripts/sim_replay.py <run> <out.parquet>
run, out = sys.argv[1], sys.argv[2]
df = pd.read_parquet(f'data/sysid/{run}.parquet')
t_cmd = df['t'].to_numpy()
speed = df['cmd_speed'].ffill().fillna(0).to_numpy()
steer = df['cmd_steer'].ffill().fillna(0).to_numpy()

rclpy.init()
node = Node('sim_replay', parameter_overrides=[Parameter('use_sim_time', value=True)])
pub_drive = node.create_publisher(AckermannDrive, 'drive', 10)
pub_arm = node.create_publisher(Bool, 'arm', 10)
rows = []


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def now():
    return node.get_clock().now().nanoseconds * 1e-9


node.create_subscription(Imu, 'imu/data_raw', lambda m: rows.append(('gyro_z', stamp(m), m.angular_velocity.z)), 10)
node.create_subscription(JointState, 'joint_states', lambda m: rows.append(('wheel', stamp(m), m.velocity[0])), 10)

while pub_arm.get_subscription_count() == 0 or now() == 0:
    rclpy.spin_once(node, timeout_sec=0.1)
for _ in range(5):    # the first ones can go before discovery ends on the other side
    pub_arm.publish(Bool(data=True))
    rclpy.spin_once(node, timeout_sec=0.05)

t0 = now() - t_cmd[0]


def tick():
    i = min(np.searchsorted(t_cmd, now() - t0), len(t_cmd) - 1)
    pub_drive.publish(AckermannDrive(speed=float(speed[i]), steering_angle=float(steer[i])))


node.create_timer(0.02, tick)    # sim time, as the real runs: 50 Hz
while now() - t0 < t_cmd[-1] + 1.0:
    rclpy.spin_once(node, timeout_sec=0.1)
pub_arm.publish(Bool(data=False))

r = pd.DataFrame(rows, columns=['topic', 'stamp', 'value'])
r['t'] = r['stamp'] - t0
r.to_parquet(out)
print(f'{run}: {len(r)} samples, {t_cmd[-1] - t_cmd[0]:.1f} s')
