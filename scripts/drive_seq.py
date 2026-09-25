import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive

T_ARM = 1.0    # s at zero speed, arming before and disarming after
T_RAMP = 1.0   # s speed ramp from rest into the first segment and back to rest
T_STEP = 0.02  # s between segments: a step at the 50 Hz command rate


class DriveSeq(Node):
    def __init__(self, segments):
        super().__init__('drive_seq')
        self.pub_drive = self.create_publisher(AckermannDrive, '/drive', 10)
        self.pub_arm = self.create_publisher(Bool, '/arm', 10)

        # breakpoints (time, speed, steer): rest, ramp into segment 1, steps between segments, ramp out
        v0, s0, _ = segments[0]
        self.t, self.v, self.s = [0.0, T_ARM], [0.0, 0.0], [s0, s0]
        for k, (v, s, dur) in enumerate(segments):
            start = self.t[-1] + (T_RAMP if k == 0 else T_STEP)
            self.t += [start, start + dur]
            self.v += [v, v]
            self.s += [s, s]
        self.t.append(self.t[-1] + T_RAMP)
        self.v.append(0.0)
        self.s.append(self.s[-1])

        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.tick)   # 50 Hz

    def tick(self):
        t = time.time() - self.t0
        t_end = self.t[-1]

        if t < T_ARM:
            self.pub_arm.publish(Bool(data=True))
        elif t > t_end + T_ARM:
            rclpy.shutdown()
            return
        elif t > t_end:
            self.pub_arm.publish(Bool(data=False))

        msg = AckermannDrive()
        msg.speed = float(np.interp(t, self.t, self.v))
        msg.steering_angle = float(np.interp(t, self.t, self.s))
        self.pub_drive.publish(msg)


def main():
    # segments as speed:steer:duration, e.g. 0.6:0.40:6 0.6:0.52:3
    segments = [tuple(float(x) for x in arg.split(':')) for arg in sys.argv[1:]]
    rclpy.init()
    node = DriveSeq(segments)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
