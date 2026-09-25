import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive

T_ARM = 1.0    # s at zero speed, arming before and disarming after
T_RAMP = 1.0   # s ramp into each step and back to zero


class DriveSteps(Node):
    def __init__(self, steer, speeds, t_hold):
        super().__init__('drive_steps')
        self.pub_drive = self.create_publisher(AckermannDrive, '/drive', 10)
        self.pub_arm = self.create_publisher(Bool, '/arm', 10)
        self.steer = steer

        # speed profile as breakpoints: rest, then ramp + hold for each step, ramp to zero
        self.times, self.speeds = [0.0, T_ARM], [0.0, 0.0]
        for v in speeds:
            self.times += [self.times[-1] + T_RAMP, self.times[-1] + T_RAMP + t_hold]
            self.speeds += [v, v]
        self.times.append(self.times[-1] + T_RAMP)
        self.speeds.append(0.0)

        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.tick)   # 50 Hz

    def tick(self):
        t = time.time() - self.t0
        t_end = self.times[-1]

        if t < T_ARM:
            self.pub_arm.publish(Bool(data=True))
        elif t > t_end + T_ARM:
            rclpy.shutdown()
            return
        elif t > t_end:
            self.pub_arm.publish(Bool(data=False))

        msg = AckermannDrive()
        msg.speed = float(np.interp(t, self.times, self.speeds))
        msg.steering_angle = float(self.steer)
        self.pub_drive.publish(msg)


def main():
    steer = float(sys.argv[1])                            # rad, positive = left
    speeds = [float(v) for v in sys.argv[2].split(',')]   # m/s, e.g. 0.3,0.5,0.7
    t_hold = float(sys.argv[3])                           # s per step
    rclpy.init()
    node = DriveSteps(steer, speeds, t_hold)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
