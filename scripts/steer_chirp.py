import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive

T_ARM = 1.0     # s at zero speed, arming before and disarming after
T_RAMP = 1.0    # s speed ramp from rest and back
T_SETTLE = 4.0  # s at the mean steering before the sweep starts


class SteerChirp(Node):
    def __init__(self, speed, mean, amp, f0, f1, duration):
        super().__init__('steer_chirp')
        self.pub_drive = self.create_publisher(AckermannDrive, '/drive', 10)
        self.pub_arm = self.create_publisher(Bool, '/arm', 10)
        self.speed, self.mean, self.amp = speed, mean, amp
        self.f0, self.f1, self.duration = f0, f1, duration
        self.t_sweep = T_ARM + T_RAMP + T_SETTLE
        self.t_end = self.t_sweep + duration
        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.tick)   # 50 Hz

    def tick(self):
        t = time.time() - self.t0
        steer = self.mean
        if t < T_ARM:
            self.pub_arm.publish(Bool(data=True))
            v = 0.0
        elif t < T_ARM + T_RAMP:
            v = self.speed * (t - T_ARM) / T_RAMP
        elif t < self.t_end:
            v = self.speed
            if t >= self.t_sweep:
                # linear chirp: frequency goes from f0 to f1 over the sweep
                tau = t - self.t_sweep
                phase = 2 * np.pi * (self.f0 * tau + (self.f1 - self.f0) * tau**2 / (2 * self.duration))
                steer = self.mean + self.amp * np.sin(phase)
        elif t < self.t_end + T_RAMP:
            v = self.speed * (self.t_end + T_RAMP - t) / T_RAMP
        elif t < self.t_end + T_RAMP + T_ARM:
            v = 0.0
            self.pub_arm.publish(Bool(data=False))
        else:
            rclpy.shutdown()
            return

        msg = AckermannDrive()
        msg.speed = float(v)
        msg.steering_angle = float(steer)
        self.pub_drive.publish(msg)


def main():
    speed = float(sys.argv[1])       # m/s
    mean = float(sys.argv[2])        # rad
    amp = float(sys.argv[3])         # rad
    f0 = float(sys.argv[4])          # Hz
    f1 = float(sys.argv[5])          # Hz
    duration = float(sys.argv[6])    # s of sweep
    rclpy.init()
    node = SteerChirp(speed, mean, amp, f0, f1, duration)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
