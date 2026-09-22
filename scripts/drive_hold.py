import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive

T_ARM = 1.0    # s at zero speed, arming before and disarming after
T_RAMP = 1.0   # s speed ramp, up and down


class DriveHold(Node):
    def __init__(self, speed, steer, t_hold):
        super().__init__('drive_hold')
        self.pub_drive = self.create_publisher(AckermannDrive, '/drive', 10)
        self.pub_arm = self.create_publisher(Bool, '/arm', 10)
        self.speed = speed
        self.steer = steer
        self.t_hold = t_hold
        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.tick)   # 50 Hz

    def tick(self):
        t = time.time() - self.t0
        t_up = T_ARM + T_RAMP
        t_down = t_up + self.t_hold
        t_end = t_down + T_RAMP

        if t < T_ARM:
            v = 0.0
            self.pub_arm.publish(Bool(data=True))
        elif t < t_up:
            v = self.speed * (t - T_ARM) / T_RAMP
        elif t < t_down:
            v = self.speed
        elif t < t_end:
            v = self.speed * (t_end - t) / T_RAMP
        elif t < t_end + T_ARM:
            v = 0.0
            self.pub_arm.publish(Bool(data=False))
        else:
            rclpy.shutdown()
            return

        msg = AckermannDrive()
        msg.speed = float(v)
        msg.steering_angle = float(self.steer)
        self.pub_drive.publish(msg)


def main():
    speed = float(sys.argv[1])           # m/s
    steer = float(sys.argv[2])           # rad, positive = left
    t_hold = float(sys.argv[3])          # s
    rclpy.init()
    node = DriveHold(speed, steer, t_hold)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
