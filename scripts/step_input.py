import sys
import time
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive


class StepInput(Node):
    def __init__(self, mode, amp, t_hold):
        super().__init__('step_input')
        self.pub = self.create_publisher(AckermannDrive, '/drive', 10)
        self.mode = mode          # 'speed' or 'steer'
        self.amp = amp
        self.t_hold = t_hold
        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.tick)   # 50 Hz

    def tick(self):
        t = time.time() - self.t0
        msg = AckermannDrive()
        if t < 1.0:                      # 1 s at zero (baseline)
            val = 0.0
        elif t < 1.0 + self.t_hold:      # step
            val = self.amp
        else:                            # back to zero, then done
            val = 0.0
            if t > 2.0 + self.t_hold:
                rclpy.shutdown()
                return
        if self.mode == 'speed':
            msg.speed = float(val)
        else:
            msg.steering_angle = float(val)
        self.pub.publish(msg)


def main():
    mode = sys.argv[1]                   # speed | steer
    amp = float(sys.argv[2])             # m/s or rad
    t_hold = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    rclpy.init()
    node = StepInput(mode, amp, t_hold)
    rclpy.spin(node)


if __name__ == '__main__':
    main()