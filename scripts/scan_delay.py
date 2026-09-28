import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

# /scan republished on /scan_late some time after its stamp, stamps unchanged: the latency of the
# A1 in the sim. usage: scan_delay.py <delay [s]> --ros-args -p use_sim_time:=true
DELAY = float(sys.argv[1])


class ScanDelay(Node):
    def __init__(self):
        super().__init__('scan_delay')
        self.queue = deque()
        self.pub = self.create_publisher(LaserScan, '/scan_late', qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.queue.append, qos_profile_sensor_data)
        self.create_timer(0.005, self.tick)

    def tick(self):
        now = self.get_clock().now()
        while self.queue and (now - Time.from_msg(self.queue[0].header.stamp)).nanoseconds * 1e-9 >= DELAY:
            self.pub.publish(self.queue.popleft())


def main():
    rclpy.init(args=sys.argv)
    rclpy.spin(ScanDelay())


if __name__ == '__main__':
    main()
