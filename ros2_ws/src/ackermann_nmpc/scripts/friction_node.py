#!/usr/bin/env python3
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray

from ackermann_nmpc import drift

# The floor friction from the drift of drift_node: the mean of its state (drift/state) from 3 s
# after the entry on, into drift.friction once a second. Ends with the drift node
SETTLE = 150    # control steps of 20 ms
EVERY = 50


class FrictionNode(Node):
    def __init__(self):
        super().__init__('friction')
        params_file = os.path.join(get_package_share_directory('ackermann_description'), 'config', 'vehicle_params.yaml')
        self.p = yaml.safe_load(open(params_file))['/**']['ros__parameters']
        self.radius = self.declare_parameter('drift_radius', 0.0).value
        self.mu = self.declare_parameter('mu_scale', 1.0).value    # what the NMPC assumes, the first guess
        self.count, self.sum = 0, np.zeros(10)
        self.drift_node_seen, self.done = False, False
        self.pub = self.create_publisher(Float32, 'drift/mu', 10)
        self.create_subscription(Float64MultiArray, 'drift/state', self.on_state, 10)
        self.create_timer(0.5, self.check_drift_node)

    def check_drift_node(self):
        publishers = self.count_publishers('drift/state')
        self.done = self.drift_node_seen and publishers == 0
        self.drift_node_seen |= publishers > 0

    def on_state(self, m):
        self.count += 1
        if self.count <= SETTLE:
            return
        self.sum += m.data
        if (self.count - SETTLE) % EVERY:
            return
        # s, n, e_psi, vx, vy, r, d, u, d_cmd, u_cmd
        n, e_psi, r, u, d_cmd = (self.sum / (self.count - SETTLE))[[1, 2, 5, 7, 8]]
        # on a steady circle the rear axle moves along it: rear slip = -e_psi
        z, ok = drift.friction(self.p, d_cmd, u, self.radius - n, (-e_psi, r, self.mu))
        if not ok or z[1] < 1.0:
            self.get_logger().warn(f'no drift with steering {np.degrees(d_cmd):.1f} deg, wheels {u:.2f} m/s')
            return
        self.mu = z[2]
        self.pub.publish(Float32(data=float(self.mu)))
        self.get_logger().info(f'mu_scale {self.mu:.3f}')


def main():
    rclpy.init()
    node = FrictionNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
