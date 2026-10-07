#!/usr/bin/env python3
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from ackermann_nmpc import valet

# Park anywhere, the plan: at rest, the localization and five scans; the two boxes the map does not have, the gap
# between them and the approach to where the parking starts, written as a path for nmpc_node (track_file). Exits 0
# with a path, 2 when the car already stands where gap_node can start, 1 without (no localization, no gap, no clear
# approach)
SCANS = 5
T_MAX = 10.0    # s to get the localization and the scans
THERE = (np.radians(10), 0.20, 0.50, 0.35, 1.20)    # as gap_node starts: heading off the faces, the faces left, the kick ahead


def yaw(q):
    return 2 * np.arctan2(q.z, q.w)


class ValetNode(Node):
    def __init__(self):
        super().__init__('valet')
        self.map_file = self.declare_parameter('map', '').value
        self.out = self.declare_parameter('path_file', '/tmp/valet_path.csv').value
        # as the gap node plans from where the approach ends
        self.kick = [self.declare_parameter(n, v).value for n, v in
                     (('kick_ahead', 0.30), ('kick_left', 0.42), ('sweep', 0.12), ('side_in', 0.02))]
        self.runup = self.declare_parameter('runup', 0.6).value    # m from the approach's end to the kick
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.points, self.scans, self.pose, self.ok = [], 0, None, 1    # the exit code
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.t0 = time.monotonic()    # wall clock: in the sim the node's clock is 0 until /clock comes

    def car_pose(self):
        # the newest localization correction on the newest odometry, as nmpc_node takes it
        c = self.tf.lookup_transform('map', 'odom', Time()).transform
        o = self.tf.lookup_transform('odom', 'base_footprint', Time()).transform
        yc = yaw(c.rotation)
        return np.array([c.translation.x + np.cos(yc) * o.translation.x - np.sin(yc) * o.translation.y,
                         c.translation.y + np.sin(yc) * o.translation.x + np.cos(yc) * o.translation.y,
                         yc + yaw(o.rotation)])

    def on_scan(self, m):
        if self.scans >= SCANS:
            return
        try:
            pose = self.car_pose()
            lidar = self.tf.lookup_transform('base_footprint', m.header.frame_id, Time()).transform.translation.x
        except TransformException:
            return    # the localization comes a while after the start
        r = np.array(m.ranges)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.15) & (r < 5.0)
        x, y = r[ok] * np.cos(a[ok]) + lidar, r[ok] * np.sin(a[ok])
        c, s = np.cos(pose[2]), np.sin(pose[2])
        self.points.append(np.c_[pose[0] + c * x - s * y, pose[1] + s * x + c * y])
        self.pose = pose
        self.scans += 1

    def run(self):
        while rclpy.ok() and self.scans < SCANS:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - self.t0 > T_MAX:
                self.get_logger().error(f'{self.scans} scans with a localization in {T_MAX:.0f} s: stop')
                return
        meta = yaml.safe_load(open(self.map_file))
        img = cv2.imread(os.path.join(os.path.dirname(self.map_file), meta['image']), -1)
        res, (ox, oy) = meta['resolution'], meta['origin'][:2]
        nr, nc = img.shape
        dist = cv2.distanceTransform((img != 0).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
        rows, cols = np.nonzero(img == 0)
        walls = np.c_[ox + (cols + 0.5) * res, oy + (nr - rows - 0.5) * res]

        def wall_dist(p):
            r = np.clip(nr - 1 - np.floor((p[:, 1] - oy) / res).astype(int), 0, nr - 1)
            c = np.clip(np.floor((p[:, 0] - ox) / res).astype(int), 0, nc - 1)
            return dist[r, c]

        new = valet.new_points(np.vstack(self.points), wall_dist)
        np.savez(os.path.splitext(self.out)[0] + '_points.npz', points=np.vstack(self.points), new=new, pose=self.pose)
        gap = valet.find_gap(new, self.pose)
        if gap is None:
            self.get_logger().error(f'no two boxes with a gap among {len(new)} new points: stop')
            return
        c, d, g_from, g_to = gap
        # the car in the gap's frame: along the faces, across them (the faces at 0, the car right of them)
        along, across = np.dot(self.pose[:2] - c, d), np.dot(self.pose[:2] - c, [-d[1], d[0]])
        off = abs(np.angle(np.exp(1j * (self.pose[2] - np.arctan2(d[1], d[0])))))
        kick = (g_from + g_to - self.kick[2]) / 2 - self.kick[0] - along
        if off < THERE[0] and THERE[1] < -across < THERE[2] and THERE[3] < kick < THERE[4]:
            self.get_logger().info(f'gap {g_to - g_from:.3f} m, the kick {kick:.2f} m ahead, the faces {-across:.2f} m left: '
                                   'the parking starts from here')
            self.ok = 2
            return
        goal = valet.parking_start(c, d, g_from, g_to, *self.kick, self.runup)
        path = valet.plan(self.pose, goal, np.vstack([walls, new]))
        self.get_logger().info(
            f'gap {g_to - g_from:.3f} m, its line at ({c[0]:.2f}, {c[1]:.2f}) heading {np.degrees(np.arctan2(d[1], d[0])):.1f} deg; '
            f'the parking starts at ({goal[0]:.2f}, {goal[1]:.2f}); from ({self.pose[0]:.2f}, {self.pose[1]:.2f})')
        if path is None:
            self.get_logger().error('no clear approach: stop')
            return
        np.savetxt(self.out, path, delimiter=',', fmt='%.4f', header='s,x,y,yaw,kappa,v', comments='')
        self.get_logger().info(f'approach {path[-1, 0]:.2f} m in {self.out}')
        self.ok = 0


def main():
    rclpy.init()
    node = ValetNode()
    node.run()
    code = node.ok
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
