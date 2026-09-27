import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ackermann_control.pure_pursuit import lateral_error, load_track, nearest, steering


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # vehicle_params.yaml
        self.declare_parameter('wheelbase', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('steer_cmd', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('steer_angle', rclpy.Parameter.Type.DOUBLE_ARRAY)
        # controller
        self.declare_parameter('track_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('lookahead_min', 0.3)     # m
        self.declare_parameter('lookahead_time', 0.4)    # s, lookahead = min + time * speed
        self.declare_parameter('speed_scale', 1.0)       # fraction of the track speed profile
        self.declare_parameter('laps', 3)
        self.declare_parameter('max_error', 0.5)         # m off the track: stop
        self.declare_parameter('max_pose_age', 0.2)      # s since the last pose: stop

        p = {n: self.get_parameter(n).value for n in [
            'wheelbase', 'steer_cmd', 'steer_angle', 'track_file', 'lookahead_min', 'lookahead_time',
            'speed_scale', 'laps', 'max_error', 'max_pose_age']}
        self.p = p
        self.track = load_track(p['track_file'])
        self.i = None
        self.lap = 0
        self.halfway = False
        self.stopped = None
        self.stop_time = None
        self.done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_drive = self.create_publisher(AckermannDrive, '/drive', 10)
        self.pub_path = self.create_publisher(
            Path, 'track', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish_track()
        self.create_timer(0.02, self.tick)   # 50 Hz, the rate of /drive in every test so far

    def publish_track(self):
        path = Path()
        path.header.frame_id = 'map'
        for x, y, yaw in zip(self.track['x'], self.track['y'], self.track['yaw']):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(pose)
        self.pub_path.publish(path)

    def stop(self, reason):
        # zero commands for half a second, then the node exits: a stopped controller
        # must not keep publishing next to another one
        now = self.get_clock().now()
        if self.stopped is None:
            self.stopped = reason
            self.stop_time = now
            self.get_logger().info(f'stopped: {reason}')
        self.pub_drive.publish(AckermannDrive())
        if now - self.stop_time > Duration(seconds=0.5):
            self.done = True

    def tick(self):
        if self.stopped is not None:
            self.stop(self.stopped)
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            if self.i is None:
                # not started yet: wait for localization
                self.pub_drive.publish(AckermannDrive())
            else:
                self.stop('no pose')
            return
        age = self.get_clock().now() - Time.from_msg(tf.header.stamp)
        if age > Duration(seconds=self.p['max_pose_age']):
            self.stop(f'pose {age.nanoseconds * 1e-9:.2f} s old')
            return

        x, y = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = 2 * math.atan2(q.z, q.w)

        i = nearest(self.track, x, y, self.i)
        self.i = i
        # a lap: through the third quarter of the track, then back to its first quarter
        n = len(self.track)
        if n // 2 < i < 3 * n // 4:
            self.halfway = True
        if self.halfway and i < n // 4:
            self.halfway = False
            self.lap += 1
            self.get_logger().info(f'lap {self.lap}')
        if self.lap >= self.p['laps']:
            self.stop(f'{self.lap} laps')
            return
        e = lateral_error(self.track, i, x, y)
        if abs(e) > self.p['max_error']:
            self.stop(f'{e:+.2f} m off the track')
            return

        v = self.p['speed_scale'] * self.track['v'][i]
        delta = steering(self.track, i, x, y, yaw,
                         self.p['lookahead_min'] + self.p['lookahead_time'] * v, self.p['wheelbase'])
        msg = AckermannDrive()
        msg.speed = float(v)
        # wheel angle -> servo command, inverse of the measured steering map
        msg.steering_angle = float(np.interp(delta, self.p['steer_angle'], self.p['steer_cmd']))
        self.pub_drive.publish(msg)


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
