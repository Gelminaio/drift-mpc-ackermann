import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    def __init__(self):
        super().__init__('ackermann_odometry')

        self.declare_parameter('wheelbase', 0.173)
        self.declare_parameter('wheel_radius', 0.0299)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.L = self.get_parameter('wheelbase').value
        self.r = self.get_parameter('wheel_radius').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.steering = 0.0
        self.last_time = None

        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Float32, 'steering_angle', self.steering_cb, 10)
        self.create_subscription(JointState, 'joint_states', self.joints_cb, 10)

    def steering_cb(self, msg):
        self.steering = msg.data

    def joints_cb(self, msg):
        # ESP32 stamp, synced to the agent clock (step 6.1)
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_time is None:
            self.last_time = stamp
            return
        dt = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time = stamp
        # skip bogus intervals (startup, agent reconnect)
        if dt <= 0.0 or dt > 0.5:
            return
        if len(msg.velocity) < 2:
            return

        # rear wheel angular velocities -> vehicle speed (order left/right as in firmware)
        v = self.r * (msg.velocity[0] + msg.velocity[1]) / 2.0
        yaw_rate = v * math.tan(self.steering) / self.L

        # midpoint integration
        mid_yaw = self.yaw + yaw_rate * dt / 2.0
        self.x += v * math.cos(mid_yaw) * dt
        self.y += v * math.sin(mid_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + yaw_rate * dt),
                              math.cos(self.yaw + yaw_rate * dt))

        self.publish_odom(stamp, v, yaw_rate)

    def publish_odom(self, stamp, v, yaw_rate):
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = yaw_rate
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02   # vy = 0 at the rear axle, sideslip < 1.5 deg in grip
        odom.twist.covariance[35] = 0.05
        self.pub_odom.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()