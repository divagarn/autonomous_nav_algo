#!/usr/bin/env python3
"""Converts wheel_odometry's nav_msgs/Odometry into the
geometry_msgs/TwistWithCovarianceStamped autoware_ekf_localizer's
in_twist_with_covariance input expects - different message types,
otherwise the same twist data (buggy_perception's wheel_odometry.py is
reused as-is from divagar_test1_ws, not modified, so this adapts at the
boundary instead).

wheel_odometry never sets odom.twist.covariance (confirmed - it's all
zeros), which a Kalman filter reads as "infinitely certain", numerically
risky and not actually true for wheel dead-reckoning. This fills in
modest, honest uncertainty instead: linear.x and angular.z are the only
two axes this 2D bicycle-model estimate actually measures, so only those
get a real (small) variance; the other four axes get a large variance so
the filter does not trust values this source never actually measured
(y/z linear velocity, roll/pitch rate are always 0.0 from wheel_odometry,
not real zero-measurements).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

# Diagonal covariance: [x, y, z, roll, pitch, yaw] linear/angular rate.
# Small variance on the two axes wheel_odometry actually estimates
# (index 0: linear.x, index 35: angular.z); large on the rest so the
# filter does not over-trust unmeasured axes.
_SMALL_VAR = 0.02
_LARGE_VAR = 100.0


def _build_covariance():
    cov = [_LARGE_VAR] * 36
    for i in range(36):
        cov[i] = 0.0
    cov[0] = _SMALL_VAR    # linear.x
    cov[7] = _LARGE_VAR    # linear.y (not measured)
    cov[14] = _LARGE_VAR   # linear.z (not measured)
    cov[21] = _LARGE_VAR   # angular.x / roll rate (not measured)
    cov[28] = _LARGE_VAR   # angular.y / pitch rate (not measured)
    cov[35] = _SMALL_VAR   # angular.z (actually estimated)
    return cov


_COVARIANCE = _build_covariance()


class OdomToTwist(Node):

    def __init__(self):
        super().__init__('odom_to_twist')
        self.input_topic = self.declare_parameter('input_topic', '/odom').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/twist_with_covariance').value

        self.pub = self.create_publisher(TwistWithCovarianceStamped, self.output_topic, 10)
        self.create_subscription(Odometry, self.input_topic, self._on_odom, 10)
        self.get_logger().info(f'odom_to_twist: {self.input_topic} -> {self.output_topic}')

    def _on_odom(self, msg):
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.header.frame_id = msg.child_frame_id or msg.header.frame_id
        out.twist.twist = msg.twist.twist
        out.twist.covariance = _COVARIANCE
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
