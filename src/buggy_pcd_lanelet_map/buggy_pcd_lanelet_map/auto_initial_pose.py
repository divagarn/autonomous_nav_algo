#!/usr/bin/env python3
"""Gives localization_manager the start pose without an RViz click (./buggy ... --init X,Y,YAW).

Publishes `initial_pose` ("x,y,yaw_deg" in map) on /initialpose - exactly what RViz 2D Pose Estimate
sends - once a LiDAR scan has arrived and /localization/state says uninitialized. NDT then aligns
around it as after a click. Retries after align_failed (max_attempts), then exits once the state is
initialized / converged / degraded; a click in RViz still re-seeds at any time.
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')
        text = self.declare_parameter('initial_pose', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.max_attempts = int(self.declare_parameter('max_attempts', 5).value)
        self.retry_sec = float(self.declare_parameter('retry_sec', 10.0).value)
        try:
            self.x, self.y, yaw_deg = (float(v) for v in text.split(','))
        except ValueError:
            raise SystemExit(f'auto_initial_pose: initial_pose must be "x,y,yaw_deg", got "{text}"')
        self.yaw = math.radians(yaw_deg)
        self.state = None
        self.scan = False
        self.attempts = 0
        self.sent_at = None
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(String, '/localization/state', self.on_state, 10)
        self.create_subscription(PointCloud2, '/localization/pointcloud_downsampled', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'start pose x={self.x:.2f} y={self.y:.2f} yaw={yaw_deg:.1f} deg '
                               '(sent when the first LiDAR scan arrives)')

    def on_state(self, msg):
        self.state = dict(p.split('=', 1) for p in msg.data.split(';') if '=' in p).get('state')

    def on_scan(self, _):
        self.scan = True

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.state in ('initialized', 'converged', 'degraded', 'lost'):
            self.get_logger().info(f'localisation {self.state}: done')
            raise SystemExit
        if not self.scan or self.state not in ('uninitialized', 'align_failed'):
            return   # no scan yet, manager not up, or aligning
        if self.sent_at is not None and self.now_s() - self.sent_at < self.retry_sec:
            return
        if self.attempts >= self.max_attempts:
            self.get_logger().error(f'no alignment after {self.attempts} tries: give the pose in RViz '
                                    '(2D Pose Estimate) or check --init')
            raise SystemExit
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = self.x, self.y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pub.publish(m)
        self.attempts += 1
        self.sent_at = self.now_s()
        self.get_logger().info(f'start pose sent (try {self.attempts}, state was {self.state})')


def main():
    rclpy.init()
    try:
        rclpy.spin(AutoInitialPose())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
