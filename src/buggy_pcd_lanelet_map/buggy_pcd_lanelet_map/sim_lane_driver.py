#!/usr/bin/env python3
"""Drives the Gazebo buggy along a Lanelet2 lane (pure pursuit) - used to
sweep the sim world once so pointcloud_map_accumulator can record a PCD.

Same steering law and limits as operation_manager (wheelbase 1.6 m, max
20 deg, lookahead 4 m) and same output convention as the real stack:
/steering_angle [deg, +right] and /vehicle_speed [km/h], consumed by
this package's sim_ackermann_bridge. Pose comes from TF <path_frame> ->
base_link (sim base_link = chassis centre, per buggy.urdf.xacro).

The lanelet's coordinates are slam_toolbox "map" coordinates, which for
my_map2 are the spawn pose frame = the sim's odom frame (path_frame).
Stops (speed 0) at the end of the lane, or after one lap.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from buggy_pcd_lanelet_map.path_utils import smooth
from buggy_pcd_lanelet_map.route_planner import load_lanelets


class SimLaneDriver(Node):
    def __init__(self):
        super().__init__('sim_lane_driver')
        osm = self.declare_parameter('lanelet2_map_path', '').value
        self.frame = self.declare_parameter('path_frame', 'odom').value
        self.speed = self.declare_parameter('speed_kmh', 6.0).value
        self.L = self.declare_parameter('wheelbase', 1.6).value
        self.max_steer = self.declare_parameter('max_steering_deg', 20.0).value
        self.lookahead = self.declare_parameter('lookahead_distance', 4.0).value
        self.base_at_center = self.declare_parameter('base_link_at_center', True).value

        pts = []
        for l in load_lanelets(osm):
            pts += l.center
        self.path, before, after = smooth(pts, self.L, 0.9 * self.max_steer, 0.6)
        self.path = [(p[0], p[1]) for p in self.path]
        self.seg = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.seg.append(self.seg[-1] + math.dist(a, b))
        self.idx = 0
        self.get_logger().info(
            f'lane {self.seg[-1]:.0f} m, peak steer {before:.0f} -> {after:.0f} deg')

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.steer_pub = self.create_publisher(Float32, '/steering_angle', 1)
        self.speed_pub = self.create_publisher(Float32, '/vehicle_speed', 1)
        self.create_timer(0.1, self.step)
        self.done = False
        self.last_log = 0.0

    def command(self, steer_deg, speed_kmh):
        self.steer_pub.publish(Float32(data=float(steer_deg)))
        self.speed_pub.publish(Float32(data=float(speed_kmh)))

    def step(self):
        try:
            t = self.tf.lookup_transform(self.frame, 'base_link', rclpy.time.Time())
        except TransformException:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        h = self.L / 2.0 if self.base_at_center else 0.0
        rx = t.transform.translation.x - h * math.cos(yaw)
        ry = t.transform.translation.y - h * math.sin(yaw)

        hi = min(len(self.path), self.idx + 80)
        self.idx = min(range(self.idx, hi), key=lambda i: math.hypot(
            self.path[i][0] - rx, self.path[i][1] - ry))
        remaining = self.seg[-1] - self.seg[self.idx]
        if self.done or remaining < 1.0:
            if not self.done:
                self.get_logger().info('end of lane reached - stopping')
            self.done = True
            self.command(0.0, 0.0)
            return

        i = self.idx
        while i < len(self.path) - 1 and self.seg[i] - self.seg[self.idx] < self.lookahead:
            i += 1
        dx, dy = self.path[i][0] - rx, self.path[i][1] - ry
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        delta = math.atan(2 * self.L * math.sin(alpha) / max(math.hypot(dx, dy), 1e-3))
        steer = max(-self.max_steer, min(self.max_steer, -math.degrees(delta)))
        self.command(steer, self.speed)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log > 10.0:
            self.last_log = now
            self.get_logger().info(
                f'progress {100 * self.seg[self.idx] / self.seg[-1]:.0f}%  '
                f'steer {steer:+.1f} deg')


def main():
    rclpy.init()
    rclpy.spin(SimLaneDriver())


if __name__ == '__main__':
    main()
