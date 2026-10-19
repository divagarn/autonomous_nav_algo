#!/usr/bin/env python3
"""Shows the vehicle in RViz once an initial pose is given (as Autoware does).

Listens on /initialpose (RViz "2D Pose Estimate"), then
  - broadcasts map -> base_link at that pose (held until a new pose arrives)
  - publishes the vehicle as a MarkerArray (chassis box + 4 wheels) on
    /vehicle/marker, in base_link, using buggy_description's dimensions.

base_link follows buggy.urdf.xacro: midway between the axles, at wheel-axle
height, so base_link z = pose z + wheel_radius.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class InitialPoseVehicle(Node):
    def __init__(self):
        super().__init__('initial_pose_vehicle')
        p = self.declare_parameter
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.length = p('chassis_length', 2.8194).value
        self.width = p('chassis_width', 1.3462).value
        self.body_top = p('body_top_height', 0.85).value  # above ground
        self.wheel_radius = p('wheel_radius', 0.23).value
        self.wheel_width = p('wheel_width', 0.18).value
        self.wheelbase = p('wheel_base', 1.6).value
        self.track = p('wheel_tread', 1.10).value
        # live: map->base_link comes from the localization (EKF), not from a
        # click - only draw the model. base_link is then the REAR AXLE at
        # ground level (velodyne_static_tf: LiDAR 1.73 m ahead of it).
        self.live = p('live', False).value
        center = p('base_link_at_center', False).value   # Gazebo URDF: chassis centre
        self.x_off = self.wheelbase / 2.0 if (self.live and not center) else 0.0
        self.z_off = self.wheel_radius if (self.live and not center) else 0.0
        self.pose = None

        self.tf = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/vehicle/marker', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_pose, 10)
        # simulated driving (operation_manager in AUTO) moves the vehicle
        if not self.live:
            self.create_subscription(PoseStamped, '/vehicle/sim_pose',
                                     lambda m: setattr(self, 'pose', m.pose), 10)
        self.steer_rad = 0.0
        if self.live:
            # real front-wheel angle from the ECU (deg, +right)
            from ground_segmentation_msgs.msg import SteerSpeed
            self.create_subscription(
                SteerSpeed, '/wheel_uart',
                lambda m: setattr(self, 'steer_rad', -math.radians(m.steering)), 10)
        else:
            self.create_subscription(
                Float32, '/sim/steering_angle',
                lambda m: setattr(self, 'steer_rad', -math.radians(m.data)), 10)  # +right -> +left
        self.create_timer(0.05, self.tick)
        self.get_logger().info('waiting for /initialpose (RViz: 2D Pose Estimate)')

    def on_pose(self, msg):
        if self.live:
            return
        self.pose = msg.pose.pose
        self.get_logger().info(
            f'initial pose x={self.pose.position.x:.2f} y={self.pose.position.y:.2f}')

    def tick(self):
        now = self.get_clock().now().to_msg()
        if self.live:
            self.marker_pub.publish(self.markers(now))
            return
        if self.pose is None:
            return
        t = TransformStamped()
        t.header.stamp, t.header.frame_id = now, self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.pose.position.x
        t.transform.translation.y = self.pose.position.y
        t.transform.translation.z = self.pose.position.z + self.wheel_radius
        t.transform.rotation = self.pose.orientation
        self.tf.sendTransform(t)
        self.marker_pub.publish(self.markers(now))

    def markers(self, stamp):
        def base(mid, mtype):
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, self.base_frame
            m.ns, m.id, m.type, m.action = 'vehicle', mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        arr = MarkerArray()
        box_h = self.body_top - self.wheel_radius
        body = base(0, Marker.CUBE)
        body.pose.position.x = self.x_off
        body.pose.position.z = box_h / 2.0 + self.z_off
        body.scale.x, body.scale.y, body.scale.z = self.length, self.width, box_h
        body.color.r, body.color.g, body.color.b, body.color.a = 0.1, 0.5, 0.9, 0.8
        arr.markers.append(body)

        # arrow along +x so heading is obvious
        arrow = base(1, Marker.ARROW)
        arrow.pose.position.x = self.x_off
        arrow.pose.position.z = box_h + 0.05 + self.z_off
        arrow.scale.x, arrow.scale.y, arrow.scale.z = self.length * 0.6, 0.15, 0.15
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0, 1.0
        arr.markers.append(arrow)

        s = math.sqrt(0.5)  # rotate cylinder axis (z) onto y
        i = 2
        for dx in (self.wheelbase / 2, -self.wheelbase / 2):
            for dy in (self.track / 2, -self.track / 2):
                w = base(i, Marker.CYLINDER)
                w.pose.position.x, w.pose.position.y = dx + self.x_off, dy
                w.pose.position.z = self.z_off
                # cylinder axis z -> y (Rx 90 deg); front wheels also yaw by steering
                yaw = self.steer_rad if dx > 0 else 0.0
                cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
                w.pose.orientation.x, w.pose.orientation.y = s * cy, s * sy
                w.pose.orientation.z, w.pose.orientation.w = s * sy, s * cy
                w.scale.x = w.scale.y = 2 * self.wheel_radius
                w.scale.z = self.wheel_width
                w.color.r = w.color.g = w.color.b = 0.1
                w.color.a = 1.0
                arr.markers.append(w)
                i += 1
        return arr


def main():
    rclpy.init()
    rclpy.spin(InitialPoseVehicle())


if __name__ == '__main__':
    main()
