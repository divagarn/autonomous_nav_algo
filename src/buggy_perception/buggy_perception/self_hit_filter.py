#!/usr/bin/env python3
"""Drop vehicle self-hit points from ground_segmentation's /non_ground_points,
before they reach the local costmap.

Simulation-only concern: ground_segmentation is the team's real-hardware
code and does no self-filtering (the real Velodyne mount doesn't clip the
chassis the way it does in Gazebo, where the sensor sits only ~15cm above
the roof). Without this, the LiDAR's self-hit returns on the chassis/hood
get classified as non-ground obstacles, forming a lethal costmap blob that
overlaps the robot's own footprint - every rollout starts already "in
collision" and MPPI's optimizer fails on every cycle regardless of motion
model. Box/range values are the same ones already validated against this
same buggy_description URDF in the original (pre-team-merge) workspace's
segment_ground.py self-footprint filter.

Also drops anything within the ROBOT_LENGTH x ROBOT_WIDTH rectangle
around the vehicle's center (base_link), regardless of simulation vs
real data - a broader, sensor-independent version of the same idea, for
ground-plane noise/grazing returns right around the chassis that the
narrower box above doesn't catch. Points arrive in the sensor frame
(frame_id=velodyne / lidar_mount_link); SENSOR_OFFSET_* is that frame's
own fixed, no-rotation mount offset from base_link (velodyne_static_tf.py:
x=1.73m forward, z=1.00m up), so converting to base_link-frame XY is
just an offset add, not a full TF lookup.

ROBOT_LENGTH/ROBOT_WIDTH are the vehicle's real overall dimensions
(user-supplied measurement), not buggy_description's URDF footprint
(2.8194m x 1.3462m) - noticeably larger, particularly in width. This
exclusion zone and the URDF-based costmap footprint/inflation_radius are
two independent things; if the real vehicle is actually this much wider,
the costmap footprint config likely needs the same update, but that
hasn't been done here - flagged, not assumed.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

SELF_FILTER_MIN_RANGE = 1.0
SELF_FILTER_X_MIN = -2.39
SELF_FILTER_X_MAX = 0.53
SELF_FILTER_Y_MIN = -0.72
SELF_FILTER_Y_MAX = 0.72
SELF_FILTER_Z_MIN = -1.05
SELF_FILTER_Z_MAX = -0.10

# base_link -> velodyne static mount offset (velodyne_static_tf.py), no
# rotation currently (roll=pitch=yaw=0), so sensor-frame -> base_link-frame
# is just this translation.
SENSOR_OFFSET_X = 1.73
SENSOR_OFFSET_Y = 0.0

# Real vehicle overall dimensions (user-measured), centered on base_link.
ROBOT_LENGTH = 4.9
ROBOT_WIDTH = 2.9
ROBOT_HALF_LENGTH = ROBOT_LENGTH / 2.0
ROBOT_HALF_WIDTH = ROBOT_WIDTH / 2.0


class SelfHitFilter(Node):
    def __init__(self):
        super().__init__('self_hit_filter')
        # configurable (config/buggy_params.yaml, section self_hit_filter); the module constants are the defaults
        p = self.declare_parameter
        self.min_range = p('min_range', SELF_FILTER_MIN_RANGE).value
        self.box = (p('box_x_min', SELF_FILTER_X_MIN).value, p('box_x_max', SELF_FILTER_X_MAX).value,
                    p('box_y_min', SELF_FILTER_Y_MIN).value, p('box_y_max', SELF_FILTER_Y_MAX).value,
                    p('box_z_min', SELF_FILTER_Z_MIN).value, p('box_z_max', SELF_FILTER_Z_MAX).value)
        self.sensor_dx = p('sensor_offset_x', SENSOR_OFFSET_X).value
        self.sensor_dy = p('sensor_offset_y', SENSOR_OFFSET_Y).value
        self.half_length = p('robot_length', ROBOT_LENGTH).value / 2.0
        self.half_width = p('robot_width', ROBOT_WIDTH).value / 2.0
        self.pub = self.create_publisher(PointCloud2, '/non_ground_points_filtered', 2)
        self.create_subscription(
            PointCloud2, '/non_ground_points', self.callback, 2)

        # Visualizes the exclusion rectangle around the vehicle in RViz - a
        # static shape in base_link, so a slow timer is enough (no need to
        # republish every point cloud callback).
        self.radius_marker_pub = self.create_publisher(Marker, '/robot_radius_marker', 1)
        self.create_timer(1.0, self.publish_radius_marker)

    def publish_radius_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_radius"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.9
        corners = [
            (self.half_length, self.half_width),
            (self.half_length, -self.half_width),
            (-self.half_length, -self.half_width),
            (-self.half_length, self.half_width),
            (self.half_length, self.half_width),
        ]
        for x, y in corners:
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            marker.points.append(p)
        self.radius_marker_pub.publish(marker)

    def callback(self, msg):
        if msg.width == 0:
            return
        pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        xyz = pts[:, :3]
        x0, x1, y0, y1, z0, z1 = self.box
        too_close = np.linalg.norm(xyz, axis=1) < self.min_range
        inside_box = (
            (xyz[:, 0] > x0) & (xyz[:, 0] < x1) &
            (xyz[:, 1] > y0) & (xyz[:, 1] < y1) &
            (xyz[:, 2] > z0) & (xyz[:, 2] < z1)
        )
        base_link_x = xyz[:, 0] + self.sensor_dx
        base_link_y = xyz[:, 1] + self.sensor_dy
        inside_robot_rect = (
            (np.abs(base_link_x) < self.half_length) &
            (np.abs(base_link_y) < self.half_width)
        )
        kept = pts[~(too_close | inside_box | inside_robot_rect)]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(kept)
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * len(kept)
        out.data = kept.tobytes()
        out.is_dense = True
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SelfHitFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
