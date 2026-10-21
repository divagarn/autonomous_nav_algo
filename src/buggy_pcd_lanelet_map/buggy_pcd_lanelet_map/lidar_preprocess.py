#!/usr/bin/env python3
"""LiDAR pre-processing for NDT scan matching (the Autoware "pointcloud
preprocessor" stage), in the velodyne sensor frame:

  drop NaN/Inf (no-return beams crash NDT's KdTree) -> crop the buggy's own
  body (static, would pull the match) -> range limit -> voxel-grid centroid
  downsample -> rate limit (VLP-16 at 1200 rpm is 20 Hz; NDT is run at
  max_rate_hz).

Input is the RAW /velodyne_points (ground included - the map contains it,
so do not feed the ground-filtered cloud). Output /localization/
pointcloud_downsampled, xyz float32, same frame/stamp.

Self crop defaults assume the real mount (LiDAR 1.73 m ahead of the rear
axle, 1.0 m high, chassis 2.82 x 1.35 m): box in the velodyne frame.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointField, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def voxel_centroids(pts, leaf):
    ijk = np.floor(pts / leaf).astype(np.int64)
    ijk -= ijk.min(axis=0)
    dims = ijk.max(axis=0) + 1
    key = (ijk[:, 0] * dims[1] + ijk[:, 1]) * dims[2] + ijk[:, 2]
    _, inv = np.unique(key, return_inverse=True)
    counts = np.bincount(inv).astype(np.float64)
    return np.column_stack(
        [np.bincount(inv, weights=pts[:, k]) / counts for k in range(3)]).astype(np.float32)


class LidarPreprocess(Node):
    def __init__(self):
        super().__init__('lidar_preprocess')
        p = self.declare_parameter
        self.leaf = float(p('voxel_size', 0.3).value)
        self.max_range = float(p('max_range', 40.0).value)
        self.min_range = float(p('min_range', 0.5).value)
        self.box = (float(p('self_x_min', -2.5).value), float(p('self_x_max', 0.6).value),
                    float(p('self_y_min', -0.9).value), float(p('self_y_max', 0.9).value),
                    float(p('self_z_min', -1.2).value), float(p('self_z_max', 0.9).value))
        self.min_period = 1.0 / float(p('max_rate_hz', 10.0).value)
        in_topic = p('input_topic', '/velodyne_points').value
        out_topic = p('output_topic', '/localization/pointcloud_downsampled').value
        self.pub = self.create_publisher(PointCloud2, out_topic, 5)
        self.create_subscription(PointCloud2, in_topic, self.on_cloud, 5)
        self.last = None
        self.n = 0
        self.get_logger().info(f'{in_topic} -> {out_topic}  voxel={self.leaf} m '
                               f'range=[{self.min_range},{self.max_range}] m')

    def on_cloud(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last is not None and now - self.last < self.min_period * 0.9:
            return
        self.last = now
        a = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if a.size == 0:
            return
        pts = np.column_stack([a['x'], a['y'], a['z']]).astype(np.float32)
        pts = pts[np.isfinite(pts).all(axis=1)]
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r > self.min_range) & (r < self.max_range)]
        x0, x1, y0, y1, z0, z1 = self.box
        inside = ((pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1) &
                  (pts[:, 2] > z0) & (pts[:, 2] < z1))
        pts = pts[~inside]
        if len(pts) < 100:
            self.get_logger().warn('too few points after filtering', throttle_duration_sec=2.0)
            return
        pts = voxel_centroids(pts, self.leaf)

        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, len(pts)
        out.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        out.is_bigendian, out.point_step = False, 12
        out.row_step, out.is_dense = 12 * len(pts), True
        out.data = pts.tobytes()
        self.pub.publish(out)
        self.n += 1
        if self.n % 100 == 1:
            self.get_logger().info(f'scan {self.n}: {len(a)} -> {len(pts)} points')


def main():
    rclpy.init()
    rclpy.spin(LidarPreprocess())


if __name__ == '__main__':
    main()
