#!/usr/bin/env python3
"""Light copy of the PCD map for RViz (the NDT / planners keep using the full map).

The full map has millions of points. Drawn as 1-pixel points in RViz's top-down view, many of them land
on the same pixel at slightly different heights; the GPU depth test then picks between them frame by
frame, so the map shimmers / flickers / changes density while anything in the scene moves. One point per
voxel (default 0.3 m) leaves at most one point per pixel at normal zoom, so there is nothing to fight over,
and the cloud is ~20x smaller to draw.

/map/pointcloud_map (latched) -> voxel grid -> /map/pointcloud_map_display (latched)
voxel_size <= 0 republishes the full map unchanged.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2


class MapDisplayCloud(Node):
    def __init__(self):
        super().__init__('map_display_cloud')
        self.voxel = float(self.declare_parameter('voxel_size', 0.3).value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, '/map/pointcloud_map_display', qos)
        self.create_subscription(PointCloud2, '/map/pointcloud_map', self.on_map, qos)

    def on_map(self, msg):
        if self.voxel <= 0.0:
            self.pub.publish(msg)
            return
        st = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.column_stack([st['x'], st['y'], st['z']]).astype(np.float32)
        idx = np.floor(xyz / self.voxel).astype(np.int64)
        idx -= idx.min(axis=0)
        span = idx.max(axis=0) + 1
        key = (idx[:, 0] * span[1] + idx[:, 1]) * span[2] + idx[:, 2]
        _, first = np.unique(key, return_index=True)
        out = np.ascontiguousarray(xyz[first])
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width = 1, len(out)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.is_bigendian, cloud.point_step = False, 12
        cloud.row_step, cloud.is_dense = 12 * len(out), True
        cloud.data = out.tobytes()
        self.pub.publish(cloud)
        self.get_logger().info(f'display map: {len(xyz)} -> {len(out)} points '
                               f'(voxel {self.voxel} m)')


def main():
    rclpy.init()
    rclpy.spin(MapDisplayCloud())


if __name__ == '__main__':
    main()
