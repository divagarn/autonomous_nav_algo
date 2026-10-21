#!/usr/bin/env python3
"""Accumulates point clouds into a single, bounded-memory map in a target
fixed frame - common to live sensors AND ros2 bag play (a subscriber
can't tell the difference), so the same node builds a point-cloud map
whether driving live or replaying a recorded bag through the same
perception pipeline (velodyne_static_tf + wheel_odometry for 'odom', or
additionally slam_toolbox in mapping mode for a loop-closure-corrected
'map').

First use case (2026-10): none of this project's existing bags
(new_delhi_bag_*, SEP_25, golfcart_*) have dynamic /tf recorded, only
raw /velodyne_points (+/wheel_uart) - so velodyne_static_tf +
wheel_odometry must be run ALONGSIDE `ros2 bag play` to reconstruct the
odom->base_link->velodyne chain this node needs. Target 'odom' first to
validate the accumulation mechanism itself; add slam_toolbox mapping
later for a true loop-closure-corrected 'map' frame accumulation.

Writes a dependency-free ASCII .pcd file (no open3d/pypcd installed in
this workspace - same "hand-roll the format, no new dependency" choice
pose_graph_to_lanelet.py already made for OSM-XML) on:
  - the /save_pointcloud_map (std_srvs/Trigger) service, or
  - shutdown (Ctrl+C) - always saves whatever's accumulated so far.

Usage (after separately running velodyne_static_tf + wheel_odometry, and
either live sensors or `ros2 bag play <bag> --clock` with
use_sim_time:=true on all three nodes):
    ros2 run buggy_mapping pointcloud_map_accumulator --ros-args \\
        -p input_topic:=/velodyne_points -p target_frame:=odom \\
        -p output_file:=/path/to/map.pcd -p use_sim_time:=true
"""
import os

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformException


def voxel_dedup(pts, voxel_size):
    """Grid-hash dedup (average point per occupied cell) - same keying
    technique segment_ground.py's c_voxel_ds() uses, kept here to bound
    memory on a long accumulation instead of growing forever."""
    if len(pts) == 0:
        return pts
    vi = np.floor(pts / voxel_size).astype(np.int64)
    keys = vi[:, 0] * 73856093 ^ vi[:, 1] * 19349663 ^ vi[:, 2] * 83492791
    _, inv, cnt = np.unique(keys, return_inverse=True, return_counts=True)
    s = np.zeros((len(cnt), 3), dtype=np.float64)
    np.add.at(s, inv, pts.astype(np.float64))
    return (s / cnt[:, None]).astype(np.float32)


def quat_to_rotation_matrix(x, y, z, w):
    """Full 3D rotation from quaternion - unlike the rest of this stack's
    2D-planar yaw-only transforms, accumulated map points carry real z
    and the sensor mount itself has a fixed tilt, so a 2D rotation isn't
    enough here."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def write_pcd(path, pts):
    """Minimal dependency-free ASCII .pcd (v0.7) writer."""
    n = len(pts)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA ascii\n'
    )
    with open(path, 'w') as f:
        f.write(header)
        for x, y, z in pts:
            f.write(f'{x:.4f} {y:.4f} {z:.4f}\n')


class PointcloudMapAccumulator(Node):

    def __init__(self):
        super().__init__('pointcloud_map_accumulator')
        self.input_topic = self.declare_parameter('input_topic', '/velodyne_points').value
        self.target_frame = self.declare_parameter('target_frame', 'odom').value
        self.voxel_size = float(self.declare_parameter('voxel_size', 0.1).value)
        self.compact_every = int(self.declare_parameter('compact_every_n_msgs', 20).value)
        self.output_file = self.declare_parameter(
            'output_file', os.path.expanduser('~/pointcloud_map.pcd')).value
        self.max_range = float(self.declare_parameter('max_range', 30.0).value)
        # TF lookups use each message's OWN stamp (not "latest") so bag
        # replay places points using the transform as it was at capture
        # time, not whatever TF happens to be live when the message is
        # finally processed - the buffer needs enough history to still
        # have that stamp's transform by the time a (possibly queued)
        # cloud callback runs.
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Compacted (deduplicated) accumulation, plus a batch of pending
        # arrays not yet folded in. Appending to `pending` is O(1) -
        # np.vstack-ing straight into `self.accum` on every single message
        # was O(total_accumulated_size) EACH call (full copy), making the
        # whole run O(n^2) in total points. With real Velodyne-scale clouds
        # (tens of thousands of points/message) that fell behind real time
        # badly enough that the subscription's own queue (depth 10) dropped
        # most of the backlog outright - confirmed live: only 9 of 872
        # clouds made it through on bag new_delhi_bag_3 before this fix.
        self.accum = np.empty((0, 3), dtype=np.float32)
        self.pending = []
        self.msgs_since_compact = 0
        self.n_received = 0
        self.n_integrated = 0
        self.n_dropped = 0
        self.waiting = []
        self.create_timer(0.05, self._retry_waiting)

        # Queue depth 50, not rclpy's default 10 - a brief processing
        # hiccup (e.g. mid-compaction) should queue messages, not silently
        # drop them, now that per-message cost no longer scales with total
        # accumulated size.
        self.create_subscription(PointCloud2, self.input_topic, self._on_cloud, 50)
        self.create_service(Trigger, '/save_pointcloud_map', self._on_save)

        self.get_logger().info(
            f'pointcloud_map_accumulator: {self.input_topic} -> {self.target_frame}, '
            f'voxel_size={self.voxel_size}m, max_range={self.max_range}m, '
            f'output_file={self.output_file}. Call /save_pointcloud_map any time, '
            f'or Ctrl+C to save on exit.')

    def _on_cloud(self, msg):
        self.n_received += 1
        if not self._integrate(msg):
            # TF for this stamp usually arrives a few ms after the cloud
            # (blocking here would stall the executor the TF listener runs
            # on) - park it and retry from a timer, drop it after 5 s.
            self.waiting.append((self.get_clock().now(), msg))

    def _retry_waiting(self):
        keep = []
        for arrived, msg in self.waiting:
            if self._integrate(msg):
                continue
            if (self.get_clock().now() - arrived).nanoseconds < 5e9:
                keep.append((arrived, msg))
            else:
                self.n_dropped += 1
        self.waiting = keep

    def _integrate(self, msg):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp)
        except TransformException:
            return False

        # read_points_numpy() asserts ALL of the message's fields share one
        # datatype before any field_names filtering happens - real Velodyne
        # clouds have float32 x/y/z alongside uint8/uint16 intensity/ring/
        # time fields, which trips that assertion every time. read_points()
        # builds a structured (mixed-dtype) array first and filters after,
        # so it handles real sensor data; read_points_numpy() does not.
        structured = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured.size == 0:
            return True
        pts = np.column_stack(
            [structured['x'], structured['y'], structured['z']]).astype(np.float32)
        rad = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)
        pts = pts[rad < self.max_range]
        if len(pts) == 0:
            return True

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        q = t.transform.rotation
        rot = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        world_pts = (pts.astype(np.float64) @ rot.T) + np.array([tx, ty, tz])
        world_pts = world_pts.astype(np.float32)

        self.pending.append(world_pts)
        self.n_integrated += 1
        self.msgs_since_compact += 1
        if self.msgs_since_compact >= self.compact_every:
            self._fold_pending()
        return True

    def _fold_pending(self):
        """Concatenate the batch of pending arrays into self.accum ONCE,
        then voxel-dedup - O(batch size + current accum size) per call
        instead of per-message, which is what keeps this able to keep up
        with a real sensor's message rate."""
        if not self.pending:
            return
        before = len(self.accum)
        merged = np.vstack([self.accum] + self.pending)
        self.pending = []
        self.accum = voxel_dedup(merged, self.voxel_size)
        self.msgs_since_compact = 0
        self.get_logger().info(
            f'Compacted: {before} -> {len(self.accum)} points '
            f'({self.n_integrated}/{self.n_received} clouds integrated so far).',
            throttle_duration_sec=5.0)

    def _on_save(self, request, response):
        ok, msg = self._save()
        response.success = ok
        response.message = msg
        return response

    def _save(self):
        self._fold_pending()
        if len(self.accum) == 0:
            return False, 'No points accumulated yet - nothing to save.'
        final = voxel_dedup(self.accum, self.voxel_size)
        try:
            write_pcd(self.output_file, final)
        except Exception as e:
            return False, f'Failed to write {self.output_file}: {e}'
        msg = (f'Saved {len(final)} points to {self.output_file} '
               f'({self.n_integrated}/{self.n_received} clouds integrated).')
        self.get_logger().info(msg)
        return True, msg

    def destroy_node(self):
        if len(self.accum) > 0:
            self.get_logger().info('Shutting down - saving accumulated map...')
            self._save()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudMapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
