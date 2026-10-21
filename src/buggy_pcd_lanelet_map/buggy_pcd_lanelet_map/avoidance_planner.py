#!/usr/bin/env python3
"""Detour planner node (see avoidance.py for the geometry).

operation_manager asks for a detour when an obstacle has blocked the route for avoid_after_sec:
  /planning/avoid_request   std_msgs/String JSON {id, points: [[x,y,z]...] route ahead from the
                            vehicle, obstacles: [[x,y,yaw,length,width]...]}
  /planning/avoid_result    std_msgs/String JSON {id, ok, reason, side, offset, segment, ib, ...}
This node knows the lanes (lanelet2_map_path), and - if use_sensors - the LiDAR free space:
/non_ground_points_filtered (solid cells) and /ground_points (ground that was actually seen).
"""
import collections
import json
import math
import time

import numpy as np
import rclpy
from matplotlib.path import Path as MplPath
from rclpy.node import Node
from scipy import ndimage
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from buggy_pcd_lanelet_map.avoidance import FreeSpace, Params, plan_detour
from buggy_pcd_lanelet_map.route_planner import load_lanelets


class LaneModel:
    def __init__(self, lanelets):
        self.polys, bounds = [], []
        for l in lanelets:
            poly = [p[:2] for p in l.left] + [p[:2] for p in reversed(l.right)]
            self.polys.append(MplPath(np.array(poly)))
            bounds += [p[:2] for p in l.left] + [p[:2] for p in l.right]
        self.tree = cKDTree(np.array(bounds)) if bounds else None

    def inside(self, xy):
        ok = np.zeros(len(xy), dtype=bool)
        for poly in self.polys:
            ok |= poly.contains_points(xy)
        return ok

    def dist(self, xy):
        return self.tree.query(xy)[0] if self.tree is not None else np.zeros(len(xy))


class AvoidancePlanner(Node):
    def __init__(self):
        super().__init__('avoidance_planner')
        p = self.declare_parameter
        self.frame = p('map_frame', 'map').value
        osm = p('lanelet2_map_path', '').value
        self.use_sensors = p('use_sensors', False).value
        self.sensor_z = p('sensor_z', 1.0).value          # LiDAR height above ground
        self.keep_sec = p('scan_memory_sec', 1.5).value
        self.res = p('grid_res', 0.2).value
        self.prm = Params(
            wheelbase=p('wheelbase', 1.6).value, max_steer_deg=p('max_steering_deg', 20.0).value,
            veh_half_width=p('vehicle_half_width', 0.72).value,
            side_margin=p('side_margin', 0.35).value,
            avoid_half_width=p('avoid_corridor_half_width', 1.05).value,
            offlane_allow=p('offlane_allow_m', 0.8).value,
            offlane_allow_sensed=p('offlane_allow_sensed_m', 8.0).value,
            max_offset=p('max_offset_m', 6.5).value)
        self.lane = LaneModel(load_lanelets(osm)) if osm else None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nonground = collections.deque()    # (t, Nx2 map-frame points of solid obstacles)
        self.ground = collections.deque()       # (t, Nx2 map-frame ground points)
        self.pub = self.create_publisher(String, '/planning/avoid_result', 5)
        self.create_subscription(String, '/planning/avoid_request', self.on_request, 5)
        if self.use_sensors:
            self.create_subscription(PointCloud2, '/non_ground_points_filtered',
                                     lambda m: self.on_cloud(m, self.nonground, True), 1)
            self.create_subscription(PointCloud2, '/ground_points',
                                     lambda m: self.on_cloud(m, self.ground, False), 1)
        self.get_logger().info(f'avoidance planner up (LiDAR free space: {self.use_sensors})')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cloud(self, msg, store, solid):
        if msg.width == 0 or msg.point_step < 12:
            return
        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
        xyz = raw[:, :12].copy().view(np.float32).reshape(n, 3).astype(float)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if solid:       # height above ground: 0.2 .. 1.9 m (a branch above the vehicle is not an obstacle)
            h = xyz[:, 2] + self.sensor_z
            xyz = xyz[(h > 0.2) & (h < 1.9)]
        try:
            t = self.tf.lookup_transform(self.frame, msg.header.frame_id, rclpy.time.Time())
        except TransformException:
            return
        q, tr = t.transform.rotation, t.transform.translation
        yaw = 2.0 * math.atan2(q.z, q.w)                  # planar rotation is enough for x, y
        c, s = math.cos(yaw), math.sin(yaw)
        pts = np.column_stack([tr.x + c * xyz[:, 0] - s * xyz[:, 1],
                               tr.y + s * xyz[:, 0] + c * xyz[:, 1]])
        store.append((self.now(), pts))
        while store and self.now() - store[0][0] > self.keep_sec:
            store.popleft()

    def free_space(self, pts):
        """FreeSpace from the recent scans around the route, or None without LiDAR data."""
        if not self.use_sensors or not self.nonground or not self.ground:
            return None
        lo = pts[:, :2].min(axis=0) - 10.0
        hi = pts[:, :2].max(axis=0) + 10.0
        shape = np.ceil((hi - lo) / self.res).astype(int) + 1

        def grid(store, grow):
            g = np.zeros(shape, dtype=bool)
            for _, xy in store:
                ij = np.floor((xy - lo) / self.res).astype(int)
                ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
                g[ij[ok, 0], ij[ok, 1]] = True
            return ndimage.binary_dilation(g, iterations=grow) if grow else g
        # ground is dilated more: the VLP-16 rings are ~0.7 m apart at 10 m, the gaps are not holes
        return FreeSpace(lo, self.res, grid(self.nonground, 1), grid(self.ground, 3))

    def on_request(self, msg):
        t0 = time.time()
        try:
            req = json.loads(msg.data)
            pts = np.array(req['points'], dtype=float)
            obstacles = [tuple(o) for o in req['obstacles']]
        except (ValueError, KeyError):
            return
        free = self.free_space(pts)
        res = plan_detour(pts, obstacles, self.lane, free, self.prm)
        res.pop('zone', None)
        res['id'] = req.get('id')
        res['sensed'] = free is not None
        self.pub.publish(String(data=json.dumps(res)))
        self.get_logger().info(
            f"detour request {req.get('id')}: " +
            (f"OK {res['side']} {res['offset']:+.2f} m, {res['length']:.0f} m long"
             if res['ok'] else f"NO - {res['reason']}") +
            f" (LiDAR free space: {free is not None}, {1000 * (time.time() - t0):.0f} ms)")


def main():
    rclpy.init()
    rclpy.spin(AvoidancePlanner())


if __name__ == '__main__':
    main()
