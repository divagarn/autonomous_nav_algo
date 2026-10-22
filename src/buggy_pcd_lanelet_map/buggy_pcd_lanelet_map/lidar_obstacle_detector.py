#!/usr/bin/env python3
"""Obstacles in the vehicle's path from the VLP-16 (ground already removed).

Input : /non_ground_points_filtered (segment_ground -> self_hit_filter), sensor
        frame; the sensor sits at (sensor_x, 0, sensor_z) in base_link
        (velodyne_static_tf: 1.73 m ahead of the rear axle, 1.00 m up, level).
Path  : /planning/route_path + TF map->base_link. Without a route, a straight
        corridor ahead is used, so objects are still reported.
Method: keep points between min_height and max_height above ground, inside a
        corridor (half_width) around the next corridor_length metres of path,
        grid-cluster them (0.3 m cells, 8-connected), keep clusters with enough
        points that were also seen in the previous scan (kills single-scan noise).
Ground aware (ground_aware:=true, default): the "height above ground" is measured from the LOCAL
        ground, not from a flat plane 1.0 m below the sensor. The ground points of the same scan
        (/ground_points) give a ground height map (median per 0.5 m cell, smoothed over 3 m) and,
        where the map is empty (far range), a plane fitted to all ground points, so a slope, a
        bump, or the vehicle pitching under braking no longer turns road surface into an obstacle.
        A cluster that is both thin (vertical extent < thin_extent) and low (below low_height) is a
        single beam ring on the road, not an object, and is dropped (reject_thin_low).
Output: /perception/lidar_objects (autoware_perception_msgs/PredictedObjects, map
        frame) published for EVERY processed scan, even when empty - it is the
        heartbeat operation_manager uses to know the detector is alive - plus
        /perception/lidar_objects_marker for RViz.
"""
import math

import numpy as np
import rclpy
from autoware_perception_msgs.msg import ObjectClassification, PredictedObject, PredictedObjects
from autoware_perception_msgs.msg import Shape
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from scipy import ndimage
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class GroundModel:
    """Local ground height from the ground points of one scan (base_link x/y, z = height above the
    flat reference plane). height(x, y): IDW-smoothed per-cell median where ground points are near,
    else the fitted ground plane, else 0 (the old flat assumption)."""

    def __init__(self, gx, gy, gz, length, cell=0.5, radius=3.0, x0=-4.0, y0=-10.0, y1=10.0,
                 plane_min_points=50, plane_gate=0.15, max_tilt_deg=15.0):
        self.cell, self.x0, self.y0 = cell, x0, y0
        self.nx = int(math.ceil((length + 10.0) / cell))
        self.ny = int(math.ceil((y1 - y0) / cell))
        ok = (gx >= x0) & (gx < x0 + self.nx * cell) & (gy >= y0) & (gy < y0 + self.ny * cell)
        gx, gy, gz = gx[ok].astype(np.float64), gy[ok].astype(np.float64), gz[ok].astype(np.float64)
        self.n_points = len(gx)
        ci = np.floor((gx - x0) / cell).astype(np.int64)
        cj = np.floor((gy - y0) / cell).astype(np.int64)
        med = np.zeros((self.nx, self.ny))
        valid = np.zeros((self.nx, self.ny))
        if len(gx):
            key = ci * self.ny + cj
            order = np.argsort(key, kind='stable')
            ks, zs = key[order], gz[order]
            starts = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1]])
            ends = np.r_[starts[1:], len(ks)]
            for a, b in zip(starts, ends):
                k = ks[a]
                med[k // self.ny, k % self.ny] = np.median(zs[a:b])
                valid[k // self.ny, k % self.ny] = 1.0
        r = int(math.floor(radius / cell))
        kernel = np.zeros((2 * r + 1, 2 * r + 1))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                d = cell * math.hypot(di, dj)
                if d <= radius:
                    kernel[di + r, dj + r] = 1.0 / max(d, 0.25) ** 2
        num = ndimage.convolve(med * valid, kernel, mode='constant', cval=0.0)
        den = ndimage.convolve(valid, kernel, mode='constant', cval=0.0)
        self.est = np.full((self.nx, self.ny), np.nan)
        m = den > 0
        self.est[m] = num[m] / den[m]
        self.plane = None
        if len(gx) >= plane_min_points:
            use = np.ones(len(gx), bool)
            coef = None
            for _ in range(3):
                A = np.column_stack([gx[use], gy[use], np.ones(use.sum())])
                if len(A) < 3:
                    coef = None
                    break
                coef = np.linalg.solve(A.T @ A, A.T @ gz[use])
                res = np.abs(gx * coef[0] + gy * coef[1] + coef[2] - gz)
                use = res < plane_gate
            if coef is not None and use.sum() >= plane_min_points and \
                    math.degrees(math.atan(math.hypot(coef[0], coef[1]))) <= max_tilt_deg:
                self.plane = coef

    def height(self, x, y):
        x, y = np.asarray(x, np.float64), np.asarray(y, np.float64)
        g = np.zeros(x.shape)
        if self.plane is not None:
            g = x * self.plane[0] + y * self.plane[1] + self.plane[2]
        ci = np.floor((x - self.x0) / self.cell)
        cj = np.floor((y - self.y0) / self.cell)
        inside = (ci >= 0) & (cj >= 0) & (ci < self.nx) & (cj < self.ny)
        ii, jj = np.where(inside, ci, 0).astype(np.int64), np.where(inside, cj, 0).astype(np.int64)
        e = self.est[ii, jj]
        use = inside & ~np.isnan(e)
        return np.where(use, e, g)


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        p = self.declare_parameter
        self.sx = p('sensor_x', 1.73).value
        self.sz = p('sensor_z', 1.00).value
        self.min_h = p('min_height', 0.20).value      # above ground
        self.max_h = p('max_height', 2.00).value
        self.half_width = p('corridor_half_width', 1.25).value
        self.length = p('corridor_length', 25.0).value
        self.cell = p('cell_size', 0.30).value
        self.min_pts = p('min_points', 5).value
        self.max_hz = p('max_rate_hz', 10.0).value
        self.ground_aware = p('ground_aware', True).value
        self.ground_min_h = p('ground_min_height', 0.30).value   # above LOCAL ground (noise + classifier band)
        self.reject_thin_low = p('reject_thin_low', True).value
        self.thin_extent = p('thin_extent', 0.08).value      # vertical extent below this = one ring
        self.low_height = p('low_height', 0.50).value        # ... and lower than this above ground
        self.ground_max_age = p('ground_max_age', 0.3).value
        self.ground = None          # (stamp seconds, gx, gy, gz) of the latest /ground_points
        self.rejected = 0
        self.last = 0.0
        self.path = None            # np (M,2) in map
        self.prev = []              # centres (base frame) of last scan's clusters

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.obj_pub = self.create_publisher(PredictedObjects, '/perception/lidar_objects', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/lidar_objects_marker', 1)
        self.create_subscription(Path, '/planning/route_path', self.on_path, 1)
        self.create_subscription(PointCloud2, '/non_ground_points_filtered', self.on_cloud, 1)
        if self.ground_aware:
            self.create_subscription(PointCloud2, '/ground_points', self.on_ground, 1)

    @staticmethod
    def read_xyz(msg):
        if msg.point_step == 16:
            return np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)[:, :3]
        st = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return np.column_stack([st['x'], st['y'], st['z']]).astype(np.float32)

    def on_ground(self, msg):
        if msg.width == 0:
            return
        xyz = self.read_xyz(msg)
        self.ground = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                       xyz[:, 0] + self.sx, xyz[:, 1], xyz[:, 2] + self.sz)

    def ground_model(self, stamp):
        """GroundModel of this scan's ground points, or None (flat assumption) if there are none."""
        if not self.ground_aware or self.ground is None:
            return None
        t = stamp.sec + stamp.nanosec * 1e-9
        if abs(t - self.ground[0]) > self.ground_max_age:
            return None
        return GroundModel(self.ground[1], self.ground[2], self.ground[3], self.length)

    def on_path(self, msg):
        pts = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
        self.path = np.array(pts) if len(pts) > 1 else None

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def corridor(self, pose):
        """Centre line ahead of the vehicle in base_link, shape (M, 2)."""
        if self.path is None:
            xs = np.arange(0.0, self.length, 0.5)
            return np.column_stack([xs, np.zeros_like(xs)])
        rx, ry, yaw = pose
        d = self.path - np.array([rx, ry])
        c, s = math.cos(-yaw), math.sin(-yaw)
        b = np.column_stack([d[:, 0] * c - d[:, 1] * s, d[:, 0] * s + d[:, 1] * c])
        i0 = int(np.argmin(np.hypot(b[:, 0], b[:, 1])))
        b = b[i0:]
        seg = np.hypot(*np.diff(b, axis=0).T) if len(b) > 1 else np.array([])
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        b = b[cum <= self.length]
        if len(b) < 2:      # route ended / too short: keep looking straight ahead
            xs = np.arange(0.0, self.length, 0.5)
            return np.column_stack([xs, np.zeros_like(xs)])
        return b

    def on_cloud(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last < 1.0 / self.max_hz:
            return
        pose = self.pose()
        if pose is None:
            return
        self.last = now
        xyz = self.read_xyz(msg)
        # sensor frame -> base_link (translation only) and height above ground
        bx, by, bz = xyz[:, 0] + self.sx, xyz[:, 1], xyz[:, 2] + self.sz
        model = self.ground_model(msg.header.stamp)
        if model is not None:       # height above the LOCAL ground instead of the flat reference
            fin = np.isfinite(bx) & np.isfinite(by) & np.isfinite(bz)
            g = np.zeros(len(bx))
            g[fin] = model.height(bx[fin], by[fin])
            bz = bz - g
        min_h = self.ground_min_h if model is not None else self.min_h
        keep = (bz > min_h) & (bz < self.max_h) & (bx > 0.0) & (bx < self.length + 3.0) \
            & np.isfinite(bx) & np.isfinite(by)
        bx, by, bz = bx[keep], by[keep], bz[keep]

        clusters = []
        if len(bx) >= self.min_pts:
            tree = cKDTree(self.corridor(pose))
            dist, _ = tree.query(np.column_stack([bx, by]))
            inside = dist < self.half_width
            bx, by, bz = bx[inside], by[inside], bz[inside]
            if len(bx) >= self.min_pts:
                clusters = self.cluster(bx, by, bz)

        if self.reject_thin_low and model is not None:
            kept = [c for c in clusters
                    if not ((c['h'] - c['hmin']) < self.thin_extent and c['h'] < self.low_height)]
            self.rejected += len(clusters) - len(kept)
            clusters = kept
        confirmed = []
        for c in clusters:
            if any(math.hypot(c['cx'] - q[0], c['cy'] - q[1]) < 0.8 for q in self.prev):
                confirmed.append(c)
        self.prev = [(c['cx'], c['cy']) for c in clusters]
        self.publish(msg.header.stamp, pose, confirmed)

    def cluster(self, bx, by, bz):
        ix = np.floor((bx - bx.min()) / self.cell).astype(int)
        iy = np.floor((by - by.min()) / self.cell).astype(int)
        grid = np.zeros((ix.max() + 1, iy.max() + 1), dtype=bool)
        grid[ix, iy] = True
        labels, n = ndimage.label(grid, structure=np.ones((3, 3)))
        lab = labels[ix, iy]
        out = []
        for k in range(1, n + 1):
            m = lab == k
            if m.sum() < self.min_pts:
                continue
            x0, x1, y0, y1 = bx[m].min(), bx[m].max(), by[m].min(), by[m].max()
            out.append({'cx': float((x0 + x1) / 2), 'cy': float((y0 + y1) / 2),
                        'lx': float(max(x1 - x0, 0.4)), 'ly': float(max(y1 - y0, 0.4)),
                        'h': float(bz[m].max()), 'hmin': float(bz[m].min()), 'n': int(m.sum()),
                        'near': float(x0)})
        return out

    def publish(self, stamp, pose, clusters):
        rx, ry, yaw = pose
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        objs = PredictedObjects()
        objs.header.stamp, objs.header.frame_id = stamp, 'map'
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for n, c in enumerate(clusters):
            o = PredictedObject()
            o.existence_probability = 1.0
            o.classification = [ObjectClassification(label=ObjectClassification.UNKNOWN,
                                                     probability=1.0)]
            pp = o.kinematics.initial_pose_with_covariance.pose
            pp.position.x = rx + c['cx'] * cy_ - c['cy'] * sy_
            pp.position.y = ry + c['cx'] * sy_ + c['cy'] * cy_
            pp.position.z = c['h'] / 2
            pp.orientation.z, pp.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            o.shape.type = Shape.BOUNDING_BOX
            o.shape.dimensions.x, o.shape.dimensions.y, o.shape.dimensions.z = \
                c['lx'], c['ly'], c['h']
            objs.objects.append(o)

            m = Marker()
            m.header = objs.header
            m.ns, m.id, m.type, m.action = 'lidar_obstacle', n, Marker.CUBE, Marker.ADD
            m.pose = pp
            m.scale.x, m.scale.y, m.scale.z = c['lx'], c['ly'], c['h']
            m.color.r, m.color.a = 1.0, 0.6
            markers.markers.append(m)
        self.obj_pub.publish(objs)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    rclpy.spin(LidarObstacleDetector())


if __name__ == '__main__':
    main()
