#!/usr/bin/env python3
"""
Rebuild a drift-corrected point cloud map from a lidar-only rosbag.

Inputs
  --bag   original bag with /velodyne_points (sqlite3 or mcap rosbag2)
  --odom  bag recorded from KISS-ICP containing /kiss/odometry
          (pose of the lidar frame in odom_lidar, same stamps as the clouds)

What it does (all offline, capture order preserved)
  1. Estimates the lidar mount tilt + height from the ground plane.
  2. FLATTENS the KISS-ICP poses: keeps x, y, yaw and forces z = 0, roll = pitch = 0.
     (3D lidar odometry on a 16-beam sensor drifts badly in z/pitch; the buggy
      drives on level ground, so this removes the vertical drift.)
  3. Loop closure: ICP the last scans onto the first scans (yaw + xyz only),
     then spread the offset over the whole trajectory by travelled distance.
  4. Writes the map (ground at z = 0), the corrected poses and a PNG report.

  python3 scripts/rebuild_flat_map.py --bag ~/bags/new_delhi_bag_3 --odom /path/kiss_odom -o map.pcd
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd_align_tool import icp_point_to_plane, save_pcd, voxel_down  # noqa: E402


def open_reader(path, topics):
    import rosbag2_py
    storage = 'sqlite3'
    if any(f.endswith('.mcap') for f in os.listdir(path)):
        storage = 'mcap'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def read_odometry(path):
    if path.endswith('.npz'):
        z = np.load(path)
        o = np.argsort(z['stamps'])
        return z['stamps'][o], z['poses'][o]
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    r = open_reader(path, ['/kiss/odometry'])
    st, T = [], []
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        M = np.eye(4)
        M[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        M[:3, 3] = [p.x, p.y, p.z]
        st.append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)
        T.append(M)
    o = np.argsort(st)
    return np.array(st)[o], np.array(T)[o]


def cloud_xyz(msg):
    from sensor_msgs_py import point_cloud2 as pc2
    a = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if a.dtype.names:
        a = np.stack([a['x'], a['y'], a['z']], axis=1)
    return np.asarray(a, dtype=np.float64)


def fit_ground(pts, rng):
    """RANSAC plane among points below the sensor; returns (unit normal pointing up, height of sensor above plane)."""
    r = np.linalg.norm(pts[:, :2], axis=1)
    c = pts[(r > 3) & (r < 15) & (pts[:, 2] < 0)]
    if len(c) < 200:
        return None
    best, bn = None, 0
    for _ in range(300):
        s = c[rng.choice(len(c), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        if n[2] < 0:
            n = -n
        if n[2] < np.cos(np.radians(25)):
            continue
        d = -n @ s[0]
        cnt = int((np.abs(c @ n + d) < 0.08).sum())
        if cnt > bn:
            best, bn = (n, d), cnt
    if best is None:
        return None
    n, d = best
    inl = c[np.abs(c @ n + d) < 0.08]
    cen = inl.mean(0)
    n = np.linalg.svd(inl - cen)[2][-1]
    if n[2] < 0:
        n = -n
    return n, float(n @ cen)  # n.p = n.cen  -> plane; height of sensor = -(n.cen)


def level_rotation(n):
    """Rotation taking the measured up-vector n to +z."""
    z = np.array([0, 0, 1.0])
    v = np.cross(n, z)
    s, c = np.linalg.norm(v), n @ z
    if s < 1e-9:
        return np.eye(3)
    return R.from_rotvec(v / s * np.arctan2(s, c)).as_matrix()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--odom', required=True)
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--topic', default='/velodyne_points')
    ap.add_argument('--voxel', type=float, default=0.1)
    ap.add_argument('--min-range', type=float, default=1.5)
    ap.add_argument('--max-range', type=float, default=40.0)
    ap.add_argument('--closure-frac', type=float, default=0.06, help='fraction of scans used at each end for loop closure')
    ap.add_argument('--max-scans', type=int, default=0, help='only the first N scans (0 = all; quick test)')
    ap.add_argument('--no-closure', action='store_true')
    ap.add_argument('--keep-pitch-roll', action='store_true', help='do NOT flatten (debug)')
    a = ap.parse_args()
    if os.path.abspath(a.out) == os.path.abspath(a.bag):
        sys.exit('bad output path')

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    ost, oT = read_odometry(a.odom)
    print('odometry poses:', len(ost))

    # ---- pass 1: read clouds, match poses ----
    r = open_reader(a.bag, [a.topic])
    scans, poses, rec = [], [], []
    miss = 0
    while r.has_next():
        if a.max_scans and len(scans) + miss >= a.max_scans:
            break
        _, data, rec_t = r.read_next()
        m = deserialize_message(data, PointCloud2)
        t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        k = int(np.searchsorted(ost, t))
        cand = [j for j in (k - 1, k) if 0 <= j < len(ost)]
        j = min(cand, key=lambda j: abs(ost[j] - t))
        if abs(ost[j] - t) > 60e6:
            miss += 1
            continue
        p = cloud_xyz(m)
        rr = np.linalg.norm(p, axis=1)
        p = p[(rr > a.min_range) & (rr < a.max_range)]
        scans.append(p.astype(np.float32))
        poses.append(oT[j])
        rec.append(rec_t)                      # bag record time [ns]: lets gnss_georef.py match GNSS fixes
    poses = np.array(poses)
    print('scans with pose: %d, without: %d' % (len(scans), miss))
    n = len(scans)

    # ---- mount tilt / height ----
    rng = np.random.default_rng(0)
    ns, hs = [], []
    for i in np.linspace(0, n - 1, 40).astype(int):
        g = fit_ground(scans[i].astype(np.float64), rng)
        if g:
            ns.append(g[0])
            hs.append(g[1])
    nrm = np.median(ns, axis=0)
    nrm /= np.linalg.norm(nrm)
    height = -float(np.median(hs))
    Rl = level_rotation(nrm)
    print('ground normal in sensor frame %s -> tilt %.2f deg, sensor height ~%.2f m (%d/40 scans fit)'
          % (np.round(nrm, 4), np.degrees(np.arccos(nrm[2])), height, len(ns)))

    # ---- flatten poses ----
    # level frame of the sensor: p_level = Rl p ; pose of level frame = T @ Rl^T
    xy = np.zeros((n, 2))
    yaw = np.zeros(n)
    for i in range(n):
        Rw = poses[i][:3, :3] @ Rl.T
        if a.keep_pitch_roll:
            yaw[i] = np.arctan2(Rw[1, 0], Rw[0, 0])
        else:
            # heading = direction of the level frame's x axis projected on the ground plane
            yaw[i] = np.arctan2(Rw[1, 0], Rw[0, 0])
        xy[i] = poses[i][:2, 3]
    yaw = np.unwrap(yaw)
    zs = poses[:, 2, 3].copy() if a.keep_pitch_roll else np.zeros(n)
    path = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    print('path length %.1f m, raw KISS z range %.1f..%.1f m' % (path[-1], poses[:, 2, 3].min(), poses[:, 2, 3].max()))

    def to_world(i, off=None):
        p = scans[i].astype(np.float64) @ Rl.T
        c, s = np.cos(yaw[i]), np.sin(yaw[i])
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return p @ Rz.T + np.array([xy[i, 0], xy[i, 1], zs[i] + height])

    # ---- loop closure ----
    corr = None
    if not a.no_closure:
        K = max(5, int(a.closure_frac * n))
        head = voxel_down(np.vstack([to_world(i) for i in range(K)]), 0.15)
        tail = voxel_down(np.vstack([to_world(i) for i in range(n - K, n)]), 0.15)
        c0 = head.mean(0)
        print('start/end gap before closure: %.2f m (xy)' % np.linalg.norm(xy[0] - xy[-1]))
        res = icp_point_to_plane(tail, head, voxel=0.2, yaw_only=True,
                                 stages=((8.0, 10), (4.0, 15), (2.0, 20), (1.0, 20), (0.5, 20)))
        if res is None:
            print('Loop closure ICP failed - writing map without closure')
        else:
            Rm, t, c, matches, resid = res
            rv = R.from_matrix(Rm).as_rotvec()
            print('closure: yaw %.2f deg, shift [%.2f %.2f %.2f] m, %d matches, plane resid %.3f m'
                  % (np.degrees(rv[2]), t[0], t[1], t[2], matches, resid))
            corr = (rv, t, c)

    # ---- assemble (capture order preserved) ----
    s_frac = path / max(path[-1], 1e-9)
    out, buf = [], []

    def compact(arrs):
        x = np.vstack(arrs)
        k = np.floor(x / a.voxel).astype(np.int64)
        _, idx = np.unique(k, axis=0, return_index=True)
        return x[np.sort(idx)]

    for i in range(n):
        w = to_world(i)
        if corr is not None:
            rv, t, c = corr
            f = s_frac[i]
            Rf = R.from_rotvec(rv * f).as_matrix()
            w = (w - c) @ Rf.T + t * f + c
            # pose correction for the report
        buf.append(w.astype(np.float32))
        if len(buf) >= 40:
            out = [compact(out + buf)] if out else [compact(buf)]
            buf = []
    final = compact(out + buf) if (out or buf) else np.empty((0, 3), np.float32)
    save_pcd(a.out, final)
    print('wrote %d points -> %s ; z range %.2f..%.2f' % (len(final), a.out, final[:, 2].min(), final[:, 2].max()))

    # corrected trajectory
    traj = np.column_stack([xy, np.full(n, height), yaw])
    if corr is not None:
        rv, t, c = corr
        for i in range(n):
            f = s_frac[i]
            Rf = R.from_rotvec(rv * f).as_matrix()
            traj[i, :3] = (traj[i, :3] - c) @ Rf.T + t * f + c
            traj[i, 3] += rv[2] * f
    np.savetxt(os.path.splitext(a.out)[0] + '_trajectory.csv', np.column_stack([traj, np.array(rec, dtype=np.float64)]),
               delimiter=',', header='x,y,z,yaw,t_rec_ns', comments='', fmt=['%.5f'] * 4 + ['%.0f'])

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(30, 10))
    s = final[::max(1, len(final) // 600000)]
    ax[0].scatter(s[:, 0], s[:, 1], c=s[:, 2], s=0.2, cmap='jet', vmin=-1, vmax=6)
    ax[0].plot(traj[:, 0], traj[:, 1], 'k-', lw=0.8)
    ax[0].set_aspect('equal')
    ax[0].set_title('top-down, colour = z (-1..6 m)')
    ax[1].scatter(s[:, 0], s[:, 2], s=0.2, c='k')
    ax[1].set_title('x-z')
    ax[2].scatter(s[:, 1], s[:, 2], s=0.2, c='k')
    ax[2].set_title('y-z')
    plt.savefig(os.path.splitext(a.out)[0] + '_report.png', dpi=45)
    print('report ->', os.path.splitext(a.out)[0] + '_report.png')


if __name__ == '__main__':
    main()
