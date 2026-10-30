#!/usr/bin/env python3
"""Run KISS-ICP (pip package, run with a venv that has `kiss-icp`) over EVERY scan of a rosbag2 and
save stamps + 4x4 poses to an .npz for rebuild_flat_map.py (--odom file.npz).
  python3 run_kiss_offline.py --bag ~/bags/new_delhi_bag_3 -o /tmp/kiss_poses.npz"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rebuild_flat_map import open_reader, cloud_xyz
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

ap = argparse.ArgumentParser()
ap.add_argument('--bag', required=True); ap.add_argument('-o', required=True)
ap.add_argument('--topic', default='/velodyne_points')
ap.add_argument('--max-scans', type=int, default=0, help='only the first N scans (0 = all; quick test)')
ap.add_argument('--max-range', type=float, default=100.0); ap.add_argument('--min-range', type=float, default=1.5)
a = ap.parse_args()
cfg = KISSConfig(); cfg.data.max_range = a.max_range; cfg.data.min_range = a.min_range; cfg.data.deskew = False; cfg.mapping.voxel_size = a.max_range / 100.0
k = KissICP(cfg)
r = open_reader(a.bag, [a.topic]); st, T = [], []
while r.has_next():
    if a.max_scans and len(st) >= a.max_scans: break
    _, d, _ = r.read_next()
    m = deserialize_message(d, PointCloud2)
    k.register_frame(cloud_xyz(m), np.empty(0))
    st.append(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec); T.append(k.last_pose.copy())
    if len(st) % 200 == 0: print(len(st), flush=True)
np.savez(a.o, stamps=np.array(st), poses=np.array(T)); print('saved', len(st), 'poses')
