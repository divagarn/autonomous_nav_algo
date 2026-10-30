#!/usr/bin/env python3
"""Publish a PCD (x y z, ascii/binary) as a latched PointCloud2 for RViz.
  python3 publish_pcd.py map.pcd [--topic /flat_map] [--frame map]"""
import argparse, os, sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd_align_tool import load_pcd

ap = argparse.ArgumentParser(); ap.add_argument('pcd'); ap.add_argument('--topic', default='/flat_map')
ap.add_argument('--frame', default='map'); a = ap.parse_args()
pts = load_pcd(a.pcd).astype(np.float32)
rclpy.init(); n = Node('pcd_publisher')
pub = n.create_publisher(PointCloud2, a.topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                           reliability=ReliabilityPolicy.RELIABLE))
m = PointCloud2(); m.header.frame_id = a.frame
m.height = 1; m.width = len(pts); m.is_dense = True; m.is_bigendian = False
m.fields = [PointField(name=c, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, c in enumerate('xyz')]
m.point_step = 12; m.row_step = 12 * len(pts); m.data = pts.tobytes()
def tick():
    m.header.stamp = n.get_clock().now().to_msg(); pub.publish(m)
tick(); n.create_timer(2.0, tick)
print('Publishing %d points on %s (frame %s). Ctrl+C to stop.' % (len(pts), a.topic, a.frame), flush=True)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
