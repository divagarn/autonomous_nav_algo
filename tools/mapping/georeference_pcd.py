#!/usr/bin/env python3
"""Rotate a LiDAR map (PCD) so +x = East, +y = North, using the georeference of gnss_georef.py.

  python3 georeference_pcd.py MAP_map_flat.pcd MAP_gps_origin.yaml -o MAP_enu.pcd

The map built from a bag has its own frame (origin = LiDAR at the first scan, +x = the buggy's heading). To draw a
lanelet2 map with REAL latitude / longitude (e.g. in Vector Map Builder with a LocalCartesian projection) the point
cloud must be in a metric frame whose origin is a known lat/lon and whose axes point East / North. This writes that
cloud (rotation about the origin only: the origin stays where it is) and the matching map_projector_info.yaml:

    projector_type: LocalCartesian
    vertical_datum: WGS84
    map_origin: {latitude: ..., longitude: ...}
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd_align_tool import load_pcd, save_pcd, voxel_down  # noqa: E402


def rotate_to_enu(pts, theta_deg_ccw):
    th = math.radians(theta_deg_ccw)
    c, s = math.cos(th), math.sin(th)
    out = np.array(pts, dtype=np.float64, copy=True)
    x, y = out[:, 0].copy(), out[:, 1].copy()
    out[:, 0] = c * x - s * y
    out[:, 1] = s * x + c * y
    return out.astype(np.float32)


def projector_yaml(geo):
    o = geo['map_origin']
    return (f"projector_type: LocalCartesian\nvertical_datum: WGS84\nmap_origin:\n"
            f"  latitude: {o['latitude']:.9f}\n  longitude: {o['longitude']:.9f}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('pcd')
    ap.add_argument('gps_origin', help='<name>_gps_origin.yaml from build_map_from_bag.sh --gnss')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--voxel', type=float, default=0.0, help='keep one point per voxel of this size [m] (0 = all): a smaller file for the browser tool')
    a = ap.parse_args()
    geo = yaml.safe_load(open(a.gps_origin))
    q = geo['gnss']['quality']
    pts = load_pcd(a.pcd)
    out = rotate_to_enu(pts, geo['transform']['theta_deg_ccw_map_to_east'])
    if a.voxel > 0:
        out = voxel_down(out, a.voxel)
    save_pcd(a.out, out)
    info = os.path.splitext(a.out)[0] + '_projector_info.yaml'
    open(info, 'w').write(projector_yaml(geo))
    print(f'{len(out)} points rotated by {geo["transform"]["theta_deg_ccw_map_to_east"]:.2f} deg -> {a.out}')
    print(f'projector info -> {info}\n' + projector_yaml(geo))
    if 'no RTK' in q or geo['agreement']['rms_m'] > 2.0:
        print(f'WARNING: GNSS quality "{q}", LiDAR/GNSS RMS {geo["agreement"]["rms_m"]} m: the georeference is only approximate')


if __name__ == '__main__':
    main()
