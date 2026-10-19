#!/usr/bin/env python3
"""Create an Autoware-style map folder from a Vector Map Builder (.osm) export.

  add_map.py <name> <path/to/map.osm> [--pcd <pointcloud.pcd>]

Creates <data>/maps/<name>/ with
  lanelet2_map.osm            (copy of your export)
  map_projector_info.yaml     (Local - VMB exports with empty lat/lon + local_x/local_y)
  pointcloud_map.pcd          (link to --pcd, default: the new_delhi_map PCD)
  pointcloud_map_metadata.yaml
and checks the osm (lanelet count, extents, lane connectivity) and whether the
PCD covers it. Then load it with:
  ./buggy live <name>
"""
import argparse
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

# portable: the data folder comes from BUGGY_DATA_DIR (the `buggy` CLI sets it)
ROOT = os.path.join(os.path.abspath(os.path.expanduser(
    os.environ.get('BUGGY_DATA_DIR', '~/autoware_data'))), 'maps')
DEFAULT_PCD = os.path.join(ROOT, 'new_delhi_map', 'pointcloud_map.pcd')


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        n = 0
        while True:
            line = f.readline().decode(errors='ignore').strip()
            if line.startswith('DATA'):
                enc = line.split()[1]
                break
        if enc != 'ascii':
            return None
        return np.loadtxt(f, usecols=(0, 1, 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('name')
    ap.add_argument('osm')
    ap.add_argument('--pcd', default=DEFAULT_PCD)
    ap.add_argument('--force', action='store_true', help='overwrite an existing folder')
    a = ap.parse_args()

    osm = os.path.expanduser(a.osm)
    pcd = os.path.realpath(os.path.expanduser(a.pcd))
    dest = os.path.join(ROOT, a.name)
    if not os.path.isfile(osm):
        sys.exit(f'no such file: {osm}')
    if not os.path.isfile(pcd):
        sys.exit(f'no such PCD: {pcd}')
    if os.path.exists(dest) and not a.force:
        sys.exit(f'{dest} exists - use --force to overwrite')

    # ---- inspect the osm ------------------------------------------------
    root = ET.parse(osm).getroot()
    nodes, local, has_latlon = {}, 0, 0
    for n in root.findall('node'):
        t = {x.get('k'): x.get('v') for x in n.findall('tag')}
        if n.get('lat') not in (None, '') and n.get('lon') not in (None, ''):
            has_latlon += 1
        if 'local_x' in t and 'local_y' in t:
            local += 1
            nodes[n.get('id')] = (float(t['local_x']), float(t['local_y']))
    ways = {w.get('id'): [r.get('ref') for r in w.findall('nd')] for w in root.findall('way')}
    lanes = []
    for rel in root.findall('relation'):
        t = {x.get('k'): x.get('v') for x in rel.findall('tag')}
        if t.get('type') != 'lanelet':
            continue
        m = {x.get('role'): x.get('ref') for x in rel.findall('member')}
        lanes.append((rel.get('id'), m.get('left'), m.get('right'), t))
    print(f'osm: {len(nodes)} nodes with local_x/y, {len(ways)} ways, {len(lanes)} lanelets')
    if not local:
        sys.exit('ERROR: no local_x/local_y tags. This export uses real lat/lon (MGRS/LocalCartesian);\n'
                 'tell me which projection you chose in Vector Map Builder and I will set up the projector.')
    if has_latlon:
        print('WARNING: nodes have real lat/lon as well - the Local projector ignores them.')
    if not lanes:
        sys.exit('ERROR: no lanelet relations (type=lanelet) in the file')

    pts = np.array([nodes[r] for w in ways.values() for r in w if r in nodes])
    print(f'extent x {pts[:,0].min():.0f}..{pts[:,0].max():.0f}  y {pts[:,1].min():.0f}..{pts[:,1].max():.0f}')
    ends = {}
    for lid, left, right, _ in lanes:
        if left in ways and right in ways:
            ends[lid] = ((ways[left][0], ways[right][0]), (ways[left][-1], ways[right][-1]))
    starts = {v[0] for v in ends.values()}
    linked = sum(1 for v in ends.values() if v[1] in starts)
    print(f'lane connections: {linked}/{len(ends)} lanelets continue into another one')

    # ---- create the folder ------------------------------------------------
    os.makedirs(dest, exist_ok=True)
    shutil.copy(osm, os.path.join(dest, 'lanelet2_map.osm'))
    with open(os.path.join(dest, 'map_projector_info.yaml'), 'w') as f:
        f.write('projector_type: Local\n')
    link = os.path.join(dest, 'pointcloud_map.pcd')
    if os.path.lexists(link):
        os.remove(link)
    # relative link when the PCD lives in the same maps folder, so the folder stays portable
    inside = os.path.commonpath([pcd, ROOT]) == ROOT
    os.symlink(os.path.relpath(pcd, dest) if inside else pcd, link)
    with open(os.path.join(dest, 'pointcloud_map_metadata.yaml'), 'w') as f:
        f.write('x_resolution: 500.0\ny_resolution: 500.0\npointcloud_map.pcd: [-150, -150]\n')
    print(f'created {dest}')

    # ---- does the PCD cover the lane? -------------------------------------
    xyz = read_pcd_xyz(pcd)
    if xyz is None:
        print('PCD is binary - skipped the coverage check')
    else:
        ground = xyz[xyz[:, 2] < np.percentile(xyz[:, 2], 25) + 0.5]   # low points ~ ground
        cells = set(zip((ground[:, 0] // 2).astype(int), (ground[:, 1] // 2).astype(int)))
        hit = sum(((int(x // 2), int(y // 2)) in cells) for x, y in pts)
        print(f'PCD coverage: {hit}/{len(pts)} lane points ({100 * hit / len(pts):.0f} %) '
              f'have PCD ground within 2 m')
        if hit < 0.9 * len(pts):
            print('WARNING: the PCD does not cover this lane well - NDT will not localize '
                  'where it is missing. Pass the right PCD with --pcd.')
    print(f'\nload it:  ./buggy live {a.name}      (or ./buggy view {a.name})')


if __name__ == '__main__':
    main()
