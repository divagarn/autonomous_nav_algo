#!/usr/bin/env python3
"""Create an Autoware-style map folder from a Vector Map Builder (.osm) export.

  add_map.py <name> <path/to/map.osm> [--pcd <pointcloud.pcd>] [--projector <map_projector_info.yaml>]

Creates <data>/maps/<name>/ with
  lanelet2_map.osm            (copy of your export)
  map_projector_info.yaml     (Local - VMB exports with empty lat/lon + local_x/local_y; or your --projector file,
                               e.g. the LocalCartesian one of georeference_pcd.py for a georeferenced map: then the
                               x, y of the lanes are computed from the lat/lon of the export, like Autoware does)
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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from buggy_pcd_lanelet_map.projection import Projector  # noqa: E402

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


def fill_latlon(src, dst, proj):
    """Copy the export to dst for a LocalCartesian map and make its latitude / longitude consistent with the point cloud.

    local_x / local_y are always in the frame of the point cloud the lanes were drawn on. The latitude / longitude Vector Map
    Builder writes next to them come from the origin ITS dialog was given (by default another place, even tens of
    kilometres away): they are replaced by the ones of this map's origin (projector file), computed from local_x / local_y.
    A node with only lat/lon is kept as it is."""
    tree = ET.parse(src)
    added, replaced, same, worst = 0, 0, 0, 0.0
    for n in tree.getroot().findall('node'):
        t = {x.get('k'): x.get('v') for x in n.findall('tag')}
        has_ll = n.get('lat') not in (None, '') and n.get('lon') not in (None, '')
        if 'local_x' not in t or 'local_y' not in t:
            continue
        lx, ly = float(t['local_x']), float(t['local_y'])
        if has_ll:
            ex, ey = proj.to_xy(float(n.get('lat')), float(n.get('lon')))
            diff = math.hypot(ex - lx, ey - ly)
            worst = max(worst, diff)
            if diff < 0.05:
                same += 1
                continue
        lat, lon = proj.to_latlon(lx, ly)
        n.set('lat', f'{lat:.10f}')
        n.set('lon', f'{lon:.10f}')
        replaced += has_ll
        added += not has_ll
    tree.write(dst, encoding='UTF-8', xml_declaration=True)
    if added:
        print(f'LocalCartesian: latitude / longitude added to {added} nodes from local_x / local_y '
              f'(origin {proj.lat0:.7f}, {proj.lon0:.7f})')
    if replaced:
        print(f'LocalCartesian: the export\'s latitude / longitude of {replaced} nodes were relative to ANOTHER origin '
              f'(up to {worst / 1000:.1f} km away from this map\'s); replaced from local_x / local_y with the origin '
              f'{proj.lat0:.7f}, {proj.lon0:.7f}. The lanes stay where you drew them on the point cloud.')
    if same:
        print(f'LocalCartesian: {same} nodes already agree with the map origin')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('name')
    ap.add_argument('osm')
    ap.add_argument('--pcd', default=DEFAULT_PCD)
    ap.add_argument('--projector', default='', help='map_projector_info.yaml of a georeferenced map (default: Local)')
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

    projector_file = os.path.expanduser(a.projector) if a.projector else ''
    if projector_file and not os.path.isfile(projector_file):
        sys.exit(f'no such projector file: {projector_file}')
    try:
        proj = Projector.load(projector_file) if projector_file else Projector()
    except RuntimeError as e:
        sys.exit(f'ERROR: {e}')

    # ---- inspect the osm ------------------------------------------------
    root = ET.parse(osm).getroot()
    nodes, local, has_latlon = {}, 0, 0
    for n in root.findall('node'):
        if n.get('lat') not in (None, '') and n.get('lon') not in (None, ''):
            has_latlon += 1
        xyz = proj.node_xyz(n)
        if xyz is not None:
            local += 1
            nodes[n.get('id')] = xyz[:2]
    ways = {w.get('id'): [r.get('ref') for r in w.findall('nd')] for w in root.findall('way')}
    lanes = []
    for rel in root.findall('relation'):
        t = {x.get('k'): x.get('v') for x in rel.findall('tag')}
        if t.get('type') != 'lanelet':
            continue
        m = {x.get('role'): x.get('ref') for x in rel.findall('member')}
        lanes.append((rel.get('id'), m.get('left'), m.get('right'), t))
    how = 'nodes with a position (local_x/y, else lat/lon)' if proj.georeferenced else 'nodes with local_x/y'
    print(f'osm: {len(nodes)} {how}, {len(ways)} ways, {len(lanes)} lanelets')
    if not local:
        sys.exit('ERROR: no usable node positions. A Local map needs local_x/local_y tags; a georeferenced export (real\n'
                 'lat/lon) needs --projector with its LocalCartesian map_projector_info.yaml (georeference_pcd.py writes it).')
    if has_latlon and not proj.georeferenced:
        print('WARNING: nodes have real lat/lon as well - the Local projector ignores them. If this map is georeferenced,\n'
              'run again with --projector <map_projector_info.yaml>.')
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
    if proj.georeferenced:
        fill_latlon(osm, os.path.join(dest, 'lanelet2_map.osm'), proj)
    else:
        shutil.copy(osm, os.path.join(dest, 'lanelet2_map.osm'))
    if projector_file:
        shutil.copy(projector_file, os.path.join(dest, 'map_projector_info.yaml'))
    else:
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
