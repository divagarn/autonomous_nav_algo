"""LocalCartesian maps: lat/lon -> map x, y, and the lane loader on a georeferenced copy of small_loop."""
import math
import os

import pytest

from buggy_pcd_lanelet_map.projection import Projector
from buggy_pcd_lanelet_map.route_planner import load_lanelets

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
GEO = os.path.join(ROOT, 'src', 'buggy_core', 'test', 'planning', 'small_loop_geo', 'lanelet2_map.osm')
LOCAL = os.path.join(ROOT, 'data', 'maps', 'small_loop', 'lanelet2_map.osm')


def test_east_and_north_offsets_match_the_ellipsoid():
    p = Projector('LocalCartesian', 28.6139, 77.2090)
    e, n = p.to_xy(28.6139, 77.2090 + 100.0 / (111319.49 * math.cos(math.radians(28.6139))))
    assert (e, n) == pytest.approx((100.0, 0.0), abs=0.5)
    e, n = p.to_xy(28.6139 + 100.0 / 110800.0, 77.2090)
    assert (e, n) == pytest.approx((0.0, 100.0), abs=0.5)
    assert p.to_xy(28.6139, 77.2090) == pytest.approx((0.0, 0.0), abs=1e-6)


def test_georeferenced_copy_gives_the_same_lanes_as_the_local_map():
    assert Projector.for_osm(GEO).georeferenced and not Projector.for_osm(LOCAL).georeferenced
    a = {l.id: l for l in load_lanelets(LOCAL)}
    b = {l.id: l for l in load_lanelets(GEO)}
    assert a.keys() == b.keys() and len(a) > 0
    for k in a:
        assert len(a[k].center) == len(b[k].center)
        for (x1, y1, *_), (x2, y2, *_) in zip(a[k].center, b[k].center):
            assert math.hypot(x1 - x2, y1 - y2) < 0.03


def test_unsupported_projector_is_refused(tmp_path):
    (tmp_path / 'map_projector_info.yaml').write_text('projector_type: MGRS\nmgrs_grid: 43RFN\n')
    with pytest.raises(RuntimeError, match='not supported'):
        Projector.load(str(tmp_path / 'map_projector_info.yaml'))
    (tmp_path / 'map_projector_info.yaml').write_text('projector_type: LocalCartesian\n')
    with pytest.raises(RuntimeError, match='map_origin'):
        Projector.load(str(tmp_path / 'map_projector_info.yaml'))


def test_to_latlon_inverts_to_xy():
    p = Projector('LocalCartesian', 12.9712936, 80.2439507)
    for x, y in ((0.0, 0.0), (120.5, -80.25), (-300.0, 410.0)):
        assert p.to_xy(*p.to_latlon(x, y)) == pytest.approx((x, y), abs=0.002)
    with pytest.raises(ValueError):
        Projector().to_latlon(1.0, 2.0)


def test_add_map_fills_latlon_for_a_local_export_drawn_on_the_enu_cloud(tmp_path):
    """small_loop drawn with the Local projector (local_x/y only) + a LocalCartesian projector file:
    add_map writes lat/lon, and the lanes come out the same."""
    import subprocess
    import sys
    data = tmp_path / 'data'
    (data / 'maps').mkdir(parents=True)
    pcd = tmp_path / 'c.pcd'
    pcd.write_text('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\n'
                   'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n0 0 0\n')
    proj = tmp_path / 'p.yaml'
    proj.write_text('projector_type: LocalCartesian\nvertical_datum: WGS84\nmap_origin:\n  latitude: 12.9712936\n  longitude: 80.2439507\n')
    script = os.path.join(ROOT, 'src', 'buggy_pcd_lanelet_map', 'scripts', 'add_map.py')
    env = dict(os.environ, BUGGY_DATA_DIR=str(data))
    r = subprocess.run([sys.executable, script, 'geo', LOCAL, '--pcd', str(pcd), '--projector', str(proj)],
                       env=env, capture_output=True, text=True)
    assert 'latitude / longitude added' in r.stdout, r.stdout + r.stderr
    out = str(data / 'maps' / 'geo' / 'lanelet2_map.osm')
    assert (data / 'maps' / 'geo' / 'map_projector_info.yaml').read_text().startswith('projector_type: LocalCartesian')
    a = {l.id: l for l in load_lanelets(LOCAL)}
    b = {l.id: l for l in load_lanelets(out)}
    assert a.keys() == b.keys()
    for k in a:
        for (x1, y1, *_), (x2, y2, *_) in zip(a[k].center, b[k].center):
            assert math.hypot(x1 - x2, y1 - y2) < 0.01


def test_local_xy_win_over_lat_lon_that_belong_to_another_origin(tmp_path):
    """The real small_loop_gnss export: lat/lon relative to Vector Map Builder's own origin (38 km away), local_x/y in the cloud frame."""
    import re
    import subprocess
    import sys
    txt = open(LOCAL).read()
    other = Projector('LocalCartesian', 12.662595, 80.079107)                      # what VMB used
    def add(m):
        x = float(re.search(r'local_x" v="([^"]*)"', m.group(0)).group(1))
        y = float(re.search(r'local_y" v="([^"]*)"', m.group(0)).group(1))
        la, lo = other.to_latlon(x, y)
        return m.group(0).replace('lat="" lon=""', f'lat="{la:.11f}" lon="{lo:.11f}"')
    bad = tmp_path / 'bad.osm'
    bad.write_text(re.sub(r'<node id=.*?</node>', add, txt, flags=re.S))
    (tmp_path / 'map_projector_info.yaml').write_text(
        'projector_type: LocalCartesian\nvertical_datum: WGS84\nmap_origin:\n  latitude: 12.9712936\n  longitude: 80.2439507\n')
    a = {l.id: l for l in load_lanelets(LOCAL)}
    b = {l.id: l for l in load_lanelets(str(bad))}                                 # the loader takes local_x/y
    for k in a:
        for (x1, y1, *_), (x2, y2, *_) in zip(a[k].center, b[k].center):
            assert math.hypot(x1 - x2, y1 - y2) < 0.01
    # add_map rewrites the lat/lon with the map's own origin, so Autoware's projector agrees as well
    data = tmp_path / 'data'
    (data / 'maps').mkdir(parents=True)
    pcd = tmp_path / 'c.pcd'
    pcd.write_text('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\n'
                   'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n0 0 0\n')
    script = os.path.join(ROOT, 'src', 'buggy_pcd_lanelet_map', 'scripts', 'add_map.py')
    r = subprocess.run([sys.executable, script, 'fix', str(bad), '--pcd', str(pcd), '--projector', str(tmp_path / 'map_projector_info.yaml')],
                       env=dict(os.environ, BUGGY_DATA_DIR=str(data)), capture_output=True, text=True)
    assert 'ANOTHER origin' in r.stdout, r.stdout + r.stderr
    import xml.etree.ElementTree as ET
    proj = Projector('LocalCartesian', 12.9712936, 80.2439507)
    for n in ET.parse(data / 'maps' / 'fix' / 'lanelet2_map.osm').getroot().findall('node'):
        t = {x.get('k'): x.get('v') for x in n.findall('tag')}
        ex, ey = proj.to_xy(float(n.get('lat')), float(n.get('lon')))
        assert math.hypot(ex - float(t['local_x']), ey - float(t['local_y'])) < 0.01
