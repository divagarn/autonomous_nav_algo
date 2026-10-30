"""Map projection of the lanelet2 map (Python twin of buggy_core/localization/projection.cpp).

<map folder>/map_projector_info.yaml (the file Autoware's map_projection_loader reads) says how the lanelet2 .osm
maps to metres in the map frame:
  Local           x, y from the local_x / local_y tags (no lat/lon): the buggy maps so far
  LocalCartesian  x = East, y = North of map_origin (WGS84) - georeferenced map, GNSS can place the vehicle in it
                  (the lane loaders still take local_x / local_y first when a node has them, see node_xyz)
MGRS / LocalCartesianUTM / TransverseMercator are Autoware options that this stack does not support: a clear error.
"""
import math
import os

import yaml

_A = 6378137.0
_F = 1 / 298.257223563
_E2 = _F * (2 - _F)


def _ecef(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    n = _A / math.sqrt(1 - _E2 * math.sin(la) ** 2)
    return (n * math.cos(la) * math.cos(lo), n * math.cos(la) * math.sin(lo), n * (1 - _E2) * math.sin(la))


class Projector:
    def __init__(self, kind='Local', lat0=0.0, lon0=0.0):
        self.kind, self.lat0, self.lon0 = kind, lat0, lon0

    @property
    def georeferenced(self):
        return self.kind == 'LocalCartesian'

    def to_xy(self, lat, lon):
        """lat/lon [deg] -> (East, North) [m] of map_origin."""
        if not self.georeferenced:
            return 0.0, 0.0
        p, o = _ecef(lat, lon), _ecef(self.lat0, self.lon0)
        dx, dy, dz = p[0] - o[0], p[1] - o[1], p[2] - o[2]
        la, lo = math.radians(self.lat0), math.radians(self.lon0)
        east = -math.sin(lo) * dx + math.cos(lo) * dy
        north = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
        return east, north

    def to_latlon(self, x, y):
        """(East, North) [m] of map_origin -> (lat, lon) [deg]: the exact inverse of to_xy (Newton steps)."""
        if not self.georeferenced:
            raise ValueError('a Local map has no latitude / longitude')
        lat = self.lat0 + y / 110574.0
        lon = self.lon0 + x / (111320.0 * math.cos(math.radians(self.lat0)))
        for _ in range(5):
            ex, ey = self.to_xy(lat, lon)
            lat += (y - ey) / 110574.0
            lon += (x - ex) / (111320.0 * math.cos(math.radians(lat)))
        return lat, lon

    def node_xyz(self, node):
        """(x, y, z) of an osm <node> element, or None when it has no usable position.

        local_x / local_y win when the node has them: Vector Map Builder always writes them in the frame of the point
        cloud the lanes were drawn on, whatever origin it used for the latitude / longitude it exports next to them.
        Only a node without them (a pure LocalCartesian export) is placed from lat/lon through map_origin."""
        tags = {t.get('k'): t.get('v') for t in node.findall('tag')}
        z = float(tags.get('ele', 0.0))
        if 'local_x' in tags and 'local_y' in tags:
            return float(tags['local_x']), float(tags['local_y']), z
        lat, lon = node.get('lat'), node.get('lon')
        if self.georeferenced and lat not in (None, '') and lon not in (None, ''):
            x, y = self.to_xy(float(lat), float(lon))
            return x, y, z
        return None

    @classmethod
    def load(cls, yaml_path):
        if not os.path.exists(yaml_path):
            return cls()
        y = yaml.safe_load(open(yaml_path)) or {}
        kind = y.get('projector_type', 'Local')
        if kind == 'Local':
            return cls()
        if kind != 'LocalCartesian':
            raise RuntimeError(f"map_projector_info.yaml: projector_type '{kind}' is not supported by the buggy stack "
                               f"(use Local or LocalCartesian): {yaml_path}")
        o = y.get('map_origin') or {}
        if 'latitude' not in o or 'longitude' not in o:
            raise RuntimeError(f'map_projector_info.yaml: LocalCartesian needs map_origin latitude and longitude: {yaml_path}')
        return cls('LocalCartesian', float(o['latitude']), float(o['longitude']))

    @classmethod
    def for_osm(cls, osm_path):
        if osm_path.endswith('.yaml'):
            return cls.load(osm_path)
        return cls.load(os.path.join(os.path.dirname(os.path.abspath(osm_path)), 'map_projector_info.yaml'))
