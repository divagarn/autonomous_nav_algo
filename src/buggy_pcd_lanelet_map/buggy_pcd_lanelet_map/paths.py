"""Where the maps and calibration data live (portable: nothing is hard-coded).

BUGGY_DATA_DIR   data folder; default ~/autoware_data. The `buggy` CLI sets it to
                 <package folder>/data so the whole folder can be copied anywhere.
  <data>/maps/<name>/        map folders (lanelet2_map.osm, pointcloud_map.pcd, ...)
  <data>/buggy_brake_calibration.yaml, buggy_brake_runs.jsonl
"""
import os


def data_dir():
    return os.path.abspath(os.path.expanduser(os.environ.get('BUGGY_DATA_DIR', '~/autoware_data')))


def maps_dir():
    return os.path.join(data_dir(), 'maps')


def resolve_map(name_or_path):
    """A map folder given by name (looked up in maps_dir) or by path."""
    p = os.path.expanduser(str(name_or_path))
    if os.sep not in p and not p.startswith('.'):
        cand = os.path.join(maps_dir(), p)
        if os.path.isdir(cand):
            return cand
    return os.path.abspath(p)
