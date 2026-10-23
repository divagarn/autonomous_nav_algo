"""Where the maps and calibration data live (portable: nothing is hard-coded).

BUGGY_CONFIG_DIR parameter folder; default <package folder>/config (set by setup.bash). It holds
                 buggy_params.yaml: every tuning parameter of the nodes (see that file).
BUGGY_DATA_DIR   data folder; default ~/autoware_data. The `buggy` CLI sets it to
                 <package folder>/data so the whole folder can be copied anywhere.
  <data>/maps/<name>/        map folders (lanelet2_map.osm, pointcloud_map.pcd, ...)
  <data>/buggy_brake_calibration.yaml, buggy_brake_runs.jsonl
"""
import os


def data_dir():
    return os.path.abspath(os.path.expanduser(os.environ.get('BUGGY_DATA_DIR', '~/autoware_data')))


def config_dir():
    d = os.environ.get('BUGGY_CONFIG_DIR')
    if d:
        return os.path.abspath(os.path.expanduser(d))
    return os.path.join(os.path.dirname(data_dir()), 'config')      # <folder>/data -> <folder>/config


def config_files(user_file=''):
    """YAML parameter files for the nodes, lowest priority first: the shipped config/buggy_params.yaml,
    then the user's own file (launch argument params:=FILE / ./buggy ... --params FILE). Missing files
    are skipped (a missing USER file is an error)."""
    files = []
    default = os.path.join(config_dir(), 'buggy_params.yaml')
    if os.path.exists(default):
        files.append(default)
    if user_file:
        user = os.path.abspath(os.path.expanduser(user_file))
        if not os.path.exists(user):
            raise RuntimeError(f'parameter file not found: {user}')
        files.append(user)
    return files


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
