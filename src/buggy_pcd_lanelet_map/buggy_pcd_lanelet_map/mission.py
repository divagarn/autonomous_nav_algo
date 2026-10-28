"""Mission file of a map: <map folder>/mission.yaml - the named stops the buggy drives to, in order.

    loop: true            # after the last stop go to the first again (default true)
    wait_sec: 10.0        # how long to stand at each stop (default 10.0)
    start_as_home: true   # "home" is optional: true = the place where the buggy stands when the mission starts
                          # (set it in RViz with 2D Pose Estimate, the usual way) becomes the last stop "home"
    stops:
      - {name: home, x: 12.3, y: -4.5}
      - {name: A,    x: 40.1, y: 8.0}
      - {name: B,    x: 71.0, y: 30.2, wait_sec: 20.0}     # own waiting time for this stop

x, y are map coordinates in metres (the same frame as RViz). A stop that is not on the lane is
snapped to the nearest lane by the route planner. Used by the launch files (the mission_manager
node) and by `./buggy mission ...`.
"""
import math
import os

import yaml

KEYS = {'loop', 'wait_sec', 'start_as_home', 'stops'}
STOP_KEYS = {'name', 'x', 'y', 'wait_sec'}
DEFAULT_WAIT = 10.0


def mission_path(map_path, user_file=''):
    return os.path.abspath(os.path.expanduser(user_file)) if user_file else os.path.join(map_path, 'mission.yaml')


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def validate(data):
    """List of problems (empty = fine)."""
    err = []
    if not isinstance(data, dict):
        return ['the file must be a mapping with a "stops:" list']
    for k in data:
        if k not in KEYS:
            err.append(f'{k}: unknown key (known: {", ".join(sorted(KEYS))})')
    for k in ('loop', 'start_as_home'):
        if k in data and not isinstance(data[k], bool):
            err.append(f'{k}: must be true or false')
    if 'wait_sec' in data and not (_num(data['wait_sec']) and data['wait_sec'] >= 0):
        err.append('wait_sec: must be a number >= 0')
    stops = data.get('stops')
    if not isinstance(stops, list) or not stops:
        return err + ['stops: needs a list with at least one stop']
    names = set()
    for i, s in enumerate(stops, 1):
        where = f'stop {i}'
        if not isinstance(s, dict):
            err.append(f'{where}: must be a mapping (name, x, y)')
            continue
        for k in s:
            if k not in STOP_KEYS:
                err.append(f'{where}: unknown key "{k}" (known: {", ".join(sorted(STOP_KEYS))})')
        name = s.get('name')
        if not isinstance(name, str) or not name.strip():
            err.append(f'{where}: name is missing')
        elif name in names:
            err.append(f'{where}: name "{name}" is used twice')
        else:
            names.add(name)
            where = f'stop "{name}"'
        for k in ('x', 'y'):
            if not _num(s.get(k)):
                err.append(f'{where}: {k} must be a number (map metres)')
        if 'wait_sec' in s and not (_num(s['wait_sec']) and s['wait_sec'] >= 0):
            err.append(f'{where}: wait_sec must be a number >= 0')
    return err


def warnings(data):
    out = []
    stops = [s for s in data.get('stops', []) if isinstance(s, dict) and _num(s.get('x')) and _num(s.get('y'))]
    for i, a in enumerate(stops):
        for b in stops[i + 1:]:
            d = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            if d < 4.0:
                out.append(f'stops "{a.get("name")}" and "{b.get("name")}" are only {d:.1f} m apart: the buggy '
                           f'stops {0.5} m before a goal, they may count as the same place')
    return out


def load(path):
    """The parsed mission file; raises RuntimeError with all problems."""
    if not os.path.exists(path):
        raise RuntimeError(f'mission file not found: {path}')
    data = yaml.safe_load(open(path)) or {}
    err = validate(data)
    if err:
        raise RuntimeError(f'{path}:\n  ' + '\n  '.join(err))
    return data


def node_params(data):
    """Parameters of the mission_manager node for a parsed mission file."""
    default = float(data.get('wait_sec', DEFAULT_WAIT))
    stops = data['stops']
    p = {'stop_names': [str(s['name']) for s in stops],
         'stop_x': [float(s['x']) for s in stops],
         'stop_y': [float(s['y']) for s in stops],
         'stop_wait_sec': [float(s.get('wait_sec', default)) for s in stops]}
    for k in ('loop', 'start_as_home'):
        if k in data:
            p[k] = bool(data[k])
    return p


def append_stop(path, name, x, y, wait_sec=None):
    """Add a stop to the mission file (created with a header when missing); the file's text stays as it is."""
    if os.path.exists(path):
        data = yaml.safe_load(open(path)) or {}
        if any(isinstance(s, dict) and s.get('name') == name for s in data.get('stops', []) or []):
            raise RuntimeError(f'a stop named "{name}" exists already in {path}')
        text = open(path).read()
        if not text.endswith('\n'):
            text += '\n'
        if 'stops:' not in text:
            text += 'stops:\n'
    else:
        text = ('# Stops of this map, in driving order: ./buggy mission list|check|start|stop ...\n'
                'loop: true            # after the last stop go to the first again\n'
                f'wait_sec: {DEFAULT_WAIT}        # standing time at every stop\n'
                'start_as_home: true   # the RViz start pose is the last stop "home"\n'
                'stops:\n')
    extra = f', wait_sec: {float(wait_sec)}' if wait_sec is not None else ''
    text += f'  - {{name: {name}, x: {x:.2f}, y: {y:.2f}{extra}}}\n'
    open(path, 'w').write(text)


# ---- editing (used by the Buggy Manager app; ./buggy mission here|pick use append_stop) ----------------
HEADER = ('# Stops of this map, in driving order: ./buggy mission list|check|start|stop ...\n'
          '# (written by the Buggy Manager app / ./buggy mission; x, y are map metres)\n')


def save(path, data):
    """Write the mission file in the canonical layout (comments of an older file are replaced by HEADER)."""
    lines = [HEADER.rstrip('\n'),
             f'loop: {"true" if data.get("loop", True) else "false"}            # after the last stop go to the first again',
             f'wait_sec: {float(data.get("wait_sec", DEFAULT_WAIT))}        # standing time at every stop without its own wait_sec',
             f'start_as_home: {"true" if data.get("start_as_home", False) else "false"}   # true = the RViz start pose is the last stop "home"',
             'stops:']
    for s in data.get('stops', []):
        extra = f', wait_sec: {float(s["wait_sec"])}' if 'wait_sec' in s else ''
        lines.append(f'  - {{name: {s["name"]}, x: {float(s["x"]):.2f}, y: {float(s["y"]):.2f}{extra}}}')
    if not data.get('stops'):
        lines[-1] = 'stops: []'
    open(path, 'w').write('\n'.join(lines) + '\n')


def read_lenient(path):
    """Parsed file, or an empty mission when it does not exist yet (no validation: for editing)."""
    if not os.path.exists(path):
        return {'loop': True, 'wait_sec': DEFAULT_WAIT, 'start_as_home': True, 'stops': []}
    data = yaml.safe_load(open(path)) or {}
    data.setdefault('stops', [])
    return data


def set_stop(data, name, x, y, wait_sec=None):
    """Add the stop, or move/replace the one with that name (keeps its position in the order)."""
    new = {'name': name, 'x': round(float(x), 2), 'y': round(float(y), 2)}
    if wait_sec is not None:
        new['wait_sec'] = float(wait_sec)
    for i, s in enumerate(data['stops']):
        if s.get('name') == name:
            data['stops'][i] = new
            return
    data['stops'].append(new)


def delete_stop(data, index):
    del data['stops'][index]


def move_stop(data, index, delta):
    j = index + delta
    if 0 <= j < len(data['stops']):
        data['stops'][index], data['stops'][j] = data['stops'][j], data['stops'][index]
        return j
    return index
