#!/usr/bin/env python3
"""Checks config/buggy_params.yaml (or any parameter file) against the code of the nodes.

  python3 check_params.py [FILE ...]        (./buggy params-check does this)

For every node section it verifies
  * the parameter exists in the Python node and/or the C++ node (a typo would otherwise be ignored),
  * the YAML type matches the code (a float written as 5 instead of 5.0 is refused by ROS at start-up),
  * for the shipped file: values that differ from the built-in default are listed as 'edited' (your changes),
    and the Python and C++ defaults of each parameter must agree with each other.
And it warns when vehicle geometry that appears in several nodes (LiDAR mount, wheelbase, steering limit,
bumper offset) does not agree between them.
Exit code 1 when there is an error.
"""
import ast
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))          # the unified folder
SRC = os.path.join(ROOT, 'src')
PY = {
    'operation_manager': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/operation_manager.py',
    'route_planner': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/route_planner.py',
    'avoidance_planner': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/avoidance_planner.py',
    'lidar_obstacle_detector': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/lidar_obstacle_detector.py',
    'localization_manager': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/localization_manager.py',
    'lidar_preprocess': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/lidar_preprocess.py',
    'map_display_cloud': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/map_display_cloud.py',
    'uart_writer': 'buggy_pcd_lanelet_map/buggy_pcd_lanelet_map/uart_writer.py',
    'wheel_odometry': 'buggy_perception/buggy_perception/wheel_odometry.py',
    'self_hit_filter': 'buggy_perception/buggy_perception/self_hit_filter.py',
    'segment_ground': 'buggy_perception/buggy_perception/segment_ground.py',
    'velodyne_static_tf': 'buggy_perception/buggy_perception/velodyne_static_tf.py',
    'uart_bridge': 'buggy_uart/buggy_uart/uart_bridge.py',
}
CPP = {
    'operation_manager': 'buggy_core/src/control/operation_manager_node.cpp',
    'mission_manager': 'buggy_core/src/control/mission_manager_node.cpp',
    'gnss_pose_initializer': 'buggy_core/src/localization/gnss_pose_initializer_node.cpp',
    'route_planner': 'buggy_core/src/planning/route_planner_node.cpp',
    'avoidance_planner': 'buggy_core/src/planning/avoidance_planner_node.cpp',
    'lidar_obstacle_detector': 'buggy_core/src/perception/lidar_obstacle_detector_node.cpp',
    'localization_manager': 'buggy_core/src/localization/localization_manager_node.cpp',
    'lidar_preprocess': 'buggy_core/src/localization/lidar_preprocess_node.cpp',
    'uart_writer': 'buggy_core/src/actuation/uart_writer_node.cpp',
    'wheel_odometry': 'buggy_core/src/localization/wheel_odometry_node.cpp',
    'self_hit_filter': 'buggy_core/src/perception/self_hit_filter_node.cpp',
    'segment_ground': 'buggy_core/src/perception/segment_ground_node.cpp',
    'velodyne_static_tf': 'buggy_core/src/localization/velodyne_static_tf_node.cpp',
    'uart_bridge': 'buggy_core/src/actuation/uart_bridge_node.cpp',
}
SAME = [  # values that describe the same physical thing in several nodes
    ('LiDAR x in base_link (m)', [('velodyne_static_tf', 'x'), ('lidar_obstacle_detector', 'sensor_x'),
                                  ('self_hit_filter', 'sensor_offset_x')]),
    ('LiDAR height (m)', [('velodyne_static_tf', 'z'), ('lidar_obstacle_detector', 'sensor_z'),
                          ('avoidance_planner', 'sensor_z')]),
    ('wheelbase (m)', [('operation_manager', 'wheelbase'), ('route_planner', 'wheelbase'),
                       ('avoidance_planner', 'wheelbase'), ('wheel_odometry', 'wheelbase')]),
    ('max steering (deg)', [('operation_manager', 'max_steering_deg'), ('route_planner', 'max_steering_deg'),
                            ('avoidance_planner', 'max_steering_deg')]),
    ('rear axle -> front bumper (m)', [('operation_manager', 'front_offset'), ('avoidance_planner', 'front')]),
]


def kind(v):
    return 'bool' if isinstance(v, bool) else 'int' if isinstance(v, int) else 'float' if isinstance(v, float) \
        else 'str' if isinstance(v, str) else type(v).__name__


def py_declarations(path):
    """{name: default} from declare_parameter('name', default) / p('name', default) calls."""
    tree = ast.parse(open(path).read())
    consts = {}
    for n in ast.walk(tree):                       # module / class level simple constants
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name):
            try:
                consts[n.targets[0].id] = eval(compile(ast.Expression(n.value), path, 'eval'),
                                               {'math': math}, dict(consts))
            except Exception:
                pass
    out = {}
    for n in ast.walk(tree):
        if not isinstance(n, ast.Call) or len(n.args) < 2:
            continue
        f = n.func
        name = f.attr if isinstance(f, ast.Attribute) else f.id if isinstance(f, ast.Name) else ''
        if name not in ('declare_parameter', 'p') or not (isinstance(n.args[0], ast.Constant) and isinstance(n.args[0].value, str)):
            continue
        try:
            out[n.args[0].value] = eval(compile(ast.Expression(n.args[1]), path, 'eval'), {'math': math}, dict(consts))
        except Exception:
            out[n.args[0].value] = None            # not a literal (a path, ...)
    return out


def cpp_declarations(path):
    text = open(path).read()
    out = {}
    for m in re.finditer(r'declare_parameter<([\w:<>]+)>\(\s*"(\w+)"\s*,', text):
        i, depth, j = m.end(), 1, m.end()
        while depth and j < len(text):
            depth += (text[j] == '(') - (text[j] == ')')
            j += 1
        expr = text[i:j - 1].strip()
        typ = m.group(1)
        if re.fullmatch(r'-?\d+\.\d*|-?\.\d+|-?\d+\.?\d*[eE][-+]?\d+', expr):
            out[m.group(2)] = float(expr)
        elif re.fullmatch(r'-?\d+', expr):
            out[m.group(2)] = float(expr) if typ == 'double' else int(expr)
        elif expr in ('true', 'false'):
            out[m.group(2)] = expr == 'true'
        elif re.fullmatch(r'"[^"]*"', expr):
            out[m.group(2)] = expr[1:-1]
        else:
            out[m.group(2)] = None
    return out


def same(a, b):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool) and not isinstance(b, bool):
        return math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-12)
    return a == b


RADAR_KEYS = {'model': str, 'can_interface': str, 'baudrate': int, 'dev_id': int, 'sensor_id': int,
              'frame_id': str, 'models': dict}
RADAR_MODEL_KEYS = {'model': str, 'uifname': str, 'uifmajorv': int, 'uifminorv': int, 'uifpatchv': int}


def check_radar_driver(body):
    """radar_driver: (read by umrr_ros2_driver/launch/radar.launch.py, not a ROS node section)."""
    err = []
    body = (body or {}).get('ros__parameters') if isinstance(body, dict) else None
    if not isinstance(body, dict):
        return ['radar_driver: needs a "ros__parameters:" section']
    for k, v in body.items():
        if k not in RADAR_KEYS:
            err.append(f'radar_driver.{k}: unknown key (known: {", ".join(RADAR_KEYS)})')
        elif not isinstance(v, RADAR_KEYS[k]) or isinstance(v, bool):
            err.append(f'radar_driver.{k}: must be {RADAR_KEYS[k].__name__}, not {kind(v)}')
    models = body.get('models') if isinstance(body.get('models'), dict) else {}
    for name, m in models.items():
        if not isinstance(m, dict):
            err.append(f'radar_driver.models.{name}: must be a mapping')
            continue
        for k, t in RADAR_MODEL_KEYS.items():
            if k not in m:
                err.append(f'radar_driver.models.{name}.{k}: missing')
            elif not isinstance(m[k], t) or isinstance(m[k], bool):
                err.append(f'radar_driver.models.{name}.{k}: must be {t.__name__}, not {kind(m[k])}')
        for k in m:
            if k not in RADAR_MODEL_KEYS:
                err.append(f'radar_driver.models.{name}.{k}: unknown key')
    if 'model' in body and models and body['model'] not in models:
        err.append(f'radar_driver.model: {body["model"]!r} is not in radar_driver.models ({", ".join(models)})')
    return err


def check_file(path, strict_defaults):
    errors, warnings, nparams = [], [], 0
    notes = []
    try:
        data = yaml.safe_load(open(path)) or {}
    except Exception as e:
        return [f'{path}: cannot read: {e}'], [], 0
    py_decl = {n: py_declarations(os.path.join(SRC, f)) for n, f in PY.items() if os.path.exists(os.path.join(SRC, f))}
    cpp_decl = {n: cpp_declarations(os.path.join(SRC, f)) for n, f in CPP.items() if os.path.exists(os.path.join(SRC, f))}
    for node, body in data.items():
        if node == 'radar_driver':
            errors += check_radar_driver(body)
            continue
        if node not in PY and node not in CPP:
            errors.append(f'{node}: unknown node (known: {", ".join(sorted(set(PY) | set(CPP)))})')
            continue
        params = (body or {}).get('ros__parameters')
        if not isinstance(params, dict):
            errors.append(f'{node}: needs a "ros__parameters:" section')
            continue
        for name, val in params.items():
            nparams += 1
            decls = [(lang, d[node][name]) for lang, d in (('python', py_decl), ('c++', cpp_decl))
                     if node in d and name in d[node]]
            if not decls:
                errors.append(f'{node}.{name}: not a parameter of this node (typo?)')
                continue
            if len(decls) == 2 and None not in (decls[0][1], decls[1][1]) and not same(decls[0][1], decls[1][1]):
                errors.append(f'{node}.{name}: the Python default ({decls[0][1]!r}) and the C++ default ({decls[1][1]!r}) differ')
            for lang, default in decls:
                if default is None:
                    continue
                if kind(val) != kind(default):
                    errors.append(f'{node}.{name}: is {kind(val)} ({val!r}) but the {lang} node declares {kind(default)} '
                                  f'({default!r}) - write floats with a decimal point')
                elif strict_defaults and not same(val, default):
                    # an edited value is the point of the file: a note, not an error
                    notes.append(f'{node}.{name} = {val!r} (built-in {lang} default: {default!r})')
            if len(decls) == 1 and node in CPP and node in PY:
                warnings.append(f'{node}.{name}: only the {decls[0][0]} node has this parameter')
    flat = {(n, k): v for n, b in data.items() if n != 'radar_driver' and isinstance(b, dict) for k, v in (b.get('ros__parameters') or {}).items()}
    for what, members in SAME:
        vals = [(f'{n}.{k}', flat[(n, k)]) for n, k in members if (n, k) in flat]
        if len({round(float(v), 9) for _, v in vals if isinstance(v, (int, float))}) > 1:
            warnings.append(f'{what} differs between nodes: ' + ', '.join(f'{a}={b}' for a, b in vals))
    check_file.notes = notes
    return errors, warnings, nparams


def main():
    files = sys.argv[1:] or [os.path.join(ROOT, 'config', 'buggy_params.yaml')]
    bad = False
    for f in files:
        strict = os.path.abspath(f) == os.path.abspath(os.path.join(ROOT, 'config', 'buggy_params.yaml'))
        errors, warnings, n = check_file(f, strict)
        print(f'{f}: {n} parameters checked' + (' against the defaults of the Python and C++ nodes' if strict else ''))
        for note in sorted(set(getattr(check_file, 'notes', []))):
            print('  edited:', note)
        for w in warnings:
            print('  warning:', w)
        for e in errors:
            print('  ERROR:', e)
        if not errors:
            print('  OK')
        bad |= bool(errors)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
