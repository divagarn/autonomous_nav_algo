#!/usr/bin/env python3
"""./buggy mission list|check|here|pick ...   edit and check <map>/mission.yaml (see buggy_pcd_lanelet_map/mission.py)

  list  [MAP]                          the stops of the map
  check [MAP]                          validate the file (fields, duplicate names, stops too close)
  here  NAME [MAP] [--wait SEC]        add the vehicle's CURRENT position (TF map->base_link) as a stop
  pick  NAME [MAP] [--wait SEC]        add the next RViz "2D Goal Pose" click as a stop (it only plans a route)
here / pick need the stack running (./buggy view|live|replay|sim).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from buggy_pcd_lanelet_map import mission  # noqa: E402
from buggy_pcd_lanelet_map.paths import resolve_map  # noqa: E402


def _ros_pose(mode, timeout=15.0):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    node = Node('mission_tool')
    out = {}
    if mode == 'pick':
        node.create_subscription(PoseStamped, '/goal_pose', lambda m: out.update(
            x=m.pose.position.x, y=m.pose.position.y), 10)
        print('click "2D Goal Pose" in RViz on the stop ...')
        timeout = 120.0
    else:
        buf = Buffer()
        TransformListener(buf, node)
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while not out and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if mode == 'here':
            try:
                t = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
                out.update(x=t.transform.translation.x, y=t.transform.translation.y)
            except Exception:
                pass
    node.destroy_node()
    rclpy.shutdown()
    return out or None


def main():
    ap = argparse.ArgumentParser(prog='./buggy mission', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['list', 'check', 'here', 'pick'])
    ap.add_argument('args', nargs='*', help='[NAME] [MAP]')
    ap.add_argument('--wait', type=float, default=None, help='standing time of this stop (default: the file\'s wait_sec)')
    ap.add_argument('--file', default='', help='mission file (default <map>/mission.yaml)')
    a = ap.parse_args()
    name = None
    rest = list(a.args)
    if a.cmd in ('here', 'pick'):
        if not rest:
            ap.error(f'{a.cmd} needs a stop NAME')
        name = rest.pop(0)
    map_path = resolve_map(rest[0] if rest else os.environ.get('BUGGY_MAP', 'new_delhi_map'))
    path = mission.mission_path(map_path, a.file)
    if a.cmd in ('here', 'pick'):
        p = _ros_pose(a.cmd)
        if not p:
            print('no pose: is the stack running and localized (./buggy view|live ...)?')
            return 1
        mission.append_stop(path, name, p['x'], p['y'], a.wait)
        print(f'added stop "{name}" ({p["x"]:.2f}, {p["y"]:.2f}) to {path}')
        return 0
    try:
        data = mission.load(path)
    except RuntimeError as e:
        print(e)
        return 1
    if a.cmd == 'list':
        d = data.get('wait_sec', mission.DEFAULT_WAIT)
        print(f'{path}   loop={data.get("loop", True)}  start_as_home={data.get("start_as_home", False)}')
        for i, s in enumerate(data['stops'], 1):
            print(f'  {i}. {s["name"]:<12} x={s["x"]:8.2f} y={s["y"]:8.2f}  wait {s.get("wait_sec", d)} s')
    else:
        for w in mission.warnings(data):
            print('warning:', w)
        print(f'{path}: {len(data["stops"])} stops OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
