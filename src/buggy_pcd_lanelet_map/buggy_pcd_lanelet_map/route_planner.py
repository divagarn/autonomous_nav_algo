#!/usr/bin/env python3
"""Lanelet2 route planner: RViz goal -> lane to navigate (Autoware-style).

Start = current map->base_link (set by initial_pose_vehicle), goal = RViz
"2D Goal Pose" (/goal_pose, also /planning/mission_planning/goal). The route
is a shortest chain of lanelets (Dijkstra over successor links, found by
shared end/start nodes), then the centreline of that chain between the
projected start and goal points.

The centreline is then smoothed (path_utils.smooth) so the steering angle
needed to follow it stays within the buggy's limit (20 deg, wheelbase 1.6 m)
while staying within smoothing_max_deviation of the lane centre.

Publishes (transient local):
  /planning/route_path    nav_msgs/Path
  /planning/route_marker  visualization_msgs/MarkerArray (route lanelets,
                          centreline, goal arrow)
Lanelet bounds are read from local_x/local_y tags like lanelet_marker_publisher.
"""
import heapq
import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Empty, Header, String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from buggy_pcd_lanelet_map.path_utils import resample, smooth
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class Lanelet:
    def __init__(self, lid, left, right, first, last):
        self.id, self.first, self.last = lid, first, last
        sl, sr = resample(left, 0.5), resample(right, 0.5)
        k = max(len(sl), len(sr))
        sl = [sl[min(len(sl) - 1, round(i * (len(sl) - 1) / (k - 1)))] for i in range(k)]
        sr = [sr[min(len(sr) - 1, round(i * (len(sr) - 1) / (k - 1)))] for i in range(k)]
        self.left, self.right = sl, sr
        self.center = [tuple((a[c] + b[c]) / 2 for c in range(3)) for a, b in zip(sl, sr)]
        self.length = sum(math.dist(a, b) for a, b in zip(self.center, self.center[1:]))


def load_lanelets(path):
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.findall('node'):
        t = {x.get('k'): x.get('v') for x in n.findall('tag')}
        if 'local_x' in t:
            nodes[n.get('id')] = (float(t['local_x']), float(t['local_y']),
                                  float(t.get('ele', 0.0)))
    ways = {w.get('id'): [r.get('ref') for r in w.findall('nd')] for w in root.findall('way')}
    out = []
    for rel in root.findall('relation'):
        t = {x.get('k'): x.get('v') for x in rel.findall('tag')}
        if t.get('type') != 'lanelet':
            continue
        m = {x.get('role'): x.get('ref') for x in rel.findall('member')}
        L, R = ways[m['left']], ways[m['right']]
        out.append(Lanelet(rel.get('id'), [nodes[i] for i in L], [nodes[i] for i in R],
                           (L[0], R[0]), (L[-1], R[-1])))
    return out


def project(lanelet, p):
    """Index of centreline point nearest p, and its xy distance."""
    best = min(range(len(lanelet.center)),
               key=lambda i: math.hypot(lanelet.center[i][0] - p[0], lanelet.center[i][1] - p[1]))
    c = lanelet.center[best]
    return best, math.hypot(c[0] - p[0], c[1] - p[1])


class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.frame = self.declare_parameter('map_frame', 'map').value
        # vehicle limits from the existing stack (steering_calculator_node)
        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        max_steer = self.declare_parameter('max_steering_deg', 20.0).value
        margin = self.declare_parameter('steering_margin', 0.9).value
        self.steer_limit = max_steer * margin
        self.max_dev = self.declare_parameter('smoothing_max_deviation', 0.6).value
        path = self.declare_parameter('lanelet2_map_path', '').value
        self.lanelets = {l.id: l for l in load_lanelets(path)}
        by_first = {}
        for l in self.lanelets.values():
            by_first.setdefault(l.first, []).append(l.id)
        self.succ = {l.id: by_first.get(l.last, []) for l in self.lanelets.values()}
        self.get_logger().info(f'{len(self.lanelets)} lanelets loaded')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_pose = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.path_pub = self.create_publisher(Path, '/planning/route_path', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/planning/route_marker', qos)
        self.status_pub = self.create_publisher(String, '/planning/route_status', qos)
        self.status('No route (use 2D Goal Pose)')
        self.create_subscription(Empty, '/planning/clear_route', lambda _: self.clear(), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_init, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(PoseStamped, '/planning/mission_planning/goal', self.on_goal, 10)
        self.get_logger().info('waiting for goal (RViz: 2D Goal Pose)')

    def on_init(self, msg):
        p = msg.pose.pose.position
        self.start_pose = (p.x, p.y)
        self.clear()

    def current_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return self.start_pose

    def nearest(self, p):
        return min(self.lanelets.values(), key=lambda l: project(l, p)[1])

    def route(self, a, b):
        """Lanelet id chain a -> b (a == b allowed: returns [a])."""
        dist, prev, pq = {}, {}, [(0.0, a, None)]
        while pq:
            d, u, pu = heapq.heappop(pq)
            if u in dist:
                continue
            dist[u], prev[u] = d, pu
            if u == b:
                break
            for v in self.succ[u]:
                if v not in dist:
                    heapq.heappush(pq, (d + self.lanelets[v].length, v, u))
        return dist, prev

    def plan(self, start, goal):
        ls, lg = self.nearest(start), self.nearest(goal)
        si, _ = project(ls, start)
        gi, _ = project(lg, goal)
        if ls.id == lg.id and gi >= si:
            return [ls.id], si, gi
        # search from successors of start lanelet to the goal lanelet
        best = None
        for s in self.succ[ls.id]:
            dist, prev = self.route(s, lg.id)
            if lg.id in dist and (best is None or dist[lg.id] < best[0]):
                chain, u = [], lg.id
                while u is not None:
                    chain.append(u)
                    u = prev[u]
                best = (dist[lg.id], [ls.id] + chain[::-1])
        if best is None:
            return None
        return best[1], si, gi

    def status(self, text):
        self.status_pub.publish(String(data=text))

    def clear(self):
        self.status('No route (use 2D Goal Pose)')
        self.path_pub.publish(Path(header=self._hdr()))
        m = Marker()
        m.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[m]))

    def _hdr(self):
        return Header(frame_id=self.frame, stamp=self.get_clock().now().to_msg())

    def on_goal(self, msg):
        start = self.current_xy()
        if start is None:
            self.status('Give an initial pose first')
            self.get_logger().warn('no start pose yet: give an initial pose first')
            return
        goal = (msg.pose.position.x, msg.pose.position.y)
        res = self.plan(start, goal)
        if res is None:
            self.status('No route to goal')
            self.get_logger().warn('no route to goal')
            return
        chain, si, gi = res
        pts = []
        for k, lid in enumerate(chain):
            c = self.lanelets[lid].center
            a = si if k == 0 else 0
            b = gi if k == len(chain) - 1 else len(c) - 1
            pts += c[a:b + 1]
        smooth_pts, before, after = smooth(
            pts, self.wheelbase, self.steer_limit, self.max_dev)
        self.publish(chain, smooth_pts, msg.pose, pts)
        length = sum(math.dist(a, b) for a, b in zip(smooth_pts, smooth_pts[1:]))
        info = (f'{len(chain)} lanelets, {length:.1f} m, '
                f'peak steer {before:.0f} -> {after:.0f} deg (limit {self.steer_limit:.0f})')
        self.status(info + ('' if after <= self.steer_limit + 0.5 else '  [TOO TIGHT]'))
        self.get_logger().info(f'route: {info}')

    def publish(self, chain, pts, goal_pose, raw_pts):
        hdr = self._hdr()
        path = Path(header=hdr)
        for i, p in enumerate(pts):
            ps = PoseStamped(header=hdr)
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p
            a, b = (pts[i], pts[i + 1]) if i + 1 < len(pts) else (pts[max(i - 1, 0)], pts[i])
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            path.poses.append(ps)
        self.path_pub.publish(path)

        arr = MarkerArray()

        def marker(mid, ns, mtype):
            m = Marker(header=hdr)
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        fill = marker(0, 'route_lanelets', Marker.TRIANGLE_LIST)
        fill.scale.x = fill.scale.y = fill.scale.z = 1.0
        fill.color.g, fill.color.a = 1.0, 0.35
        for lid in chain:
            l = self.lanelets[lid]
            for i in range(len(l.left) - 1):
                for p in (l.left[i], l.right[i], l.left[i + 1],
                          l.right[i], l.right[i + 1], l.left[i + 1]):
                    fill.points.append(Point(x=p[0], y=p[1], z=p[2] + 0.05))
        arr.markers.append(fill)

        line = marker(1, 'route_path', Marker.LINE_STRIP)
        line.scale.x = 0.3
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.9, 0.0, 1.0
        line.points = [Point(x=p[0], y=p[1], z=p[2] + 0.1) for p in pts]
        arr.markers.append(line)

        raw = marker(3, 'route_path_raw', Marker.LINE_STRIP)
        raw.scale.x = 0.08
        raw.color.r = raw.color.g = raw.color.b = 0.6
        raw.color.a = 0.8
        raw.points = [Point(x=p[0], y=p[1], z=p[2] + 0.08) for p in raw_pts]
        arr.markers.append(raw)

        goal = marker(2, 'goal', Marker.ARROW)
        goal.pose = goal_pose
        goal.scale.x, goal.scale.y, goal.scale.z = 2.0, 0.4, 0.4
        goal.color.r, goal.color.a = 1.0, 1.0
        arr.markers.append(goal)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(RoutePlanner())


if __name__ == '__main__':
    main()
