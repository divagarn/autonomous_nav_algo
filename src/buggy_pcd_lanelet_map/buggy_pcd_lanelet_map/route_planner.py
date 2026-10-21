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

Closed loops: a lane whose end meets its own start (or another lane's start) within link_gap
metres and roughly the same heading is treated as connected even when the OSM has no shared node
(Vector Map Builder exports a ring as one lanelet with separate start/end nodes). Without this a
goal "behind" the vehicle on a ring had NO ROUTE.

Auto goal / loop (requests on /system/request, sent by the panel buttons):
  home:set      remember the current pose as HOME (also done automatically when AUTO is first engaged)
  home:go       route back to HOME
  home:clear    forget HOME (also forgotten when a new initial pose is given)
  loop:<n>      drive to HOME, then n-1 more laps (n laps if the buggy is already at HOME), stopping at HOME

Publishes (transient local):
  /planning/route_path    nav_msgs/Path
  /planning/route_marker  visualization_msgs/MarkerArray (route lanelets,
                          centreline, goal arrow)
  /planning/home_marker   HOME disc + label
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
        self.succ = {l.id: list(by_first.get(l.last, [])) for l in self.lanelets.values()}
        self.link_gap = self.declare_parameter('link_gap', 4.0).value
        self.link_angle = math.radians(self.declare_parameter('link_angle_deg', 60.0).value)
        self.home_reach = self.declare_parameter('home_reach', 2.5).value
        self.link_geometrically()
        self.home = None            # (x, y, yaw) in map
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
        self.home_pub = self.create_publisher(MarkerArray, '/planning/home_marker', qos)
        self.create_subscription(String, '/system/request', self.on_request, 10)
        self.create_subscription(Empty, '/planning/clear_route', lambda _: self.clear(), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_init, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(PoseStamped, '/planning/mission_planning/goal', self.on_goal, 10)
        self.get_logger().info('waiting for goal (RViz: 2D Goal Pose)')

    def on_init(self, msg):
        p = msg.pose.pose.position
        self.start_pose = (p.x, p.y)
        self.clear()
        self.clear_home()           # a new initial pose starts a new session

    def link_geometrically(self):
        """Connect lane A -> lane B when A's end meets B's start (same direction), node ids or not."""
        def tangent(c, end):
            k = min(4, len(c) - 1)               # ~2 m apart: the first/last segment is too noisy
            a, b = (c[-1 - k], c[-1]) if end else (c[0], c[k])
            return math.atan2(b[1] - a[1], b[0] - a[0])
        added = []
        for a in self.lanelets.values():
            for b in self.lanelets.values():
                if b.id in self.succ[a.id] or len(a.center) < 2 or len(b.center) < 2:
                    continue
                if math.dist(a.center[-1][:2], b.center[0][:2]) > self.link_gap:
                    continue
                d = tangent(a.center, True) - tangent(b.center, False)
                if abs(math.atan2(math.sin(d), math.cos(d))) > self.link_angle:
                    continue
                self.succ[a.id].append(b.id)
                added.append((a.id, b.id))
        if added:
            self.get_logger().info(f'linked {len(added)} lane end(s) to a lane start by position '
                                   f'(closed loop / unshared nodes): {added}')

    def current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

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

    def chain_points(self, chain, si, gi):
        pts = []
        for k, lid in enumerate(chain):
            c = self.lanelets[lid].center
            a = si if k == 0 else 0
            b = gi if k == len(chain) - 1 else len(c) - 1
            pts += c[a:b + 1]
        return pts

    def smooth_pts(self, pts):
        return smooth(pts, self.wheelbase, self.steer_limit, self.max_dev)

    def lap_points(self, home):
        """(chain, raw points) of one full lap starting and ending at `home`, or None if the lanes
        do not lead back to it."""
        lh = self.nearest(home)
        h, _ = project(lh, home)
        best = None
        for s0 in self.succ[lh.id]:
            dist, prev = self.route(s0, lh.id)
            if lh.id in dist and (best is None or dist[lh.id] < best[0]):
                chain, u = [], lh.id
                while u is not None:
                    chain.append(u)
                    u = prev[u]
                best = (dist[lh.id], chain[::-1])
        if best is None:
            return None
        chain = best[1]
        pts = list(lh.center[h:])
        for k, lid in enumerate(chain):
            c = self.lanelets[lid].center
            pts += c[:h + 1] if (k == len(chain) - 1 and lid == lh.id) else c
        return [lh.id] + chain, pts

    # -- auto goal / loop -------------------------------------------------------
    def home_pose_msg(self):
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x, p.position.y = self.home[0], self.home[1]
        p.orientation.z, p.orientation.w = math.sin(self.home[2] / 2), math.cos(self.home[2] / 2)
        return p

    def set_home(self):
        cur = self.current_pose()
        if cur is None:
            self.status('Give an initial pose first')
            return False
        self.home = cur
        m = Marker(header=self._hdr())
        m.ns, m.id, m.type, m.action = 'home', 0, Marker.CYLINDER, Marker.ADD
        m.pose = self.home_pose_msg()
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = 2 * self.home_reach
        m.scale.z = 0.1
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 0.55
        t = Marker(header=m.header)
        t.ns, t.id, t.type, t.action = 'home_label', 1, Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position.x, t.pose.position.y, t.pose.position.z = cur[0], cur[1], 1.5
        t.pose.orientation.w = 1.0
        t.scale.z = 0.8
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text = 'HOME'
        self.home_pub.publish(MarkerArray(markers=[m, t]))
        self.get_logger().info(f'HOME set at ({cur[0]:.1f}, {cur[1]:.1f})')
        return True

    def clear_home(self):
        if self.home is not None:
            self.home = None
            m = Marker()
            m.action = Marker.DELETEALL
            self.home_pub.publish(MarkerArray(markers=[m]))

    def on_request(self, msg):
        key, _, val = msg.data.partition(':')
        if key == 'mode' and val == 'auto' and self.home is None:
            self.set_home()                      # HOME = where AUTO was first engaged
        elif key == 'home' and val == 'set':
            self.set_home()
        elif key == 'home' and val == 'clear':
            self.clear_home()
            self.status('HOME cleared')
        elif key == 'home' and val == 'go':
            self.go_home()
        elif key == 'loop':
            try:
                self.go_loop(max(1, int(float(val))))
            except ValueError:
                self.status(f'loop: bad lap count "{val}"')

    def go_home(self):
        cur = self.current_pose()
        if cur is None or self.home is None:
            self.status('No HOME yet: press Set home (or engage AUTO once) first')
            return
        if math.dist(cur[:2], self.home[:2]) < self.home_reach:
            self.status('Already at HOME')
            return
        res = self.plan(cur[:2], self.home[:2])
        if res is None:
            self.status('No route home')
            return
        chain, si, gi = res
        raw = self.chain_points(chain, si, gi)
        pts, before, after = self.smooth_pts(raw)
        self.publish(list(dict.fromkeys(chain)), pts, self.home_pose_msg(), raw)
        self.status(f'Back to HOME: {self.length(pts):.0f} m, peak steer {before:.0f} -> {after:.0f} deg')

    def go_loop(self, laps):
        cur = self.current_pose()
        if cur is None:
            self.status('Give an initial pose first')
            return
        if self.home is None and not self.set_home():
            return
        at_home = math.dist(cur[:2], self.home[:2]) < self.home_reach
        pieces, raw_all, chains = [], [], []
        before = after = 0.0
        more = laps
        if not at_home:
            res = self.plan(cur[:2], self.home[:2])
            if res is None:
                self.status('No route back to HOME')
                return
            chain, si, gi = res
            raw = self.chain_points(chain, si, gi)
            sm, b, a = self.smooth_pts(raw)
            pieces.append(sm); raw_all += raw; chains += chain
            before, after, more = max(before, b), max(after, a), laps - 1
        if more > 0:
            lap = self.lap_points(self.home[:2])
            if lap is None:
                self.status('Not a closed loop: the lanes do not lead back to HOME')
                return
            chain, raw = lap
            sm, b, a = self.smooth_pts(raw)
            for _ in range(more):
                pieces.append(sm); raw_all += raw; chains += chain
            before, after = max(before, b), max(after, a)
        pts = []
        for piece in pieces:
            pts += piece if not pts or math.dist(pts[-1], piece[0]) > 0.3 else piece[1:]
        if len(pts) < 2:
            self.status('Already at HOME')
            return
        self.publish(list(dict.fromkeys(chains)), pts, self.home_pose_msg(), raw_all)
        self.status(f'Loop x{laps}: {self.length(pts):.0f} m, back to HOME at the end; '
                    f'peak steer {before:.0f} -> {after:.0f} deg')
        self.get_logger().info(f'loop route: {laps} lap(s), {self.length(pts):.0f} m')

    @staticmethod
    def length(pts):
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

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
        pts = self.chain_points(chain, si, gi)
        smooth_pts, before, after = self.smooth_pts(pts)
        self.publish(list(dict.fromkeys(chain)), smooth_pts, msg.pose, pts)
        length = self.length(smooth_pts)
        info = (f'{len(chain)} lanelets, {length:.1f} m, '
                f'peak steer {before:.0f} -> {after:.0f} deg (limit {self.steer_limit:.0f})')
        lane_d = project(self.nearest(goal), goal)[1]
        if lane_d > 6.0:
            info += f'  [goal is {lane_d:.0f} m from the lane: snapped to it]'
        if len(chain) > 1 and chain[0] == chain[-1]:
            info += '  [goal is behind: around the loop]'
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
