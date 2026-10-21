#!/usr/bin/env python3
"""Operation mode manager (the backend of the RViz "Buggy State" panel).

Autoware-style state machine:
  routing       unset | set | arrived    (from /planning/route_path)
  localization  uninitialized | initialized   (from /initialpose)
  mode          stop | auto | local | remote
  motion        stopped | moving
AUTO is only accepted when routing == set, localization == initialized and
no emergency. In AUTO the vehicle is *simulated* with a kinematic bicycle
model (wheelbase 1.6 m, max steer 20 deg - same as steering_calculator_node)
tracking /planning/route_path with pure pursuit (same law as
buggy_nav/pure_pursuit.py: steer = atan(2 L sin(alpha) / Ld)). The steering
command is clamped and rate limited and published like the real stack
(deg, +right) on /sim/steering_angle and /sim/vehicle_speed - NOT on
/steering_angle, so a running uart stack is never driven by this simulation
(set publish_real_topics:=true to also publish /steering_angle).
Pose goes to initial_pose_vehicle on /vehicle/sim_pose; on arrival mode
returns to stop. local/remote just set the mode (no simulated motion).

LIVE mode (live:=true, real vehicle): pose = TF map->base_link (rear axle) from
NDT+EKF. AUTO needs drive_enabled:=true AND localization 'converged' AND a route
AND no emergency. In AUTO the same pure pursuit drives the REAL buggy: it
publishes /steering_angle [deg, +right], /final_red_detected (stop) and
/final_yellow_detected (speed preset: yellow=1 -> 2 km/h, else 4 km/h) at 20 Hz;
uart_writer turns those into UART frames. Anything else - STOP/other modes,
emergency, localization loss > 1 s, stale pose, > 2.5 m off the path, obstacle
ahead, near the goal - publishes red=1, steering 0. The node never writes a
serial port itself.

Requests (std_msgs/String on /system/request):
  mode:<stop|auto|local|remote>   emergency:<true|false>
  velocity_limit:<km/h>           gear:<park|reverse|neutral|drive>
  turn_signal:<none|left|right|hazard>
State (std_msgs/String "k=v;k=v;..." on /system/state, 10 Hz).
"""
import math
import json
import os

import rclpy
from autoware_perception_msgs.msg import PredictedObjects
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node

import yaml
from buggy_pcd_lanelet_map.paths import data_dir
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

ACCEL = 1.0  # m/s^2, used for both speeding up and braking
STOP_MARGIN = 2.5     # m kept between vehicle front and an obstacle
FRONT_OFFSET = 2.2    # rear axle -> front bumper (wheelbase 1.6 + overhang 0.61)
DETECT_RANGE = 30.0   # m along the path
SEARCH_WINDOW = 60  # path points searched ahead when matching the vehicle


class OperationManager(Node):
    def __init__(self):
        super().__init__('operation_manager')
        self.limit_kmh = self.declare_parameter('velocity_limit_kmh', 5.0).value
        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        self.max_steer = self.declare_parameter('max_steering_deg', 20.0).value
        self.lookahead = self.declare_parameter('lookahead_distance', 4.0).value
        # steering actuator slew rate - placeholder, not measured on hardware
        self.steer_rate = self.declare_parameter('max_steering_rate_deg_s', 40.0).value
        self.publish_real = self.declare_parameter('publish_real_topics', False).value
        # live: pose comes from NDT localization, speed/steering from the ECU
        # (/wheel_uart); no simulated driving and AUTO is refused.
        self.live = self.declare_parameter('live', False).value
        # live only: must be true or AUTO is refused (the "arm" switch)
        self.drive_enabled = self.declare_parameter('drive_enabled', False).value
        # real stack: base_link = REAR AXLE. Gazebo URDF: base_link = chassis CENTRE
        self.base_at_center = self.declare_parameter('base_link_at_center', False).value
        # objects closer than this to the path centre line (edge to centre) block the
        # vehicle: vehicle half width 0.72 (self_hit_filter box) + lateral clearance
        self.safe_half_width = self.declare_parameter('corridor_half_width', 1.25).value
        # placeholders until measured on the vehicle:
        self.max_cte = self.declare_parameter('max_cross_track', 2.5).value
        self.loc_grace = self.declare_parameter('localization_grace_sec', 1.0).value
        # Stopping model (Autoware-style stop-distance planning). The ECU has NO
        # deceleration command: only red=1 (stop) and the 2/4 km/h presets, so a
        # smooth decel profile is impossible. Instead: slow to the 2 km/h preset
        # early, and stop (red) at the last moment that still lets the vehicle
        # halt before the obstacle:  s_stop(v) = v * react + v^2 / (2 * decel).
        # Defaults are conservative GUESSES; brake_calibration.py measures them.
        self.react = self.declare_parameter('stop_react_sec', 1.0).value
        self.decel = self.declare_parameter('stop_decel', 0.8).value
        self.stop_margin = self.declare_parameter('stop_margin', 2.5).value
        self.slow_zone = self.declare_parameter('slow_zone', 6.0).value
        self.goal_margin = self.declare_parameter('goal_margin', 0.5).value
        self.calib_source = 'default (NOT measured)'
        self.load_calibration(self.declare_parameter(
            'brake_calibration',
            os.path.join(data_dir(), 'buggy_brake_calibration.yaml')).value)
        # live: no AUTO unless the LiDAR obstacle detector is alive
        self.require_detector = self.declare_parameter('require_obstacle_detector', True).value
        self.cmd_steer = 0.0
        self.preset_kmh = 4.0
        self.cmd_go = False
        self.hold_obstacle = False
        self.clear_since = None
        self.calib_time = -10.0
        self.lidar_time = -10.0
        # optional extra sensors (launch args radar:=true / camera:=true)
        self.radar_enabled = self.declare_parameter('radar_enabled', False).value
        self.camera_enabled = self.declare_parameter('camera_enabled', False).value
        # Obstacle memory: a sensor obstacle that was blocking the path is kept for this many
        # seconds after it was last seen (LiDAR/radar drop-outs close to the bumper must not
        # release the stop or skip it). It is dropped once the vehicle has passed it.
        # Detour around a blocked lane: see avoidance.py / avoidance_planner.py
        self.avoid_enabled = self.declare_parameter('avoid_enabled', False).value
        self.avoid_after = self.declare_parameter('avoid_after_sec', 5.0).value
        self.avoid_hw = self.declare_parameter('avoid_corridor_half_width', 1.05).value
        self.avoid_retry = self.declare_parameter('avoid_retry_sec', 5.0).value
        self.avoid_max_tries = self.declare_parameter('avoid_max_tries', 6).value
        self.avoiding, self.avoid_end_idx, self.avoid_seg = False, 0, []
        self.block_since, self.block_last = None, 0.0
        self.avoid_id, self.avoid_pending, self.avoid_next, self.avoid_tries = 0, None, 0.0, 0
        self.avoid_status, self.avoid_note = 'idle', ''
        self.memory_sec = self.declare_parameter('obstacle_memory_sec', 4.0).value
        # while the vehicle is still approaching the stop line a lost obstacle is kept up to this long
        self.memory_max_sec = self.declare_parameter('obstacle_memory_max_sec', 25.0).value
        # after a hold for a lost obstacle, creep at the 2 km/h preset for this long
        self.creep_sec = self.declare_parameter('lost_obstacle_creep_sec', 10.0).value
        self.creep_until = 0.0
        # a single flicker is not an obstacle: it must have been seen this many times to be remembered
        self.memory_min_hits = self.declare_parameter('obstacle_memory_min_hits', 5).value
        self.memory = []        # {'obj': tuple, 't': last seen, 'hold_t': when the stop line was reached}
        self.diag_t = 0.0
        self.last_gap = None
        self.radar_objs, self.radar_time = [], -10.0
        self.names, self.names_time = [], -10.0   # camera names for obstacles
        self.loc_ok_time = 0.0
        self.uart_status = {}
        self.uart_time = 0.0
        self.loc = {}
        self.mode = 'stop'
        self.routing = 'unset'
        self.localized = False
        self.emergency = False
        self.gear = 'park'
        self.turn_signal = 'none'
        self.msg = ''
        self.path = []          # [(x, y, z, qz, qw)]
        self.seg = []           # cumulative arclength
        self.idx = 0            # nearest path index to the rear axle
        self.v = 0.0
        self.rx = self.ry = self.ryaw = 0.0   # rear axle pose (bicycle model)
        self.rz = 0.0
        self.steer = 0.0        # actual steering, deg, +right (buggy convention)
        self.cte = 0.0

        self.create_subscription(String, '/system/request', self.on_request, 10)
        self.create_subscription(Path, '/planning/route_path', self.on_path, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_init, 10)
        if self.live:
            from ground_segmentation_msgs.msg import SteerSpeed
            self.create_subscription(String, '/localization/state', self.on_loc, 10)
            self.create_subscription(SteerSpeed, '/wheel_uart', self.on_wheel, 10)
            self.create_subscription(String, '/uart/status', self.on_uart, 10)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.cmd_steer_pub = self.create_publisher(Float32, '/steering_angle', 1)
            self.cmd_red_pub = self.create_publisher(Bool, '/final_red_detected', 1)
            self.cmd_yellow_pub = self.create_publisher(Bool, '/final_yellow_detected', 1)
        self.dummy_objects = []   # (x, y, yaw, length, width, label)
        self.lidar_objs = []
        self.create_subscription(
            PredictedObjects, '/perception/object_recognition/objects', self.on_objects, 10)
        self.create_subscription(
            PredictedObjects, '/perception/lidar_objects', self.on_lidar_objects, 10)
        self.create_subscription(Bool, '/calibration/active', self.on_calib, 10)
        self.create_subscription(String, '/planning/avoid_result', self.on_avoid_result, 10)
        self.avoid_pub = self.create_publisher(String, '/planning/avoid_request', 5)
        self.detour_pub = self.create_publisher(MarkerArray, '/planning/detour_marker', 1)
        self.create_subscription(
            PredictedObjects, '/perception/radar_objects', self.on_radar_objects, 10)
        self.create_subscription(String, '/perception/object_names', self.on_names, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/vehicle/sim_pose', 10)
        self.steer_pub = self.create_publisher(Float32, '/sim/steering_angle', 1)
        self.speed_pub = self.create_publisher(Float32, '/sim/vehicle_speed', 1)
        if self.publish_real:
            self.real_steer_pub = self.create_publisher(Float32, '/steering_angle', 1)
        self.state_pub = self.create_publisher(String, '/system/state', 10)
        # Autoware-style planning display: slow zone (orange), stop zone (blue), stop wall
        self.zone_pub = self.create_publisher(MarkerArray, '/planning/stop_zone_marker', 1)
        self.zone_half_width = self.declare_parameter('zone_half_width', 1.5).value
        self.create_timer(0.2, self.publish_zones)
        self.dt = 0.05
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.publish_state)

    # -- inputs ------------------------------------------------------------
    def on_loc(self, msg):
        self.loc = dict(kv.split('=', 1) for kv in msg.data.split(';') if '=' in kv)
        self.localized = self.loc.get('state') == 'converged'
        if self.localized:
            self.loc_ok_time = self.mono()

    def on_uart(self, msg):
        self.uart_status = dict(kv.split('=', 1) for kv in msg.data.split(';') if '=' in kv)
        self.uart_time = self.mono()

    def mono(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_wheel(self, msg):
        self.steer = float(msg.steering)
        self.v = float(msg.speed) / 3.6

    def on_init(self, msg):
        if self.live:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # initial pose is base_link (mid-wheelbase); simulate at the rear axle
        h = self.wheelbase / 2.0
        self.rx, self.ry, self.rz = p.x - h * math.cos(yaw), p.y - h * math.sin(yaw), p.z
        self.ryaw, self.steer = yaw, 0.0
        self.localized = True
        self.mode, self.v = 'stop', 0.0

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z,
                      p.pose.orientation.z, p.pose.orientation.w) for p in msg.poses]
        self.seg = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.seg.append(self.seg[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.idx, self.v = 0, 0.0
        self.avoiding, self.avoid_pending, self.avoid_tries = False, None, 0
        self.block_since, self.avoid_status = None, 'idle'
        self.detour_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        if self.mode == 'auto':
            self.mode = 'stop'
        self.routing = 'set' if len(self.path) > 1 else 'unset'
        self.msg = ''

    @staticmethod
    def parse_objects(msg):
        out = []
        for o in msg.objects:
            p = o.kinematics.initial_pose_with_covariance.pose
            yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
            label = o.classification[0].label if o.classification else 0
            out.append((p.position.x, p.position.y, yaw,
                        o.shape.dimensions.x, o.shape.dimensions.y, label))
        return out

    def on_objects(self, msg):          # dummy objects from the RViz tools
        self.dummy_objects = self.parse_objects(msg)

    def on_lidar_objects(self, msg):    # real detections; every message is a heartbeat
        self.lidar_objs = self.parse_objects(msg)
        self.lidar_time = self.mono()

    def on_radar_objects(self, msg):    # radar_fusion tracks (optional); a message = radar alive
        self.radar_objs = self.parse_objects(msg)
        self.radar_time = self.mono()

    def on_names(self, msg):            # camera class names from object_labeler (optional)
        try:
            self.names = json.loads(msg.data)
        except ValueError:
            self.names = []
        self.names_time = self.mono()

    def name_near(self, x, y):
        """Camera class name of the obstacle at (x, y), or None."""
        if self.mono() - self.names_time > 1.5:
            return None
        best = min(self.names, key=lambda n: math.hypot(n['x'] - x, n['y'] - y), default=None)
        if best and math.hypot(best['x'] - x, best['y'] - y) < 1.5 and best['name'] != 'unknown':
            return best['name']
        return None

    def on_calib(self, msg):
        if msg.data:
            self.calib_time = self.mono()

    def load_calibration(self, path):
        try:
            with open(path) as f:
                c = yaml.safe_load(f)
            self.react, self.decel = float(c['react_sec']), float(c['decel_ms2'])
            self.calib_source = f'measured ({os.path.basename(path)})'
        except (OSError, KeyError, TypeError, ValueError):
            pass

    def stopping_distance(self, v):
        return v * self.react + v * v / (2.0 * self.decel)

    def all_objects(self):
        lidar = self.lidar_objs if self.mono() - self.lidar_time < 0.7 else []
        radar = self.radar_objs if self.mono() - self.radar_time < 1.0 else []
        return self.dummy_objects + lidar + radar

    def block_dist(self, o):
        """Distance (front bumper -> near edge) if object `o` blocks the route ahead, else None."""
        ox, oy, oyaw, length, width, label = o[:6]
        hi = len(self.path)
        j = min(range(self.idx, hi), key=lambda i: math.hypot(
            self.path[i][0] - ox, self.path[i][1] - oy))
        px, py = self.path[j][0], self.path[j][1]
        lat = math.hypot(px - ox, py - oy)
        k = min(j, hi - 2)
        tang = math.atan2(self.path[k + 1][1] - self.path[k][1],
                          self.path[k + 1][0] - self.path[k][0])
        th = oyaw - tang
        half_lat = abs(math.cos(th)) * width / 2 + abs(math.sin(th)) * length / 2
        half_lon = abs(math.cos(th)) * length / 2 + abs(math.sin(th)) * width / 2
        corridor = self.avoid_hw if self.avoiding else self.safe_half_width
        if lat - half_lat > corridor:
            return None
        dist = self.seg[j] - self.seg[self.idx] - FRONT_OFFSET - half_lon
        return dist if -half_lon * 2 < dist < DETECT_RANGE else None

    def update_memory(self):
        """Remember blocking LiDAR/radar obstacles through sensor drop-outs.

        A remembered obstacle that is no longer seen stays valid
          * until the vehicle reaches its stop line (up to obstacle_memory_max_sec), so a drop-out
            during the approach can never cancel the slow-down / stop, and
          * for obstacle_memory_sec after the buggy has stopped at that line;
        then it is forgotten and the buggy creeps (2 km/h) for lost_obstacle_creep_sec.
        Entries are also dropped once the vehicle has passed them.
        """
        if not self.path or self.memory_sec <= 0.0:    # obstacle_memory_sec:=0 switches it off
            self.memory = []
            return
        now = self.mono()
        fresh = (self.lidar_objs if now - self.lidar_time < 0.7 else []) + \
            (self.radar_objs if now - self.radar_time < 1.0 else [])
        for o in fresh:
            if self.block_dist(o) is None:
                continue
            for m in self.memory:
                if math.hypot(m['obj'][0] - o[0], m['obj'][1] - o[1]) < 1.0:
                    m['obj'], m['t'], m['hold_t'] = o, now, None
                    m['n'] += 1
                    break
            else:
                self.memory.append({'obj': o, 't': now, 'hold_t': None, 'n': 1})
        s_stop = self.stopping_distance(max(self.v, 0.55))
        keep = []
        for m in self.memory:
            d = self.block_dist(m['obj'])
            if d is None:                      # passed, or no longer in the corridor
                continue
            if now - m['t'] > 1.0 and m['n'] < self.memory_min_hits:
                continue                       # was only a flicker
            if now - m['t'] > 0.35:            # lost: not shown by the sensors right now
                if m['hold_t'] is None and d - self.stop_margin < s_stop:
                    m['hold_t'] = now          # the buggy has reached the stop line
                if m['hold_t'] is not None:
                    if now - m['hold_t'] > self.memory_sec:
                        self.creep_until = now + self.creep_sec
                        continue
                elif now - m['t'] > self.memory_max_sec:
                    continue
            keep.append(m)
        self.memory = keep

    def remembered(self):
        """Remembered obstacles that the sensors do not currently show."""
        now = self.mono()
        return [m['obj'] for m in self.memory
                if now - m['t'] > 0.35 and m['n'] >= self.memory_min_hits]

    def obstacle_ahead(self):
        """(bumper -> nearest blocking object edge [m], label, camera name) or None."""
        best = None
        for o in self.all_objects() + self.remembered():
            dist = self.block_dist(o)
            if dist is not None and (best is None or dist < best[0]):
                best = (dist, o[5], self.name_near(o[0], o[1]))
        return best

    @staticmethod
    def obs_name(obs):
        """Camera class name if known, else the label's name, else 'obstacle'."""
        if obs[2]:
            return obs[2]
        return {1: 'car', 2: 'truck', 3: 'bus', 5: 'motorcycle', 6: 'bicycle',
                7: 'pedestrian', 8: 'animal'}.get(obs[1], 'obstacle')

    # -- detour around a blocked lane ----------------------------------------------
    def update_avoidance(self, obs, now):
        """Blocked for avoid_after_sec -> ask the planner for a detour; adopt it; leave it when done."""
        if not self.avoid_enabled:
            return
        if self.avoiding:
            if self.idx >= self.avoid_end_idx:
                self.avoiding, self.avoid_tries, self.block_since = False, 0, None
                self.avoid_status = 'idle'
                self.detour_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
                self.get_logger().info('detour finished: back on the lanelet path')
                self.msg = ''
            return
        if obs is not None:
            self.block_since = self.block_since or now
            self.block_last = now
        elif now - self.block_last > 1.0:
            self.block_since = None
            if self.avoid_status != 'idle':
                self.avoid_status = 'idle'
            return
        if self.avoid_pending is not None:
            if now - self.avoid_pending[1] > 4.0:
                self.avoid_pending = None
                self.avoid_next = now + self.avoid_retry
                self.avoid_status = 'planner not answering'
            return
        if (self.block_since is not None and now - self.block_since >= self.avoid_after
                and now >= self.avoid_next and self.avoid_tries < self.avoid_max_tries):
            self.request_avoidance(now)

    def request_avoidance(self, now):
        pts = [[p[0], p[1], p[2]] for p in self.path[self.idx:self.idx + 130]]
        if len(pts) < 6:
            return
        obstacles = []
        for o in self.all_objects() + self.remembered():
            obstacles.append([o[0], o[1], o[2], o[3], o[4]])
        self.avoid_id += 1
        self.avoid_pending = (self.avoid_id, now, self.idx)
        self.avoid_status = 'planning'
        self.avoid_pub.publish(String(data=json.dumps(
            {'id': self.avoid_id, 'points': pts, 'obstacles': obstacles})))
        self.get_logger().info(f'blocked for {now - self.block_since:.0f} s: asking for a detour')

    def on_avoid_result(self, msg):
        if self.avoid_pending is None:
            return
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        pid, _, req_idx = self.avoid_pending
        if d.get('id') != pid:
            return
        now = self.mono()
        self.avoid_pending = None
        if not d.get('ok'):
            self.avoid_tries += 1
            self.avoid_next = now + self.avoid_retry
            self.avoid_status = 'no detour'
            self.avoid_note = d.get('reason', '')
            self.msg = 'Blocked, no detour: ' + self.avoid_note[:140]
            self.get_logger().warn(self.msg)
            return
        seg = d['segment']
        ib = int(d['ib'])
        new = []
        for i, q in enumerate(seg):
            a, b = seg[i], seg[min(i + 1, len(seg) - 1)]
            if i == len(seg) - 1:
                a, b = seg[i - 1], seg[i]
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            new.append((q[0], q[1], q[2], math.sin(yaw / 2), math.cos(yaw / 2)))
        self.path = self.path[:req_idx] + new + self.path[req_idx + ib:]
        self.seg = [0.0]
        for a, b in zip(self.path, self.path[1:]):
            self.seg.append(self.seg[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.avoid_end_idx = req_idx + len(new)
        self.avoiding = True
        self.avoid_status = f"avoiding ({d['side']} {abs(d['offset']):.1f} m)"
        self.hold_obstacle, self.clear_since = False, None
        self.msg = f"Detour {d['side']} by {abs(d['offset']):.1f} m around the obstacle"
        self.get_logger().info(self.msg + f" ({d['length']:.0f} m, LiDAR free space: {d.get('sensed')})")
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'detour', 0, Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.35
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.9, 1.0, 1.0
        m.points = [Point(x=q[0], y=q[1], z=q[2] + 0.15) for q in seg]
        self.detour_pub.publish(MarkerArray(markers=[m]))

    # -- planning display (Autoware-style stop / slow zones) -----------------
    def ribbon(self, a, b, half_w, z, rgba, mid, ns, now):
        """Strip of road between path arclengths a..b (clamped to the path), as a marker."""
        a, b = max(a, 0.0), min(b, self.seg[-1])
        if b - a < 0.05:
            return None
        pts, i = [], 0
        samples = [a] + [t for t in self.seg if a < t < b] + [b]
        for sv in samples:
            while i < len(self.seg) - 2 and self.seg[i + 1] < sv:
                i += 1
            span = self.seg[i + 1] - self.seg[i]
            f = 0.0 if span <= 0 else (sv - self.seg[i]) / span
            p0, p1 = self.path[i], self.path[i + 1]
            x, y = p0[0] + f * (p1[0] - p0[0]), p0[1] + f * (p1[1] - p0[1])
            zc = p0[2] + f * (p1[2] - p0[2])
            tx, ty = p1[0] - p0[0], p1[1] - p0[1]
            n = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / n, tx / n
            pts.append(((x + nx * half_w, y + ny * half_w, zc + z),
                        (x - nx * half_w, y - ny * half_w, zc + z)))
        m = Marker()
        m.header.frame_id, m.header.stamp = 'map', now
        m.ns, m.id, m.type, m.action = ns, mid, Marker.TRIANGLE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime.sec, m.lifetime.nanosec = 0, 500_000_000
        for (l0, r0), (l1, r1) in zip(pts, pts[1:]):
            for q in (l0, r0, l1, r0, r1, l1):
                m.points.append(Point(x=q[0], y=q[1], z=q[2]))
        return m

    def zone_markers(self, line, label, now, mid, buffer_m=0.0):
        """Slow / stop zones leading to a stop line at bumper arclength `line`."""
        preset = 2.0 if self.limit_kmh <= 3.0 else 4.0
        cruise = 2.0 if self.limit_kmh <= 3.0 else 4.0
        s_stop = self.stopping_distance(max(self.v, cruise / 3.6))
        s_slow = self.stopping_distance(preset / 3.6) + self.slow_zone
        hw = self.zone_half_width
        sb = self.seg[self.idx] + FRONT_OFFSET          # front bumper, arclength
        # a stop line / zone is measured at the front bumper; the strip drawn is the road there
        out = []
        for a, b, rgba, ns, z in (
                (line - s_slow, line - s_stop, (1.0, 0.62, 0.0, 0.45), 'slow_zone', 0.07),
                (line - s_stop, line, (0.10, 0.35, 1.0, 0.60), 'stop_zone', 0.08),
                (line, line + buffer_m, (0.85, 0.10, 0.10, 0.40), 'keep_clear', 0.07)):
            r = self.ribbon(max(a, sb), b, hw, z, rgba, mid, ns, now)
            if r is not None:
                out.append(r)
                mid += 1
        if line > sb - 0.2 and line <= self.seg[-1]:
            w = Marker()
            w.header.frame_id, w.header.stamp = 'map', now
            w.ns, w.id, w.type, w.action = 'stop_wall', mid, Marker.CUBE, Marker.ADD
            j = min(range(len(self.seg)), key=lambda i: abs(self.seg[i] - line))
            k = min(j, len(self.path) - 2)
            yaw = math.atan2(self.path[k + 1][1] - self.path[k][1],
                             self.path[k + 1][0] - self.path[k][0])
            w.pose.position.x, w.pose.position.y = self.path[j][0], self.path[j][1]
            w.pose.position.z = self.path[j][2] + 0.6
            w.pose.orientation.z, w.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            w.scale.x, w.scale.y, w.scale.z = 0.12, 2 * hw, 1.2
            w.color.r, w.color.g, w.color.b, w.color.a = 1.0, 0.1, 0.1, 0.85
            w.lifetime.sec, w.lifetime.nanosec = 0, 500_000_000
            out.append(w)
            t = Marker()
            t.header = w.header
            t.ns, t.id, t.type, t.action = 'stop_label', mid + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y = w.pose.position.x, w.pose.position.y
            t.pose.position.z = w.pose.position.z + 1.1
            t.pose.orientation.w = 1.0
            t.scale.z = 0.6
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f'{label} {max(line - sb, 0.0):.1f} m'
            t.lifetime.sec, t.lifetime.nanosec = 0, 500_000_000
            out.append(t)
        return out

    def publish_zones(self):
        if not self.path or self.routing != 'set' or len(self.path) < 2:
            return
        if self.localized or not self.live:
            self.match()
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        obs = self.obstacle_ahead()
        sb = self.seg[self.idx] + FRONT_OFFSET
        mid = 0
        if obs is not None:
            what = self.obs_name(obs)
            arr.markers += self.zone_markers(sb + obs[0] - self.stop_margin, f'STOP: {what}',
                                             now, mid, buffer_m=self.stop_margin)
            mid += 10
        arr.markers += self.zone_markers(self.seg[-1] - self.goal_margin + FRONT_OFFSET,
                                         'GOAL', now, mid)
        self.zone_pub.publish(arr)

    def auto_available(self):
        if self.live and not self.drive_enabled:
            return False
        return self.routing == 'set' and self.localized and not self.emergency

    def on_request(self, msg):
        key, _, val = msg.data.partition(':')
        if key == 'mode':
            if val == 'auto' and self.live and not self.drive_enabled:
                self.msg = 'AUTO disabled: not armed (launch with drive:=true)'
                return
            if val == 'auto' and self.live and self.live_pose() is None:
                self.msg = 'AUTO unavailable: no fresh map->base_link pose'
                return
            if val == 'auto' and not self.auto_available():
                self.msg = 'AUTO unavailable: needs localization + route'
                return
            self.mode, self.msg = val, ''
            if val != 'auto' and not self.live:
                self.v = 0.0 if val == 'stop' else self.v
            if val == 'auto':
                self.gear = 'drive'
        elif key == 'emergency':
            self.emergency = val == 'true'
            if self.emergency:
                self.mode, self.v, self.msg = 'stop', 0.0, 'EMERGENCY'
            else:
                self.msg = ''
        elif key == 'velocity_limit':
            try:
                self.limit_kmh = max(0.0, min(30.0, float(val)))
            except ValueError:
                pass
        elif key == 'gear' and val in ('park', 'reverse', 'neutral', 'drive'):
            self.gear = val
        elif key == 'turn_signal' and val in ('none', 'left', 'right', 'hazard'):
            self.turn_signal = val

    # -- simulation --------------------------------------------------------
    def match(self):
        """Advance idx to the path point nearest the rear axle (forward only)."""
        hi = min(len(self.path), self.idx + SEARCH_WINDOW)
        self.idx = min(range(self.idx, hi), key=lambda i: math.hypot(
            self.path[i][0] - self.rx, self.path[i][1] - self.ry))

    def pure_pursuit(self):
        """Steering demand (deg, +right) toward the lookahead point."""
        i = self.idx
        while i < len(self.path) - 1 and self.seg[i] - self.seg[self.idx] < self.lookahead:
            i += 1
        tx, ty = self.path[i][0], self.path[i][1]
        dx, dy = tx - self.rx, ty - self.ry
        ld = max(math.hypot(dx, dy), 1e-3)
        alpha = math.atan2(dy, dx) - self.ryaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        delta_left = math.atan(2.0 * self.wheelbase * math.sin(alpha) / ld)
        return max(-self.max_steer, min(self.max_steer, -math.degrees(delta_left)))

    # -- live (real vehicle) ------------------------------------------------
    def live_pose(self):
        """(rear-axle x, y, yaw) from map->base_link if fresh (<0.5 s), else None."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        stamp = rclpy.time.Time.from_msg(t.header.stamp, clock_type=self.get_clock().clock_type)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > 0.5:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y = t.transform.translation.x, t.transform.translation.y
        if self.base_at_center:   # everything below works at the rear axle
            x, y = x - self.wheelbase / 2.0 * math.cos(yaw), y - self.wheelbase / 2.0 * math.sin(yaw)
        return x, y, yaw, t.transform.translation.z

    def cross_track(self):
        i = min(self.idx, len(self.path) - 2)
        ax, ay, bx, by = *self.path[i][:2], *self.path[i + 1][:2]
        seg_len = math.hypot(bx - ax, by - ay) or 1.0
        return ((bx - ax) * (self.ry - ay) - (by - ay) * (self.rx - ax)) / seg_len

    def live_fault(self, pose):
        if self.emergency:
            return 'EMERGENCY'
        if pose is None:
            return 'FAULT: pose stale - stopped'
        if self.mono() - self.loc_ok_time > self.loc_grace:
            return 'FAULT: localization not converged - stopped'
        if self.loc.get('tf_conflict'):
            return f"FAULT: TF conflict ({self.loc['tf_conflict']}) - stopped"
        if self.require_detector and self.mono() - self.lidar_time > 0.7:
            return 'FAULT: obstacle detector not running - stopped'
        if self.routing != 'set':
            return 'FAULT: no route - stopped'
        if abs(self.cte) > self.max_cte:
            return f'FAULT: {abs(self.cte):.1f} m off the path - stopped'
        return ''

    def step_live(self):
        steer, red, yellow = 0.0, True, False     # safe default = STOP
        if self.mode != 'auto':
            self.hold_obstacle, self.clear_since, self.memory = False, None, []
            self.creep_until = 0.0
        pose = self.live_pose()
        if pose is not None:
            self.rx, self.ry, self.ryaw, self.rz = pose
        if self.mode == 'auto' and self.drive_enabled:
            if pose is not None and self.path:
                self.match()
                self.cte = self.cross_track()
            fault = self.live_fault(pose)
            if fault:
                self.mode, self.msg = 'stop', fault
                self.get_logger().error(fault)
            else:
                remaining = self.seg[-1] - self.seg[self.idx]
                self.update_memory()
                obs = self.obstacle_ahead()
                self.update_avoidance(obs, self.mono())
                if self.avoiding:       # the detour may have just replaced the path
                    obs = self.obstacle_ahead()
                preset = 2.0 if self.limit_kmh <= 3.0 else 4.0
                # the ECU takes seconds to ramp up, so plan with the larger of
                # the measured speed and the preset we are currently asking for
                v_eff = max(self.v, (self.preset_kmh / 3.6) if self.cmd_go else 0.0)
                s_stop = self.stopping_distance(v_eff)
                s_slow = self.stopping_distance(preset / 3.6) + self.slow_zone
                now = self.mono()
                # obstacle hold: stop, and only resume after it has been clear for 1 s
                # resume once there is no obstacle, or it is far enough away that the
                # normal slow-down / stop logic can handle it again (1 m hysteresis)
                far = obs is None or \
                    obs[0] - self.stop_margin > self.stopping_distance(preset / 3.6) + 1.0
                if far:
                    self.clear_since = self.clear_since or now
                    if self.hold_obstacle and now - self.clear_since > 1.0:
                        self.hold_obstacle = False
                else:
                    self.clear_since = None
                    if obs[0] - self.stop_margin < s_stop:
                        self.hold_obstacle = True
                slow = (obs is not None and obs[0] - self.stop_margin < s_slow) or \
                    remaining - self.goal_margin < s_slow
                self.last_gap = obs[0] if obs is not None else None
                if now - self.diag_t > 1.0:
                    self.diag_t = now
                    gap = f'{obs[0]:.1f}' if obs is not None else '-'
                    self.get_logger().info(
                        f'AUTO: v={self.v * 3.6:.1f} km/h gap={gap} m stop_below='
                        f'{self.stop_margin + s_stop:.1f} m slow_below={self.stop_margin + s_slow:.1f} m '
                        f'goal_left={remaining:.1f} m hold={self.hold_obstacle} '
                        f'remembered={len(self.remembered())} creep={max(self.creep_until - now, 0):.0f}s -> '
                        f'{"RED" if (self.hold_obstacle or remaining - self.goal_margin < s_stop) else ("SLOW" if slow else "GO")}')
                if remaining - self.goal_margin < s_stop:
                    self.routing, self.mode, self.msg = 'arrived', 'stop', 'Arrived at goal'
                elif self.hold_obstacle:
                    gap = f'{max(obs[0], 0):.1f} m' if obs is not None else 'clearing'
                    what = f': {self.obs_name(obs)}' if obs is not None else ''
                    self.msg = f'Obstacle ahead{what} ({gap}) - stopped'
                else:
                    if self.msg.startswith('Obstacle'):
                        self.msg = ''
                    demand = self.pure_pursuit()
                    max_d = self.steer_rate * self.dt
                    self.cmd_steer += max(-max_d, min(max_d, demand - self.cmd_steer))
                    steer, red = self.cmd_steer, False
                    yellow = (slow or preset <= 2.0 or now < self.creep_until
                              or self.avoiding)                 # 2 km/h when slowing / detouring
                    self.preset_kmh = 2.0 if yellow else 4.0
                    if slow and obs is not None:
                        self.msg = f'Slowing: {self.obs_name(obs)} {max(obs[0], 0):.1f} m ahead'
                    elif slow:
                        self.msg = 'Slowing for goal'
                    elif self.msg.startswith('Slowing'):
                        self.msg = ''
        self.cmd_go = not red
        if red:
            self.cmd_steer = 0.0
        if self.mono() - self.calib_time < 1.0:
            return      # brake_calibration.py owns the command topics right now
        self.cmd_steer_pub.publish(Float32(data=float(steer)))
        self.cmd_red_pub.publish(Bool(data=red))
        self.cmd_yellow_pub.publish(Bool(data=yellow))

    def step(self):
        if self.live:
            self.step_live()   # real vehicle: state comes from the ECU / NDT
            return
        driving = self.mode == 'auto' and self.routing == 'set'
        if driving:
            self.match()
            remaining = self.seg[-1] - self.seg[self.idx]
            target = min(self.limit_kmh / 3.6,
                         math.sqrt(2.0 * ACCEL * max(remaining, 0.0)) + 0.05)
            self.update_memory()
            obs = self.obstacle_ahead()
            self.update_avoidance(obs, self.mono())
            if self.avoiding:
                obs = self.obstacle_ahead()
                target = min(target, 2.0 / 3.6)
            if obs is not None:
                target = min(target, math.sqrt(2.0 * ACCEL * max(obs[0] - STOP_MARGIN, 0.0)))
                self.msg = f'Obstacle ahead: {self.obs_name(obs)} {max(obs[0], 0):.1f} m'
            elif self.msg.startswith('Obstacle'):
                self.msg = ''
            demand = self.pure_pursuit()
        else:
            target, demand = 0.0, self.steer

        self.v = (min(target, self.v + ACCEL * self.dt) if self.v < target
                  else max(target, self.v - ACCEL * self.dt))
        max_d = self.steer_rate * self.dt
        self.steer += max(-max_d, min(max_d, demand - self.steer))

        if self.v > 0.0 and self.localized:
            delta = -math.radians(self.steer)  # +left for the bicycle model
            self.rx += self.v * math.cos(self.ryaw) * self.dt
            self.ry += self.v * math.sin(self.ryaw) * self.dt
            self.ryaw += self.v / self.wheelbase * math.tan(delta) * self.dt
            if driving:
                self.rz = self.path[self.idx][2]

        if self.localized and (driving or self.v > 0.0):
            h = self.wheelbase / 2.0
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = self.rx + h * math.cos(self.ryaw)
            ps.pose.position.y = self.ry + h * math.sin(self.ryaw)
            ps.pose.position.z = self.rz
            ps.pose.orientation.z = math.sin(self.ryaw / 2.0)
            ps.pose.orientation.w = math.cos(self.ryaw / 2.0)
            self.pose_pub.publish(ps)

        self.steer_pub.publish(Float32(data=float(self.steer)))
        self.speed_pub.publish(Float32(data=float(self.v * 3.6)))
        if self.publish_real:
            self.real_steer_pub.publish(Float32(data=float(self.steer)))

        if driving:
            i = min(self.idx, len(self.path) - 2)
            ax, ay, bx, by = *self.path[i][:2], *self.path[i + 1][:2]
            seg_len = math.hypot(bx - ax, by - ay) or 1.0
            self.cte = ((bx - ax) * (self.ry - ay) - (by - ay) * (self.rx - ax)) / seg_len
            if self.seg[-1] - self.seg[self.idx] < 0.5 and obs is None:
                self.routing, self.mode, self.v = 'arrived', 'stop', 0.0
                self.msg = 'Arrived at goal'

    def sensors_text(self):
        t = self.mono()
        lidar = 'off' if not self.live else ('ok' if t - self.lidar_time < 0.7 else 'STALE')
        parts = [f'lidar {lidar} ({len(self.lidar_objs)})']
        if self.radar_enabled:
            parts.append('radar ' + ('ok' if t - self.radar_time < 1.0 else 'STALE')
                         + f' ({len(self.radar_objs)})')
        if self.camera_enabled:
            named = sum(1 for n in self.names if n['name'] != 'unknown')
            parts.append('camera ' + ('ok' if t - self.names_time < 1.5 else 'STALE/no objects')
                         + f' ({named} named)')
        return ' | '.join(parts)

    def drive_text(self):
        if not self.live:
            return 'simulation'
        if not self.drive_enabled:
            return 'DISARMED (AUTO off)'
        fresh = self.mono() - self.uart_time < 2.0
        if not fresh:
            return 'armed - no uart_writer running'
        mode = self.uart_status.get('mode', '?')
        ok = self.uart_status.get('state', '?')
        return ('armed - UART LOOPBACK (not writing)' if mode == 'loopback'
                else 'ARMED - UART WRITING') + ('' if ok == 'ok' else ' [STALE]')

    def publish_state(self):
        total = self.seg[-1] if self.seg else 0.0
        progress = 100.0 * self.seg[self.idx] / total if total > 0 and self.seg else 0.0
        kv = {
            'mode': self.mode, 'routing': self.routing,
            'localization': ((self.loc.get('state', 'no_data')
                              + (' TF-CONFLICT ' + self.loc['tf_conflict']
                                 if self.loc.get('tf_conflict') else '')
                              + (f" ({self.loc['jumps']} pose jumps/30s)"
                                 if int(self.loc.get('jumps', '0') or 0) > 3 else ''))
                             if self.live else
                             'initialized' if self.localized else 'uninitialized'),
            'drive': self.drive_text(),
            'detector': ('off' if not self.live else 'ok'
                         if self.mono() - self.lidar_time < 0.7 else 'STALE'),
            'objects': str(len(self.all_objects())),
            'avoid': ('off' if not self.avoid_enabled else self.avoid_status),
            'obs_gap': f'{self.last_gap:.1f}' if self.last_gap is not None else '-',
            'stop_below': f'{self.stop_margin + self.stopping_distance(max(self.v, 0.55)):.1f}',
            'sensors': self.sensors_text(),
            'stop_model': (f'react {self.react:.1f}s decel {self.decel:.2f} '
                           f'-> {self.stopping_distance(4 / 3.6):.1f} m @4km/h, '
                           f'{self.calib_source}'),
            'cmd_steer': f'{self.cmd_steer:.1f}' if self.live else f'{self.steer:.1f}',
            'ndt_score': self.loc.get('score', '-'),
            'ndt_age': self.loc.get('age', '-'),
            'motion': 'moving' if self.v > 0.05 else 'stopped',
            'emergency': str(self.emergency).lower(),
            'velocity_limit': f'{self.limit_kmh:.1f}', 'speed': f'{self.v * 3.6:.1f}',
            'gear': self.gear, 'turn_signal': self.turn_signal,
            'auto_available': str(self.auto_available()).lower(),
            'progress': f'{progress:.0f}', 'steering': f'{self.steer:.1f}',
            'cte': f'{self.cte:.2f}', 'msg': self.msg + ((' | no detour: ' + self.avoid_note[:110])
                               if self.avoid_enabled and self.avoid_note and self.avoid_status == 'no detour'
                               and self.msg.startswith('Obstacle') else ''),
        }
        self.state_pub.publish(String(data=';'.join(f'{k}={v}' for k, v in kv.items())))


def main():
    rclpy.init()
    rclpy.spin(OperationManager())


if __name__ == '__main__':
    main()
