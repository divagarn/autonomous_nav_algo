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
import os

import rclpy
from autoware_perception_msgs.msg import PredictedObjects
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node

import yaml
from buggy_pcd_lanelet_map.paths import data_dir
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

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
        self.pose_pub = self.create_publisher(PoseStamped, '/vehicle/sim_pose', 10)
        self.steer_pub = self.create_publisher(Float32, '/sim/steering_angle', 1)
        self.speed_pub = self.create_publisher(Float32, '/sim/vehicle_speed', 1)
        if self.publish_real:
            self.real_steer_pub = self.create_publisher(Float32, '/steering_angle', 1)
        self.state_pub = self.create_publisher(String, '/system/state', 10)
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
        return self.dummy_objects + lidar

    def obstacle_ahead(self):
        """(distance from front bumper to nearest blocking object edge, label) or None."""
        best = None
        hi = len(self.path)
        for (ox, oy, oyaw, length, width, label) in self.all_objects():
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
            if lat - half_lat > self.safe_half_width:
                continue
            dist = self.seg[j] - self.seg[self.idx] - FRONT_OFFSET - half_lon
            if -half_lon * 2 < dist < DETECT_RANGE and (best is None or dist < best[0]):
                best = (dist, label)
        return best

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
            self.hold_obstacle, self.clear_since = False, None
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
                if remaining - self.goal_margin < s_stop:
                    self.routing, self.mode, self.msg = 'arrived', 'stop', 'Arrived at goal'
                elif self.hold_obstacle:
                    gap = f'{max(obs[0], 0):.1f} m' if obs is not None else 'clearing'
                    self.msg = f'Obstacle ahead ({gap}) - stopped'
                else:
                    if self.msg.startswith('Obstacle'):
                        self.msg = ''
                    demand = self.pure_pursuit()
                    max_d = self.steer_rate * self.dt
                    self.cmd_steer += max(-max_d, min(max_d, demand - self.cmd_steer))
                    steer, red = self.cmd_steer, False
                    yellow = slow or preset <= 2.0       # 2 km/h preset when slowing
                    self.preset_kmh = 2.0 if yellow else 4.0
                    if slow and obs is not None:
                        self.msg = f'Slowing: obstacle {max(obs[0], 0):.1f} m ahead'
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
            obs = self.obstacle_ahead()
            if obs is not None:
                target = min(target, math.sqrt(2.0 * ACCEL * max(obs[0] - STOP_MARGIN, 0.0)))
                names = {1: 'car', 3: 'bus', 7: 'pedestrian'}
                self.msg = f'Obstacle ahead: {names.get(obs[1], "object")} {max(obs[0], 0):.1f} m'
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
            'localization': (self.loc.get('state', 'no_data') if self.live else
                             'initialized' if self.localized else 'uninitialized'),
            'drive': self.drive_text(),
            'detector': ('off' if not self.live else 'ok'
                         if self.mono() - self.lidar_time < 0.7 else 'STALE'),
            'objects': str(len(self.all_objects())),
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
            'cte': f'{self.cte:.2f}', 'msg': self.msg,
        }
        self.state_pub.publish(String(data=';'.join(f'{k}={v}' for k, v in kv.items())))


def main():
    rclpy.init()
    rclpy.spin(OperationManager())


if __name__ == '__main__':
    main()
