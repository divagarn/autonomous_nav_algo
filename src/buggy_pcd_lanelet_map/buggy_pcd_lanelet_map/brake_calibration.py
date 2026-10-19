#!/usr/bin/env python3
"""Measure the buggy's real stopping behaviour (reaction delay, deceleration).

The ECU has no deceleration command - only red=1 (stop) and the 2/4 km/h
presets - so how it stops is a property of the vehicle that has to be measured.
operation_manager uses the result (react_sec, decel_ms2) in
    s_stop(v) = v * react + v^2 / (2 * decel)
to decide when to slow down and when to send red=1 for an obstacle or the goal.

Test: from standstill, GO at the chosen preset (steering 0) until the wheel
feedback holds >= 90 % of the preset, then red=1 and record until standstill.
Reaction = red command -> first speed drop; stopping distance = travelled
distance (NDT pose if available, else /odom) until speed is 0; decel is then
derived from distance, so the 1 km/h feedback resolution does not matter.
Each run is appended to <data>/buggy_brake_runs.jsonl; the yaml
operation_manager reads (<data>/buggy_brake_calibration.yaml) is
rewritten from ALL runs using the WORST case (longest reaction, weakest decel)
times safety_factor.

SAFETY
  * Default is a DRY RUN: only listens (checks feedback + pose are available).
  * --live needs a typed YES. It drives the buggy straight at the preset: clear
    flat space of >= 15 m ahead, nobody near, safety driver ready, wheels straight.
  * Run the stack with uart_write:=true loopback:=false (uart_writer has the
    0.5 s watchdog); while this tool is active operation_manager stays silent
    (/calibration/active) so the two never fight over the command topics.
  * Aborts to red=1 on Ctrl+C, travel > --max-dist, or timeouts.

  ros2 run buggy_pcd_lanelet_map brake_calibration --preset 4            # dry run
  ros2 run buggy_pcd_lanelet_map brake_calibration --preset 4 --live
"""
import argparse
import json
import math
import os
import sys
import time
import traceback

import rclpy
import yaml
from buggy_pcd_lanelet_map.paths import data_dir
from ground_segmentation_msgs.msg import SteerSpeed
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener

DATA = data_dir()
RUNS = os.path.join(DATA, 'buggy_brake_runs.jsonl')
OUT = os.path.join(DATA, 'buggy_brake_calibration.yaml')


def analyze(v_steady, dist, t_react, t_stop):
    """-> (react_sec, decel_ms2). dist [m] from the red command to standstill."""
    react = max(t_react, 0.0)
    braking_dist = dist - v_steady * react
    if braking_dist > 0.05:
        decel = v_steady * v_steady / (2.0 * braking_dist)
    else:   # essentially instant stop: fall back to the time-based estimate
        decel = v_steady / max(t_stop - react, 0.1)
    return react, decel


def write_calibration(safety_factor):
    runs = []
    if os.path.exists(RUNS):
        with open(RUNS) as f:
            runs = [json.loads(line) for line in f if line.strip()]
    if not runs:
        return None
    react = max(r['react_sec'] for r in runs) * safety_factor
    decel = min(r['decel_ms2'] for r in runs) / safety_factor
    out = {'react_sec': round(react, 3), 'decel_ms2': round(decel, 3), 'runs': len(runs),
           'safety_factor': safety_factor, 'updated': time.strftime('%Y-%m-%d %H:%M:%S'),
           'note': 'worst case over all runs in buggy_brake_runs.jsonl, times safety_factor'}
    with open(OUT, 'w') as f:
        yaml.safe_dump(out, f)
    return out


class Calibrator(Node):
    def __init__(self):
        super().__init__('brake_calibration')
        self.speed, self.speed_time = None, 0.0
        self.odom_xy = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(SteerSpeed, '/wheel_uart', self.on_wheel, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.steer = self.create_publisher(Float32, '/steering_angle', 1)
        self.red = self.create_publisher(Bool, '/final_red_detected', 1)
        self.yellow = self.create_publisher(Bool, '/final_yellow_detected', 1)
        self.active = self.create_publisher(Bool, '/calibration/active', 1)

    def on_wheel(self, m):
        self.speed, self.speed_time = float(m.speed) / 3.6, time.monotonic()

    def on_odom(self, m):
        self.odom_xy = (m.pose.pose.position.x, m.pose.pose.position.y)

    def position(self):
        """(x, y, source) - NDT pose if fresh, else wheel odometry."""
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            age = (self.get_clock().now() -
                   rclpy.time.Time.from_msg(t.header.stamp,
                                            clock_type=self.get_clock().clock_type)
                   ).nanoseconds / 1e9
            if age < 0.5:
                return t.transform.translation.x, t.transform.translation.y, 'ndt'
        except TransformException:
            pass
        if self.odom_xy:
            return self.odom_xy[0], self.odom_xy[1], 'odom'
        return None

    def command(self, red, yellow, active=True):
        self.steer.publish(Float32(data=0.0))
        self.red.publish(Bool(data=red))
        self.yellow.publish(Bool(data=yellow))
        self.active.publish(Bool(data=active))

    def spin_for(self, sec, red=True, yellow=False, hz=20.0):
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            self.command(red, yellow)
            rclpy.spin_once(self, timeout_sec=1.0 / hz)

    def wait_inputs(self, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.speed is not None and self.position() is not None:
                return True
        return False


def run(node, preset_kmh, max_dist, go_timeout, stop_timeout):
    v_target = preset_kmh / 3.6
    yellow = preset_kmh <= 2.0
    # 1. confirm standstill
    node.spin_for(1.0)
    if node.speed > 0.1:
        raise RuntimeError('vehicle is not stationary')
    p_start = node.position()
    # 2. GO until steady at the preset
    t0, steady_since = time.monotonic(), None
    while True:
        node.command(False, yellow)
        rclpy.spin_once(node, timeout_sec=0.05)
        x, y, _ = node.position()
        travelled = math.hypot(x - p_start[0], y - p_start[1])
        if node.speed >= 0.9 * v_target:
            steady_since = steady_since or time.monotonic()
            if time.monotonic() - steady_since > 1.0:
                break
        else:
            steady_since = None
        if travelled > max_dist or time.monotonic() - t0 > go_timeout:
            raise RuntimeError(f'did not reach {preset_kmh} km/h steadily '
                               f'(travelled {travelled:.1f} m, speed {node.speed * 3.6:.1f} km/h)')
    v_steady = node.speed
    # 3. RED: record
    p0 = node.position()
    t_red = time.monotonic()
    t_drop, zero_since, samples = None, None, []
    while True:
        node.command(True, yellow)
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic() - t_red
        samples.append((now, node.speed))
        if t_drop is None and node.speed <= v_steady - 0.25:    # >= 1 km/h drop
            t_drop = now
        if node.speed < 0.05:
            zero_since = zero_since or now
            if now - zero_since > 1.0:
                break
        else:
            zero_since = None
        if now > stop_timeout:
            raise RuntimeError('did not come to a stop within the timeout')
    # the odometer keeps integrating until the speed reads 0, so measure at zero_since
    p1 = node.position()
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    react, decel = analyze(v_steady, dist, t_drop if t_drop is not None else 0.0, zero_since)
    return {'preset_kmh': preset_kmh, 'v_steady_ms': round(v_steady, 3),
            'react_sec': round(react, 3), 'decel_ms2': round(decel, 3),
            'stop_dist_m': round(dist, 3), 'time_to_stop_s': round(zero_since, 2),
            'pose_source': p1[2], 'time': time.strftime('%Y-%m-%d %H:%M:%S')}


def main():
    code = 0
    try:
        _main()
    except BaseException:       # incl. Ctrl+C: report, but still shut down below
        traceback.print_exc()
        code = 1
    finally:
        # without this the TF listener thread keeps the process alive after a dry run
        if rclpy.ok():
            rclpy.shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)


def _main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--preset', type=float, choices=[2.0, 4.0], default=4.0, help='km/h')
    ap.add_argument('--live', action='store_true', help='really drive (default: dry run)')
    ap.add_argument('--max-dist', type=float, default=15.0, help='abort if GO travels more [m]')
    ap.add_argument('--go-timeout', type=float, default=15.0)
    ap.add_argument('--stop-timeout', type=float, default=12.0)
    ap.add_argument('--safety-factor', type=float, default=1.2)
    args = ap.parse_args()

    rclpy.init()
    node = Calibrator()
    print('waiting for /wheel_uart feedback and a pose (NDT or /odom)...')
    if not node.wait_inputs():
        print('FAIL: no /wheel_uart feedback and/or no pose. Is the ECU serial cable connected (see $BUGGY_DATA_DIR/calibration_base.log)?')
        return
    x, y, src = node.position()
    print(f'OK: feedback {node.speed * 3.6:.1f} km/h, pose from {src} at ({x:.1f}, {y:.1f})')
    if not args.live:
        print(f'DRY RUN: would GO straight at {args.preset} km/h (yellow={args.preset <= 2.0}), '
              f'then red=1 and measure the stop. Nothing was sent. Re-run with --live.')
        return
    print('\n!!! LIVE: the buggy WILL DRIVE straight at '
          f'{args.preset} km/h, then brake. Needs >= {args.max_dist:.0f} m of clear flat space,')
    print('    nobody near, safety driver ready, uart_writer running with loopback:=false.')
    if input('Type YES to continue: ').strip() != 'YES':
        print('aborted')
        return
    try:
        result = run(node, args.preset, args.max_dist, args.go_timeout, args.stop_timeout)
    except (RuntimeError, KeyboardInterrupt) as e:
        node.spin_for(1.5, red=True)
        print(f'ABORTED -> red=1 sent: {e}')
        return
    finally:
        node.spin_for(0.5, red=True)
    os.makedirs(DATA, exist_ok=True)
    with open(RUNS, 'a') as f:
        f.write(json.dumps(result) + '\n')
    print('\nRESULT', json.dumps(result, indent=1))
    cal = write_calibration(args.safety_factor)
    print(f'\nwrote {OUT}: {cal}')
    print('Repeat 3+ times per preset (also 2 km/h); operation_manager reads this file at start-up.')


if __name__ == '__main__':
    main()
