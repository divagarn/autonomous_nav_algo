#!/usr/bin/env python3
"""NDT localization supervisor (Autoware pose_initializer + a status monitor).

  RViz 2D Pose Estimate (/initialpose)
    -> ndt_scan_matcher ndt_align_srv (particle search around the click, on
       the latest scan)
    -> aligned pose to ekf_localizer (/initialpose3d)
    -> activate ekf_localizer, then ndt_scan_matcher (trigger_node, once)
  Re-clicking re-seeds the EKF at any time.

State (/localization/state, "k=v;k=v" String, 5 Hz):
  uninitialized  no pose given yet
  aligning       ndt_align_srv running
  align_failed   no successful alignment (click again, closer to the truth)
  initialized    seed sent, waiting for the first NDT result
  converged      fresh NDT pose and score >= score_ok
  degraded       NDT pose fresh but score < score_ok, or stale 1-3 s
  lost           no NDT pose for > lost_timeout_sec
Keys: state, score (nearest-voxel likelihood), iterations, exe_ms,
age (s since last NDT pose), x, y, yaw (map->base_link TF, deg), msg.
"""
import math
import threading
import time

import rclpy
from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped
from autoware_internal_localization_msgs.srv import PoseWithCovarianceStamped as AlignSrv
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizationManager(Node):
    def __init__(self):
        super().__init__('localization_manager')
        p = self.declare_parameter
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.score_ok = float(p('score_ok', 1.4).value)
        self.lost_timeout = float(p('lost_timeout_sec', 3.0).value)
        self.align_timeout = float(p('align_timeout_sec', 40.0).value)
        self.max_retries = int(p('max_retries', 2).value)
        self.ekf_settle = float(p('ekf_settle_sec', 1.5).value)
        self.state, self.msg = 'uninitialized', 'Give an initial pose (2D Pose Estimate)'
        self.activated = False
        self.busy = False
        self.score = self.iters = self.exe_ms = 0.0
        self.last_ndt = None

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.seed_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose3d', qos)
        self.state_pub = self.create_publisher(String, '/localization/state', 1)
        self.align = self.create_client(AlignSrv, '/ndt_align_srv')
        self.ekf_trigger = self.create_client(SetBool, '/ekf_localizer/trigger_node')
        self.ndt_trigger = self.create_client(SetBool, '/ndt_scan_matcher/trigger_node')

        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_click, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ndt_pose_with_covariance',
                                 self.on_ndt_pose, 10)
        self.create_subscription(Float32Stamped, '/nearest_voxel_transformation_likelihood',
                                 lambda m: setattr(self, 'score', m.data), 10)
        self.create_subscription(Int32Stamped, '/iteration_num',
                                 lambda m: setattr(self, 'iters', m.data), 10)
        self.create_subscription(Float32Stamped, '/exe_time_ms',
                                 lambda m: setattr(self, 'exe_ms', m.data), 10)
        self.create_timer(0.2, self.publish_state)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_ndt_pose(self, _):
        self.last_ndt = self.now_s()

    def on_click(self, msg):
        if self.busy:
            self.get_logger().warn('alignment already running, click ignored')
            return
        self.busy = True
        self.state, self.msg = 'aligning', 'NDT aligning...'
        threading.Thread(target=self.run_align, args=(msg,), daemon=True).start()

    def wait(self, future, timeout):
        t = 0.0
        while not future.done() and t < timeout:
            time.sleep(0.05)
            t += 0.05
        return future.result() if future.done() else None

    def trigger(self, client, name):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{name} trigger service not available')
            return False
        req = SetBool.Request()
        req.data = True
        r = self.wait(client.call_async(req), 5.0)
        ok = r is not None and r.success
        self.get_logger().info(f'{name} activated' if ok else f'{name} trigger failed')
        return ok

    def run_align(self, click):
        try:
            guess = click
            guess.header.frame_id = guess.header.frame_id or self.map_frame
            # generous search uncertainty (2 m, ~17 deg) around the click
            guess.pose.covariance[0] = guess.pose.covariance[7] = 4.0
            guess.pose.covariance[35] = 0.09
            aligned = None
            for attempt in range(1, self.max_retries + 1):
                if not self.align.wait_for_service(timeout_sec=2.0):
                    self.msg = 'ndt_align_srv not available (is ndt_scan_matcher up?)'
                    continue
                req = AlignSrv.Request()
                req.pose_with_covariance = guess
                req.pose_with_covariance.header.stamp = self.get_clock().now().to_msg()
                r = self.wait(self.align.call_async(req), self.align_timeout)
                if r is not None and r.success:
                    aligned = r.pose_with_covariance
                    self.get_logger().info(f'aligned (attempt {attempt}, reliable={r.reliable})')
                    break
                self.msg = f'align attempt {attempt}/{self.max_retries} failed'
                self.get_logger().warn(self.msg)
                time.sleep(1.0)
            if aligned is None:
                self.state = 'align_failed'
                self.msg = 'Alignment failed: click closer to the real position, check LiDAR'
                return
            aligned.header.stamp = self.get_clock().now().to_msg()
            aligned.header.frame_id = aligned.header.frame_id or self.map_frame
            self.seed_pub.publish(aligned)
            self.last_ndt = None
            self.state, self.msg = 'initialized', 'Seed sent, waiting for NDT'
            if not self.activated:
                if self.trigger(self.ekf_trigger, 'ekf_localizer'):
                    time.sleep(self.ekf_settle)
                    self.activated = self.trigger(self.ndt_trigger, 'ndt_scan_matcher')
                if not self.activated:
                    self.state, self.msg = 'align_failed', 'Could not activate EKF/NDT'
        finally:
            self.busy = False

    def update_state(self):
        if self.state in ('uninitialized', 'aligning', 'align_failed') or not self.activated:
            return
        if self.last_ndt is None:
            return  # initialized, waiting for the first result
        age = self.now_s() - self.last_ndt
        if age > self.lost_timeout:
            self.state, self.msg = 'lost', f'No NDT pose for {age:.0f} s'
        elif age > 1.0:
            self.state, self.msg = 'degraded', f'NDT pose stale ({age:.1f} s)'
        elif self.score < self.score_ok:
            self.state, self.msg = 'degraded', f'Low match score {self.score:.2f}'
        else:
            self.state, self.msg = 'converged', ''

    def publish_state(self):
        self.update_state()
        x = y = yaw = float('nan')
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                rclpy.time.Time())
            q = t.transform.rotation
            x, y = t.transform.translation.x, t.transform.translation.y
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))
        except TransformException:
            pass
        age = self.now_s() - self.last_ndt if self.last_ndt is not None else -1.0
        kv = {'state': self.state, 'score': f'{self.score:.2f}', 'iterations': f'{self.iters:.0f}',
              'exe_ms': f'{self.exe_ms:.0f}', 'age': f'{age:.1f}', 'x': f'{x:.2f}',
              'y': f'{y:.2f}', 'yaw': f'{yaw:.1f}', 'msg': self.msg}
        self.state_pub.publish(String(data=';'.join(f'{k}={v}' for k, v in kv.items())))


def main():
    rclpy.init()
    rclpy.spin(LocalizationManager())


if __name__ == '__main__':
    main()
