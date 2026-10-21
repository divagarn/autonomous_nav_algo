#!/usr/bin/env python3
"""Camera names for LiDAR / radar obstacles (instead of "unknown").

Takes the obstacles the stack already detects - /perception/lidar_objects (LiDAR clusters,
always "unknown") and /perception/radar_objects (radar_fusion tracks) - projects each 3D box
into the camera image (fisheye model of camera_lidar_fusion_node, camera TF from
camera_static_tf) and matches it to the 2D boxes of the object-detection model
(Detection2DArray, e.g. camera_processing's YOLOX 18-class detector) by IoU. A matched
obstacle gets the detector's class name.

Publishes (10 Hz)
  /perception/objects_labeled   PredictedObjects: lidar + radar obstacles with the camera class
  /perception/object_names      std_msgs/String JSON [{x, y, name, score, src}] - operation_manager
                                uses it to say "Obstacle ahead: pedestrian 3.2 m"
  /perception/objects_marker    RViz: class-coloured wireframe + "name score" text
This node only ADDS names. It never removes or delays an obstacle, so the stop logic works the
same with the camera off, blocked or wrong.

ACCURACY NOTE: base_link->camera_link is a rough estimate (camera_static_tf.py), so the projected
boxes can be offset; calibrate it before trusting names at long range.
"""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from autoware_perception_msgs.msg import ObjectClassification, PredictedObjects
from ground_segmentation_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from buggy_pcd_lanelet_map.object_classes import COLORS, label_of

# fisheye calibration, given at 1920x1080 (same as camera_lidar_fusion_node)
CALIB_W, CALIB_H = 1920, 1080
CALIB_K = np.array([[865.8168558653789, 0.0, 946.40124792976326],
                    [0.0, 866.93611621490459, 539.44936044049723], [0.0, 0.0, 1.0]])
CALIB_D = np.array([-0.02884450658109073, 0.0037397682678126918,
                    -0.011255146215826033, 0.0045604499621032029])
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]


def quat_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


class ObjectLabeler(Node):
    def __init__(self):
        super().__init__('object_labeler')
        p = self.declare_parameter
        self.frame = p('map_frame', 'map').value
        self.cam_frame = p('camera_frame', 'camera_link').value
        w, h = int(p('image_width', 960).value), int(p('image_height', 540).value)
        self.min_iou = p('min_iou', 0.15).value
        self.max_age = p('detection_max_age_sec', 0.8).value
        self.min_score = p('min_score', 0.3).value
        self.debug = p('debug_image', False).value
        self.w, self.h = w, h
        self.K = CALIB_K.copy()
        self.K[0] *= w / CALIB_W
        self.K[1] *= h / CALIB_H
        self.D = CALIB_D.copy()

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.lidar, self.radar = None, None
        self.dets, self.dets_time = [], 0.0
        self.last_image = None
        self.create_subscription(PredictedObjects, '/perception/lidar_objects',
                                 lambda m: setattr(self, 'lidar', m), 1)
        self.create_subscription(PredictedObjects, '/perception/radar_objects',
                                 lambda m: setattr(self, 'radar', m), 1)
        self.create_subscription(Detection2DArray, 'camera/detections/array', self.on_dets, 2)
        if self.debug:
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.create_subscription(Image, 'camera/image_raw',
                                     lambda m: setattr(self, 'last_image', m), 1)
            self.dbg_pub = self.create_publisher(Image, '/perception/labeler_debug/image', 1)
        self.out_pub = self.create_publisher(PredictedObjects, '/perception/objects_labeled', 1)
        self.names_pub = self.create_publisher(String, '/perception/object_names', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/objects_marker', 1)
        self.create_timer(0.1, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def stamp_of(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_dets(self, msg):
        self.dets = [(float(d.x1), float(d.y1), float(d.x2), float(d.y2), d.class_name,
                      float(d.score)) for d in msg.detections if d.score >= self.min_score]
        self.dets_time = self.now()

    def project(self, obj):
        """Projected pixel bbox (x1, y1, x2, y2) of an object's 3D box, or None."""
        pose = obj.kinematics.initial_pose_with_covariance.pose
        cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        l, w, h = (obj.shape.dimensions.x, obj.shape.dimensions.y, obj.shape.dimensions.z)
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for dz in (-h / 2, h / 2):
            for dx, dy in ((-l / 2, -w / 2), (l / 2, -w / 2), (l / 2, w / 2), (-l / 2, w / 2)):
                pts.append((cx + c * dx - s * dy, cy + s * dx + c * dy, cz + dz))
        pts = np.array(pts)
        try:
            t = self.tf.lookup_transform(self.cam_frame, self.frame, rclpy.time.Time())
        except TransformException:
            return None
        q, tr = t.transform.rotation, t.transform.translation
        body = pts @ quat_to_matrix(q.x, q.y, q.z, q.w).T + np.array([tr.x, tr.y, tr.z])
        opt = np.stack([-body[:, 1], -body[:, 2], body[:, 0]], axis=1)   # body -> optical
        front = opt[:, 2] > 0.5
        if front.sum() < 4:
            return None
        px, _ = cv2.fisheye.projectPoints(opt[front].reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                                          self.K, self.D)
        px = px.reshape(-1, 2)
        x1, y1 = px.min(axis=0)
        x2, y2 = px.max(axis=0)
        x1, y1, x2, y2 = max(x1, 0), max(y1, 0), min(x2, self.w), min(y2, self.h)
        if x2 - x1 < 4 or y2 - y1 < 4:
            return None
        return float(x1), float(y1), float(x2), float(y2)

    def step(self):
        objs = []     # (src, PredictedObject)
        for src, msg in (('lidar', self.lidar), ('radar', self.radar)):
            if msg is not None and self.now() - self.stamp_of(msg) < 2.0:
                objs += [(src, o) for o in msg.objects]
        if not objs:
            return
        have_cam = self.dets and self.now() - self.dets_time < self.max_age
        boxes = [self.project(o) for _, o in objs] if have_cam else [None] * len(objs)
        names = [None] * len(objs)          # (name, score)
        if have_cam:
            cand = []
            for i, b in enumerate(boxes):
                if b is None:
                    continue
                for j, d in enumerate(self.dets):
                    v = iou(b, d[:4])
                    if v >= self.min_iou:
                        cand.append((v * (0.5 + 0.5 * d[5]), i, j))
            used_i, used_j = set(), set()
            for v, i, j in sorted(cand, reverse=True):
                if i in used_i or j in used_j:
                    continue
                used_i.add(i)
                used_j.add(j)
                names[i] = (self.dets[j][4], self.dets[j][5])

        out = PredictedObjects()
        out.header.stamp, out.header.frame_id = self.get_clock().now().to_msg(), self.frame
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        info = []
        for k, ((src, o), nm) in enumerate(zip(objs, names)):
            cur = o.classification[0].label if o.classification else ObjectClassification.UNKNOWN
            name, score = nm if nm else (None, 0.0)
            if name is not None:
                o.classification = [ObjectClassification(label=label_of(name),
                                                         probability=float(score))]
                label = label_of(name)
            else:
                label = cur
            out.objects.append(o)
            shown = name or 'unknown'
            p = o.kinematics.initial_pose_with_covariance.pose
            info.append({'x': p.position.x, 'y': p.position.y, 'name': shown,
                         'score': round(score, 2), 'src': src})
            markers.markers += self.draw(o, k, label, shown, score, src)
        self.out_pub.publish(out)
        self.names_pub.publish(String(data=json.dumps(info)))
        self.marker_pub.publish(markers)
        if self.debug and self.last_image is not None:
            self.publish_debug(boxes, names)

    def draw(self, o, k, label, shown, score, src):
        pose = o.kinematics.initial_pose_with_covariance.pose
        l, w, h = o.shape.dimensions.x, o.shape.dimensions.y, o.shape.dimensions.z
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        c, s = math.cos(yaw), math.sin(yaw)
        cx, cy, cz = pose.position.x, pose.position.y, pose.position.z
        rgb = COLORS.get(label, COLORS[ObjectClassification.UNKNOWN])
        corners = [Point(x=cx + c * dx - s * dy, y=cy + s * dx + c * dy, z=cz + dz)
                   for dz in (-h / 2, h / 2)
                   for dx, dy in ((-l / 2, -w / 2), (l / 2, -w / 2), (l / 2, w / 2), (-l / 2, w / 2))]
        m = Marker()
        m.header.frame_id, m.header.stamp = self.frame, self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'labeled_box', k, Marker.LINE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
        for a, b in EDGES:
            m.points += [corners[a], corners[b]]
        m.lifetime.sec, m.lifetime.nanosec = 0, 400_000_000
        t = Marker()
        t.header = m.header
        t.ns, t.id, t.type, t.action = 'labeled_text', k, Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position.x, t.pose.position.y, t.pose.position.z = cx, cy, cz + h / 2 + 0.5
        t.pose.orientation.w = 1.0
        t.scale.z = 0.6
        t.color.r, t.color.g, t.color.b, t.color.a = (*rgb, 1.0)
        t.text = f'{shown} {score:.2f}' if shown != 'unknown' else f'unknown ({src})'
        t.lifetime.sec, t.lifetime.nanosec = 0, 400_000_000
        return [m, t]

    def publish_debug(self, boxes, names):
        img = self.bridge.imgmsg_to_cv2(self.last_image, 'bgr8').copy()
        for d in self.dets:
            cv2.rectangle(img, (int(d[0]), int(d[1])), (int(d[2]), int(d[3])), (0, 255, 0), 2)
            cv2.putText(img, f'{d[4]} {d[5]:.2f}', (int(d[0]), max(int(d[1]) - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        for b, nm in zip(boxes, names):
            if b is not None:
                cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])),
                              (0, 0, 255) if nm is None else (255, 0, 255), 2)
        self.dbg_pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))


def main():
    rclpy.init()
    rclpy.spin(ObjectLabeler())


if __name__ == '__main__':
    main()
