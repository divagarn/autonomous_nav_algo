#!/usr/bin/env python3
"""Dummy perception objects (Autoware's dummy_perception_publisher, simplified).

RViz tools Add Car / Add Bus / Add Pedestrian (buggy_rviz_plugins) and the
panel's "Delete all dummies" button send String requests on /dummy/request:
  add:<car|bus|pedestrian>:<x>:<y>:<yaw_rad>
  delete_all
Objects are static. Published at 5 Hz:
  /perception/object_recognition/objects  autoware_perception_msgs/PredictedObjects
  /simulation/dummy_objects/marker        visualization_msgs/MarkerArray
operation_manager stops the simulated vehicle for objects on its path.
"""
import math
import uuid

import rclpy
from autoware_perception_msgs.msg import ObjectClassification, PredictedObject, PredictedObjects
from autoware_perception_msgs.msg import Shape
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

# type -> (length, width, height, classification label, rgb)
TYPES = {
    'car': (4.5, 1.8, 1.6, ObjectClassification.CAR, (0.2, 0.5, 1.0)),
    'bus': (10.5, 2.5, 3.2, ObjectClassification.BUS, (1.0, 0.7, 0.0)),
    'pedestrian': (0.6, 0.6, 1.8, ObjectClassification.PEDESTRIAN, (1.0, 0.2, 0.4)),
}


class DummyObjects(Node):
    def __init__(self):
        super().__init__('dummy_objects')
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.objects = []   # (uuid bytes, type, x, y, yaw)
        self.create_subscription(String, '/dummy/request', self.on_request, 10)
        self.obj_pub = self.create_publisher(
            PredictedObjects, '/perception/object_recognition/objects', 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/simulation/dummy_objects/marker', 1)
        self.create_timer(0.2, self.publish)

    def on_request(self, msg):
        parts = msg.data.split(':')
        if parts[0] == 'delete_all':
            self.objects.clear()
        elif parts[0] == 'add' and len(parts) == 5 and parts[1] in TYPES:
            try:
                x, y, yaw = float(parts[2]), float(parts[3]), float(parts[4])
            except ValueError:
                return
            self.objects.append((uuid.uuid4().bytes, parts[1], x, y, yaw))
            self.get_logger().info(f'added {parts[1]} at ({x:.1f}, {y:.1f})')

    def publish(self):
        now = self.get_clock().now().to_msg()
        objs = PredictedObjects()
        objs.header.stamp, objs.header.frame_id = now, self.frame
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for n, (uid, typ, x, y, yaw) in enumerate(self.objects):
            length, width, height, label, rgb = TYPES[typ]
            o = PredictedObject()
            o.object_id.uuid = list(uid)
            o.existence_probability = 1.0
            o.classification = [ObjectClassification(label=label, probability=1.0)]
            p = o.kinematics.initial_pose_with_covariance.pose
            p.position.x, p.position.y, p.position.z = x, y, height / 2.0
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            o.shape.type = Shape.BOUNDING_BOX
            o.shape.dimensions.x, o.shape.dimensions.y, o.shape.dimensions.z = \
                length, width, height
            objs.objects.append(o)

            m = Marker()
            m.header.stamp, m.header.frame_id = now, self.frame
            m.ns, m.id, m.type, m.action = 'dummy', n, Marker.CUBE, Marker.ADD
            m.pose = p
            m.scale.x, m.scale.y, m.scale.z = length, width, height
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 0.85)
            markers.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns, t.id, t.type, t.action = 'dummy_label', n, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, height + 0.6
            t.pose.orientation.w = 1.0
            t.scale.z = 0.7
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = typ
            markers.markers.append(t)

        self.obj_pub.publish(objs)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    rclpy.spin(DummyObjects())


if __name__ == '__main__':
    main()
