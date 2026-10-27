#!/usr/bin/env python3
"""sensor_fusion objects -> the obstacle format the AUTO logic uses.

/sensor_fusion/objects (objects_topic; perception_msgs/FusedObjectArray, sensor_fusion's radar
tracks (EKF tracker or IMM), class from the camera fusion, "unknown" without a camera box) is converted to
  /perception/radar_objects         autoware_perception_msgs/PredictedObjects   (map frame)
  /perception/radar_objects_marker  RViz boxes (orange; a velocity arrow if moving) + text
Published for EVERY radar cycle, even empty, so subscribers can tell "radar alive, nothing
there" from "radar silent".
"""
import math

import rclpy
from autoware_perception_msgs.msg import ObjectClassification, PredictedObject, PredictedObjects
from autoware_perception_msgs.msg import Shape
from geometry_msgs.msg import Point
from perception_msgs.msg import FusedObjectArray
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from buggy_pcd_lanelet_map.object_classes import SIZES, label_of


class RadarObjectBridge(Node):
    def __init__(self):
        super().__init__('radar_object_bridge')
        self.frame = self.declare_parameter('map_frame', 'map').value
        self.min_existence = self.declare_parameter('min_existence', 0.5).value
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(PredictedObjects, '/perception/radar_objects', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/radar_objects_marker', 1)
        topic = self.declare_parameter('objects_topic', '/sensor_fusion/objects').value
        self.create_subscription(FusedObjectArray, topic, self.on_objects, 5)

    def to_map(self, frame, x, y, yaw):
        """(x, y, yaw) in `frame` -> map frame (identity if already in map)."""
        if frame in ('', self.frame):
            return x, y, yaw
        t = self.tf.lookup_transform(self.frame, frame, rclpy.time.Time())
        q = t.transform.rotation
        fyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(fyaw), math.sin(fyaw)
        return (t.transform.translation.x + c * x - s * y,
                t.transform.translation.y + s * x + c * y, yaw + fyaw)

    def on_objects(self, msg):
        out = PredictedObjects()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        try:
            for i, o in enumerate(msg.objects):
                if o.ghost or o.existence_probability < self.min_existence:
                    continue
                label = label_of(o.class_name)
                dl, dw, dh = SIZES[label]
                length = o.dimensions.x if o.dimensions.x > 0.3 else dl
                width = o.dimensions.y if o.dimensions.y > 0.3 else dw
                height = o.dimensions.z if o.dimensions.z > 0.3 else dh
                speed = math.hypot(o.velocity.x, o.velocity.y)
                yaw0 = o.heading if o.is_moving else 0.0
                x, y, yaw = self.to_map(msg.header.frame_id, o.position.x, o.position.y, yaw0)
                obj = PredictedObject()
                obj.existence_probability = float(o.existence_probability)
                obj.classification = [ObjectClassification(
                    label=label, probability=float(max(o.class_confidence, 0.5)))]
                p = obj.kinematics.initial_pose_with_covariance.pose
                p.position.x, p.position.y, p.position.z = x, y, height / 2.0
                p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                obj.kinematics.initial_twist_with_covariance.twist.linear.x = float(speed)
                obj.shape.type = Shape.BOUNDING_BOX
                obj.shape.dimensions.x, obj.shape.dimensions.y, obj.shape.dimensions.z = \
                    float(length), float(width), float(height)
                out.objects.append(obj)

                m = Marker()
                m.header = out.header
                m.ns, m.id, m.type, m.action = 'radar', i, Marker.CUBE, Marker.ADD
                m.pose = p
                m.scale.x, m.scale.y, m.scale.z = float(length), float(width), float(height)
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.55, 0.0, 0.5
                m.lifetime.sec, m.lifetime.nanosec = 0, 400_000_000
                markers.markers.append(m)
                t = Marker()
                t.header = out.header
                t.ns, t.id, t.type, t.action = 'radar_text', i, Marker.TEXT_VIEW_FACING, Marker.ADD
                t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, height + 0.7
                t.pose.orientation.w = 1.0
                t.scale.z = 0.5
                t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 0.7, 0.2, 1.0
                t.text = f'radar {o.class_name or "unknown"}' + \
                    (f' {speed * 3.6:.0f} km/h' if o.is_moving else '')
                t.lifetime.sec, t.lifetime.nanosec = 0, 400_000_000
                markers.markers.append(t)
                if o.is_moving and speed > 0.3:
                    a = Marker()
                    a.header = out.header
                    a.ns, a.id, a.type, a.action = 'radar_vel', i, Marker.ARROW, Marker.ADD
                    a.scale.x, a.scale.y, a.scale.z = 0.12, 0.3, 0.3
                    a.color.r, a.color.g, a.color.b, a.color.a = 1.0, 0.3, 0.0, 1.0
                    vx, vy, _ = self.to_map(msg.header.frame_id, o.position.x + o.velocity.x,
                                            o.position.y + o.velocity.y, 0.0)
                    a.points = [Point(x=x, y=y, z=height / 2), Point(x=vx, y=vy, z=height / 2)]
                    a.lifetime.sec, a.lifetime.nanosec = 0, 400_000_000
                    markers.markers.append(a)
        except TransformException:
            return    # no map pose yet: publish nothing rather than objects in the wrong frame
        self.pub.publish(out)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    rclpy.spin(RadarObjectBridge())


if __name__ == '__main__':
    main()
