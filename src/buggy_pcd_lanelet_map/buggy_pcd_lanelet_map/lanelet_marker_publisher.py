#!/usr/bin/env python3
"""Publishes a Lanelet2 .osm as a latched MarkerArray (/map/vector_map_marker).

Reads local_x/local_y/ele tags straight from the XML - the non-georeferenced
convention Autoware's Local projector and Vector Map Builder use (lat/lon are
empty, so lanelet2.io.load() would collapse every point to the origin).
Lanelet bounds are drawn white, centrelines (computed from left/right bound
midpoints) green.
"""
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def parse_osm(path):
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.findall('node'):
        tags = {t.get('k'): t.get('v') for t in n.findall('tag')}
        if 'local_x' in tags and 'local_y' in tags:
            nodes[n.get('id')] = (float(tags['local_x']), float(tags['local_y']),
                                  float(tags.get('ele', 0.0)))
    ways = {w.get('id'): [nodes[r.get('ref')] for r in w.findall('nd') if r.get('ref') in nodes]
            for w in root.findall('way')}
    lanelets = []
    for rel in root.findall('relation'):
        tags = {t.get('k'): t.get('v') for t in rel.findall('tag')}
        if tags.get('type') != 'lanelet':
            continue
        m = {mem.get('role'): mem.get('ref') for mem in rel.findall('member')}
        if m.get('left') in ways and m.get('right') in ways:
            lanelets.append((ways[m['left']], ways[m['right']]))
    return ways, lanelets


def line_marker(mid, ns, rgb, width, pts_list, frame):
    m = Marker()
    m.header.frame_id = frame
    m.ns, m.id = ns, mid
    m.type, m.action = Marker.LINE_LIST, Marker.ADD
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
    m.pose.orientation.w = 1.0
    for pts in pts_list:
        for a, b in zip(pts, pts[1:]):
            m.points.append(Point(x=a[0], y=a[1], z=a[2]))
            m.points.append(Point(x=b[0], y=b[1], z=b[2]))
    return m


def main():
    rclpy.init()
    node = Node('lanelet_marker_publisher')
    path = node.declare_parameter('lanelet2_map_path', '').value
    frame = node.declare_parameter('frame_id', 'map').value
    topic = node.declare_parameter('topic', '/map/vector_map_marker').value
    ways, lanelets = parse_osm(path)

    centerlines = []
    for left, right in lanelets:
        n = min(len(left), len(right))
        centerlines.append([tuple((l + r) / 2 for l, r in zip(left[i], right[i]))
                            for i in range(n)])
    arr = MarkerArray()
    arr.markers.append(line_marker(0, 'bounds', (1.0, 1.0, 1.0), 0.15, ways.values(), frame))
    arr.markers.append(line_marker(1, 'centerline', (0.0, 1.0, 0.0), 0.1, centerlines, frame))

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(MarkerArray, topic, qos)
    pub.publish(arr)
    node.get_logger().info(f'{len(lanelets)} lanelets from {path} -> {topic}')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
