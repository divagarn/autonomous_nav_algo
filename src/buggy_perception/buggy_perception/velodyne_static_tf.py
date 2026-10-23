#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# NumPy compatibility for older transforms3d used by tf_transformations.
if not hasattr(np, "float"):
    np.float = float

from tf_transformations import quaternion_from_euler


class VelodyneStaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('velodyne_static_tf_broadcaster')

        broadcaster = StaticTransformBroadcaster(self)

        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        # configurable (config/buggy_params.yaml, section velodyne_static_tf)
        p = self.declare_parameter
        static_transformStamped.header.frame_id = p('parent_frame', 'base_link').value   # parent frame
        static_transformStamped.child_frame_id = p('child_frame', 'velodyne').value      # child frame

        # Translation: x = rear axle -> LiDAR, y = 0 centred, z = height of the LiDAR above the ground
        static_transformStamped.transform.translation.x = float(p('x', 1.73).value)
        static_transformStamped.transform.translation.y = float(p('y', 0.0).value)
        static_transformStamped.transform.translation.z = float(p('z', 1.00).value)

        # Euler angles (roll, pitch, yaw) in radians
        roll = float(p('roll', 0.0).value)
        pitch = float(p('pitch', 0.0).value)      # e.g. 0.296706 for a LiDAR tilted down 17 deg
        yaw = float(p('yaw', 0.0).value)
        # Convert Euler to quaternion
        quat = quaternion_from_euler(roll, pitch, yaw)

        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        # Broadcast once (static transform)
        broadcaster.sendTransform(static_transformStamped)

        tr = static_transformStamped.transform.translation
        self.get_logger().info(
            f"Broadcasted static transform from {static_transformStamped.header.frame_id} to "
            f"{static_transformStamped.child_frame_id}: x={tr.x:.2f}m, z={tr.z:.2f}m")


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneStaticTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
