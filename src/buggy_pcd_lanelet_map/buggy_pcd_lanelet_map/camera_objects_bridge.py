#!/usr/bin/env python3
"""sensor_fusion's camera detector boxes -> the detection format object_labeler reads.

/object_detection/objects (perception_msgs/CameraObjectArray, YOLOX boxes in the RECTIFIED image of
object_detection) is converted to
  /camera/detections/array   ground_segmentation_msgs/Detection2DArray, boxes in the RAW (fisheye)
                             image of /camera/image_raw, as camera_processing's detector published them
so object_labeler names LiDAR / radar obstacles from the same detector sensor_fusion uses.

Rectified box -> raw box: points along the box border, rectified pixel -> normalised ray (new camera
matrix of the rectification, cv2.fisheye.estimateNewCameraMatrixForUndistortRectify with the same
balance / fov_scale as object_detection) -> fisheye distortion with the calibrated K, D -> bounding
box of the distorted points.
"""
import cv2
import numpy as np
import rclpy
from ground_segmentation_msgs.msg import Detection2D, Detection2DArray
from perception_msgs.msg import CameraObjectArray
from rclpy.node import Node


class CameraObjectsBridge(Node):
    def __init__(self):
        super().__init__('camera_objects_bridge')
        intrinsics = self.declare_parameter('intrinsics_file', '').value
        self.balance = self.declare_parameter('rectify_balance', 0.0).value
        self.fov_scale = self.declare_parameter('rectify_fov_scale', 1.0).value
        self.samples = int(self.declare_parameter('border_samples', 6).value)   # points per box edge
        fs = cv2.FileStorage(intrinsics, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise RuntimeError(f'cannot open intrinsics_file "{intrinsics}"')
        self.K = fs.getNode('K').mat().astype(np.float64)
        self.D = fs.getNode('D').mat().astype(np.float64).reshape(-1)[:4]
        self.calib_size = (int(fs.getNode('image_width').real()), int(fs.getNode('image_height').real()))
        self.maps = {}   # image size -> (K scaled, new K inverse)
        self.pub = self.create_publisher(Detection2DArray, '/camera/detections/array', 2)
        self.create_subscription(CameraObjectArray, '/object_detection/objects', self.on_objects, 5)
        self.get_logger().info(f'/object_detection/objects -> /camera/detections/array (raw image), {intrinsics}')

    def models(self, w, h):
        """Calibrated K scaled to the image size and the inverse of the rectified camera matrix."""
        if (w, h) not in self.maps:
            K = self.K.copy()
            K[0] *= w / self.calib_size[0]
            K[1] *= h / self.calib_size[1]
            new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, self.D, (w, h), np.eye(3), balance=self.balance, new_size=(w, h), fov_scale=self.fov_scale)
            self.maps[(w, h)] = (K, np.linalg.inv(new_k))
        return self.maps[(w, h)]

    def on_objects(self, msg):
        out = Detection2DArray()
        out.header = msg.header
        w, h = int(msg.image_width), int(msg.image_height)
        if w > 0 and h > 0 and msg.objects:
            K, new_k_inv = self.models(w, h)
            t = np.linspace(0.0, 1.0, self.samples)
            for o in msg.objects:
                x0, y0, x1, y1 = o.x_min, o.y_min, o.x_max, o.y_max
                # border of the rectified box
                u = np.concatenate([x0 + (x1 - x0) * t, np.full_like(t, x1), x1 - (x1 - x0) * t, np.full_like(t, x0)])
                v = np.concatenate([np.full_like(t, y0), y0 + (y1 - y0) * t, np.full_like(t, y1), y1 - (y1 - y0) * t])
                # rectified pixel -> normalised coordinates: [x, y, 1] = new_K^-1 [u, v, 1]
                n = new_k_inv @ np.vstack([u, v, np.ones_like(u)])
                raw = cv2.fisheye.distortPoints(n[:2].T.reshape(-1, 1, 2).astype(np.float64), K, self.D).reshape(-1, 2)
                d = Detection2D()
                d.class_name = o.class_name
                d.score = float(o.score)
                d.x1, d.y1 = int(np.clip(raw[:, 0].min(), 0, w - 1)), int(np.clip(raw[:, 1].min(), 0, h - 1))
                d.x2, d.y2 = int(np.clip(raw[:, 0].max(), 0, w - 1)), int(np.clip(raw[:, 1].max(), 0, h - 1))
                out.detections.append(d)
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CameraObjectsBridge())


if __name__ == '__main__':
    main()
