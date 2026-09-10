# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Detection-only YOLO cone node. No depth, no 3D, no custom messages --
# purely to assess how well the existing weights detect cones in Gazebo.

from pathlib import Path

import cv2
import numpy as np
import rclpy
import message_filters

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from ultralytics import YOLO
from geometry_msgs.msg import PoseArray, Pose


BOX_COLORS = [
    (255, 160, 40),
    (40, 220, 255),
    (80, 220, 120),
    (200, 120, 255),
]


def find_default_model():
    """Look for best.pt in the installed share dir, then in the source tree."""
    candidates = []

    try:
        share = Path(get_package_share_directory('roboracer_perception'))
        candidates.append(share / 'models' / 'best.pt')
    except Exception:
        pass

    # Source tree: <pkg>/roboracer_perception/<this file> -> <pkg>/models/best.pt
    candidates.append(Path(__file__).resolve().parent.parent / 'models' / 'best.pt')

    for path in candidates:
        if path.is_file():
            return str(path)
    return ''


class YoloConeDebugNode(Node):
    """Runs YOLO on the camera feed and reports detection quality."""

    def __init__(self):
        super().__init__('yolo_cone_debug_node')

        self.declare_parameter('image_topic', '/camera/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/camera_info')
        self.declare_parameter('model_path', find_default_model())
        self.declare_parameter('confidence_threshold', 0.05)
        self.declare_parameter('blue_conf', 0.3)
        self.declare_parameter('yellow_conf', 0.5)
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('iou_threshold', 0.4)
        self.declare_parameter('position_confidence_threshold', 0.4)
        self.declare_parameter('max_depth', 10.0)

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        model_path = self.get_parameter('model_path').value
        report_period = self.get_parameter('report_period').value
        self._iou = self.get_parameter('iou_threshold').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self._pos_conf = self.get_parameter('position_confidence_threshold').value
        self._conf = self.get_parameter('confidence_threshold').value
        self._class_conf = {
            'blue_cones': self.get_parameter('blue_conf').value,
            'yellow_cones': self.get_parameter('yellow_conf').value,
        }
        self._max_depth = self.get_parameter('max_depth').value


        if not model_path:
            raise RuntimeError(
                'No model found. Put best.pt in roboracer_perception/models/ '
                'or pass -p model_path:=/full/path/to/best.pt'
            )
        if not Path(model_path).is_file():
            raise RuntimeError(f'model_path does not exist: {model_path}')

        self.get_logger().info(f'loading model: {model_path}')
        self._model = YOLO(model_path)
        self._names = self._model.names
        self.get_logger().info(f'classes: {self._names}')

        self._bridge = CvBridge()

        rgb_sub = message_filters.Subscriber(self, Image, image_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.1
        )
        self._info_sub = self.create_subscription(
        CameraInfo, camera_info_topic, self._camera_info_cb, 1
        )
        self._blue_pub = self.create_publisher(PoseArray, '/cones/blue', 5)
        self._yellow_pub = self.create_publisher(PoseArray, '/cones/yellow', 5)
        self._sync.registerCallback(self._synced_cb)

        self._frames = 0
        self._frames_with_det = 0
        self._confs = []
        self._per_class = {}
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
        self.create_timer(report_period, self._report)

        self.get_logger().info(
            f'listening on {image_topic} + {depth_topic}, '
            f'inference conf={self._conf}, iou={self._iou}, '
            f'publish gates: {self._class_conf}'
        )

    def _camera_info_cb(self, msg):
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]
        self.get_logger().info(
            f'camera intrinsics: fx={self._fx:.2f} fy={self._fy:.2f} '
            f'cx={self._cx:.2f} cy={self._cy:.2f}'
        )
        self.destroy_subscription(self._info_sub)


    def _synced_cb(self, msg, depth_msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return
        
        self._cone_points = []

        results = self._model(frame, conf=self._conf, iou=self._iou, verbose=False)

        n_det = 0
        for res in results:
            boxes = getattr(res, 'boxes', None)
            if boxes is None:
                continue
            for box in boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self._names.get(cls_id, str(cls_id))
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])

                u = (x1 + x2) // 2
                v = (y1 + y2) // 2
                z = float(depth[v, u])

                if z <= 0.0 or z > 10.0 or not np.isfinite(z) or self._fx is None:
                    continue

                x = (u - self._cx) * z / self._fx
                y = (v - self._cy) * z / self._fy

                if conf >= self._class_conf.get(label, 1.0):
                    self._cone_points.append((z, x, label))

                self._confs.append(conf)
                self._per_class.setdefault(label, []).append(conf)
                n_det += 1

        blue_msg = PoseArray()
        blue_msg.header = msg.header
        yellow_msg = PoseArray()
        yellow_msg.header = msg.header

        for z, x, label in self._cone_points:
            p = Pose()
            p.position.x = z
            p.position.y = -x
            p.position.z = 0.0

            if label == 'blue_cones':
                blue_msg.poses.append(p)
            elif label == 'yellow_cones':
                yellow_msg.poses.append(p)

        self._blue_pub.publish(blue_msg)
        self._yellow_pub.publish(yellow_msg)
        self._frames += 1
        if n_det:
            self._frames_with_det += 1

    def _report(self):
        if self._frames == 0:
            self.get_logger().info('no frames received yet')
            return

        hit_rate = 100.0 * self._frames_with_det / self._frames
        lines = [
            f'frames={self._frames}  frames_with_detections={hit_rate:.0f}%  '
            f'total_detections={len(self._confs)}'
        ]

        if self._confs:
            arr = np.array(self._confs)
            lines.append(
                f'  confidence  min={arr.min():.2f}  median={np.median(arr):.2f}  '
                f'mean={arr.mean():.2f}  max={arr.max():.2f}'
            )
            for label, vals in sorted(self._per_class.items()):
                v = np.array(vals)
                lines.append(
                    f'  {label:<10} n={len(v):<5} median={np.median(v):.2f}  '
                    f'>0.5: {100.0 * (v > 0.5).mean():.0f}%  '
                    f'>0.8: {100.0 * (v > 0.8).mean():.0f}%'
                )
        else:
            lines.append('  no detections at all')

        self.get_logger().info('\n'.join(lines))

        self._frames = 0
        self._frames_with_det = 0
        self._confs = []
        self._per_class = {}


def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
