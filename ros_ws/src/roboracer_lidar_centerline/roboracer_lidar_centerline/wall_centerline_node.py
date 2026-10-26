#!/usr/bin/env python3
import math
from typing import List, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .common import XY, scan_to_base_points

class WallCenterlineNode(Node):
    """Generate a local centerline from left and right LiDAR walls."""

    def __init__(self) -> None:
        super().__init__("wall_centerline_node")

        self._declare_parameters()
        self._read_parameters()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            10,
        )

        self.path_publisher = self.create_publisher(
            Path,
            self.path_topic,
            10,
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10,
        )

        self.get_logger().info(
            f"Subscribed to LiDAR topic: {self.scan_topic}"
        )

    def _declare_parameters(self) -> None:
        """Declare all parameters and their fallback values."""

        parameter_defaults = {
            "scan_topic": "/scan",
            "base_frame": "base_link",
            "path_topic": "/local_centerline",
            "marker_topic": "/wall_centerline/markers",

            "minimum_forward_distance": 0.15,
            "maximum_forward_distance": 7.0,
            "maximum_lateral_distance": 4.0,

            "wall_track_width": 1.8,

            "wall_path_start": 0.45,
            "wall_path_horizon": 6.5,
            "wall_station_spacing": 0.25,
            "wall_bin_half_width": 0.20,

            # "wall_minimum_side_distance": 0.20,
            # "wall_minimum_points_per_bin": 2,

            # "allow_single_wall_estimation": True,
            # "smoothing_window": 3,
        }

        for parameter_name, default_value in parameter_defaults.items():
            self.declare_parameter(parameter_name, default_value)

    def _read_parameters(self) -> None:
        """Read parameter values after YAML overrides have been applied."""

        self.scan_topic = str(
            self.get_parameter("scan_topic").value
        )

        self.base_frame = str(
            self.get_parameter("base_frame").value
        )

        self.path_topic = str(
            self.get_parameter("path_topic").value
        )

        self.marker_topic = str(
            self.get_parameter("marker_topic").value
        )

        self.minimum_forward_distance = float(
            self.get_parameter("minimum_forward_distance").value
        )

        self.maximum_forward_distance = float(
            self.get_parameter("maximum_forward_distance").value
        )

        self.maximum_lateral_distance = float(
            self.get_parameter("maximum_lateral_distance").value
        )

        self.wall_track_width = float(
            self.get_parameter("wall_track_width").value
        )

        self.wall_path_start = float(
            self.get_parameter("wall_path_start").value
        )

        self.wall_path_horizon = float(
            self.get_parameter("wall_path_horizon").value
        )

        self.wall_station_spacing = float(
            self.get_parameter("wall_station_spacing").value
        )

        self.wall_bin_half_width = float(
            self.get_parameter("wall_bin_half_width").value
        )

        # self.wall_minimum_side_distance = float(
        #     self.get_parameter("wall_minimum_side_distance").value
        # )

        # self.wall_minimum_points_per_bin = int(
        #     self.get_parameter("wall_minimum_points_per_bin").value
        # )

        # self.allow_single_wall_estimation = bool(
        #     self.get_parameter("allow_single_wall_estimation").value
        # )

        # self.smoothing_window = int(
        #     self.get_parameter("smoothing_window").value
        # )
    
    def _scan_callback(self, scan_message: LaserScan) -> None:
        """
        Called automatically whenever a new LaserScan message arrives.
        """

        self.get_logger().info(
            f"Received scan with {len(scan_message.ranges)} measurements.",
            throttle_duration_sec=1.0,
        )    # Find left wall, right wall and the center between them.

        ordered_points = scan_to_base_points(
            self,
            self.tf_buffer,
            scan_message,
            self.base_frame,
            self.minimum_forward_distance,
            self.maximum_forward_distance,
            self.maximum_lateral_distance,
        )

        valid_points: List[XY] = [
            point
            for point in ordered_points
            if point is not None
        ]

        centerline = self._get_centerline(
            valid_points
        )

        # Publish it as nav_msgs/Path.
        self._publish_path(
            centerline,
            scan_message,
        )

        

    def _get_centerline(
        self,
        points: Sequence[XY],
    ) -> List[XY]:
        """
        Find the midpoint between the left and right walls
        at several forward x positions.

        Coordinate system:
            +x = forward
            +y = left
            -y = right
        """

        centerline: List[XY] = []

        # Forward positions where we calculate the track center.
        x_stations = np.arange(
            self.wall_path_start,
            self.wall_path_horizon,
            self.wall_station_spacing,
        )

        for x_station in x_stations:

            # Get points close to this forward x position.
            nearby_points = [
                (x, y)
                for x, y in points
                if abs(x - x_station) <= self.wall_bin_half_width
            ]

            # Points with positive y are on the left.
            left_points = [
                (x, y)
                for x, y in nearby_points
                if y > 0.0
            ]

            # Points with negative y are on the right.
            right_points = [
                (x, y)
                for x, y in nearby_points
                if y < 0.0
            ]

            # We need both walls to calculate a midpoint.
            if not left_points or not right_points:
                continue

            # Select the wall point closest to the vehicle centre.
            left_wall_point = min(
                left_points,
                key=lambda point: abs(point[1]),
            )

            right_wall_point = min(
                right_points,
                key=lambda point: abs(point[1]),
            )

            left_y = left_wall_point[1]
            right_y = right_wall_point[1]

            # Midpoint between the left and right walls.
            center_y = (
                left_y + right_y
            ) / 2.0

            centerline.append(
                (
                    float(x_station),
                    float(center_y),
                )
            )

        return centerline

    def _publish_path(
        self,
        path: Sequence[XY],
        scan_message: LaserScan,
    ) -> None:
        path_message = Path()
        path_message.header.frame_id = self.base_frame
        path_message.header.stamp = scan_message.header.stamp

        for x, y in path:
            pose = PoseStamped()
            pose.header = path_message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_message.poses.append(pose)

        self.path_publisher.publish(path_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WallCenterlineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
