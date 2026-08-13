#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time

XY = Tuple[float, float]

class PurePursuitNode(Node):
    """
    Follow a local centerline using the Pure Pursuit algorithm.
    """

    def __init__(self) -> None:
        super().__init__("pure_pursuit_node")

        self._declare_parameters()
        self._read_parameters()
        self.latest_path: List[XY] = []
        self.latest_path_time: Optional[Time] = None

        self.path_subscription = self.create_subscription(
            Path,
            self.path_topic,
            self._path_callback,
            10,
        )

        self.command_publisher = self.create_publisher(
            Twist,
            self.command_topic,
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self._control_callback,
        )

    def _declare_parameters(self) -> None:
        """
        Declare Pure Pursuit parameters and their default values.
        """

        parameter_defaults = {
            "path_topic": "/local_centerline",
            "command_topic": "/cmd_vel",

            "lookahead_distance": 0.85,
            "wheelbase": 0.32,

            "target_speed": 0.45,
            "minimum_speed": 0.22,
            "maximum_speed": 0.60,

            "maximum_steering_angle": 0.55,
            "curvature_speed_gain": 1.2,

            "minimum_forward_distance": 0.10,
            "minimum_path_points": 2,

            "control_frequency": 30.0,
            "path_timeout": 0.5,
        }

        for parameter_name, default_value in parameter_defaults.items():
            self.declare_parameter(
                parameter_name,
                default_value,
            )

    def _read_parameters(self) -> None:
        """
        Read the parameter values after YAML overrides are applied.
        """

        self.path_topic = str(
            self.get_parameter("path_topic").value
        )

        self.command_topic = str(
            self.get_parameter("command_topic").value
        )

        self.lookahead_distance = float(
            self.get_parameter("lookahead_distance").value
        )

        self.wheelbase = float(
            self.get_parameter("wheelbase").value
        )

        self.target_speed = float(
            self.get_parameter("target_speed").value
        )

        self.minimum_speed = float(
            self.get_parameter("minimum_speed").value
        )

        self.maximum_speed = float(
            self.get_parameter("maximum_speed").value
        )

        self.maximum_steering_angle = float(
            self.get_parameter("maximum_steering_angle").value
        )

        self.curvature_speed_gain = float(
            self.get_parameter("curvature_speed_gain").value
        )

        self.minimum_forward_distance = float(
            self.get_parameter("minimum_forward_distance").value
        )

        self.minimum_path_points = int(
            self.get_parameter("minimum_path_points").value
        )

        self.control_frequency = float(
            self.get_parameter("control_frequency").value
        )

        self.path_timeout = float(
            self.get_parameter("path_timeout").value
        )

    def _path_callback(
        self,
        path_message: Path,
    ) -> None:
        """
        Convert the received ROS Path into a Python list of x-y points.
        """

        received_path: List[XY] = []

        for pose_stamped in path_message.poses:
            x = float(
                pose_stamped.pose.position.x
            )

            y = float(
                pose_stamped.pose.position.y
            )

            received_path.append(
                (x, y)
            )

        self.latest_path = received_path
        self.latest_path_time = self.get_clock().now()

    def _find_nearest_forward_index(
        self,
        path: List[XY],
    ) -> Optional[int]:
        """
        Find the nearest path point that lies in front of the robot.
        """

        nearest_index: Optional[int] = None
        nearest_distance = float("inf")

        for index, point in enumerate(path):
            x, y = point

            if x < self.minimum_forward_distance:
                continue

            distance = math.hypot(
                x,
                y,
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index

        return nearest_index

    def _find_lookahead_target(
        self,
        path: List[XY],
        nearest_index: int,
    ) -> Optional[XY]:
        """
        Find the first forward path point that reaches the lookahead distance.
        """

        for index in range(
            nearest_index,
            len(path),
        ):
            x, y = path[index]

            if x < self.minimum_forward_distance:
                continue

            distance = math.hypot(
                x,
                y,
            )

            if distance >= self.lookahead_distance:
                return (x, y)

        # If no point reaches the requested lookahead distance,
        # use the farthest available forward point.
        for index in range(
            len(path) - 1,
            nearest_index - 1,
            -1,
        ):
            x, y = path[index]

            if x >= self.minimum_forward_distance:
                return (x, y)

        return None

    def _calculate_alpha(
        self,
        target: XY,
    ) -> float:
        """
        Calculate the angle between the robot's forward direction
        and the selected target point.
        """

        target_x, target_y = target

        alpha = math.atan2(
            target_y,
            target_x,
        )

        return alpha

    def _calculate_curvature(
        self,
        target: XY,
        alpha: float,
    ) -> float:
        """
        Calculate the circular-path curvature required
        to reach the selected target.
        """

        target_x, target_y = target

        actual_lookahead_distance = math.hypot(
            target_x,
            target_y,
        )

        if actual_lookahead_distance < 1e-6:
            return 0.0

        curvature = (
            2.0
            * math.sin(alpha)
            / actual_lookahead_distance
        )

        return curvature

    def _calculate_steering_angle(
        self,
        curvature: float,
    ) -> float:
        """
        Convert path curvature into an Ackermann steering angle.
        """

        steering_angle = math.atan(
            self.wheelbase * curvature
        )

        steering_angle = max(
            -self.maximum_steering_angle,
            min(
                self.maximum_steering_angle,
                steering_angle,
            ),
        )

        return steering_angle

    def _calculate_speed(
        self,
        curvature: float,
    ) -> float:
        """
        Reduce the vehicle speed when the requested curve is sharp.
        """

        speed = self.target_speed / (
            1.0
            + self.curvature_speed_gain
            * abs(curvature)
        )

        speed = max(
            self.minimum_speed,
            min(
                self.maximum_speed,
                speed,
            ),
        )

        return speed

    def publish_stop(self) -> None:
        """
        Stop the vehicle safely.
        """

        stop_command = Twist()

        stop_command.linear.x = 0.0
        stop_command.angular.z = 0.0

        self.command_publisher.publish(
            stop_command
        )

    def _control_callback(self) -> None:
        """
        Execute one complete Pure Pursuit control cycle.
        """

        # Step 1: ensure a usable path exists.
        if len(self.latest_path) < self.minimum_path_points:
            self.publish_stop()
            return

        # Step 1b: ensure that path is still recent. Without this the node
        # would keep steering along the last path it ever received if the
        # centerline node stopped publishing.
        if self.latest_path_time is None:
            self.publish_stop()
            return

        path_age = (
            self.get_clock().now() - self.latest_path_time
        ).nanoseconds * 1e-9

        if path_age > self.path_timeout:
            self.publish_stop()
            self.get_logger().warn(
                f"no centerline for {path_age:.2f} s; stopping",
                throttle_duration_sec=1.0,
            )
            return

        # Step 2: find the nearest forward waypoint.
        nearest_index = self._find_nearest_forward_index(
            self.latest_path
        )

        if nearest_index is None:
            self.publish_stop()
            return

        # Step 3: select the lookahead goal point.
        target = self._find_lookahead_target(
            self.latest_path,
            nearest_index,
        )

        if target is None:
            self.publish_stop()
            return

        # Step 4: calculate the angle to the target.
        alpha = self._calculate_alpha(
            target
        )

        # Step 5: calculate required path curvature.
        curvature = self._calculate_curvature(
            target,
            alpha,
        )

        # Step 6: calculate steering.
        steering_angle = self._calculate_steering_angle(
            curvature
        )

        # Step 7: calculate forward speed.
        speed = self._calculate_speed(
            curvature
        )

        # Step 8: create and publish the command.
        command = Twist()

        command.linear.x = float(speed)
        command.angular.z = float(steering_angle)

        self.command_publisher.publish(
            command
        )

        self.get_logger().info(
            f"target=({target[0]:.2f}, {target[1]:.2f}), "
            f"alpha={alpha:.3f}, "
            f"curvature={curvature:.3f}, "
            f"steering={steering_angle:.3f}, "
            f"speed={speed:.3f}",
            throttle_duration_sec=1.0,
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
