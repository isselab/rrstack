#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Pure Pursuit on the Delaunay waypoints (PoseArray).
# Same algorithm and steps as roboracer_lidar_centerline/pure_pursuit_node.py
# (which stays untouched for the LiDAR pipeline); only the input type changes
# and the HOLD / EXPLORE / STOP recovery shared with the PID node is added.

import math
from typing import List, Optional, Tuple

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .recovery import EXPLORE, HOLD, STOP, WAIT, Recovery

XY = Tuple[float, float]


class WaypointPursuitNode(Node):
    """Follow the cone waypoints using the Pure Pursuit algorithm."""

    def __init__(self) -> None:
        super().__init__('waypoint_pursuit_node')

        self._declare_parameters()
        self._read_parameters()
        self.latest_path: List[XY] = []
        self.current_speed = 0.0
        self.latency: Optional[float] = None
        self.mode: Optional[str] = None
        self.prev_time = None

        self.recovery = Recovery(
            fresh_timeout=self.fresh_timeout,
            hold_timeout=self.hold_timeout,
            explore_speed=self.explore_speed,
            explore_step=self.explore_step,
            explore_pause=self.explore_pause,
            explore_max_distance=self.explore_max_distance,
            weave_amplitude=self.weave_amplitude,
            weave_period=self.weave_period,
            weave_bias=self.weave_bias,
        )

        self.waypoint_subscription = self.create_subscription(
            PoseArray, self.waypoint_topic, self._waypoint_callback, 5)
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 10)
        self.command_publisher = self.create_publisher(
            AckermannDriveStamped, self.command_topic, 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, 10)
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self._control_callback)

        self.get_logger().info(
            f'waypoint_pursuit_node: lookahead={self.lookahead_distance} m, '
            f'speed {self.minimum_speed}-{self.maximum_speed} m/s'
        )

    # ------------------------------------------------------------ parameters
    def _declare_parameters(self) -> None:
        parameter_defaults = {
            'waypoint_topic': '/waypoints',
            'odom_topic': '/odom',
            'command_topic': '/drive',
            'marker_topic': '/planning/controller_markers',

            # Same values as config/planning_params.yaml (tuned in Gazebo).
            'lookahead_distance': 1.8,    # > ~1.5 m camera blind zone
            'wheelbase': 0.32,

            'target_speed': 1.5,
            'minimum_speed': 0.5,
            'maximum_speed': 1.5,

            'maximum_steering_angle': 0.55,
            'curvature_speed_gain': 3.0,  # ~0.6 m/s in tight corners

            'minimum_forward_distance': 0.10,
            'minimum_path_points': 2,

            'control_frequency': 30.0,

            # recovery (R2 / R8)
            'fresh_timeout': 0.5,
            'hold_timeout': 1.0,
            'hold_speed': 0.3,
            'explore_speed': 0.3,
            'explore_step': 1.0,
            'explore_pause': 0.6,
            'explore_max_distance': 3.0,
            'weave_amplitude': 0.2,
            'weave_period': 2.0,
            'weave_bias': 0.1,
            'max_latency_warn': 1.0,
            'publish_markers': True,
        }
        for name, default in parameter_defaults.items():
            self.declare_parameter(name, default)

    def _read_parameters(self) -> None:
        g = self.get_parameter
        self.waypoint_topic = str(g('waypoint_topic').value)
        self.odom_topic = str(g('odom_topic').value)
        self.command_topic = str(g('command_topic').value)
        self.marker_topic = str(g('marker_topic').value)
        self.lookahead_distance = float(g('lookahead_distance').value)
        self.wheelbase = float(g('wheelbase').value)
        self.target_speed = float(g('target_speed').value)
        self.minimum_speed = float(g('minimum_speed').value)
        self.maximum_speed = float(g('maximum_speed').value)
        self.maximum_steering_angle = float(g('maximum_steering_angle').value)
        self.curvature_speed_gain = float(g('curvature_speed_gain').value)
        self.minimum_forward_distance = float(g('minimum_forward_distance').value)
        self.minimum_path_points = int(g('minimum_path_points').value)
        self.control_frequency = float(g('control_frequency').value)
        self.fresh_timeout = float(g('fresh_timeout').value)
        self.hold_timeout = float(g('hold_timeout').value)
        self.hold_speed = float(g('hold_speed').value)
        self.explore_speed = float(g('explore_speed').value)
        self.explore_step = float(g('explore_step').value)
        self.explore_pause = float(g('explore_pause').value)
        self.explore_max_distance = float(g('explore_max_distance').value)
        self.weave_amplitude = float(g('weave_amplitude').value)
        self.weave_period = float(g('weave_period').value)
        self.weave_bias = float(g('weave_bias').value)
        self.max_latency_warn = float(g('max_latency_warn').value)
        self.publish_markers = bool(g('publish_markers').value)

    # ------------------------------------------------------------ inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_callback(self, msg: Odometry) -> None:
        t = msg.twist.twist
        self.current_speed = math.hypot(t.linear.x, t.linear.y)

    def _waypoint_callback(self, msg: PoseArray) -> None:
        """Convert the PoseArray into a list of x-y points (one stamp for all)."""
        now = self._now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latency = now - stamp
        # Sim-time pitfall: if a node runs on wall time, this is huge/negative.
        if self.latency < -0.1 or self.latency > self.max_latency_warn:
            self.get_logger().warn(
                f'waypoint latency {self.latency:.2f} s looks wrong: '
                f'is use_sim_time set on every node?',
                throttle_duration_sec=2.0,
            )
        path = [(float(p.position.x), float(p.position.y)) for p in msg.poses]
        if len(path) >= self.minimum_path_points:
            self.latest_path = path
            idx = self._find_nearest_forward_index(path)
            tgt = self._find_lookahead_target(path, idx) if idx is not None else None
            self.recovery.mark_good(now, tgt[1] if tgt else 0.0)

    # ------------------------------------------------------------ algorithm
    def _find_nearest_forward_index(self, path: List[XY]) -> Optional[int]:
        """Find the nearest path point that lies in front of the robot."""
        nearest_index, nearest_distance = None, float('inf')
        for index, (x, y) in enumerate(path):
            if x < self.minimum_forward_distance:
                continue
            distance = math.hypot(x, y)
            if distance < nearest_distance:
                nearest_distance, nearest_index = distance, index
        return nearest_index

    def _find_lookahead_target(self, path: List[XY], nearest_index: int) -> Optional[XY]:
        """Find the first forward path point that reaches the lookahead distance."""
        for index in range(nearest_index, len(path)):
            x, y = path[index]
            if x < self.minimum_forward_distance:
                continue
            if math.hypot(x, y) >= self.lookahead_distance:
                return (x, y)
        # If no point reaches the lookahead distance, use the farthest forward point.
        for index in range(len(path) - 1, nearest_index - 1, -1):
            x, y = path[index]
            if x >= self.minimum_forward_distance:
                return (x, y)
        return None

    def _calculate_alpha(self, target: XY) -> float:
        """Angle between the robot's forward direction and the target."""
        return math.atan2(target[1], target[0])

    def _calculate_curvature(self, target: XY, alpha: float) -> float:
        """Circular-path curvature required to reach the target."""
        ld = math.hypot(target[0], target[1])
        if ld < 1e-6:
            return 0.0
        return 2.0 * math.sin(alpha) / ld

    def _calculate_steering_angle(self, curvature: float) -> float:
        """Convert path curvature into an Ackermann steering angle."""
        steering = math.atan(self.wheelbase * curvature)
        return max(-self.maximum_steering_angle, min(self.maximum_steering_angle, steering))

    def _calculate_speed(self, curvature: float) -> float:
        """Reduce the vehicle speed when the requested curve is sharp."""
        speed = self.target_speed / (1.0 + self.curvature_speed_gain * abs(curvature))
        return max(self.minimum_speed, min(self.maximum_speed, speed))

    # ------------------------------------------------------------ output
    def _make_drive(self, speed: float, steering: float) -> AckermannDriveStamped:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        return msg

    def publish_stop(self) -> None:
        self.command_publisher.publish(self._make_drive(0.0, 0.0))

    def _set_mode(self, mode: str) -> None:
        if mode != self.mode:
            if mode in (EXPLORE, STOP):
                self.get_logger().warn(
                    f'mode: {mode} (explored {self.recovery.explored:.1f} m)')
            else:
                self.get_logger().info(f'mode: {mode}')
            self.mode = mode

    def _dt(self) -> float:
        now = self._now()
        dt = 1.0 / self.control_frequency if self.prev_time is None else max(now - self.prev_time, 1e-3)
        self.prev_time = now
        return dt

    def _control_callback(self) -> None:
        """One Pure Pursuit control cycle, with recovery when waypoints are missing."""
        now = self._now()
        dt = self._dt()
        mode = self.recovery.update(now)
        self._set_mode(mode)

        target, alpha, curvature = None, 0.0, 0.0
        if mode in (WAIT, STOP):
            speed, steering = 0.0, 0.0
        elif mode == EXPLORE:
            speed, steering = self.recovery.explore_command(now, dt, self.current_speed)
            steering = max(-self.maximum_steering_angle,
                           min(self.maximum_steering_angle, steering))
        else:  # TRACK or HOLD
            # Steps 1-3: nearest forward waypoint, then the lookahead goal point.
            nearest_index = self._find_nearest_forward_index(self.latest_path)
            if nearest_index is not None:
                target = self._find_lookahead_target(self.latest_path, nearest_index)
            if target is None:
                speed, steering = 0.0, 0.0
            else:
                # Steps 4-7: alpha, curvature, steering, speed.
                alpha = self._calculate_alpha(target)
                curvature = self._calculate_curvature(target, alpha)
                steering = self._calculate_steering_angle(curvature)
                speed = self._calculate_speed(curvature)
                if mode == HOLD:
                    # R2: waypoints briefly missing -> keep going, but slowly
                    speed = min(speed, self.hold_speed)

        # Step 8: publish the command.
        self.command_publisher.publish(self._make_drive(speed, steering))
        self._publish_markers(target, alpha, curvature, steering, speed, mode)

    # ------------------------------------------------------------ debug
    def _marker(self, ns, mid, mtype, scale, color) -> Marker:
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(self, target, alpha, curvature, steering, speed, mode) -> None:
        if not self.publish_markers:
            return
        arr = MarkerArray()
        tgt = self._marker('target', 0, Marker.SPHERE, 0.18, (0.95, 0.55, 0.1, 1.0))
        if target is not None:
            tgt.pose.position.x, tgt.pose.position.y = float(target[0]), float(target[1])
        else:
            tgt.action = Marker.DELETE
        arr.markers.append(tgt)
        text = self._marker('readout', 1, Marker.TEXT_VIEW_FACING, 0.15, (1.0, 1.0, 1.0, 1.0))
        text.pose.position.z = 0.5
        lat = f'{self.latency:.2f}s' if self.latency is not None else 'n/a'
        text.text = (f'PP {mode}  alpha={math.degrees(alpha):+.1f}deg k={curvature:+.2f}\n'
                     f'steer={steering:+.2f} v={speed:.2f} latency={lat}')
        arr.markers.append(text)
        self.marker_publisher.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointPursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
