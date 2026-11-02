# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Centreline PID follower on the Delaunay waypoints.
# Same structure as roboracer_wallfollowing/wall_follow_pid_node.py:
#   - lookahead target (here: point on the waypoint polyline, not a wall beam)
#   - PID on the angle to that target, D-term cap, steering-rate limit
#   (steering-lag compensation for Ignition lives in the sim-only
#    roboracer_drive_to_cmd_vel/steering_lag_compensator node)
#   - speed drops with steering
# plus the HOLD / EXPLORE / STOP recovery shared with the pursuit node.

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .path_utils import lookahead_point, path_length
from .recovery import EXPLORE, HOLD, STOP, WAIT, Recovery


class CenterlinePIDNode(Node):
    """PID on the angle to a lookahead point on the cone centreline."""

    def __init__(self):
        super().__init__('centerline_pid_node')

        self.declare_parameter('base_lookahead', 1.5)
        self.declare_parameter('lookahead_speed_gain', 0.3)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('max_steering_rate', 4.0)
        self.declare_parameter('max_d_term', 0.3)
        self.declare_parameter('nominal_speed', 1.0)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('min_path_length', 2.5)
        self.declare_parameter('hold_speed', 0.4)
        self.declare_parameter('min_waypoints', 2)
        self.declare_parameter('control_period', 0.02)
        self.declare_parameter('fresh_timeout', 0.5)
        self.declare_parameter('hold_timeout', 1.0)
        self.declare_parameter('explore_speed', 0.3)
        self.declare_parameter('explore_step', 1.0)
        self.declare_parameter('explore_pause', 0.6)
        self.declare_parameter('explore_max_distance', 3.0)
        self.declare_parameter('weave_amplitude', 0.2)
        self.declare_parameter('weave_period', 2.0)
        self.declare_parameter('weave_bias', 0.1)
        self.declare_parameter('max_latency_warn', 1.0)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('waypoint_topic', '/waypoints')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('marker_topic', '/planning/controller_markers')

        p = self.get_parameter
        self._base_lookahead = p('base_lookahead').value
        self._lookahead_speed_gain = p('lookahead_speed_gain').value
        self._kp = p('kp').value
        self._ki = p('ki').value
        self._kd = p('kd').value
        self._max_steering = p('max_steering_angle').value
        self._max_steering_rate = p('max_steering_rate').value
        self._max_d_term = p('max_d_term').value
        self._nominal_speed = p('nominal_speed').value
        self._min_speed = p('min_speed').value
        self._min_path_length = p('min_path_length').value
        self._hold_speed = p('hold_speed').value
        self._min_waypoints = p('min_waypoints').value
        self._control_period = p('control_period').value
        self._max_latency_warn = p('max_latency_warn').value
        self._publish_markers = p('publish_markers').value

        self._recovery = Recovery(
            fresh_timeout=p('fresh_timeout').value,
            hold_timeout=p('hold_timeout').value,
            explore_speed=p('explore_speed').value,
            explore_step=p('explore_step').value,
            explore_pause=p('explore_pause').value,
            explore_max_distance=p('explore_max_distance').value,
            weave_amplitude=p('weave_amplitude').value,
            weave_period=p('weave_period').value,
            weave_bias=p('weave_bias').value,
        )

        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None
        self._prev_steering = 0.0
        self._current_speed = 0.0
        self._path = []
        self._latency = None
        self._mode = None

        self._cmd_pub = self.create_publisher(AckermannDriveStamped, p('drive_topic').value, 10)
        self._marker_pub = self.create_publisher(MarkerArray, p('marker_topic').value, 10)
        self.create_subscription(PoseArray, p('waypoint_topic').value, self._waypoint_cb, 5)
        self.create_subscription(Odometry, p('odom_topic').value, self._odom_cb, 10)
        self.create_timer(self._control_period, self._control_tick)

        self.get_logger().info(
            f'centerline_pid_node: kp={self._kp} kd={self._kd} '
            f'lookahead={self._base_lookahead}+{self._lookahead_speed_gain}*v, '
            f'speed {self._min_speed}-{self._nominal_speed} m/s'
        )

    # ------------------------------------------------------------ inputs
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg):
        t = msg.twist.twist
        self._current_speed = math.hypot(t.linear.x, t.linear.y)

    def _waypoint_cb(self, msg):
        now = self._now()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latency = now - stamp
        # Sim-time pitfall: if a node runs on wall time, this is huge/negative.
        if self._latency < -0.1 or self._latency > self._max_latency_warn:
            self.get_logger().warn(
                f'waypoint latency {self._latency:.2f} s looks wrong: '
                f'is use_sim_time set on every node?',
                throttle_duration_sec=2.0,
            )
        pts = [(ps.position.x, ps.position.y) for ps in msg.poses]
        if len(pts) >= self._min_waypoints:
            self._path = pts
            target = lookahead_point(pts, self._lookahead())
            self._recovery.mark_good(now, target[1] if target else 0.0)

    # ------------------------------------------------------------ control
    def _lookahead(self):
        return self._base_lookahead + self._lookahead_speed_gain * self._current_speed

    def _pid(self, error, dt):
        if self._prev_error is None:
            self._prev_error = error
        self._integral += error * dt
        raw_d = (error - self._prev_error) / dt if dt > 0.0 else 0.0
        d_term = max(-self._max_d_term, min(self._max_d_term, self._kd * raw_d))
        self._prev_error = error
        return self._kp * error + self._ki * self._integral + d_term

    def _reset_pid(self):
        self._integral = 0.0
        self._prev_error = None

    def _dt_seconds(self):
        now = self.get_clock().now()
        if self._prev_time is None:
            dt = self._control_period
        else:
            dt = max((now - self._prev_time).nanoseconds * 1e-9, 1e-3)
        self._prev_time = now
        return dt

    def _apply_rate_limit(self, steering, dt):
        max_step = self._max_steering_rate * dt
        delta = steering - self._prev_steering
        if delta > max_step:
            steering = self._prev_steering + max_step
        elif delta < -max_step:
            steering = self._prev_steering - max_step
        self._prev_steering = steering
        return steering

    def _speed(self, steering, mode):
        ratio = min(abs(steering) / self._max_steering, 1.0) if self._max_steering > 0 else 0.0
        speed = self._nominal_speed - ratio * (self._nominal_speed - self._min_speed)
        # R5 - Slow updates (~4 Hz)
        # Problem: between updates the car drives on old waypoints.
        # Fix:     when the visible path is short (few waypoints / sharp bend),
        #          drive at min_speed so the next update arrives in time.
        if path_length(self._path) < self._min_path_length:
            speed = self._min_speed
        if mode == HOLD:
            speed = min(speed, self._hold_speed)
        return speed

    def _publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self._cmd_pub.publish(msg)

    def _set_mode(self, mode):
        if mode != self._mode:
            if mode in (EXPLORE, STOP):
                self.get_logger().warn(
                    f'mode: {mode} (explored {self._recovery.explored:.1f} m)')
            else:
                self.get_logger().info(f'mode: {mode}')
            self._mode = mode

    def _control_tick(self):
        now = self._now()
        dt = self._dt_seconds()
        mode = self._recovery.update(now)
        self._set_mode(mode)

        target, error, steering = None, 0.0, 0.0
        if mode in (WAIT, STOP):
            self._reset_pid()
            speed, steering = 0.0, 0.0
        elif mode == EXPLORE:
            self._reset_pid()
            speed, steering = self._recovery.explore_command(now, dt, self._current_speed)
            steering = max(-self._max_steering, min(self._max_steering, steering))
            steering = self._apply_rate_limit(steering, dt)
        else:  # TRACK or HOLD
            target = lookahead_point(self._path, self._lookahead())
            if target is None:
                speed, steering = 0.0, 0.0
            else:
                # error = angle from base_link (rear axle) to the target
                error = math.atan2(target[1], target[0])
                steering = self._pid(error, dt)
                steering = max(-self._max_steering, min(self._max_steering, steering))
                steering = self._apply_rate_limit(steering, dt)
                speed = self._speed(steering, mode)

        self._publish(speed, steering)
        self._publish_debug_markers(target, error, steering, speed, mode)

    # ------------------------------------------------------------ debug
    def _marker(self, ns, mid, mtype, scale, color):
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

    def _publish_debug_markers(self, target, error, steering, speed, mode):
        if not self._publish_markers:
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
        lat = f'{self._latency:.2f}s' if self._latency is not None else 'n/a'
        text.text = (f'PID {mode}  err={math.degrees(error):+.1f}deg\n'
                     f'steer={steering:+.2f} v={speed:.2f} latency={lat}')
        arr.markers.append(text)
        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CenterlinePIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
