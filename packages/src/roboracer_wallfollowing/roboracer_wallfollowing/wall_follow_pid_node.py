# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# V-beam wall follower: three LiDAR beams (forward arm, perpendicular, rear
# arm) define a fitted wall line. PID on distance error, with RViz markers
# published for debugging.

import math

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class WallFollowPIDNode(Node):
    """Wall follower using a V of three LiDAR beams plus a PID controller."""

    def __init__(self):
        super().__init__('wall_follow_pid_node')

        self.declare_parameter('side', 'left')
        self.declare_parameter('theta_front_deg', 35.0)
        self.declare_parameter('theta_rear_deg', 35.0)
        self.declare_parameter('desired_distance', 0.6)
        self.declare_parameter('lidar_forward_offset', 0.37)
        self.declare_parameter('base_lookahead', 0.4)
        self.declare_parameter('lookahead_speed_gain', 0.2)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('max_steering_rate', 4.0)
        self.declare_parameter('max_d_term', 0.3)
        self.declare_parameter('nominal_speed', 1.2)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('recovery_speed', 0.3)
        self.declare_parameter('min_lidar_range', 0.25)
        self.declare_parameter('max_lidar_range', 10.0)
        self.declare_parameter('residual_threshold', 0.35)
        self.declare_parameter('control_period', 0.02)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_topic', '/wall_follow/markers')

        p = self.get_parameter
        self._side = p('side').value
        self._wall_sign = 1.0 if self._side == 'left' else -1.0
        self._theta_f = math.radians(p('theta_front_deg').value)
        self._theta_r = math.radians(p('theta_rear_deg').value)
        self._desired_distance = p('desired_distance').value
        self._lidar_offset = p('lidar_forward_offset').value
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
        self._recovery_speed = p('recovery_speed').value
        self._min_range = p('min_lidar_range').value
        self._max_range = p('max_lidar_range').value
        self._residual_threshold = p('residual_threshold').value
        self._control_period = p('control_period').value
        self._publish_markers = p('publish_markers').value

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._prev_steering = 0.0
        self._current_speed = 0.0
        self._latest_scan = None

        self._cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic').value, 10)
        self._marker_pub = self.create_publisher(MarkerArray, p('marker_topic').value, 10)
        self.create_subscription(LaserScan, p('scan_topic').value, self._scan_cb, 10)
        self.create_subscription(Odometry, p('odom_topic').value, self._odom_cb, 10)
        self.create_timer(self._control_period, self._control_tick)

        rear_deg = math.degrees(self._theta_r)
        if rear_deg > 44.0:
            self.get_logger().warn(
                f'theta_rear_deg={rear_deg:.1f} puts the rear arm outside the '
                f'+/-135 deg scan; clamp it below 45.'
            )

        self.get_logger().info(
            f'wall_follow_pid_node: side={self._side}, '
            f'desired={self._desired_distance} m, '
            f'arms={math.degrees(self._theta_f):.0f}/{rear_deg:.0f} deg'
        )

    def _odom_cb(self, msg):
        t = msg.twist.twist
        self._current_speed = math.hypot(t.linear.x, t.linear.y)

    def _scan_cb(self, msg):
        self._latest_scan = msg

    def _beam(self, scan, angle):
        """Return (range, x, y) for the beam nearest `angle`, or None."""
        idx = int(round((angle - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return None
        r = scan.ranges[idx]
        if math.isnan(r) or math.isinf(r):
            return None
        if r < self._min_range or r > self._max_range:
            return None
        return r, r * math.cos(angle), r * math.sin(angle)

    def _geometry(self, scan):
        """Fit a wall line through the two arm hits.

        Returns a dict with the beam hits, alpha, signed distances and the
        residual of the perpendicular beam against the fitted line.
        """
        ws = self._wall_sign
        ang_b = ws * (math.pi / 2.0)
        ang_f = ws * (math.pi / 2.0 - self._theta_f)
        ang_r = ws * (math.pi / 2.0 + self._theta_r)

        hit_b = self._beam(scan, ang_b)
        hit_f = self._beam(scan, ang_f)
        hit_r = self._beam(scan, ang_r)
        if hit_f is None or hit_r is None:
            return None

        fx, fy = hit_f[1], hit_f[2]
        rx, ry = hit_r[1], hit_r[2]
        dx, dy = fx - rx, fy - ry
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None

        alpha = math.atan2(dy, dx)
        nx, ny = -dy / norm, dx / norm
        d_signed = nx * fx + ny * fy

        residual = None
        if hit_b is not None:
            denom = ny * ws
            if abs(denom) > 1e-6:
                b_pred = d_signed / denom
                if b_pred > 0.0:
                    residual = abs(hit_b[0] - b_pred)

        return {
            'hit_b': hit_b, 'hit_f': hit_f, 'hit_r': hit_r,
            'alpha': alpha, 'd_signed': d_signed,
            'nx': nx, 'ny': ny, 'residual': residual,
        }

    def _pid(self, error, dt):
        self._integral += error * dt
        raw_d = (error - self._prev_error) / dt if dt > 0.0 else 0.0
        d_term = max(-self._max_d_term, min(self._max_d_term, self._kd * raw_d))
        self._prev_error = error
        return self._kp * error + self._ki * self._integral + d_term

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

    def _publish(self, speed, steering):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steering)
        self._cmd_pub.publish(msg)

    def _recover(self, dt, reason):
        """Beams unusable: steer away from the wall instead of freezing."""
        steering = -self._wall_sign * self._max_steering * 0.6
        steering = self._apply_rate_limit(steering, dt)
        self._publish(self._recovery_speed, steering)
        self.get_logger().warn(f'recovery: {reason}', throttle_duration_sec=1.0)

    def _control_tick(self):
        scan = self._latest_scan
        if scan is None:
            return
        dt = self._dt_seconds()

        geo = self._geometry(scan)
        if geo is None:
            self._recover(dt, 'arm beam invalid or too close')
            self._publish_debug_markers(scan, None, 0.0, 0.0)
            return

        alpha = geo['alpha']
        residual = geo['residual']
        if residual is not None and residual > self._residual_threshold:
            self._recover(dt, f'arms disagree (residual {residual:.2f} m)')
            self._publish_debug_markers(scan, geo, 0.0, 0.0)
            return

        ws = self._wall_sign
        d_base = geo['d_signed'] - self._lidar_offset * math.sin(alpha)
        lookahead = self._base_lookahead + self._lookahead_speed_gain * self._current_speed
        d_proj = d_base + lookahead * math.sin(alpha)
        dt_proj = ws * d_proj

        error = self._desired_distance - dt_proj
        steering = -ws * self._pid(error, dt)
        steering = max(-self._max_steering, min(self._max_steering, steering))
        steering = self._apply_rate_limit(steering, dt)

        ratio = min(abs(steering) / self._max_steering, 1.0) if self._max_steering > 0 else 0.0
        speed = self._nominal_speed - ratio * (self._nominal_speed - self._min_speed)
        self._publish(speed, steering)
        self._publish_debug_markers(scan, geo, dt_proj, steering)

    def _marker(self, scan, ns, mid, mtype, scale, color):
        m = Marker()
        m.header.frame_id = scan.header.frame_id
        m.header.stamp = scan.header.stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = scale
        m.scale.y = scale
        m.scale.z = scale
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        m.pose.orientation.w = 1.0
        return m

    def _publish_debug_markers(self, scan, geo, dt_proj, steering):
        if not self._publish_markers:
            return
        arr = MarkerArray()

        if geo is not None:
            beams = self._marker(scan, 'beams', 0, Marker.LINE_LIST, 0.015,
                                 (0.25, 0.75, 0.15, 0.9))
            hits = self._marker(scan, 'hits', 1, Marker.SPHERE_LIST, 0.06,
                                (0.1, 0.5, 0.05, 1.0))
            for key in ('hit_f', 'hit_b', 'hit_r'):
                hit = geo[key]
                if hit is None:
                    continue
                beams.points.append(Point(x=0.0, y=0.0, z=0.0))
                beams.points.append(Point(x=hit[1], y=hit[2], z=0.0))
                hits.points.append(Point(x=hit[1], y=hit[2], z=0.0))
            arr.markers.append(beams)
            arr.markers.append(hits)

            nx, ny = geo['nx'], geo['ny']
            d_signed = geo['d_signed']
            foot_x, foot_y = nx * d_signed, ny * d_signed
            dxu, dyu = -ny, nx
            line = self._marker(scan, 'fitted_wall', 2, Marker.LINE_STRIP, 0.02,
                                (0.85, 0.2, 0.2, 0.9))
            line.points.append(Point(x=foot_x - 2.0 * dxu, y=foot_y - 2.0 * dyu, z=0.0))
            line.points.append(Point(x=foot_x + 3.0 * dxu, y=foot_y + 3.0 * dyu, z=0.0))
            arr.markers.append(line)

            perp = self._marker(scan, 'dt', 3, Marker.LINE_LIST, 0.02,
                                (0.95, 0.55, 0.1, 1.0))
            perp.points.append(Point(x=0.0, y=0.0, z=0.0))
            perp.points.append(Point(x=foot_x, y=foot_y, z=0.0))
            arr.markers.append(perp)

            base = self._marker(scan, 'base_link', 4, Marker.SPHERE_LIST, 0.07,
                                (0.2, 0.4, 0.9, 1.0))
            base.points.append(Point(x=-self._lidar_offset, y=0.0, z=0.0))
            arr.markers.append(base)

        text = self._marker(scan, 'readout', 5, Marker.TEXT_VIEW_FACING, 0.12,
                            (1.0, 1.0, 1.0, 1.0))
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = 0.6
        if geo is None:
            text.text = 'RECOVERY: beams invalid'
        else:
            res = geo['residual']
            res_s = f'{res:.2f}' if res is not None else 'n/a'
            text.text = (
                f"side={self._side}  Dt={dt_proj:.2f}  "
                f"alpha={math.degrees(geo['alpha']):+.1f}deg\n"
                f"err={self._desired_distance - dt_proj:+.2f}  "
                f"steer={steering:+.2f}  resid={res_s}"
            )
        arr.markers.append(text)
        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()