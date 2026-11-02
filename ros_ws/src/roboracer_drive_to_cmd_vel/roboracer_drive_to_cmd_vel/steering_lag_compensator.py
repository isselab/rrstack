# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# SIMULATION ONLY. Compensates the steering lag of Ignition Fortress so the
# controllers can stay identical to the real car.
#
#   controller --/drive--> steering_lag_compensator --/drive_sim--> drive_to_cmd_vel
#                                   ^ /joint_states (measured steering-hinge angles)
#
# Problem: Fortress' AckermannSteering moves the steering hinges towards the
#          commanded angle as a first-order lag with tau ~1.05 s (measured with
#          step tests: 63 % after ~1.0 s, independent of speed and step size).
#          `steer_p_gain` does not exist in Fortress, so it cannot be tuned in
#          the SDF. A real servo reacts in ~0.1-0.2 s.
# Fix:     command past the goal while the wheels are still far from it,
#          using the MEASURED wheel angle (closed loop):
#              cmd = delta + boost * (wanted - delta)
#          With a first-order lag this behaves like tau / boost. The extra
#          command shrinks to zero as the wheels arrive, so they settle at the
#          wanted angle. With enabled: auto it only acts in simulation
#          (use_sim_time true); on the real car it is a passthrough.

from collections import deque

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SteeringLagCompensator(Node):

    def __init__(self):
        super().__init__('steering_lag_compensator')

        self.declare_parameter('input_topic', '/drive')
        self.declare_parameter('output_topic', '/drive_sim')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_joint', 'left_steering_hinge')
        self.declare_parameter('right_joint', 'right_steering_hinge')
        # 'auto' (default): compensate only in simulation, i.e. when the node
        # runs on Gazebo's clock (use_sim_time: true, set by every sim launch).
        # On the real car use_sim_time is false -> plain passthrough, so the
        # same launch line works everywhere. 'true' / 'false' force it.
        self.declare_parameter('enabled', 'auto',
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('tau', 1.05)            # measured plant lag [s] (reference / logs)
        self.declare_parameter('boost', 3.0)           # tau_effective = tau / boost
        self.declare_parameter('steering_limit', 0.6458)  # <steering_limit> in f110_car/model.sdf
        self.declare_parameter('joint_states_timeout', 0.2)
        self.declare_parameter('estimate_window', 6.0)  # seconds of data for the live tau fit
        self.declare_parameter('log_period', 10.0)

        p = self.get_parameter
        self._left = p('left_joint').value
        self._right = p('right_joint').value
        self._enabled, why = self._resolve_enabled(p('enabled').value)
        self._tau = p('tau').value
        self._boost = p('boost').value
        self._limit = p('steering_limit').value
        self._js_timeout = p('joint_states_timeout').value
        self._window = p('estimate_window').value

        self._delta = None          # measured wheel angle (mean of both hinges)
        self._delta_time = None
        self._samples = deque()     # (t, delta, wanted) for the live lag estimate

        self._pub = self.create_publisher(AckermannDriveStamped, p('output_topic').value, 10)
        self.create_subscription(AckermannDriveStamped, p('input_topic').value, self._drive_cb, 10)
        self.create_subscription(JointState, p('joint_states_topic').value, self._js_cb, 50)
        self.create_timer(p('log_period').value, self._log_estimate)

        mode = (f'boost {self._boost} (tau {self._tau} s -> ~{self._tau / self._boost:.2f} s)'
                if self._enabled else 'passthrough')
        self.get_logger().info(
            f'steering_lag_compensator: {p("input_topic").value} -> '
            f'{p("output_topic").value}, {mode} [{why}]')

    def _resolve_enabled(self, value):
        """Decide whether to compensate: forced on/off, or 'auto' = simulation only."""
        text = str(value).strip().lower()
        if text in ('true', '1', 'yes'):
            return True, 'forced on'
        if text in ('false', '0', 'no'):
            return False, 'forced off'
        sim = bool(self.get_parameter('use_sim_time').value)
        return sim, ('auto: simulation detected (use_sim_time)' if sim
                     else 'auto: no simulation clock -> real car, passthrough')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _js_cb(self, msg):
        try:
            left = msg.position[msg.name.index(self._left)]
            right = msg.position[msg.name.index(self._right)]
        except (ValueError, IndexError):
            return
        # Ackermann: the inner wheel steers more than the outer one; their
        # mean matches the bicycle-model angle the controllers command.
        self._delta = 0.5 * (left + right)
        self._delta_time = self._now()

    def _drive_cb(self, msg):
        wanted = msg.drive.steering_angle
        now = self._now()
        fresh = (self._delta is not None
                 and now - self._delta_time <= self._js_timeout)

        if not self._enabled:
            cmd = wanted
        elif not fresh:
            # Never boost blind: without a measured angle, pass through.
            cmd = wanted
            self.get_logger().warn('no recent /joint_states: passing steering through',
                                   throttle_duration_sec=2.0)
        else:
            cmd = self._delta + self._boost * (wanted - self._delta)
            cmd = max(-self._limit, min(self._limit, cmd))

        out = AckermannDriveStamped()
        out.header = msg.header
        out.drive = msg.drive
        out.drive.steering_angle = float(cmd)
        self._pub.publish(out)

        if fresh:
            self._samples.append((now, self._delta, wanted))
            while self._samples and now - self._samples[0][0] > self._window:
                self._samples.popleft()

    def _log_estimate(self):
        """Live check from /joint_states: how fast do the wheels follow the controller?

        Fits d(delta)/dt = (wanted - delta) / tau_eff by least squares over the
        last `estimate_window` seconds, where `wanted` is the controller's
        steering. Passthrough: tau_eff is the plant lag (~1.05 s).
        Compensated: it should be near tau / boost. A large mismatch means the
        compensation is mis-set or the measured angle arrives late.
        """
        s = list(self._samples)
        num = den = 0.0
        n = 0
        for (t0, d0, c0), (t1, d1, _) in zip(s, s[1:]):
            dt = t1 - t0
            err = c0 - d0   # wanted - measured
            if dt <= 1e-4 or abs(err) < 0.02:
                continue  # skip tiny errors: rate would be mostly noise
            num += err * (d1 - d0) / dt
            den += err * err
            n += 1
        if n < 20 or num <= 0.0:
            return
        tau_eff = den / num
        expected = self._tau / self._boost if self._enabled else self._tau
        self.get_logger().info(
            f'effective steering lag {tau_eff:.2f} s from {n} samples '
            f'(expected ~{expected:.2f} s)')


def main(args=None):
    rclpy.init(args=args)
    node = SteeringLagCompensator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
