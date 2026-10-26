# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri, Thorsten Berger
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Part of the rrstack RoboRacer reinforcement learning package.

import argparse
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PolicyNode(Node):
    def __init__(self, args):
        super().__init__('roboracer_rl_policy')
        self.args = args
        self.latest_scan = None
        self.latest_odom = None
        self.last_steering = 0.0
        self.max_lidar_range = 30.0
        self.steering_values = [-0.6, -0.3, 0.0, 0.3, 0.6]

        try:
            from stable_baselines3 import DQN, PPO
        except ImportError as exc:
            raise RuntimeError(
                'Policy inference requires Stable-Baselines3. Install dependencies with: '
                'pip install gymnasium stable-baselines3'
            ) from exc

        loader = PPO if args.algorithm == 'ppo' else DQN
        self.model = loader.load(args.model)

        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.create_subscription(LaserScan, args.scan_topic, self._scan_callback, 10)
        self.create_subscription(Odometry, args.odom_topic, self._odom_callback, 10)
        self.timer = self.create_timer(args.period, self._tick)

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _sectorize_scan(self):
        ranges = np.asarray(self.latest_scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges,
            nan=self.max_lidar_range,
            posinf=self.max_lidar_range,
            neginf=0.0,
        )
        ranges = np.clip(ranges, 0.0, self.max_lidar_range)
        sectors = np.array_split(ranges, self.args.lidar_bins)
        mins = np.asarray([sector.min(initial=self.max_lidar_range) for sector in sectors])
        return (mins / self.max_lidar_range).astype(np.float32)

    def _speed(self):
        twist = self.latest_odom.twist.twist
        return math.hypot(twist.linear.x, twist.linear.y)

    def _observation(self):
        lidar = self._sectorize_scan()
        speed = np.clip(self._speed() / 10.0, 0.0, 1.0)
        steering = np.clip((self.last_steering + 0.6) / 1.2, 0.0, 1.0)
        return np.concatenate([lidar, [speed, steering]]).astype(np.float32)

    def _publish_cmd(self, speed, steering):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steering)
        self.cmd_pub.publish(msg)

    def _tick(self):
        if self.latest_scan is None or self.latest_odom is None:
            return

        observation = self._observation()
        action, _ = self.model.predict(observation, deterministic=True)
        self.last_steering = self.steering_values[int(action)]
        self._publish_cmd(self.args.fixed_speed, self.last_steering)

    def stop(self):
        self._publish_cmd(0.0, 0.0)
        time.sleep(0.05)


def parse_args():
    parser = argparse.ArgumentParser(description='Run a trained RL policy on /cmd_vel.')
    parser.add_argument('--model', required=True)
    parser.add_argument('--algorithm', choices=['dqn', 'ppo'], default='dqn')
    parser.add_argument('--lidar-bins', type=int, default=36)
    parser.add_argument('--fixed-speed', type=float, default=1.5)
    parser.add_argument('--period', type=float, default=0.1)
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = PolicyNode(args)
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
