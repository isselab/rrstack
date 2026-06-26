# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri, Thorsten Berger
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Part of the rrstack RoboRacer reinforcement learning package.

import argparse
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class RandomPolicy(Node):
    def __init__(self, args):
        super().__init__('roboracer_random_policy')
        self.args = args
        self.steering_values = [-0.6, -0.3, 0.0, 0.3, 0.6]
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.timer = self.create_timer(args.period, self._tick)

    def _tick(self):
        msg = Twist()
        msg.linear.x = self.args.fixed_speed
        msg.angular.z = random.choice(self.steering_values)
        self.cmd_pub.publish(msg)

    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(description='Publish random discrete steering commands.')
    parser.add_argument('--fixed-speed', type=float, default=1.0)
    parser.add_argument('--period', type=float, default=0.2)
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = RandomPolicy(args)
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
