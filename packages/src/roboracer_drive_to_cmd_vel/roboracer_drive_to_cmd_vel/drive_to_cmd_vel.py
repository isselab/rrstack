# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Simulation only: converts /drive (AckermannDriveStamped) : used by the VESC on the car -> /cmd_vel (Twist) used in the simulation.
# Controllers publish /drive (the real F1TENTH interface, read by the VESC on the car).
# Gazebo's Ackermann plugin only reads /cmd_vel, so this node translates.
# Not started on the real car.

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveToCmdVel(Node):
    """Converts Ackermann drive commands into Twist for the Gazebo Ackermann plugin."""

    def __init__(self):
        super().__init__('drive_to_cmd_vel')

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        drive_topic = self.get_parameter('drive_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self._pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(AckermannDriveStamped, drive_topic, self._drive_cb, 10)

        self.get_logger().info(f'bridging {drive_topic} -> {cmd_vel_topic}')

    def _drive_cb(self, msg):
        twist = Twist()
        # speed (m/s) -> forward velocity
        twist.linear.x = float(msg.drive.speed)
        # steering angle (rad) -> angular.z; the Gazebo Ackermann plugin reads
        # angular.z as the steering angle (same convention as the wall follower).
        twist.angular.z = float(msg.drive.steering_angle)
        self._pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DriveToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()