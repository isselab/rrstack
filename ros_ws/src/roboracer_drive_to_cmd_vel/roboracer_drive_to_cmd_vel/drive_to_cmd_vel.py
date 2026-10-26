# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Simulation only: converts /drive (AckermannDriveStamped, read by the VESC on the car)
# -> /cmd_vel (Twist, read by Ignition Gazebo's AckermannSteering system).
# Controllers publish /drive (the real F1TENTH interface).
# Not started on the real car.

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveToCmdVel(Node):
    """Converts Ackermann drive commands into Twist for the Ignition Ackermann system."""

    def __init__(self):
        super().__init__('drive_to_cmd_vel')

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # Must match <wheel_base> of the AckermannSteering system in f110_car/model.sdf
        self.declare_parameter('wheelbase', 0.32)

        drive_topic = self.get_parameter('drive_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._wheelbase = self.get_parameter('wheelbase').value

        self._pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(AckermannDriveStamped, drive_topic, self._drive_cb, 10)

        self.get_logger().info(
            f'bridging {drive_topic} -> {cmd_vel_topic} (wheelbase={self._wheelbase} m)'
        )

    def _drive_cb(self, msg):
        speed = float(msg.drive.speed)
        steering = float(msg.drive.steering_angle)

        twist = Twist()
        # speed (m/s) -> forward velocity
        twist.linear.x = speed
        # Ignition's AckermannSteering reads angular.z as YAW RATE (rad/s), not
        # steering angle. Bicycle model: yaw_rate = speed * tan(steering) / wheelbase.
        # At speed 0 the yaw rate is 0 (an Ackermann car cannot turn on the spot).
        twist.angular.z = speed * math.tan(steering) / self._wheelbase
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()