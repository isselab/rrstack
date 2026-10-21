# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Delaunay waypoint node -- STUB.
# Interface only: subscribes to cone topics and owns the waypoint publisher,
# but does not compute or publish waypoints yet.


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray 



class DelaunayNode(Node):
    """Builds waypoints from blue/yellow cone positions."""

    def __init__(self):
        super().__init__('delaunay_node')

        self.declare_parameter('blue_topic', '/cones/blue')
        self.declare_parameter('yellow_topic', '/cones/yellow')
        self.declare_parameter('waypoint_topic', '/waypoints')

        blue_topic = self.get_parameter('blue_topic').value
        yellow_topic = self.get_parameter('yellow_topic').value
        waypoint_topic = self.get_parameter('waypoint_topic').value

        self._blue = []
        self._yellow = []
        self.create_subscription(PoseArray, blue_topic, self._blue_cb, 5)
        self.create_subscription(PoseArray, yellow_topic, self._yellow_cb, 5)
        self._wp_pub = self.create_publisher(PoseArray, waypoint_topic, 5)

        self.get_logger().warn(
            f'delaunay_node is a STUB: listening on {blue_topic} + '
            f'{yellow_topic}, nothing is published on {waypoint_topic}'
        )

    def _blue_cb(self, msg):
        self._frame_id = msg.header.frame_id
        self._blue = [(p.position.x, p.position.y) for p in msg.poses]

    def _yellow_cb(self, msg):
        self._yellow = [(p.position.x, p.position.y) for p in msg.poses]
        self._triangulation_process()

    def _triangulation_process(self):
        if len(self._blue) < 2 or len(self._yellow) < 2:
            return


def main(args=None):
    rclpy.init(args=args)
    node = DelaunayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()