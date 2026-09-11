# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Delaunay triangulation on detected cone positions -> track waypoints.

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node
from scipy.spatial import Delaunay


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
        self._frame_id = 'camera_color_optical_frame'

        self.create_subscription(PoseArray, blue_topic, self._blue_cb, 5)
        self.create_subscription(PoseArray, yellow_topic, self._yellow_cb, 5)
        self._wp_pub = self.create_publisher(PoseArray, waypoint_topic, 5)

        self.get_logger().info(
            f'listening on {blue_topic} + {yellow_topic}, '
            f'publishing waypoints on {waypoint_topic}'
        )

    def _blue_cb(self, msg):
        self._frame_id = msg.header.frame_id
        self._blue = [(p.position.x, p.position.y) for p in msg.poses]

    def _yellow_cb(self, msg):
        self._yellow = [(p.position.x, p.position.y) for p in msg.poses]
        self._triangulation_process()()

    def _triangulation_process(self):
        if len(self._blue) < 2 or len(self._yellow) < 2:
            return

        pts = np.array(self._blue + self._yellow)
        n_blue = len(self._blue)

        try:
            tri = Delaunay(pts)
        except Exception as exc:
            self.get_logger().warn(f'triangulation failed: {exc}')
            return

        midpoints = set()
        for simplex in tri.simplices:
            for i in range(3):
                a = simplex[i]
                b = simplex[(i + 1) % 3]
                # keep only edges that cross the track (blue <-> yellow)
                if (a < n_blue) != (b < n_blue):
                    mx = (pts[a][0] + pts[b][0]) / 2.0
                    my = (pts[a][1] + pts[b][1]) / 2.0
                    midpoints.add((round(mx, 3), round(my, 3)))

        waypoints = sorted(midpoints, key=lambda p: p[0])

        msg = PoseArray()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in waypoints:
            p = Pose()
            p.position.x = wx
            p.position.y = wy
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)

        self._wp_pub.publish(msg)


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