# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Delaunay waypoint planner with a colour constraint.
#   1. Take the blue (left) and yellow (right) cones from yolo_node.
#   2. Triangulate all cones (Delaunay).
#   3. Keep only blue-yellow edges of a plausible track width.
#   4. The midpoints of those edges, ordered forward, are the waypoints.
# Output: /waypoints (PoseArray, base_link, original camera stamp) and RViz
# markers showing cones, kept / rejected edges, midpoints and the path.

import math
import warnings

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

with warnings.catch_warnings():
    # system scipy 1.8 warns about the newer pip numpy; it works fine
    warnings.simplefilter('ignore')
    import numpy as np
    from scipy.spatial import Delaunay

from .path_utils import dist, order_chain, unit

BLUE, YELLOW = 0, 1


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class DelaunayNode(Node):
    """Builds ordered track waypoints from blue/yellow cone positions."""

    def __init__(self):
        super().__init__('delaunay_node')

        self.declare_parameter('blue_topic', '/cones/blue')
        self.declare_parameter('yellow_topic', '/cones/yellow')
        self.declare_parameter('waypoint_topic', '/waypoints')
        self.declare_parameter('marker_topic', '/planning/markers')
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('camera_forward_offset', 0.37)
        self.declare_parameter('max_range', 6.5)
        self.declare_parameter('min_edge', 0.8)
        self.declare_parameter('max_edge', 2.2)
        self.declare_parameter('half_track_width', 0.7)
        self.declare_parameter('merge_radius', 0.5)
        self.declare_parameter('cone_link', 1.5)
        self.declare_parameter('waypoint_link', 1.6)
        self.declare_parameter('max_gap', 3.0)
        self.declare_parameter('pair_slop', 0.05)
        self.declare_parameter('publish_markers', True)

        p = self.get_parameter
        self._frame = p('output_frame').value
        self._cam_offset = p('camera_forward_offset').value
        self._max_range = p('max_range').value
        self._min_edge = p('min_edge').value
        self._max_edge = p('max_edge').value
        self._half_width = p('half_track_width').value
        self._merge_radius = p('merge_radius').value
        self._cone_link = p('cone_link').value
        self._wp_link = p('waypoint_link').value
        self._max_gap = p('max_gap').value
        self._pair_slop = p('pair_slop').value
        self._publish_markers = p('publish_markers').value

        self._blue_msg = None
        self._yellow_msg = None

        self.create_subscription(PoseArray, p('blue_topic').value, self._blue_cb, 5)
        self.create_subscription(PoseArray, p('yellow_topic').value, self._yellow_cb, 5)
        self._wp_pub = self.create_publisher(PoseArray, p('waypoint_topic').value, 5)
        self._marker_pub = self.create_publisher(MarkerArray, p('marker_topic').value, 5)

        self.get_logger().info(
            f'delaunay_node: edges kept if {self._min_edge}-{self._max_edge} m, '
            f'range <= {self._max_range} m, camera offset {self._cam_offset} m, '
            f'output frame {self._frame}'
        )

    # ------------------------------------------------------------ inputs
    # yolo_node publishes blue and yellow for every frame with the same
    # header, so we wait until both halves of one frame have arrived.
    def _blue_cb(self, msg):
        self._blue_msg = msg
        self._try_process()

    def _yellow_cb(self, msg):
        self._yellow_msg = msg
        self._try_process()

    def _try_process(self):
        if self._blue_msg is None or self._yellow_msg is None:
            return
        tb = stamp_sec(self._blue_msg.header.stamp)
        ty = stamp_sec(self._yellow_msg.header.stamp)
        if abs(tb - ty) > self._pair_slop:
            return  # halves of different frames: wait for the matching one
        stamp = self._blue_msg.header.stamp
        blue = self._to_base(self._blue_msg)
        yellow = self._to_base(self._yellow_msg)
        self._blue_msg = None
        self._yellow_msg = None
        self._plan(blue, yellow, stamp)

    def _to_base(self, msg):
        """Camera-level points -> base_link, dropping cones we can't trust.

        R10 - Far cones overlap in the image
          Problem: far away, cones are a few pixels wide; boxes merge and the
                   depth lands on the wrong cone, so far positions are wrong.
          Fix:     ignore everything beyond max_range (and behind the car).
        """
        pts = []
        for pose in msg.poses:
            x = pose.position.x + self._cam_offset
            y = pose.position.y
            if x <= 0.0 or math.hypot(x, y) > self._max_range:
                continue
            pts.append((x, y))
        return pts

    # ------------------------------------------------------------ planning
    def _plan(self, blue, yellow, stamp):
        kept, rejected = self._triangulate(blue, yellow)
        midpoints = [((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0) for a, b in kept]

        paired = set()
        for a, b in kept:
            paired.add(a)
            paired.add(b)
        fallback = self._single_colour_fallback(blue, yellow, paired, midpoints)

        waypoints, gaps = order_chain(
            midpoints + fallback, self._wp_link, self._max_gap,
            max_first=self._max_range,
        )

        if midpoints and fallback:
            mode = 'pairs+fallback'
        elif midpoints:
            mode = 'pairs'
        elif fallback:
            mode = 'blue only' if blue else 'yellow only'
        else:
            mode = 'no waypoints'

        self._publish_waypoints(waypoints, stamp)
        if self._publish_markers:
            self._publish_debug(blue, yellow, kept, rejected, midpoints,
                                fallback, waypoints, gaps, mode, stamp)

    def _triangulate(self, blue, yellow):
        """Delaunay over all cones; keep blue-yellow edges of track width.

        R7 - Wrong pairing with uneven spacing
          Problem: pairing each cone with its nearest neighbour crosses the
                   track diagonally or pairs two cones of the same colour.
          Fix:     Delaunay triangles only connect natural neighbours, and we
                   keep only edges that join a blue cone to a yellow cone.

        R4 - False or wrong-colour cones
          Problem: a ghost cone or a wrong colour creates a midpoint that is
                   outside the track.
          Fix:     a kept edge must be as long as a track is wide
                   (min_edge .. max_edge); anything else is rejected.
        """
        pts = blue + yellow
        labels = [BLUE] * len(blue) + [YELLOW] * len(yellow)
        if not blue or not yellow:
            return [], []

        edges = set()
        if len(pts) >= 3:
            try:
                tri = Delaunay(np.array(pts))
                for s in tri.simplices:
                    for i, j in ((s[0], s[1]), (s[1], s[2]), (s[0], s[2])):
                        edges.add((min(i, j), max(i, j)))
            except Exception:
                # all cones on one line (degenerate): fall back to all pairs
                edges = {(i, j) for i in range(len(pts)) for j in range(i + 1, len(pts))}
        else:
            edges = {(0, 1)}

        kept, rejected = [], []
        for i, j in edges:
            a, b = pts[i], pts[j]
            if labels[i] == labels[j]:
                rejected.append((a, b))
                continue
            length = dist(a, b)
            if self._min_edge <= length <= self._max_edge:
                kept.append((a, b) if labels[i] == BLUE else (b, a))
            else:
                rejected.append((a, b))
        return kept, rejected

    def _single_colour_fallback(self, blue, yellow, paired, midpoints):
        """Guess centre points from cones that have no partner.

        R1 - One colour not visible
          Problem: in tight corners the narrow camera may see only blue or
                   only yellow cones, so there are no pairs at all.
          Fix:     shift each unpaired cone sideways by half the track width,
                   towards the inside of the track (blue = left boundary, so
                   shift right; yellow = right boundary, so shift left).

        R8 - Missing cone or group of cones
          Problem: a gap in one boundary leaves the cones opposite the gap
                   without partners, so no midpoints along that stretch.
          Fix:     the same shift fills the gap from the boundary we can see.
        """
        out = []
        for cones, side in ((blue, -1.0), (yellow, 1.0)):
            chain, _ = order_chain(cones, self._cone_link, self._max_gap,
                                   max_first=self._max_range)
            for k, c in enumerate(chain):
                if c in paired:
                    continue
                # local boundary direction from the neighbouring cones of the
                # same boundary only (never from the car's own position, which
                # is not on the boundary and would tilt the shift)
                prev_c = chain[k - 1] if k > 0 else None
                next_c = chain[k + 1] if k + 1 < len(chain) else None
                if prev_c is not None and next_c is not None:
                    tx, ty = unit(next_c[0] - prev_c[0], next_c[1] - prev_c[1])
                elif next_c is not None:
                    tx, ty = unit(next_c[0] - c[0], next_c[1] - c[1])
                elif prev_c is not None:
                    tx, ty = unit(c[0] - prev_c[0], c[1] - prev_c[1])
                else:
                    tx, ty = 1.0, 0.0  # single cone: assume the track runs ahead
                # normal pointing into the track: right of blue, left of yellow
                nx, ny = side * -ty, side * tx
                g = (c[0] + self._half_width * nx, c[1] + self._half_width * ny)
                if all(dist(g, m) > self._merge_radius for m in midpoints + out):
                    out.append(g)
        return out

    # ------------------------------------------------------------ outputs
    def _publish_waypoints(self, waypoints, stamp):
        # One header for the whole set: it is the camera stamp of the frame
        # the cones came from, so controllers can measure end-to-end latency.
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame
        for k, (x, y) in enumerate(waypoints):
            nxt = waypoints[k + 1] if k + 1 < len(waypoints) else None
            prv = waypoints[k - 1] if k > 0 else (0.0, 0.0)
            ref = (nxt[0] - x, nxt[1] - y) if nxt else (x - prv[0], y - prv[1])
            yaw = math.atan2(ref[1], ref[0])
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.z = math.sin(yaw / 2.0)
            pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)
        self._wp_pub.publish(msg)

    def _marker(self, stamp, ns, mid, mtype, scale, color):
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        m.pose.orientation.w = 1.0
        return m

    def _points_marker(self, stamp, ns, mid, mtype, scale, color, points):
        m = self._marker(stamp, ns, mid, mtype, scale, color)
        for x, y in points:
            m.points.append(Point(x=float(x), y=float(y), z=0.0))
        if not m.points:
            m.action = Marker.DELETE  # RViz complains about empty lists
        return m

    def _publish_debug(self, blue, yellow, kept, rejected, midpoints,
                       fallback, waypoints, gaps, mode, stamp):
        arr = MarkerArray()
        S = Marker.SPHERE_LIST
        arr.markers.append(self._points_marker(
            stamp, 'cones_blue', 0, S, 0.15, (0.1, 0.3, 1.0, 1.0), blue))
        arr.markers.append(self._points_marker(
            stamp, 'cones_yellow', 1, S, 0.15, (1.0, 0.85, 0.0, 1.0), yellow))
        arr.markers.append(self._points_marker(
            stamp, 'edges_kept', 2, Marker.LINE_LIST, 0.02, (0.1, 0.9, 0.2, 1.0),
            [p for e in kept for p in e]))
        arr.markers.append(self._points_marker(
            stamp, 'edges_rejected', 3, Marker.LINE_LIST, 0.01, (0.6, 0.6, 0.6, 0.35),
            [p for e in rejected for p in e]))
        arr.markers.append(self._points_marker(
            stamp, 'midpoints', 4, S, 0.08, (1.0, 1.0, 1.0, 1.0), midpoints))
        arr.markers.append(self._points_marker(
            stamp, 'fallback', 5, S, 0.08, (1.0, 0.5, 0.0, 1.0), fallback))
        arr.markers.append(self._points_marker(
            stamp, 'path', 6, Marker.LINE_STRIP, 0.03, (0.9, 0.1, 0.9, 1.0),
            [(0.0, 0.0)] + waypoints if waypoints else []))

        text = self._marker(stamp, 'readout', 7, Marker.TEXT_VIEW_FACING, 0.15,
                            (1.0, 1.0, 1.0, 1.0))
        text.pose.position.x = 0.0
        text.pose.position.z = 0.8
        text.text = (f'{mode}  blue={len(blue)} yellow={len(yellow)}\n'
                     f'edges={len(kept)} fallback={len(fallback)} '
                     f'waypoints={len(waypoints)} gaps={gaps}')
        arr.markers.append(text)
        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DelaunayNode()
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
