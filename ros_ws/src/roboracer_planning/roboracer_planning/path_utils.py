# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Small geometry helpers shared by the planner and both controllers.
# All points are (x, y) tuples in base_link: x forward, y left.

import math


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(dx, dy):
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return 1.0, 0.0
    return dx / n, dy / n


def order_chain(points, max_link, max_gap, start=(0.0, 0.0), heading=(1.0, 0.0),
                max_first=10.0, max_turn_deg=75.0, gap_turn_deg=30.0):
    """Order loose points into a forward chain, starting at the car.

    R7 - Wrong pairing / zig-zag order
      Problem: sorting points by distance mixes up the order in hairpins.
      Fix:     walk from the car and always take the next point that is
               close AND roughly ahead of the current direction.

    R8 - Gap (missing cone or group of cones)
      Problem: if cones are missing, the next point can be further away than
               the normal spacing and the chain would end early.
      Fix:     if nothing is within max_link, allow one longer jump (up to
               max_gap) but only almost straight ahead, i.e. continue the
               track in its current direction across the gap.

    Returns (ordered_points, number_of_gaps_bridged).
    """
    remaining = list(points)
    ordered = []
    current = start
    hx, hy = heading
    gaps = 0
    cos_turn = math.cos(math.radians(max_turn_deg))
    cos_gap = math.cos(math.radians(gap_turn_deg))

    while remaining:
        first = not ordered
        reach = max_first if first else max_link
        best, best_cost = None, float('inf')
        for p in remaining:
            d = dist(current, p)
            if d < 1e-6 or d > reach:
                continue
            ux, uy = (p[0] - current[0]) / d, (p[1] - current[1]) / d
            dot = ux * hx + uy * hy
            if dot < cos_turn:
                continue
            cost = d * (2.0 - dot)      # prefer near and straight ahead
            if cost < best_cost:
                best, best_cost = p, cost

        if best is None and not first:
            # R8: try one longer, straight-ahead jump across a gap
            for p in remaining:
                d = dist(current, p)
                if d <= max_link or d > max_gap:
                    continue
                ux, uy = (p[0] - current[0]) / d, (p[1] - current[1]) / d
                dot = ux * hx + uy * hy
                if dot < cos_gap:
                    continue
                if d < best_cost:
                    best, best_cost = p, d
            if best is not None:
                gaps += 1

        if best is None:
            break
        hx, hy = unit(best[0] - current[0], best[1] - current[1])
        ordered.append(best)
        remaining.remove(best)
        current = best

    return ordered, gaps


def lookahead_point(path, lookahead, min_forward=0.0):
    """Point on the polyline `path` at `lookahead` metres from the car.

    Walks the ordered waypoints and linearly interpolates between the two
    waypoints that straddle the lookahead circle, so no smoothing is needed.
    If the whole path is closer than lookahead, the last point is used; if
    the first point is already beyond it, the first point is used.
    """
    pts = [p for p in path if p[0] >= min_forward]
    if not pts:
        return None
    if math.hypot(*pts[0]) >= lookahead:
        return pts[0]
    for a, b in zip(pts, pts[1:]):
        da, db = math.hypot(*a), math.hypot(*b)
        if da < lookahead <= db:
            t = (lookahead - da) / max(db - da, 1e-9)
            return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
    return pts[-1]


def path_length(path):
    """Distance from the car along the path to its last point."""
    if not path:
        return 0.0
    total = math.hypot(*path[0])
    for a, b in zip(path, path[1:]):
        total += dist(a, b)
    return total
