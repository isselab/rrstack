# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# What the controllers do when the planner gives no usable waypoints.
# Shared by centerline_pid_node and waypoint_pursuit_node.
#
#   WAIT    -> no waypoints received yet since start: stand still
#   TRACK   -> fresh waypoints: normal driving
#   HOLD    -> waypoints briefly missing: keep the last ones, drive slowly   (R2)
#   EXPLORE -> missing longer: creep forward in steps while weaving          (R8)
#   STOP    -> explored too far without finding cones: stand still

import math

WAIT, TRACK, HOLD, EXPLORE, STOP = 'wait', 'track', 'hold', 'explore', 'stop'


class Recovery:

    def __init__(self, fresh_timeout, hold_timeout, explore_speed, explore_step,
                 explore_pause, explore_max_distance, weave_amplitude,
                 weave_period, weave_bias):
        self.fresh_timeout = fresh_timeout
        self.hold_timeout = hold_timeout
        self.explore_speed = explore_speed
        self.explore_step = explore_step
        self.explore_pause = explore_pause
        self.explore_max_distance = explore_max_distance
        self.weave_amplitude = weave_amplitude
        self.weave_period = weave_period
        self.weave_bias = weave_bias

        self.last_good = None       # time [s] of the last usable waypoint set
        self.last_side = 0.0        # +1 track was bending left, -1 right
        self.mode = WAIT
        self._explored = 0.0        # distance driven in EXPLORE [m]
        self._step_done = 0.0       # distance driven in the current step [m]
        self._pause_until = None
        self._explore_t0 = None

    def mark_good(self, now, target_y):
        """Called whenever a usable waypoint set arrives."""
        self.last_good = now
        if abs(target_y) > 0.05:
            self.last_side = 1.0 if target_y > 0.0 else -1.0

    def update(self, now):
        """Pick the mode from how long waypoints have been missing."""
        if self.last_good is None:
            self.mode = WAIT
            return self.mode
        lost_for = now - self.last_good
        if lost_for <= self.fresh_timeout:
            new = TRACK
        elif lost_for <= self.hold_timeout:
            # R2 - No cones for a moment
            # Problem: YOLO drops a frame or two, /waypoints arrives empty.
            # Fix:     keep driving on the last waypoints, but slowly.
            new = HOLD
        elif self._explored < self.explore_max_distance:
            # R8 - Missing cone / group of cones, nothing usable in view
            # Problem: the camera sees no pair and no boundary to follow.
            # Fix:     creep forward in small steps and weave left/right so
            #          the narrow camera sweeps a wider area to find cones.
            new = EXPLORE
        else:
            # Explored explore_max_distance without finding cones: give up.
            new = STOP

        if new in (TRACK, HOLD):
            self._explored = 0.0
            self._step_done = 0.0
            self._pause_until = None
            self._explore_t0 = None
        self.mode = new
        return new

    def explore_command(self, now, dt, measured_speed):
        """(speed, steering) for EXPLORE: step forward, pause, look, repeat."""
        if self._explore_t0 is None:
            self._explore_t0 = now

        # pause between steps: stand still so YOLO gets a few sharp frames
        if self._pause_until is not None:
            if now < self._pause_until:
                return 0.0, 0.0
            self._pause_until = None
            self._step_done = 0.0

        travelled = max(measured_speed, 0.0) * dt
        self._step_done += travelled
        self._explored += travelled
        if self._step_done >= self.explore_step:
            self._pause_until = now + self.explore_pause
            return 0.0, 0.0

        # weave around a small bias toward the side the track was bending to
        t = now - self._explore_t0
        steer = (self.weave_bias * self.last_side
                 + self.weave_amplitude * math.sin(2.0 * math.pi * t / self.weave_period))
        return self.explore_speed, steer

    @property
    def explored(self):
        return self._explored
