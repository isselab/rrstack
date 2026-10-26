# SPDX-License-Identifier: MIT
# Authors: Sai Tarun Bhyri, Thorsten Berger
# Copyright (c) 2026 AVAI Team, Chair of Software Engineering, Ruhr University Bochum
#
# Part of the rrstack RoboRacer reinforcement learning package.

import math
import subprocess
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from ros_gz_interfaces.srv import ControlWorld
except ImportError:  # pragma: no cover
    ControlWorld = None

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover - exercised only when dependency is missing
    gym = None
    spaces = None


class RoboRacerEnv(gym.Env if gym is not None else object):
    """Small Gymnasium wrapper around the existing scan/odom/cmd_vel interface."""

    metadata = {'render_modes': []}

    def __init__(
        self,
        scan_topic: str = '/scan',
        odom_topic: str = '/odom',
        cmd_vel_topic: str = '/cmd_vel',
        reset_service: str = '/world/roboracer_track/control',
        world_name: str = 'roboracer_track',
        model_name: str = 'f110_car',
        spawn_x: float = 0.0,
        spawn_y: float = 0.0,
        spawn_z: float = 0.1,
        spawn_yaw: float = 0.0,
        lidar_bins: int = 36,
        fixed_speed: float = 1.5,
        step_duration: float = 0.1,
        episode_seconds: float = 30.0,
        crash_distance: float = 0.25,
        wall_distance: float = 0.6,
        max_lidar_range: float = 30.0,
        steering_values: Optional[Tuple[float, ...]] = None,
    ):
        if gym is None or spaces is None:
            raise ImportError(
                'RoboRacerEnv requires gymnasium. Install it with '
                '`pip install gymnasium stable-baselines3`.'
            )

        super().__init__()
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = Node('roboracer_rl_env')
        self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.lidar_bins = lidar_bins
        self.fixed_speed = fixed_speed
        self.step_duration = step_duration
        self.episode_seconds = episode_seconds
        self.crash_distance = crash_distance
        self.wall_distance = wall_distance
        self.max_lidar_range = max_lidar_range
        self.steering_values = steering_values or (-0.6, -0.3, 0.0, 0.3, 0.6)
        self.world_name = world_name
        self.model_name = model_name
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.spawn_z = spawn_z
        self.spawn_yaw = spawn_yaw

        self.latest_scan = None
        self.latest_odom = None
        self.episode_started_at = time.monotonic()
        self.last_action = 0.0

        self.cmd_pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )
        self.reset_client = (
            self.node.create_client(ControlWorld, reset_service)
            if ControlWorld is not None
            else None
        )

        self.action_space = spaces.Discrete(len(self.steering_values))
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(self.lidar_bins + 2,),
            dtype=np.float32,
        )

        self._wait_for_messages()

    def _scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _spin_for(self, seconds: float) -> None:
        end_time = time.monotonic() + seconds
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _wait_for_messages(self, timeout: float = 10.0) -> None:
        end_time = time.monotonic() + timeout
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest_scan is not None and self.latest_odom is not None:
                return
        raise TimeoutError(
            f'Timed out waiting for {self.scan_topic} and {self.odom_topic}. '
            'Start the Gazebo launch file before training.'
        )

    def _publish_cmd(self, speed: float, steering: float) -> None:
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steering)
        self.cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_cmd(0.0, 0.0)
        self._spin_for(0.05)

    def _call_service(self, client, request, timeout: float = 3.0) -> bool:
        if client is None or not client.wait_for_service(timeout_sec=2.0):
            return False
        future = client.call_async(request)
        end_time = time.monotonic() + timeout
        while time.monotonic() < end_time and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.done()

    def _teleport_home(self) -> bool:
        # The Ignition /world/<world>/set_pose service expects a flat
        # ignition.msgs.Pose request that the ros_gz parameter_bridge cannot
        # map onto ros_gz_interfaces/srv/SetEntityPose. Calling the `ign
        # service` CLI directly is the simplest reliable way to teleport the
        # car back to its spawn pose between episodes.
        half = self.spawn_yaw * 0.5
        qz = math.sin(half)
        qw = math.cos(half)
        service = f'/world/{self.world_name}/set_pose'
        request = (
            f'name: "{self.model_name}", '
            f'position: {{x: {self.spawn_x}, y: {self.spawn_y}, z: {self.spawn_z}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'
        )
        try:
            result = subprocess.run(
                [
                    'ign', 'service', '-s', service,
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', request,
                ],
                capture_output=True, text=True, timeout=6.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.node.get_logger().warning(f'Failed to run `ign service`: {exc}')
            return False
        return 'data: true' in result.stdout

    def _reset_gazebo(self) -> None:
        # Ignition's world-control "reset" only rewinds the clock and plugin
        # state; it does NOT teleport the model back to its spawn pose. So the
        # car is teleported home first, then the clock is reset. This mirrors
        # what the RL environment expects at the start of every episode.

        # 1) Teleport the car back to its spawn pose.
        if not self._teleport_home():
            self.node.get_logger().warning(
                'Failed to teleport car to spawn pose; continuing anyway.'
            )

        # 2) Reset world time / plugin state.
        if self.reset_client is not None:
            req = ControlWorld.Request()
            req.world_control.reset.all = True
            if not self._call_service(self.reset_client, req):
                self.node.get_logger().warning('Timed out waiting for Gazebo reset.')
        else:
            self.node.get_logger().warning(
                'ros_gz_interfaces is not available; continuing without clock reset.'
            )

    def _sectorize_scan(self) -> np.ndarray:
        ranges = np.asarray(self.latest_scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges,
            nan=self.max_lidar_range,
            posinf=self.max_lidar_range,
            neginf=0.0,
        )
        ranges = np.clip(ranges, 0.0, self.max_lidar_range)
        sectors = np.array_split(ranges, self.lidar_bins)
        mins = np.asarray([sector.min(initial=self.max_lidar_range) for sector in sectors])
        return (mins / self.max_lidar_range).astype(np.float32)

    def _speed(self) -> float:
        twist = self.latest_odom.twist.twist

        vel = np.array(
            [twist.linear.x, twist.linear.y],
            dtype=np.float32
        )

        vel = np.nan_to_num(
            vel,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )

        speed = float(np.linalg.norm(vel))

        if not np.isfinite(speed):
            return 0.0

        return speed

    def _observation(self) -> np.ndarray:
        lidar = self._sectorize_scan()
        speed = np.clip(self._speed() / 10.0, 0.0, 1.0)
        steering = np.clip((self.last_action + 0.6) / 1.2, 0.0, 1.0)

        obs = np.concatenate([lidar, [speed, steering]]).astype(np.float32)

        obs = np.nan_to_num(
            obs,
            nan=0.0,
            posinf=1.0,
            neginf=0.0,
        )

        return np.clip(obs, 0.0, 1.0).astype(np.float32)

    def _min_lidar(self) -> float:
        ranges = np.asarray(self.latest_scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges,
            nan=self.max_lidar_range,
            posinf=self.max_lidar_range,
            neginf=0.0,
        )
        return float(
            np.clip(ranges, 0.0, self.max_lidar_range).min(
                initial=self.max_lidar_range
            )
        )

    def _reward(self, steering: float, min_lidar: float, crashed: bool) -> float:
        if crashed:
            return -10.0

        reward = 0.1 * self._speed()
        reward -= 0.02 * abs(steering)
        if min_lidar < self.wall_distance:
            reward -= (self.wall_distance - min_lidar) / self.wall_distance
        return float(reward)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._stop()
        self._reset_gazebo()
        self.latest_scan = None
        self.latest_odom = None
        self.last_action = 0.0
        self.episode_started_at = time.monotonic()
        self._wait_for_messages()
        self._spin_for(0.2)
        return self._observation(), {}

    def step(self, action):
        steering = self.steering_values[int(action)]
        self.last_action = steering
        self._publish_cmd(self.fixed_speed, steering)
        self._spin_for(self.step_duration)

        min_lidar = self._min_lidar()
        crashed = min_lidar < self.crash_distance
        elapsed = time.monotonic() - self.episode_started_at
        truncated = elapsed >= self.episode_seconds
        reward = self._reward(steering, min_lidar, crashed)
        observation = self._observation()
        info = {
            'min_lidar': min_lidar,
            'speed': self._speed(),
            'steering': steering,
        }
        if crashed:
            self._stop()
        return observation, reward, crashed, truncated, info

    def close(self):
        self._stop()
        self.node.destroy_node()


def make_env(**kwargs):
    return RoboRacerEnv(**kwargs)
