# roboracer_rl

Minimal reinforcement-learning controller for the existing Gazebo RoboRacer setup.

The first controller is intentionally simple:

- Observation: 36 lidar sectors from `/scan`, odometry speed from `/odom`, previous steering command.
- Action: one of five discrete steering commands.
- Command: fixed forward speed plus learned steering on `/cmd_vel`.
- Episode end: lidar range below the crash threshold or timeout.

## Dependencies

Install the Python RL dependencies in the same environment used by ROS 2:

```bash
pip install gymnasium stable-baselines3
```

## Build

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Start Gazebo

Run the existing simulator first:

```bash
ros2 launch roboracer_gazebo gazebo.launch.py
```

The RL environment expects these defaults:

- `/scan`: `sensor_msgs/LaserScan`
- `/odom`: `nav_msgs/Odometry`
- `/cmd_vel`: `geometry_msgs/Twist`
- `/reset_world`: Gazebo reset service

## Smoke Test Control

Before training, verify the car accepts `/cmd_vel` commands:

```bash
ros2 run roboracer_rl rl_random_policy --ros-args
```

Stop it with `Ctrl+C`.

## Train

Train a discrete-steering DQN policy:

```bash
ros2 run roboracer_rl rl_train --algorithm dqn --timesteps 100000 --output models/roboracer_dqn
```

The default reward is deliberately basic: move forward, avoid nearby walls, avoid excessive steering, and heavily penalize crashes. It is enough for the first closed-loop experiment, but not enough for racing-line optimization.

## Run A Trained Policy

```bash
ros2 run roboracer_rl rl_policy_node --model models/roboracer_dqn.zip --algorithm dqn
```

## Next Improvements

- Add a track centerline and reward progress along the lap.
- Randomize spawn pose on reset.
- Move from fixed speed to learned speed and steering with PPO or SAC.
- Add collision/contact feedback instead of using only lidar distance.
