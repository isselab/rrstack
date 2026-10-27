# roboracer_wallfollowing

PID wall follower for the RoboRacer. The car keeps a fixed distance (`desired_distance`, default `0.6 m`) from the left or right wall using four LiDAR beams.

---

## Algorithm

### 1. The V: three beams fit the nearby wall

| Beam | Angle from heading | Role |
|---|---|---|
| `f` | `90° − 35°` | front arm, fits the wall line |
| `b` | `90°` | perpendicular, checks the fit |
| `r` | `90° + 35°` | rear arm, fits the wall line |

- `f` and `r` give two wall points, which define a straight wall line.
- `b` is not used in the fit, so it can test it: if `b` disagrees with the line by more than `residual_threshold`, the line is rejected (usually a corner).

### 2. Beam 4: look ahead

- **Aim:** beam 4 points at the wall one lookahead ahead of the car: `lookahead = base_lookahead + lookahead_speed_gain · speed`.
- **Target:** the hit point, moved `desired_distance` away from the wall, is where the car should be.
- **Continuity check:** beam 4 is trusted only if the wall is continuous from `b` to beam 4. A jump means an outer corner (the wall turns away), so the target is placed beside the wall's visible end instead.

### 3. PID

```text
error    = angle from base_link to the target
steering = Kp·error + Ki·∫error + Kd·d(error)/dt
```

- **Inputs combined:** position and heading errors are both in one number, so corners are seen before the car reaches them.
- **Speed:** slows from `nominal_speed` to `min_speed` as steering increases, and brakes for a wall straight ahead.

### 4. Steering-lag compensation (simulation)

- **Problem:** Ignition Fortress steers with a ~1 s lag. Without compensation, the car weaves.
- **Fix:** the node estimates the actual wheel angle and over-commands until the wheels catch up (`steer_lag_tau`, `lag_boost`).
- **Real car:** set `steer_lag_tau: 0.0`.

### 5. Recovery

If no wall is found, the car drives slowly (`recovery_speed`) toward the most open direction until the wall is visible again.

---

## Launch

```bash
cd ~/rrstack/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch roboracer_wallfollowing wall_follow.launch.py              # left wall
ros2 launch roboracer_wallfollowing wall_follow.launch.py side:=right  # right wall
```

Starts Ignition Gazebo, spawns the car, the controller, and the `/drive` → `/cmd_vel` bridge.

| Argument | Default | Description |
|---|---|---|
| `side` | `left` | Wall to follow: `left` or `right` |
| `world` | `walls` | `walls`, `cones`, or a path to a `.world` file |
| `x`, `y`, `z`, `yaw` | `0`, `0`, `0.1`, `0` | Spawn pose |

---

## RViz markers (`/wall_follow/markers`)

In a second terminal (after sourcing):

```bash
rviz2 -d ~/rrstack/ros_ws/src/roboracer_wallfollowing/rviz/wall_following_visual.rviz \
    --ros-args -p use_sim_time:=true
```

| Marker | Meaning |
|---|---|
| Green lines + dots | Beams `f`, `b`, `r` and beam 4, with their wall hits |
| Red line | Fitted wall line from the V |
| Orange ball | Lookahead target |
| Blue dot | `base_link` (rear axle) |
| Text | Mode, error, steering, residual |

Modes: `pid` (normal), `pid_line` (beam 4 sees nothing, uses the V line), `corner` (outer corner), `gap` (wall lost).

---

## Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribed | `/scan` | `sensor_msgs/LaserScan` |
| Subscribed | `/odom` | `nav_msgs/Odometry` (speed only) |
| Published | `/drive` | `ackermann_msgs/AckermannDriveStamped` |
| Published | `/wall_follow/markers` | `visualization_msgs/MarkerArray` |

---

## Main parameters (`config/wall_follow_params.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `side` | `left` | Wall to follow |
| `desired_distance` | `0.6` | Distance to keep from the wall [m] |
| `theta_front_deg`, `theta_rear_deg` | `35.0` | V arm angles from the perpendicular |
| `base_lookahead`, `lookahead_speed_gain` | `0.8`, `0.6` | Lookahead = base + gain · speed [m] |
| `kp`, `ki`, `kd` | `0.6`, `0.0`, `0.05` | PID gains (error in rad) |
| `max_steering_angle` | `0.4` | Steering limit [rad] |
| `steer_lag_tau` | `1.1` | Steering lag of the simulator [s]; `0.0` on the real car |
| `lag_boost` | `3.0` | Over-command factor while the wheels catch up |
| `nominal_speed`, `min_speed` | `1.5`, `0.5` | Speed on straights / at full steering [m/s] |
| `recovery_speed` | `0.5` | Speed while the wall is lost [m/s] |
| `residual_threshold` | `0.35` | Max disagreement of beam `b` with the fitted line [m] |

YAML values must keep their decimal point (`2.0`, not `2`).

---

## Tuning

- **Weaving in simulation:** raise `lag_boost` (e.g. `4.0`), or lower `nominal_speed`.
- **Too close to the wall in curves:** lower `nominal_speed`.
- **Real car:** `steer_lag_tau: 0.0`, then tune `kp`, `kd` at low speed.
