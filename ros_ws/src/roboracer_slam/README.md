# roboracer_slam

LiDAR SLAM for the RoboRacer simulation using `slam_toolbox`.

- **Mapping:** builds an occupancy-grid map from LiDAR scans.
- **Localization:** estimates the car's pose in that map.

---

## Launch

```bash
ros2 launch roboracer_slam sim_slam_rviz.launch.py                  # walls track
ros2 launch roboracer_slam sim_slam_rviz.launch.py world:=cones     # cone track
```

Starts Ignition Gazebo (via `roboracer_gazebo`), SLAM Toolbox after 7 s, and RViz (with `rviz/roboracer_slam.rviz`) after 9 s.

| Argument | Default | Description |
|---|---|---|
| `world` | track world | `walls`, `cones`, or a full path |
| `use_sim_time` | `true` | Use the Gazebo clock |
| `use_rviz` | `true` | Start RViz |
| `x`, `y`, `z`, `yaw` | `0.0`, `0.0`, `0.1`, `0.0` | Spawn pose |

On a VM or without a GPU, prefix with `LIBGL_ALWAYS_SOFTWARE=1`.

---

## Drive the car

The map only grows while the car moves. In a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Teleop publishes `/cmd_vel`. Do not run it together with another controller unless command arbitration is configured.

---

## Data flow

```text
LiDAR (gpu_lidar) ──► /scan ──► slam_toolbox ──► /map
                                       └──────► map → odom transform
AckermannSteering ──► /odom
```

---

## RViz

The launch file loads a ready configuration. To set it up manually:

| Setting / display | Value |
|---|---|
| Fixed Frame | `map` |
| Map | `/map` |
| LaserScan | `/scan` |
| Odometry | `/odom` |
| TF | — |

---

## Inspect

```bash
ros2 node info /slam_toolbox
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
rqt_graph
```

---

## Save the map

```bash
mkdir -p ~/roboracer_maps
ros2 run nav2_map_server map_saver_cli -f ~/roboracer_maps/roboracer_track_map
```

Produces `roboracer_track_map.pgm` (occupancy image) and `roboracer_track_map.yaml` (resolution, origin, thresholds).