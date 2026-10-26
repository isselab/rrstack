# roboracer_gazebo

Ignition Gazebo (Fortress) simulation of the RoboRacer F110 car: car model, worlds, cone models, and the ROS ↔ Ignition bridge.

---

## Launch

```bash
ros2 launch roboracer_gazebo gazebo.launch.py                  # walls track (default)
ros2 launch roboracer_gazebo gazebo.launch.py world:=cones     # cone track
ros2 launch roboracer_gazebo gazebo.launch.py gui:=false       # headless
```

| Argument | Default | Description |
|---|---|---|
| `world` | `walls` | `walls`, `cones`, or a full path to a `.world` file |
| `gui` | `true` | `false` runs without the Ignition GUI |
| `x`, `y`, `z`, `yaw` | `0.0`, `0.0`, `0.1`, `0.0` | Spawn pose of the car |
| `world_name` | `auto` | Must match `<world name="...">` in the world file. Inferred automatically for `walls` and `cones`; only needed for custom worlds (used for the reset service) |

---

## Worlds

| `world:=` | File | Content |
|---|---|---|
| `walls` | `worlds/roboracer_track.world` | FLW-Halle-2 indoor track with walls |
| `cones` | `worlds/flw_cone_track_from_walls.world` | Blue/yellow cones along the track boundaries (no walls) |

The cone world is generated from the track walls by `tools/generate_flw_cone_track.py`.

---

## Car model

| File | Used by | Purpose |
|---|---|---|
| `models/f110_car/model.sdf` | Ignition (`ros_gz_sim create`) | Physics, Ackermann drive, sensors |
| `models/f110_car/f110_car.urdf` | `robot_state_publisher` | TF tree of the static frames |

Both files describe the same car and must be kept consistent.

---

## ROS topics (via `ros_gz_bridge`)

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo → ROS |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS |
| `/imu` | `sensor_msgs/Imu` | Gazebo → ROS |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gazebo → ROS |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Gazebo → ROS |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Gazebo → ROS |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | Gazebo → ROS |

`/cmd_vel` is read by Ignition's AckermannSteering system: `linear.x` = speed (m/s), `angular.z` = **yaw rate** (rad/s). Controllers publish `/drive` instead; `roboracer_drive_to_cmd_vel` converts it.

---

## Cone models

```text
models/
├── blue_cone/    model.config, model.sdf, meshes/cone_mesh.dae
└── yellow_cone/  model.config, model.sdf, meshes/cone_mesh.dae
```

- Visual: cone mesh (with normals, required for correct colours in Ignition).
- Collision: cylinder primitive (radius 0.117 m, height 0.2925 m).

Check the models and their use in the world:

```bash
find src/roboracer_gazebo/models/blue_cone src/roboracer_gazebo/models/yellow_cone -maxdepth 2 -type f
grep -c "blue_cone\|yellow_cone" src/roboracer_gazebo/worlds/flw_cone_track_from_walls.world
```

If cones do not appear, check that the model names in `model.config`, `model.sdf`, and the world file match. Ignition finds models via `IGN_GAZEBO_RESOURCE_PATH`, which the launch file sets automatically.

---

