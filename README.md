
## Attribution

This repository is maintained for the RoboRacer/AVAI setup at the Chair of Software Engineering, Ruhr University Bochum.

Parts of this repository are adapted from TUD RoboRacer reference material and have been modified for the `rrstack` setup. Where applicable, source files include license headers or reference notes.

---

## LiDAR SLAM

The `roboracer_slam` package provides a LiDAR-based mapping pipeline for the RoboRacer simulation using ROS 2 and `slam_toolbox`.

SLAM, or Simultaneous Localization and Mapping, performs two closely related tasks:

- **Mapping:** The vehicle uses LiDAR measurements to construct an occupancy-grid representation of the environment.
- **Localization:** The vehicle estimates its current pose relative to the generated map.

The SLAM pipeline uses the simulated LiDAR scan, odometry information, and ROS 2 transform tree to incrementally construct the map as the vehicle moves through the environment.

### Required ROS Package

Install SLAM Toolbox if it is not already installed:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

The ROS 2 workspace must already be built before launching SLAM:

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch the SLAM Simulation

Open the first terminal and run:

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 ros2 launch roboracer_slam sim_slam_rviz.launch.py
```

This launch file starts the RoboRacer Gazebo simulation, SLAM Toolbox, and RViz.

`LIBGL_ALWAYS_SOFTWARE=1` enables software rendering and can be useful when running the simulation inside a virtual machine or on a system with limited graphics acceleration.

### RViz Configuration

In RViz, set:

```text
Fixed Frame: map
```

Add the following displays manually:

| RViz display | ROS topic |
|---|---|
| Map | `/map` |
| LaserScan | `/scan` |
| Odometry | `/odom` |
| TF | Transform tree |

The main SLAM data flow is:

```text
Gazebo LiDAR
    |
    v
/scan
    |
    v
slam_toolbox
    |
    +----> /map
    |
    +----> map -> odom transform

Vehicle odometry
    |
    v
/odom
```

The vehicle must move through the environment so that SLAM Toolbox receives LiDAR measurements from different positions and gradually constructs the occupancy-grid map.

### Run the Trained RL Controller

Open a second terminal while Gazebo and SLAM remain active:

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash
source rr_rl/bin/activate

python3 src/roboracer_rl/roboracer_rl/policy_node.py \
    --model models/roboracer_dqn.zip \
    --algorithm dqn
```

The trained reinforcement-learning controller publishes vehicle commands while the SLAM pipeline generates the map.

### Keyboard Teleoperation

The vehicle can alternatively be driven manually using keyboard teleoperation.

Open another terminal:

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The teleoperation node publishes movement commands to `/cmd_vel`.

Do not run the RL controller and keyboard teleoperation simultaneously unless command arbitration is configured, because both nodes may publish commands to the same topic.

### Inspect the SLAM Pipeline

List the active ROS 2 topics:

```bash
ros2 topic list
```

Inspect the SLAM Toolbox node:

```bash
ros2 node info /slam_toolbox
```

Inspect the main topics:

```bash
ros2 topic info /scan
ros2 topic info /odom
ros2 topic info /map
```

Check whether data is being published:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
```

Inspect the complete ROS 2 node and topic graph:

```bash
rqt_graph
```

The graph should show the LiDAR and odometry data flowing from the simulation into SLAM Toolbox, and the generated map being published to RViz.

### Save the Generated Map

After driving through the complete environment, save the occupancy-grid map:

```bash
mkdir -p ~/roboracer_maps
cd ~/roboracer_maps

ros2 run nav2_map_server map_saver_cli \
    -f roboracer_track_map
```

The command generates:

```text
roboracer_track_map.pgm
roboracer_track_map.yaml
```

The `.pgm` file contains the occupancy-grid image, and the `.yaml` file contains the map resolution, origin, occupancy thresholds, and image reference.

The generated map files are the final SLAM output.

---

## Wall Following

The `roboracer_wallfollowing` package provides a PID wall-following controller for the RoboRacer simulation.

The vehicle maintains a fixed lateral distance from one track wall using LiDAR and odometry. The controller is entirely classical: geometry and a PID loop, with no learned component.

Three LiDAR beams are used, arranged as a V around the perpendicular direction. The forward and rear arms produce an estimate of the wall, from which the controller obtains:

- **`Dt`** — the perpendicular distance from the vehicle to the wall
- **`alpha`** — the heading error relative to the wall

The perpendicular beam is excluded from that estimate and used instead as an independent validity check, so that a beam landing on the wrong surface at a corner can be detected and the estimate rejected.

The method extends the two-beam approach described in the F1TENTH Lab 3 material. A full description of the geometry, the control law, and the complete parameter list is given in the package README:

```text
packages/src/roboracer_wallfollowing/README.md
```

### Build the Wall Following Package

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash

colcon build \
    --symlink-install \
    --packages-select roboracer_wallfollowing

source install/setup.bash
```

### Launch Wall Following

The launch file starts the Gazebo simulation, spawns the vehicle, and runs the controller.

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 ros2 launch roboracer_wallfollowing wall_follow.launch.py
```

The wall to follow is selected at launch:

```bash
ros2 launch roboracer_wallfollowing wall_follow.launch.py side:=left
```

A different world can be supplied in the same way as for the other packages:

```bash
ros2 launch roboracer_wallfollowing wall_follow.launch.py \
    world:=$(ros2 pkg prefix roboracer_gazebo)/share/roboracer_gazebo/worlds/flw_cone_track_from_walls.world
```

Available launch arguments are `side`, `world`, `x`, `y`, `z`, and `yaw`.

### Topics

| Direction | ROS topic | Type |
|---|---|---|
| Subscribed | `/scan` | `sensor_msgs/LaserScan` |
| Subscribed | `/odom` | `nav_msgs/Odometry` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` |
| Published | `/wall_follow/markers` | `visualization_msgs/MarkerArray` |

The controller publishes commands to `/cmd_vel`. Do not run it at the same time as keyboard teleoperation or the RL controller unless command arbitration is configured, because all three publish to the same topic.

### Visualise the Controller

In RViz, set:

```text
Fixed Frame: laser
```

Add the following display:

| RViz display | ROS topic |
|---|---|
| MarkerArray | `/wall_follow/markers` |

The markers show the three beams, the fitted wall line, the measured distance `Dt`, and a text readout of the current distance, heading error, control error, and steering command.

### Inspect the Controller

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /cmd_vel
```

```bash
ros2 param list /wall_follow_pid_node
ros2 param get /wall_follow_pid_node side
ros2 param get /wall_follow_pid_node desired_distance
```

---

## Cone Track World

The Gazebo package includes the world:

```text
packages/src/roboracer_gazebo/worlds/flw_cone_track_from_walls.world
```

This environment contains blue and yellow cones positioned along the track boundaries. It can be used for cone detection, LiDAR perception, boundary extraction, path planning, and autonomous-driving experiments.

### Build the Gazebo Package

After adding or modifying the world or cone models, rebuild the workspace:

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash

colcon build \
    --symlink-install \
    --packages-select roboracer_gazebo roboracer_slam

source install/setup.bash
```

### Launch the Cone Track in Gazebo

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch roboracer_gazebo gazebo.launch.py \
    world:=$(ros2 pkg prefix roboracer_gazebo)/share/roboracer_gazebo/worlds/flw_cone_track_from_walls.world
```

### Launch SLAM with the Cone Track

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 ros2 launch roboracer_slam sim_slam_rviz.launch.py \
    world:=$(ros2 pkg prefix roboracer_gazebo)/share/roboracer_gazebo/worlds/flw_cone_track_from_walls.world
```

The cone models referenced by the world must be available to Gazebo. The expected model names are:

```text
model://blue_cone
model://yellow_cone
```

A typical model structure is:

```text
roboracer_gazebo/
└── models/
    ├── blue_cone/
    │   ├── model.config
    │   └── model.sdf
    └── yellow_cone/
        ├── model.config
        └── model.sdf
```

When the cone geometry is defined directly using SDF primitives, separate mesh files are not required. When the SDF contains a `<mesh>` URI, the corresponding mesh file must also be present in the model directory.

### Verify the Cone Models

```bash
find packages/src/roboracer_gazebo/models/blue_cone \
    -maxdepth 2 \
    -type f \
    -print

find packages/src/roboracer_gazebo/models/yellow_cone \
    -maxdepth 2 \
    -type f \
    -print
```

### Verify the Cone World

```bash
grep -n "blue_cone\|yellow_cone" \
    packages/src/roboracer_gazebo/worlds/flw_cone_track_from_walls.world
```

If the cones do not appear in Gazebo, verify that the model names inside `model.config`, `model.sdf`, and the world file are consistent.

