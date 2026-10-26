# RUB RoboRacer

ROS 2 software stack for the RoboRacer (F1TENTH-style) car of the AVAI team, Chair of Software Engineering, Ruhr University Bochum.
The car is simulated in **Ignition Gazebo (Fortress)** with the same ROS interfaces as the real car: LiDAR, RealSense RGB-D camera, IMU, and Ackermann drive commands on `/drive`.

Parts of this repository are adapted from TUD RoboRacer reference material and have been modified for this setup. Where applicable, source files include license headers or reference notes.

---

## Repository layout

```text
rrstack/
├── ros_ws/          ROS 2 workspace (build here)
│   └── src/         ROS 2 packages
├── tools/           Helper scripts (e.g. cone-track world generator)
└── README.md        Installation and quick start (this file)
```

| Package | Purpose | Documentation |
|---|---|---|
| `roboracer_gazebo` | Simulation: car model, worlds, ROS ↔ Ignition bridge | [README](ros_ws/src/roboracer_gazebo/README.md) |
| `roboracer_drive_to_cmd_vel` | Simulation only: converts `/drive` (Ackermann) to `/cmd_vel` | — |
| `roboracer_wallfollowing` | PID wall-following controller | [README](ros_ws/src/roboracer_wallfollowing/README.md) |
| `roboracer_lidar_centerline` | LiDAR centerline extraction and Pure Pursuit | [README](ros_ws/src/roboracer_lidar_centerline/README.md) |
| `roboracer_slam` | LiDAR SLAM with `slam_toolbox` | [README](ros_ws/src/roboracer_slam/README.md) |
| `roboracer_perception` | YOLO cone detection, Delaunay waypoints (stub) | — |
| `roboracer_rl` | Reinforcement-learning controller (optional) | [README](ros_ws/src/roboracer_rl/README.md) |


---

## Installation

### 1. ROS 2 Humble

Requires **Ubuntu 22.04**. Follow the official guide: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2. Ignition Gazebo (Fortress) and stack dependencies

```bash
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip
```

Check the simulator (expected: version 6.x, Fortress):

```bash
ign gazebo --version
```

### 3. Clone and build

```bash
git clone https://github.com/isselab/rrstack.git ~/rrstack
cd ~/rrstack/ros_ws

source /opt/ros/humble/setup.bash
sudo rosdep init      # only once per machine; skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y
pip install -r src/roboracer_perception/requirements.txt

colcon build --symlink-install
source install/setup.bash
```

Always run `colcon build` from `ros_ws/`, never from inside `src/`.

Optional, only for `roboracer_rl`:

```bash
pip install gymnasium stable-baselines3
```

---

## Launch the simulation

Every new terminal:

```bash
cd ~/rrstack/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Start Ignition Gazebo and spawn the car

```bash
ros2 launch roboracer_gazebo gazebo.launch.py                  # walls track (default)
ros2 launch roboracer_gazebo gazebo.launch.py world:=cones     # cone track
```

This starts Ignition Gazebo, loads the world, spawns the car at the origin, and starts the ROS ↔ Ignition bridge.

| Argument | Default | Description |
|---|---|---|
| `world` | `walls` | `walls`, `cones`, or a full path to a `.world` file |
| `gui` | `true` | `false` runs the simulator headless |
| `x`, `y`, `z`, `yaw` | `0.0`, `0.0`, `0.1`, `0.0` | Spawn pose of the car |
| `world_name` | `auto` | World name for the reset service; only needed for custom worlds |

Examples:

```bash
ros2 launch roboracer_gazebo gazebo.launch.py gui:=false
ros2 launch roboracer_gazebo gazebo.launch.py x:=1.0 y:=0.5 yaw:=1.57
```

### Check that the car is running

```bash
ros2 topic list                      # /scan /odom /imu /tf /clock /cmd_vel /camera/...
ros2 topic hz /scan
```

### Drive the car manually

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

On a virtual machine or a system without a GPU, prefix launch commands with `LIBGL_ALWAYS_SOFTWARE=1`.

Controllers, SLAM, and perception are described in the package READMEs linked above.