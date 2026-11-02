# roboracer_planning

Drives the RoboRacer between blue (left) and yellow (right) cones using only the camera.
YOLO detects the cones, a Delaunay triangulation turns them into track waypoints, and
**pure pursuit** follows them. Best result in Gazebo: 1.25 laps, no cone contact,
1.03 m/s average (up to 1.5 m/s).

```text
camera ─► yolo_node ─► delaunay_node ─► waypoint_pursuit_node ─► /drive ─► steering_lag_compensator ─► drive_to_cmd_vel ─► Gazebo
          (perception)  (waypoints)      (controller)                       (sim only)                   (sim only)
```

A second controller, `centerline_pid_node`, is kept as an alternative (`controller:=pid`).

---

## Algorithm

### 1. Cone detection (`yolo_node`, roboracer_perception)

- YOLO finds blue and yellow cones in the colour image; the depth image gives their distance.
- Published as two `PoseArray`s in the camera frame, both with the camera frame's timestamp.
- Yellow cones on the shaded side of a corner score only 0.3–0.5, so the launch lowers `yellow_conf` from 0.5 to 0.3 (R9).

### 2. Waypoints (`delaunay_node`)

1. Move the cones from the camera to `base_link` (camera is 0.37 m ahead). Drop cones behind the car or further than `max_range` (6.5 m).
2. Delaunay triangulation over all cones. Keep only **blue–yellow** edges that are track-width long (`min_edge` 0.8 … `max_edge` 2.2 m). Same-colour edges and too long or too short edges are wrong pairs.
3. The midpoints of the kept edges are the waypoints. If one colour is missing, the visible cones are shifted half a track width (0.7 m) into the track instead (R1/R8).
4. Chain them forward from the car: always the next point that is close **and** roughly ahead. No smoothing.
5. Publish `/waypoints` with **one timestamp for the whole set** (the camera frame's), so the controller can measure end-to-end latency.

### 3. Pure pursuit (`waypoint_pursuit_node`)

```text
target    = first waypoint at least lookahead_distance (1.8 m) ahead
alpha     = angle from the car's heading to the target
curvature = 2·sin(alpha) / lookahead_distance
steering  = atan(wheelbase · curvature)
speed     = target_speed / (1 + curvature_speed_gain · |curvature|)    → 1.5 m/s straight, ~0.6 m/s in tight corners
```

- **Lookahead 1.8 m:** the camera sees nothing closer than ~1.5 m. A shorter lookahead just picks the nearest visible point, so it has no effect.
- **Corner slowing (`curvature_speed_gain` 3.0):** this is what makes 1.5 m/s safe at the hairpin.

### 4. Steering-lag compensation (simulation) — `steering_lag_compensator`

This is the part that made every controller in this repository work in Ignition.
It lives in the sim-only package [`roboracer_drive_to_cmd_vel`](../roboracer_drive_to_cmd_vel/README.md),
so the controllers contain only real-car code.

#### 4.1 Problem

Ignition Fortress' `AckermannSteering` plugin does not set the wheel angle you command. It moves the wheels **slowly** towards it:

```text
d(delta)/dt = (command − delta) / tau          tau ≈ 1.05 s
```

`delta` is the real wheel angle. After a step the wheels reach 63 % after ~1 s and 90 % after ~2.3 s. A real servo does this in 0.1–0.2 s.

Effect on driving: at 1.5 m/s the car travels ~1.6 m before the wheels reach most of the commanded angle. It turns in too late,
the controller asks for more, the wheels arrive too late again and overshoot → weaving and cones hit at corner exits.
Before the compensator, pure pursuit could only drive 0.22–0.6 m/s (0.38 m/s average); the PID hit a cone at the first sharp corner.

#### 4.2 Measurement (step tests, empty world)

A test script commanded steering steps and recorded the real wheel angle from `/joint_states` (~735 Hz, `left_steering_hinge` / `right_steering_hinge`):

| Test | Result |
|---|---|
| Time to 63 % / 90 % of the step | 1.01–1.10 s / 2.2–2.4 s → first-order lag, tau ≈ 1.05 s |
| Speed 0.5 vs 1.0 m/s | same tau |
| Step size 0.15 / 0.3 / 0.5 rad | same tau |
| Dead time (before the wheels start moving) | 0.06–0.15 s, small |
| `steer_p_gain` 10 vs 100 in `model.sdf` | no change: Fortress does not have this parameter, so the lag cannot be fixed in the car model |
| Speed 0 | wheels do not steer at all (`drive_to_cmd_vel` sends yaw rate = v·tan(delta)/L = 0) |

Same tau for every speed and step size → **one tau for all controllers**, no per-controller tuning.

#### 4.3 Fix: over-command, closed loop on the measured angle

```text
command = delta + boost · (wanted − delta)          boost = 3, clamped to ±0.6458 rad
```

- `wanted` = the controller's steering on `/drive`; `delta` = measured wheel angle (mean of the two hinges, which matches the bicycle-model angle).
- While the wheels are far from `wanted`, the command overshoots on purpose, so the plugin moves them faster.
- As `delta` approaches `wanted`, the extra part shrinks to zero → the wheels settle **exactly** at `wanted`, no overshoot.

Why it works — put the command into the plant equation:

```text
d(delta)/dt = (command − delta) / tau = boost · (wanted − delta) / tau
            → the same lag, but with tau_effective = tau / boost = 1.05 / 3 ≈ 0.35 s
```

| Step 0.15 rad | 63 % | 90 % | Final angle |
|---|---|---|---|
| no compensation | 1.02 s | 2.30 s | still settling after 2.5 s |
| **boost 3 (default)** | **0.39 s** | **0.80 s** | exact, no overshoot |
| boost 5 | 0.29 s | 0.52 s | exact; large steps hit the steering limit |

#### 4.4 Design choices

| Choice | Why |
|---|---|
| **Closed loop** (measured `delta` from `/joint_states`) | The older version inside the controllers *estimated* `delta` from a guessed tau. A wrong guess over-steers for a long time. The measured angle is always right. |
| **Separate node** between controller and bridge | Controllers publish `/drive` exactly as on the car. Only the sim pipeline adds `/drive → /drive_sim`. |
| **One tau, boost 3** | tau was the same in all tests. Boost 3 is fast without hitting the steering limit on normal corrections; 5 saturates on large steps. |
| **Clamp to ±0.6458 rad** | `<steering_limit>` in `f110_car/model.sdf`. Over-commanding past it does nothing. |
| **No fresh `/joint_states` (> 0.2 s) → pass through** | Never boost blind; logs a warning. |
| **`enabled: auto`** | Compensates only when `use_sim_time` is true (every sim launch sets it). On the real car it would be a passthrough, so no launch argument is needed anywhere. `true` / `false` force it. |

#### 4.5 Live check

Every 10 s the node logs, e.g.:

```text
effective steering lag 0.39 s from 29 samples (expected ~0.35 s)
```

It fits `d(delta)/dt = (wanted − delta) / tau_eff` to the last 6 s. With pure pursuit it reads 0.36–0.45 s.

- Near 0.35 s → working.
- Near 1.05 s → compensation not acting (check `enabled`, `/joint_states`).
- Higher with the wall follower (1.3–2.9 s): its steering jumps between modes and saturates, which the fit does not model. Only the log is affected, not the control.

To re-measure the raw plant lag: launch with `lag_comp:=false`; the log then reports tau itself (~1.05 s).

#### 4.6 Real car

The node is not started. A real servo has ~0.1–0.2 s lag, so nothing needs to be compensated and the controllers run unchanged.

### 5. Recovery (both controllers)

```text
WAIT ──first waypoints──▶ TRACK ──none for 0.5 s──▶ HOLD ──none for 1.0 s──▶ EXPLORE ──3 m──▶ STOP
                            ▲                         │                         │
                            └────────── usable waypoints again ─────────────────┘
```

- **HOLD (R2):** keep the last waypoints, drive at `hold_speed`.
- **EXPLORE (R8):** creep forward in 1 m steps while weaving, so the camera sweeps for cones; STOP after 3 m.

---

## Launch

```bash
cd ~/rrstack/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch roboracer_planning cone_drive.launch.py                       # pure pursuit (recommended)
ros2 launch roboracer_planning cone_drive.launch.py controller:=pid       # centreline PID
ros2 launch roboracer_planning cone_drive.launch.py gui:=false rviz:=false  # headless
```

Starts everything in stages; each stage waits until the previous one is ready (no timers). The car starts driving by itself.

| Stage | Starts | When |
|---|---|---|
| 1 | Gazebo (cones world) + car | car spawns once `/clock` runs |
| 2 | `yolo_node` | first `/camera/color/camera_info` |
| 3 | `delaunay_node`, controller, `steering_lag_compensator`, `drive_to_cmd_vel` | first `/cones/blue` |
| 4 | RViz | first `/waypoints` |

The launch log prints `[stage N]`, so a stuck start shows where it is waiting.

| Argument | Default | Description |
|---|---|---|
| `controller` | `pursuit` | `pursuit` or `pid` |
| `world` | `cones` | `cones`, `walls`, or a path to a `.world` file |
| `gui` / `rviz` | `true` / `true` | Gazebo GUI / RViz |
| `yellow_conf` | `0.3` | YOLO gate for yellow cones (R9) |
| `lag_comp` | `auto` | Steering-lag compensation: `auto` = on in simulation, `false` = off (to measure the raw lag) |
| `lag_boost` | `3.0` | Effective lag = 1.05 s / boost |
| `x`, `y`, `z`, `yaw` | `0`, `0`, `0.1`, `0` | Spawn pose |

### Running the nodes one by one (for debugging)

One terminal each, after sourcing. `P` is the parameter file:

```bash
P=install/roboracer_planning/share/roboracer_planning/config/planning_params.yaml

ros2 launch roboracer_gazebo gazebo.launch.py world:=cones                                                # 1 Gazebo + car
ros2 run roboracer_perception yolo_node --ros-args -p use_sim_time:=true -p yellow_conf:=0.3              # 2 cones
ros2 run roboracer_planning delaunay_node --ros-args --params-file $P -p use_sim_time:=true               # 3 waypoints
ros2 run roboracer_planning waypoint_pursuit_node --ros-args --params-file $P -p use_sim_time:=true       # 4 controller
ros2 run roboracer_drive_to_cmd_vel steering_lag_compensator --ros-args -p use_sim_time:=true             # 5 lag (sim)
ros2 run roboracer_drive_to_cmd_vel drive_to_cmd_vel --ros-args -p use_sim_time:=true -p drive_topic:=/drive_sim   # 6 bridge
rviz2 -d src/roboracer_planning/rviz/cone_planning.rviz --ros-args -p use_sim_time:=true                 # 7 RViz
```

`use_sim_time:=true` is required on every node, otherwise latencies are wrong and the compensator stays off.

### Running on the real car

With the RealSense driver and the motor driver (`/drive` → VESC) already running:

```bash
ros2 launch roboracer_planning cone_drive_car.launch.py                  # pure pursuit, max 0.8 m/s
ros2 launch roboracer_planning cone_drive_car.launch.py max_speed:=1.2   # faster once it works
ros2 launch roboracer_planning cone_drive_car.launch.py controller:=pid  # centreline PID
```

Starts `yolo_node` once the camera publishes, then the planner and the controller once
cones are detected. Without a camera nothing starts and nothing is sent on `/drive`.

| Argument | Default | Description |
|---|---|---|
| `controller` | `pursuit` | `pursuit` or `pid` |
| `camera_prefix` | `/camera/realsense2_camera` | RealSense topic prefix (check with `ros2 topic list`) |
| `max_speed` | `0.8` | Speed cap for first runs [m/s]; Gazebo-tested up to 1.5 (keep the decimal point) |
| `yellow_conf` | `0.3` | YOLO gate for yellow cones (R9) |
| `rviz` | `false` | RViz (needs a screen) |
| `use_sim_time` | `false` | `true` only to test this launch against Gazebo |

Why it differs from the simulation launch:

| Left out / changed | Why |
|---|---|
| Gazebo | The car has its own sensors and drivers |
| `steering_lag_compensator` | Compensates Gazebo's 1.05 s steering lag; a real servo reacts in ~0.1–0.2 s. Its output (`/drive_sim`) would not reach the VESC anyway |
| `drive_to_cmd_vel` | Gazebo needs `/cmd_vel`; the VESC reads `/drive` directly from the controller |
| `use_sim_time` | No simulation clock on the car; nodes use the system clock |
| Camera topics | RealSense names, set only in this launch: `<prefix>/color/image_raw`, `<prefix>/aligned_depth_to_color/image_raw` (depth aligned to the colour image, so it matches the YOLO boxes pixel by pixel), `<prefix>/color/camera_info`. The simulator keeps `/camera/color/...` |
| Depth units | The RealSense sends millimetres (`16UC1`); `yolo_node` converts them to metres. Gazebo sends metres (`32FC1`) |

Tested against Gazebo with a stand-in that republishes the simulated camera under the
RealSense names with millimetre depth: cones at correct distances, staged start, waits
without a camera. Before the first drive, check `ros2 topic echo /cones/blue` shows cones
at plausible distances, and the camera offset (`camera_forward_offset`, 0.37 m in the yaml).

---

## Which controller runs, and when

```text
ros2 launch ... controller:=<value>
        │
        ▼
check_controller()  ── not 'pursuit' / 'pid' ──▶  launch stops with an error (nothing started)
        │
        ▼  stage 3 (first /cones/blue)
IfCondition('<value>' == 'pursuit') ──▶ waypoint_pursuit_node ─┐
IfCondition('<value>' == 'pid')     ──▶ centerline_pid_node  ──┴─▶ /drive ─▶ steering_lag_compensator ─▶ /drive_sim ─▶ drive_to_cmd_vel ─▶ /cmd_vel
                                                                              (enabled: auto → on when use_sim_time)
```

| Decision | Where | Condition | Result |
|---|---|---|---|
| Controller name valid? | `launch/cone_drive.launch.py`, `check_controller()` | value is `pursuit` or `pid` | otherwise the launch stops before Gazebo starts: `Caught exception in launch ...: unknown controller 'purepursuit': use controller:=pursuit or controller:=pid` |
| Which controller | `launch/cone_drive.launch.py`, the two `IfCondition`s | `controller == 'pursuit'` / `== 'pid'` | exactly one controller node starts; both publish `/drive`, so the rest of the pipeline is the same |
| When it starts | `launch/cone_drive.launch.py`, stage 3 | first `/cones/blue` message | starts together with `delaunay_node`, the compensator and the bridge |
| Compensator on / off | `roboracer_drive_to_cmd_vel/steering_lag_compensator.py`, `_resolve_enabled()` | `enabled: auto` and `use_sim_time: true` | simulation: compensates; real car: passthrough |
| What the controller does | `roboracer_planning/recovery.py` (used by both controllers) | age of the newest `/waypoints` | TRACK → HOLD (0.5 s) → EXPLORE (1.0 s) → STOP (3 m) |
| Wall follower | `roboracer_wallfollowing/launch/wall_follow.launch.py` | separate launch | never started by `cone_drive.launch.py` |

---

## RViz markers (`/planning/markers`, `/planning/controller_markers`, fixed frame `base_link`)

| Marker | Meaning |
|---|---|
| Blue / yellow spheres | Cones used by the planner |
| Green lines | Kept blue–yellow edges |
| Faint grey lines | Rejected edges (same colour or wrong length) |
| White dots | Edge midpoints |
| Orange dots | Fallback points (R1/R8) |
| Magenta line + arrows | Ordered waypoints |
| Orange sphere | Controller's current target |
| Text | Planner counts; controller mode, steering, speed, latency |

---

## Topics

| Node | Direction | Topic | Type |
|---|---|---|---|
| `yolo_node` | Subscribed | `/camera/color/image_raw`, `/camera/depth/image_raw` | `sensor_msgs/Image` |
| | Subscribed | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| | Published | `/cones/blue`, `/cones/yellow` | `geometry_msgs/PoseArray` (camera frame) |
| `delaunay_node` | Subscribed | `/cones/blue`, `/cones/yellow` | `geometry_msgs/PoseArray` |
| | Published | `/waypoints` | `geometry_msgs/PoseArray` (`base_link`, camera stamp) |
| | Published | `/planning/markers` | `visualization_msgs/MarkerArray` |
| `waypoint_pursuit_node` / `centerline_pid_node` | Subscribed | `/waypoints` | `geometry_msgs/PoseArray` |
| | Subscribed | `/odom` | `nav_msgs/Odometry` (speed only) |
| | Published | `/drive` | `ackermann_msgs/AckermannDriveStamped` |
| | Published | `/planning/controller_markers` | `visualization_msgs/MarkerArray` |
| `steering_lag_compensator` (sim) | Subscribed | `/drive` | `ackermann_msgs/AckermannDriveStamped` |
| | Subscribed | `/joint_states` | `sensor_msgs/JointState` |
| | Published | `/drive_sim` | `ackermann_msgs/AckermannDriveStamped` |
| `drive_to_cmd_vel` (sim) | Subscribed | `/drive_sim` | `ackermann_msgs/AckermannDriveStamped` |
| | Published | `/cmd_vel` | `geometry_msgs/Twist` |

---

## Main parameters (`config/planning_params.yaml`)

**`waypoint_pursuit_node`**

| Parameter | Default | Description |
|---|---|---|
| `lookahead_distance` | `1.8` | Target distance [m]; must exceed the ~1.5 m camera blind zone |
| `wheelbase` | `0.32` | [m] |
| `target_speed`, `maximum_speed`, `minimum_speed` | `1.5`, `1.5`, `0.5` | Speed range [m/s] |
| `curvature_speed_gain` | `3.0` | Corner slowing; ~0.6 m/s in tight corners |
| `maximum_steering_angle` | `0.55` | [rad] |
| `control_frequency` | `30.0` | [Hz] |
| `fresh_timeout`, `hold_timeout` | `0.5`, `1.0` | TRACK → HOLD → EXPLORE [s] |
| `hold_speed`, `explore_speed` | `0.3`, `0.3` | [m/s] |

**`delaunay_node`**

| Parameter | Default | Description |
|---|---|---|
| `camera_forward_offset` | `0.37` | Camera ahead of `base_link` [m] |
| `max_range` | `6.5` | Ignore cones further away (R10) [m] |
| `min_edge`, `max_edge` | `0.8`, `2.2` | Blue–yellow edge length gate (R4) [m] |
| `half_track_width` | `0.7` | Shift for unpaired cones (R1/R8) [m] |

**`steering_lag_compensator`** (set in the launch file)

| Parameter | Default | Description |
|---|---|---|
| `enabled` | `auto` | `auto` (on with `use_sim_time`) / `true` / `false` |
| `boost` | `3.0` | Effective lag = tau / boost |
| `tau` | `1.05` | Measured plant lag [s] (log reference) |
| `steering_limit` | `0.6458` | Must match `f110_car/model.sdf` [rad] |
| `joint_states_timeout` | `0.2` | Older measurement → pass through [s] |

Centreline PID parameters: see the yaml. YAML values must keep their decimal point (`2.0`, not `2`).

---

## Robustness

Functions handling these carry a comment with the same tag (`grep -rn "R8" roboracer_planning/`).

| # | Issue | Solution | Status |
|---|---|---|---|
| R1 | One colour not visible (narrow camera in tight corners) → no pairs | Shift unpaired cones half a track width into the track | v1 |
| R2 | No cones for a moment (dropped frames) | HOLD: keep last waypoints, drive at `hold_speed` | v1 |
| R3 | Detections flicker between frames | Cone memory moved with odometry | later |
| R4 | False / wrong-colour cones → midpoint outside the track | Keep only edges of track width (`min_edge`..`max_edge`) | v1 |
| R5 | Slow updates → driving on old waypoints | Short visible path (`min_path_length`) → `min_speed` | v1 |
| R6 | Position error grows with distance | Prefer near waypoints, short lookahead | later |
| R7 | Wrong pairing with uneven spacing (zig-zag) | Delaunay + blue–yellow edges + forward chaining | v1 |
| R8 | Missing cone or group of cones | Fill from the visible boundary (R1 shift), bridge short gaps straight ahead; EXPLORE: creep in `explore_step` steps while weaving to sweep the camera, then STOP | v1 |
| R9 | Lighting: shaded cones score low and are dropped | `yellow_conf` 0.5 → 0.3 (shaded yellow cones scored 0.30–0.50); contrast normalisation | gate: v1, CLAHE: later |
| R10 | Far cones overlap in the image → merged boxes, wrong depth | Ignore cones beyond `max_range` (6.5 m); median depth over the box | cap: v1, depth: later |

---

## Results (Gazebo, cones world, scored with Gazebo's true pose and the car's footprint)

| Controller / setting | Laps | Cone contact | Closest gap | Avg speed |
|---|---|---|---|---|
| Pure pursuit, before compensator (0.22–0.6 m/s, lookahead 0.85 m) | 1 | none | +0.23 m | 0.38 m/s |
| **Pure pursuit 1.5 m/s, lookahead 1.8 m (default)** | **1.25** | **none** | **+0.20 m** | **1.03 m/s** |
| Pure pursuit 2.0 m/s, lookahead 2.2 m | 1.4 | 1 cone, lap 2 hairpin | −0.03 m | 1.12 m/s |
| Centreline PID 0.4–1.0 m/s | 1 | none (grazes corner) | +0.00 m | 0.84 m/s |
| Wall follower (walls world, reference) | 2.2 | none | — | 1.34 m/s |

---

## Tuning

- **Weaving in simulation:** check the compensator log (~0.35 s expected); raise `lag_boost` to 4.0, or lower `target_speed`.
- **Cone hit at corner exits:** raise `curvature_speed_gain` or lower `target_speed`.
- **Real car:** no lag setting; start at `target_speed` 0.8 and retune `lookahead_distance` for the real camera's blind zone.

---

## Debugging

| Symptom | Cause / check |
|---|---|
| `yolo_node`: `no frames received yet` forever | Car not spawned (launch log: `spawn_f110_car`), or camera topics not bridged: `ros2 topic hz /camera/color/image_raw` |
| Cone positions shifted sideways | Wrong camera intrinsics: `ros2 topic echo --once /camera/color/camera_info` must show fx≈462, cx=320, cy=240 (set in `f110_car/model.sdf`) |
| Controller warns `waypoint latency ... looks wrong` | A node runs without `use_sim_time` |
| Controller stays in `wait` | Check `ros2 topic echo /waypoints`; the planner readout in RViz shows cone / edge counts |
| Car drives slowly in wall-clock time | Real-time factor: `ign topic -e -t /stats -n 1`. YOLO runs on the CPU; a hot laptop throttles. Driving stays correct (all nodes on sim time) |
| Car "off track" in an odometry plot | `/odom` is wheel odometry and drifts (~1.7 m in a few metres). Use Gazebo's pose (`/world/<world>/dynamic_pose/info`) for evaluation |
| `UserWarning: A NumPy version ...` from scipy | Harmless (system scipy 1.8 with newer numpy) |

---

## Known limitations / next steps

- **PID corner margin**: with the compensator it completes the lap but grazes the top-left corner apex (+0.00 m). The camera sees nothing closer than ~1.4 m, so the target is always ≥ 1.5 m ahead; candidates: curvature-based speed cap from the waypoints (pure pursuit's `curvature_speed_gain` is what made 1.5 m/s safe there), an inside margin on the target.
- **Pure pursuit above 1.5 m/s**: 2.0 m/s completes a lap but touches a cone at the hairpin exit on lap 2.
- **No reverse in recovery**: once pinned against a cone, EXPLORE keeps pushing forward.
- **Latency compensation** (R5, parked): shift waypoints by odometry over the measured detection latency; matters mostly on the real car.
- **No steering at speed 0 in simulation** (`drive_to_cmd_vel` sends yaw rate = 0).
- **Later robustness items**: R3 cone memory (note: needs better odometry than wheel odometry), R6, R9 contrast normalisation, R10 median depth.
- **Real car (`yolo_node`)**: aligned depth (`/camera/aligned_depth_to_color/image_raw`), `16UC1` mm → m conversion, RealSense topic namespace.
