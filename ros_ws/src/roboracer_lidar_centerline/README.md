# RoboRacer LiDAR Centerline and Pure Pursuit

This ROS 2 package provides local path generation from LiDAR measurements and path following using the **Pure Pursuit** algorithm.

The package currently supports **wall-based track centerline generation**. LiDAR measurements are used to detect the left and right boundaries of the track, generate a local centerline, and provide this path to the Pure Pursuit controller.

## Overview

The processing pipeline is:

```text
LiDAR /scan
    ↓
Convert LaserScan to (x, y) points
    ↓
Filter useful points in front of the vehicle
    ↓
Detect left and right wall positions
    ↓
Calculate wall midpoints
    ↓
Generate /local_centerline
    ↓
Pure Pursuit Controller
    ↓
Calculate lookahead target
    ↓
Calculate curvature and steering
    ↓
Publish /cmd_vel
    ↓
Vehicle follows the track
```

## 1. Wall Centerline Generation

The wall centerline node subscribes to the LiDAR topic:

```text
/scan
```

The goal of this node is to estimate the middle of the track from the detected left and right walls.

### LiDAR Point Conversion

The LiDAR provides range measurements at different angles. Each valid measurement is converted into an `(x, y)` point in the vehicle coordinate frame.

In `base_link`:

```text
+x = forward
+y = left
-y = right
```

Only points within the configured forward and lateral detection region are retained.

### Centerline Stations

Several positions are defined in front of the vehicle at fixed intervals, for example:

```text
0.45 m
0.70 m
0.95 m
1.20 m
1.45 m
...
```

These positions are called **x-stations**.

At every station, LiDAR points within a small tolerance around that forward position are considered.

### Left and Right Wall Detection

For every x-station:

- Points with positive `y` are considered candidates for the left wall.
- Points with negative `y` are considered candidates for the right wall.
- The closest suitable point on each side is selected.

For example:

```text
Left wall point  = (2.0, +1.20)
Right wall point = (2.0, -1.40)
```

### Midpoint Calculation

The center between the two detected wall positions is calculated as:

```text
center_y = (left_y + right_y) / 2
```

For the previous example:

```text
center_y = (1.20 + (-1.40)) / 2
         = -0.10 m
```

Therefore, the generated center point is approximately:

```text
(2.0, -0.10)
```

Repeating this calculation at the different x-stations produces a sequence of local center points:

```text
•
  •
    •
       •
          •
```

These points are published as a ROS `nav_msgs/Path` on:

```text
/local_centerline
```

The centerline is continuously regenerated as new LiDAR scans arrive, allowing the path to change as the vehicle moves through the track.

---

## 2. Pure Pursuit Controller

The generated `/local_centerline` is followed using a Pure Pursuit controller.

The Pure Pursuit implementation is based conceptually on:

**Jeff Paulo Fernandez — "Pure Pursuit in ROS Noetic"**

Reference:

https://medium.com/@jefffer705/pure-pursuit-in-ros-noetic-7b2c0a3c36ef

The reference describes Pure Pursuit as a path-tracking controller that selects a goal point ahead of the vehicle and calculates the curvature required for the vehicle to reach that point.

The implementation in this package is adapted for ROS 2 and for the locally generated `base_link` centerline.

### Local Path

The Pure Pursuit node subscribes to:

```text
/local_centerline
```

Because this centerline is already expressed relative to `base_link`, the vehicle can be considered to be at:

```text
(x, y) = (0, 0)
```

with its forward direction along the positive x-axis.

This simplifies the controller because the local centerline is continuously regenerated relative to the current vehicle position.

### Lookahead Target Selection

Pure Pursuit does not steer toward a point directly underneath or immediately in front of the vehicle.

Instead, a **lookahead distance** is defined.

For example:

```text
lookahead_distance = 0.85 m
```

Consider centerline points at distances:

```text
0.45 m
0.70 m
0.96 m
1.20 m
1.45 m
```

The first point that reaches or exceeds the lookahead distance is selected:

```text
Target = 0.96 m
```

Using a point ahead of the vehicle produces smoother path following than continuously steering toward the closest point.

### Target Angle

Once the target `(x, y)` is selected, the angle between the vehicle's forward direction and the target is calculated:

```text
alpha = atan2(target_y, target_x)
```

Therefore:

```text
alpha = 0     → target straight ahead
alpha > 0     → target to the left
alpha < 0     → target to the right
```

### Curvature Calculation

The required path curvature is calculated using:

```text
curvature = 2 * sin(alpha) / lookahead_distance
```

The curvature represents how sharply the vehicle needs to turn:

```text
curvature ≈ 0   → approximately straight
positive        → left turn
negative        → right turn
```

### Steering Calculation

The required curvature is converted into a steering angle using the vehicle wheelbase:

```text
steering_angle = atan(wheelbase * curvature)
```

The RoboRacer wheelbase used in this package is:

```text
wheelbase = 0.32 m
```

The steering command is limited to the configured maximum steering angle.

### Vehicle Speed

Pure Pursuit primarily determines the lateral steering required to follow the path.

The forward velocity is therefore configured separately.

In this implementation, the vehicle is allowed to drive faster on approximately straight sections and slower when the requested curvature increases.

Conceptually:

```text
Low curvature
    ↓
Straight section
    ↓
Higher speed

High curvature
    ↓
Sharp turn
    ↓
Lower speed
```

### Vehicle Command

The resulting control command is published on:

```text
/cmd_vel
```

using `geometry_msgs/Twist`.

The main values are:

```text
linear.x  → forward velocity
angular.z → turning/steering command
```

The vehicle model in Gazebo then converts these commands into vehicle motion.

---

## 3. Continuous Control Loop

The complete system operates continuously:

```text
1. LiDAR produces a new scan
            ↓
2. Wall points are extracted
            ↓
3. Left and right walls are detected
            ↓
4. Their midpoints generate a local centerline
            ↓
5. /local_centerline is published
            ↓
6. Pure Pursuit selects a lookahead target
            ↓
7. Target angle is calculated
            ↓
8. Required curvature is calculated
            ↓
9. Curvature is converted into steering
            ↓
10. Speed and steering are published on /cmd_vel
            ↓
11. Vehicle moves
            ↓
12. LiDAR observes the new vehicle position
            ↓
13. Process repeats
```

This creates a closed-loop local path-following system where the trajectory is continuously updated from the latest LiDAR measurements.

## Package Structure

```text
roboracer_lidar_centerline/
├── config/
│   └── lidar_centerline.yaml
├── launch/
│   └── wall_pure_pursuit.launch.py
├── resource/
│   └── roboracer_lidar_centerline
├── roboracer_lidar_centerline/
│   ├── __init__.py
│   ├── wall_centerline_node.py
│   └── pure_pursuit_node.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

### Main Nodes

`wall_centerline_node.py`

Processes LiDAR measurements, estimates the left and right wall boundaries, calculates their midpoint, and publishes the resulting local centerline.

`pure_pursuit_node.py`

Receives the local centerline, selects a lookahead target, calculates the required curvature and steering command, and publishes the vehicle command.

## Reference

Jeff Paulo Fernandez, **"Pure Pursuit in ROS Noetic"**, Medium, July 29, 2022.

https://medium.com/@jefffer705/pure-pursuit-in-ros-noetic-7b2c0a3c36ef

The reference was used for the conceptual Pure Pursuit sequence: selecting a waypoint/goal using a lookahead distance, calculating the target angle, determining the required curvature, and obtaining the steering command. The implementation in this package is adapted for ROS 2 and a continuously generated local LiDAR centerline.