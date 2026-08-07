# roboracer_wallfollowing

A PID wall-following controller for the RoboRacer simulation.

The vehicle keeps a fixed lateral distance from one track wall using three LiDAR beams and a PID controller. The wall side is selectable at launch, and the controller publishes RViz markers so the estimated wall geometry can be inspected while driving.

This package is intended as a classical-control reference: the complete behaviour is produced by geometry and a PID loop, without any learned component.

---

## Background: the F1TENTH two-beam method

The starting point for this package is the wall-following method described in the F1TENTH Lab 3 material.

Reference: https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/lab3.html

That method uses **two** LiDAR beams:

- `b` → fired perpendicular to the vehicle
- `a` → fired at an angle `theta` ahead of `b`

From those two ranges it computes:

```text
alpha = arctan( (a * cos(theta) - b) / (a * sin(theta)) )
Dt    = b * cos(alpha)
```

where

- `alpha` → the heading error, the angle between the vehicle and the wall
- `Dt` → the perpendicular distance from the vehicle to the wall

`Dt` is then compared against a desired distance, and the difference is fed to a PID controller.

---

## How this package extends it

The two quantities being estimated are the same. `alpha` and `Dt` mean exactly what they mean in Lab 3, and the relation `Dt = b * cos(alpha)` still holds here.

The difference is which beams produce them.

| | F1TENTH Lab 3 | This package |
|---|---|---|
| Beams used | 2 → `a`, `b` | 3 → `f`, `b`, `r` |
| Beams that build the wall estimate | `a` and `b` | `f` and `r` |
| Where `Dt` comes from | `b * cos(alpha)` | the fitted wall line |
| Is `b` used in the result? | yes | no |
| Beam positions | both on one side of the perpendicular | one ahead, one behind it |
| Can the estimate be checked? | no | yes, using `b` |

The three beams form a **V straddling the perpendicular**:

| Beam | Angle from vehicle heading | Role |
|---|---|---|
| `f` | `90° - theta_front` | forward arm, builds the wall estimate |
| `b` | `90°` | perpendicular, used only as a check |
| `r` | `90° + theta_rear` | rear arm, builds the wall estimate |

Angles are mirrored when the right wall is followed.

Two consequences follow from this arrangement:

- `b` is not needed for the answer → it is free to test the answer
- the perpendicular direction lies **between** the two samples rather than at the edge of them → the estimate is interpolated rather than extrapolated

---

## How the estimate is built

Step by step, once per scan:

**1. Pick three beams out of the scan**

- desired angle → array index → range
- reject the reading if it is `NaN`, `inf`, below `min_lidar_range`, or above `max_lidar_range`

**2. Turn ranges into points**

- a range alone is only a length
- range + angle → a point on the wall
- gives `F`, `B`, `R`

**3. Fit the wall from `f` and `r` only**

- two points → one straight line
- that line is the wall estimate
- `B` takes no part in this

**4. Read `alpha` off the line**

- the line's direction relative to the vehicle heading

```text
alpha = arctan( (f * cos(theta_front) - r * cos(theta_rear))
              / (f * sin(theta_front) + r * sin(theta_rear)) )
```

**5. Read `Dt` off the line**

The LiDAR, `F` and `R` form a triangle:

- sides `f` and `r` meet at the LiDAR with an included angle `psi = theta_front + theta_rear`
- the line `FR` is the triangle's base
- `Dt` is the triangle's **height**, measured from the LiDAR to that base

Writing the triangle area two ways gives a closed form:

```text
                f * r * sin(psi)
Dt = ------------------------------------------
     sqrt( f^2 + r^2 - 2 * f * r * cos(psi) )
```

The denominator is the law of cosines applied to the distance between the two wall points.

Neither `alpha` nor `Dt` uses `b`.

---

## Checking the estimate with `b`

Two points always produce a line — whether or not those two points belong to the same wall. Nothing in the arithmetic reveals a bad fit, so an outside opinion is needed. That is what `b` provides.

**1. Predict what `b` should read**

- if the fitted line really is the wall
- then a beam fired perpendicular would travel this far before meeting it

```text
b_predicted = Dt / cos(alpha)
```

**2. Compare against what `b` actually measured**

```text
residual = | b_measured - b_predicted |
```

**3. Act on the result**

- small residual → all three beams lie on one straight wall → trust the estimate
- large residual → they do not → discard it and enter recovery

Typical track curvature produces a residual of a few centimetres. A large residual usually means an arm has passed a corner and landed on a different surface. The default rejection threshold is `0.35 m`.

This check is only possible because `b` was excluded from the fit. A beam that helped produce an answer cannot be used to test that same answer.

---

## Corrections applied before control

Two adjustments are made to `Dt` before it reaches the controller.

**Sensor offset** — the LiDAR sits ahead of `base_link`, so the measurement is taken at the sensor rather than at the vehicle reference point:

```text
Dt_centre = Dt - lidar_forward_offset * sin(alpha)
```

**Look-ahead** — the controller acts on the distance the vehicle will have a short way ahead rather than the distance it has now. The look-ahead grows with speed, which is the only use made of odometry:

```text
L       = base_lookahead + lookahead_speed_gain * speed
Dt_proj = Dt_centre + L * sin(alpha)
```

---

## Control law

The controlled quantity is the difference between the requested wall distance and the projected distance:

```text
error = desired_distance - Dt_proj
```

A positive error means the vehicle is closer to the wall than requested.

The error is passed through a PID controller, and the result is converted into a steering command. The sign of the conversion depends on which wall is being followed, because steering away from a left wall and steering away from a right wall are opposite actions:

```text
steering = -wall_sign * PID(error)
```

where `wall_sign` is `+1` for the left wall and `-1` for the right. This single sign term is the whole difference between the two sides; the geometry above is identical for both.

The integral gain is zero by default. Through a sustained corner the error does not settle at zero, so an integral term accumulates and overshoots on corner exit.

The steering command is clamped to `max_steering_angle` and then rate limited, so it cannot change faster than `max_steering_rate`. Speed is reduced linearly with steering magnitude, from `nominal_speed` when travelling straight to `min_speed` at full lock.

### Recovery

The controller does not hold its last command when the geometry is unusable. If either arm is rejected, or the residual exceeds the threshold, the node reduces speed to `recovery_speed`, steers away from the wall, and logs a warning.

---

## Build

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash

colcon build \
    --symlink-install \
    --packages-select roboracer_wallfollowing

source install/setup.bash
```

## Launch

The launch file starts Gazebo, spawns the vehicle, and starts the controller.

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash

LIBGL_ALWAYS_SOFTWARE=1 ros2 launch roboracer_wallfollowing wall_follow.launch.py
```

To follow the left wall instead of the default:

```bash
ros2 launch roboracer_wallfollowing wall_follow.launch.py side:=left
```

To use a different world:

```bash
ros2 launch roboracer_wallfollowing wall_follow.launch.py \
    world:=$(ros2 pkg prefix roboracer_gazebo)/share/roboracer_gazebo/worlds/flw_cone_track_from_walls.world
```

Launch arguments are `side`, `world`, `x`, `y`, `z`, and `yaw`.

---

## Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribed | `/scan` | `sensor_msgs/LaserScan` |
| Subscribed | `/odom` | `nav_msgs/Odometry` |
| Published | `/cmd_vel` | `geometry_msgs/Twist` |
| Published | `/wall_follow/markers` | `visualization_msgs/MarkerArray` |

Odometry supplies vehicle speed only, which scales the look-ahead distance.

---

## Parameters

Defaults are declared in the node and can be overridden in `config/wall_follow_params.yaml` or on the command line.

### Wall selection and geometry

| Parameter | Default | Description |
|---|---|---|
| `side` | `right` | Wall to follow, `left` or `right` |
| `theta_front_deg` | `35.0` | Forward arm angle from the perpendicular |
| `theta_rear_deg` | `35.0` | Rear arm angle from the perpendicular |
| `desired_distance` | `0.6` | Requested distance from the wall, in metres |
| `lidar_forward_offset` | `0.37` | LiDAR offset ahead of `base_link`, in metres |

The scan covers `±135°`. The rear arm sits at `90° + theta_rear`, so `theta_rear_deg` must remain below `45`. The node logs a warning otherwise.

### Look-ahead

| Parameter | Default | Description |
|---|---|---|
| `base_lookahead` | `0.4` | Fixed look-ahead distance, in metres |
| `lookahead_speed_gain` | `0.2` | Additional look-ahead per m/s of speed |

### PID

| Parameter | Default | Description |
|---|---|---|
| `kp` | `1.2` | Proportional gain |
| `ki` | `0.0` | Integral gain, disabled by default |
| `kd` | `0.15` | Derivative gain |
| `max_d_term` | `0.3` | Limit on the derivative contribution |

### Actuation limits

| Parameter | Default | Description |
|---|---|---|
| `max_steering_angle` | `0.4` | Steering limit, in radians |
| `max_steering_rate` | `4.0` | Maximum steering change, in rad/s |
| `nominal_speed` | `1.2` | Speed when travelling straight, in m/s |
| `min_speed` | `0.4` | Speed at full steering, in m/s |
| `recovery_speed` | `0.3` | Speed during recovery, in m/s |

### Beam validity

| Parameter | Default | Description |
|---|---|---|
| `min_lidar_range` | `0.25` | Readings below this are rejected |
| `max_lidar_range` | `10.0` | Readings above this are rejected |
| `residual_threshold` | `0.35` | Maximum accepted disagreement of `b` |

The simulated LiDAR has a minimum range of `0.2 m`. The default rejection floor sits above it so that readings taken inside the sensor blind zone are not used.

### Node

| Parameter | Default | Description |
|---|---|---|
| `control_period` | `0.02` | Control loop period, giving 50 Hz |
| `publish_markers` | `true` | Publish RViz debug markers |
| `scan_topic` | `/scan` | Scan input topic |
| `odom_topic` | `/odom` | Odometry input topic |
| `cmd_vel_topic` | `/cmd_vel` | Command output topic |
| `marker_topic` | `/wall_follow/markers` | Marker output topic |

---

## Visual debugging

Start RViz alongside the simulation:

```bash
cd ~/project_repo/roboracer_state_estimation/packages

source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2
```

Set the fixed frame and add the marker display:

```text
Fixed Frame: laser
```

| RViz display | ROS topic |
|---|---|
| MarkerArray | `/wall_follow/markers` |

The markers show:

| Marker | Meaning |
|---|---|
| Green lines and points | The three beams and where they meet the wall |
| Red line | The fitted wall line |
| Orange line | `Dt`, perpendicular to the fitted wall |
| Blue point | `base_link`, behind the LiDAR |
| Text | `Dt`, `alpha`, error, steering, and residual |

The text readout is the quickest way to see what the controller believes. If the red line leaves the visible wall, or the residual rises sharply, the two arms are no longer on the same surface.

---

## Inspect the controller

Check that both inputs are being received:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
```

Check the commands being produced:

```bash
ros2 topic echo /cmd_vel
ros2 topic hz /cmd_vel
```

Read the parameters in use:

```bash
ros2 param list /wall_follow_pid_node
ros2 param get /wall_follow_pid_node side
ros2 param get /wall_follow_pid_node desired_distance
```

Recovery events are logged by the node:

```text
recovery: arm beam invalid or too close
recovery: arms disagree (residual 0.40 m)
```

Occasional warnings at corners are expected. Continuous warnings indicate that the arm angles or the requested distance do not suit the track.

---

## Tuning notes

The default gains are starting values rather than tuned ones. A conventional manual procedure is to set `ki` and `kd` to zero, raise `kp` until the vehicle oscillates, halve it, and then add `kd` until the oscillation settles. `ki` is normally left at zero for this application.

`desired_distance` should suit the track. The wall track used here has a lane width of roughly `1.25 m` to `1.48 m`, so the centre line lies near `0.7 m` from either wall and the default of `0.6 m` places the vehicle slightly toward the followed wall.

Wider arms give a longer baseline and a less noisy heading estimate, but they also make it more likely that one arm meets a different surface at a corner. Narrow arms behave in the opposite way. The symmetric default of `35°` is a compromise.
