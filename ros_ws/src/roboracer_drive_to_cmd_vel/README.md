# roboracer_drive_to_cmd_vel

**Simulation only.** Glue between the controllers (which speak the real-car
interface, `/drive`) and Ignition Gazebo (which reads `/cmd_vel`).

```
controller --/drive--> steering_lag_compensator --/drive_sim--> drive_to_cmd_vel --/cmd_vel--> Gazebo
                                ^ /joint_states (steering-hinge angles)
```

| Node | Purpose |
|---|---|
| `steering_lag_compensator` | Removes Ignition's steering lag so controllers need no sim-specific code |
| `drive_to_cmd_vel` | `AckermannDriveStamped` → `Twist` (yaw rate = v·tan(δ)/L) |

Started automatically by `cone_drive.launch.py` (roboracer_planning) and
`wall_follow.launch.py` (roboracer_wallfollowing). Other launches use
`drive_to_cmd_vel` directly on `/drive`.

---

## Why the compensator exists

Ignition Fortress' `AckermannSteering` plugin moves the steering hinges towards the
commanded angle as a **first-order lag with τ ≈ 1.05 s**. A real servo reacts in
~0.1–0.2 s. Measured with step tests in an empty world (`/joint_states`, ~735 Hz):

| Measurement | Result |
|---|---|
| Time to 63 % / 90 % of a step | 1.01–1.10 s / 2.2–2.4 s (first-order: t90 ≈ 2.3 τ) |
| Speed 0.5 vs 1.0 m/s, steps 0.15 / 0.3 / 0.5 rad | same τ → **one value for all controllers** |
| Dead time | 0.06–0.15 s (small) |
| `steer_p_gain` 10 vs 100 in `model.sdf` | no change: **Fortress has no such parameter**, so the lag cannot be tuned in the SDF |
| Speed 0 | wheels do not steer at all (`drive_to_cmd_vel` sends yaw rate = 0) |

## How it compensates

Closed loop on the **measured** wheel angle δ (mean of left/right steering hinge):

```
cmd = δ + boost · (wanted − δ)        clamped to ±0.6458 rad (steering_limit)
```

The wheels then follow like a lag of τ / boost; the extra command fades to zero as
they arrive, so they settle exactly at `wanted`.

| Step 0.15 rad | 63 % | 90 % | Final angle |
|---|---|---|---|
| no compensation | 1.02 s | 2.30 s | still settling after 2.5 s |
| boost 3 (default) | 0.39 s | 0.80 s | exact, no overshoot |
| boost 5 | 0.29 s | 0.52 s | exact; larger steps limited by the steering clamp |

## Simulation vs real car — no launch argument needed

`enabled` defaults to **`auto`**: compensate only when the node runs on the
simulation clock (`use_sim_time: true`, set by every sim launch). Without it
(real car) the node is a plain passthrough. `true` / `false` force it.

## Parameters

| Parameter | Default | |
|---|---|---|
| `enabled` | `auto` | `auto` / `true` / `false` |
| `boost` | `3.0` | effective lag = `tau / boost` |
| `tau` | `1.05` | measured plant lag [s] (used for the log's expected value) |
| `steering_limit` | `0.6458` | must match `<steering_limit>` in `f110_car/model.sdf` |
| `joint_states_timeout` | `0.2` | older measurement → pass through (never boost blind) |
| `input_topic` / `output_topic` | `/drive` / `/drive_sim` | |

## Live check

Every 10 s the node logs `effective steering lag X s (expected ~Y s)`, a least-squares
fit of how fast the measured angle follows the controller's wanted angle. Values a
bit above `tau / boost` are normal when the command saturates at the steering limit
(large steps); a value near `tau` while enabled means the compensation is not acting.

To re-measure the plant lag: launch with `lag_comp:=false` (cone_drive) or run the
node with `-p enabled:=false`; the log then reports the raw lag.
