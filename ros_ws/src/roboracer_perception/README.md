# roboracer_perception

Cone perception for the RoboRacer: YOLO detects blue/yellow cones in the RGB image, the depth image gives their distance, and each cone is published as a 2D position in front of the car.

---

## Nodes

| Node | Status | Purpose |
|---|---|---|
| `yolo_node` | Working | Cone detection + 3D position from RGB-D |
| `delaunay_node` | Stub | Will build track waypoints from cone positions (Delaunay triangulation); currently only subscribes |

---

## Run

```bash
ros2 launch roboracer_gazebo gazebo.launch.py world:=cones     # terminal 1
ros2 run roboracer_perception yolo_node                        # terminal 2
ros2 topic echo /cones/blue --once                             # terminal 3
```

Every 5 s, `yolo_node` logs a detection report: frames processed, share of frames with detections, and confidence per class.

---

## Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribed | `/camera/color/image_raw` | `sensor_msgs/Image` |
| Subscribed | `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1, metres) |
| Subscribed | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| Published | `/cones/blue` | `geometry_msgs/PoseArray` |
| Published | `/cones/yellow` | `geometry_msgs/PoseArray` |

Cone positions are in the car's convention: **x forward, y left** (metres).

---

## How a cone position is computed

1. RGB and depth images are synchronised by timestamp.
2. YOLO returns a bounding box per cone; its centre pixel `(u, v)` is used.
3. The depth image gives the distance `z` at `(u, v)`.
4. Back-projection with the camera intrinsics: `x = (u - cx) · z / fx`.
5. Published as forward = `z`, left = `-x`.

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/camera/color/image_raw` | RGB input |
| `depth_topic` | `/camera/depth/image_raw` | Depth input |
| `camera_info_topic` | `/camera/color/camera_info` | Intrinsics |
| `model_path` | `models/best.pt` (installed share dir) | YOLO weights |
| `confidence_threshold` | `0.05` | YOLO inference threshold |
| `blue_conf` / `yellow_conf` | `0.3` / `0.5` | Minimum confidence to publish a cone |
| `iou_threshold` | `0.4` | Non-maximum suppression |
| `max_depth` | `10.0` | Cones further away are ignored (m) |

---

## Dependencies

Python: `ultralytics` (installs `torch`), see `requirements.txt`:

```bash
pip install -r requirements.txt
```

---

## Known limitations

- The message header keeps the camera optical frame, but positions are already in car axes (x forward, y left).
- Orange cones are detected but not published.
- On the real RealSense, the default topics are `/camera/camera/color/...`; pass them with `--ros-args -p image_topic:=...`.