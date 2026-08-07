# RoboRacer LiDAR Centerline v2

Separated nodes:
- `wall_centerline_node.py`: `/scan` -> `/local_centerline`
- `cone_centerline_node.py`: `/scan` -> `/local_centerline`
- `pure_pursuit_node.py`: `/local_centerline` + `/odom` -> `/cmd_vel`

Wheelbase is 0.32 m. Default target speed is 0.45 m/s.

## Build
```bash
cd ~/rrstack
colcon build --symlink-install --base-paths packages/src --packages-select roboracer_lidar_centerline
source install/setup.bash
```

## Run walls
```bash
ros2 launch roboracer_lidar_centerline wall_pure_pursuit.launch.py
```

## Run cones
```bash
ros2 launch roboracer_lidar_centerline cone_pure_pursuit.launch.py
```

## RViz displays
Set Fixed Frame to `odom` and add:
- Path `/pure_pursuit/trajectory`
- Path `/local_centerline` (works through TF from base_link)
- Marker `/pure_pursuit/target_marker`
- Marker `/pure_pursuit/lookahead_line`
- MarkerArray `/wall_centerline/markers` or `/cone_centerline/markers`
- LaserScan `/scan`
