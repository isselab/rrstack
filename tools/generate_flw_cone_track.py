#!/usr/bin/env python3
"""
Generate a cone-only version of the original FLW wall track.

Corrections included:
- Single blue boundary only, no duplicate blue line from wall thickness.
- Origin is shifted to the lane midpoint between the yellow and blue boundaries
  closest to the original wall-track origin. This keeps (0,0) at the centre of
  the track spacing, not at the full bounding-box centre.
"""
import math
from pathlib import Path

OUT_WORLD = Path("packages/src/roboracer_gazebo/worlds/flw_cone_track_from_walls.world")
CONE_SPACING = 0.80  # meters. Smaller = more cones, larger = fewer cones.
CENTER_LANE_ORIGIN = True

YELLOW_OUTER_BOUNDARY = [(-0.972648, 4.688479), (-1.478251, 4.937815), (-2.370825, 5.622711), (-11.29976, 14.69378), (-11.85937, 15.18455), (-13.14581, 15.92727), (-14.58065, 16.31174), (-15.80506, 16.32885), (-16.73559, 16.07952), (-17.56988, 15.59784), (-17.9328, 15.27957), (-18.51925, 14.51528), (-18.73275, 14.08236), (-18.98209, 13.15183), (-19.01366, 12.67015), (-19.01366, 10.67015), (-18.98182, 10.15087), (-18.14055, 7.700926), (-18.01366, 6.67015), (-18.01366, 5.670149), (-17.81976, 4.197399), (-17.25131, 2.825015), (-16.34701, 1.646521), (-15.16852, 0.742231), (-13.79614, 0.17377), (-12.32339, -0.020122), (-9.323386, -0.020119), (-8.29261, -0.147013), (-5.842663, -0.988277), (-5.323385, -1.020119), (6.676615, -1.020117), (7.419344, -0.971438), (8.854187, -0.586972), (9.52175, -0.257765), (10.70024, 0.646524), (11.60453, 1.825018), (11.93374, 2.49258), (12.3182, 3.927422), (12.36688, 4.670152), (12.36688, 12.67015), (12.35242, 12.89078), (12.23822, 13.31699), (12.0176, 13.69913), (11.87182, 13.86535), (9.871579, 15.86511), (9.940523, 15.934057), (9.871817, 15.86535), (9.705588, 16.01114), (9.323452, 16.23176), (8.89724, 16.34596), (8.676613, 16.36042), (-1.323388, 16.36042), (-1.760862, 16.30283), (-1.970226, 16.23176), (-2.35236, 16.01113), (-2.66437, 15.69912), (-2.884994, 15.31699), (-2.956064, 15.10762), (-3.013657, 14.67015), (-3.013656, 12.67015), (-2.956062, 12.23268), (-2.884992, 12.02331), (-2.664368, 11.64118), (-2.352358, 11.32917), (-1.970223, 11.10855), (-1.544008, 10.99434), (-1.323385, 10.97988), (2.676614, 10.97988), (2.676615, 10.97988), (2.978095, 10.96012), (3.560511, 10.80406), (3.831482, 10.67043), (4.309841, 10.30338), (4.509047, 10.07623), (4.810526, 9.554047), (4.966585, 8.971632), (4.949474, 8.107621), (4.658287, 7.020889), (4.408951, 6.515285), (3.724054, 5.622711), (2.831481, 4.937815), (1.792055, 4.507271), (1.239147, 4.397291), (0.114082, 4.397291), (-0.972648, 4.688479)]
BLUE_INNER_BOUNDARY_RAW = [(-17.55441, 13.26795), (-17.4573, 13.55405), (-17.15582, 14.07622), (-16.72946, 14.50258), (-16.20728, 14.80406), (-15.62487, 14.96012), (-15.32339, 14.97988), (-14.20795, 14.83303), (-13.16852, 14.40249), (-12.27595, 13.71759), (-3.347013, 4.646523), (-2.787401, 4.155757), (-1.500957, 3.413027), (-0.066115, 3.028561), (1.419345, 3.028561), (2.854187, 3.413028), (3.521749, 3.742233), (4.700243, 4.646523), (5.604532, 5.825017), (5.933739, 6.49258), (6.318203, 7.927422), (6.335314, 9.151827), (6.085981, 10.08236), (5.604303, 10.91664), (4.923109, 11.59784), (4.08882, 12.07952), (3.15829, 12.32885), (2.676614, 12.36042), (-1.323386, 12.36042), (-1.363814, 12.36307), (-1.441915, 12.384), (-1.511938, 12.42442), (-1.569112, 12.4816), (-1.60954, 12.55162), (-1.630466, 12.62972), (-1.633116, 12.67015), (-1.633117, 14.67015), (-1.630468, 14.71058), (-1.609542, 14.78868), (-1.569114, 14.8587), (-1.51194, 14.91588), (-1.441915, 14.9563), (-1.403552, 14.96933), (-1.323388, 14.97988), (8.676613, 14.97988), (8.717043, 14.97723), (8.795144, 14.95631), (8.865168, 14.91588), (8.895627, 14.88917), (8.79209, 14.78563), (8.77603, 14.79971), (8.739109, 14.82103), (8.697932, 14.83206), (8.676613, 14.83346), (-1.323388, 14.83346), (-1.365654, 14.82789), (-1.40504, 14.81158), (-1.438862, 14.78562), (-1.464814, 14.7518), (-1.474262, 14.73264), (-1.485296, 14.69147), (-1.486692, 14.67015), (-1.486692, 12.67015), (-1.485294, 12.64883), (-1.47426, 12.60766), (-1.464812, 12.5885), (-1.43886, 12.55468), (-1.4228, 12.54059), (-1.38588, 12.51928), (-1.365652, 12.51241), (-1.323386, 12.50685), (2.676614, 12.50685), (3.177403, 12.47402), (4.144855, 12.2148), (5.012247, 11.714), (5.720468, 11.00578), (6.221258, 10.13839), (6.480486, 9.170941), (6.463377, 7.90831), (6.069017, 6.436546), (5.73134, 5.751804), (4.803781, 4.542985), (3.594963, 3.615425), (2.187263, 3.032337), (1.438457, 2.883389), (-0.085227, 2.883389), (-0.834033, 3.032337), (-2.241734, 3.615425), (-3.450551, 4.542985), (-12.37948, 13.61405), (-13.24173, 14.27568), (-14.24584, 14.69159), (-15.32339, 14.83346), (-15.60575, 14.81495), (-16.15125, 14.66878), (-16.40504, 14.54363), (-16.85307, 14.19984), (-17.19687, 13.7518), (-17.41298, 13.23005), (-17.48669, 12.67015), (-17.48669, 10.67015), (-17.39844, 9.808533), (-16.57495, 7.531768), (-16.4867, 6.67015), (-16.48669, 5.67015), (-16.34483, 4.592607), (-16.16978, 4.076921), (-15.62636, 3.135689), (-15.26729, 2.726247), (-14.40504, 2.06462), (-13.40093, 1.648704), (-12.32339, 1.506842), (-9.323386, 1.506845), (-8.632361, 1.45029), (-6.185003, 0.595101), (-5.323385, 0.506845), (6.676615, 0.506846), (7.754156, 0.648708), (8.758267, 1.064624), (9.211075, 1.36718), (9.979588, 2.135692), (10.52301, 3.076924), (10.69806, 3.59261), (10.83992, 4.670152), (10.83992, 12.67015), (10.83435, 12.71242), (10.82749, 12.73265), (10.80617, 12.76957), (10.79209, 12.78563), (8.791851, 14.78539), (8.895388, 14.88892), (10.89563, 12.88917), (10.92234, 12.85871), (10.96277, 12.78868), (10.98369, 12.71058), (10.98635, 12.67015), (10.98635, 4.670152), (10.94947, 4.10762), (10.65829, 3.02089), (10.40895, 2.515286), (9.724055, 1.622712), (8.83148, 0.937816), (8.325877, 0.68848), (7.239147, 0.397292), (6.676615, 0.360422), (-5.323385, 0.360421), (-6.354161, 0.487315), (-8.632361, 1.303866), (-9.323386, 1.360421), (-12.32339, 1.360418), (-12.88592, 1.397288), (-13.97265, 1.688477), (-14.47825, 1.937812), (-15.37083, 2.622709), (-16.05572, 3.515283), (-16.48627, 4.554709), (-16.63312, 5.67015), (-16.63312, 6.67015), (-16.76001, 7.700926), (-17.54486, 9.808533), (-17.63312, 10.67015), (-17.63311, 12.67015), (-17.55441, 13.26795)]
BLUE_INNER_BOUNDARY = BLUE_INNER_BOUNDARY_RAW[:49] + BLUE_INNER_BOUNDARY_RAW[130:]


def sample_closed(points, spacing):
    pts = list(points)
    if len(pts) > 1 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-9:
        pts = pts[:-1]

    segs = []
    total = 0.0
    for i, p in enumerate(pts):
        q = pts[(i + 1) % len(pts)]
        length = math.hypot(q[0] - p[0], q[1] - p[1])
        if length > 1e-9:
            segs.append((i, length, total))
            total += length

    n = max(1, int(round(total / spacing)))
    actual_spacing = total / n
    samples = []
    seg_idx = 0

    for k in range(n):
        d = k * actual_spacing
        while seg_idx + 1 < len(segs) and d >= segs[seg_idx][2] + segs[seg_idx][1]:
            seg_idx += 1

        i, length, start = segs[seg_idx]
        p = pts[i]
        q = pts[(i + 1) % len(pts)]
        t = (d - start) / length
        x = p[0] + t * (q[0] - p[0])
        y = p[1] + t * (q[1] - p[1])
        yaw = math.atan2(q[1] - p[1], q[0] - p[0])
        samples.append((x, y, yaw))

    return samples, total, actual_spacing


def compute_lane_origin(yellow_samples, blue_samples):
    best = None
    for yx, yy, _ in yellow_samples:
        for bx, by, _ in blue_samples:
            width = math.hypot(yx - bx, yy - by)
            if 1.0 <= width <= 5.0:
                mx = (yx + bx) / 2.0
                my = (yy + by) / 2.0
                score = math.hypot(mx, my)
                if best is None or score < best[0]:
                    best = (score, width, mx, my)
    if best is None:
        raise RuntimeError("Could not find a valid lane midpoint")
    return best[2], best[3], best[1]


def shift_samples(samples, ox, oy):
    return [(x - ox, y - oy, yaw) for x, y, yaw in samples]


def include_model(name, uri, x, y, z=0.0, yaw=0.0):
    return f"""
    <include>
      <name>{name}</name>
      <uri>{uri}</uri>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>
    </include>"""


WORLD_HEADER = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="flw_cone_track_from_walls_lane_origin">

    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.4</sor>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry><material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material></visual>
      </link>
    </model>
"""


def main():
    yellow_samples, yellow_len, yellow_actual = sample_closed(YELLOW_OUTER_BOUNDARY, CONE_SPACING)
    blue_samples, blue_len, blue_actual = sample_closed(BLUE_INNER_BOUNDARY, CONE_SPACING)

    if CENTER_LANE_ORIGIN:
        ox, oy, lane_width = compute_lane_origin(yellow_samples, blue_samples)
        yellow_samples = shift_samples(yellow_samples, ox, oy)
        blue_samples = shift_samples(blue_samples, ox, oy)
        print(f"Origin shifted to lane midpoint: x={ox:.4f}, y={oy:.4f}, lane width≈{lane_width:.3f} m")

    items = []
    for i, (x, y, yaw) in enumerate(yellow_samples):
        items.append(include_model(f"yellow_cone_outer_{i:03d}", "model://yellow_cone", x, y, 0.0, yaw))
    for i, (x, y, yaw) in enumerate(blue_samples):
        items.append(include_model(f"blue_cone_inner_{i:03d}", "model://blue_cone", x, y, 0.0, yaw))

    world = WORLD_HEADER + "\n".join(items) + """

  </world>
</sdf>
"""
    OUT_WORLD.write_text(world)
    print(f"Wrote: {OUT_WORLD}")
    print(f"Yellow cones: {len(yellow_samples)}")
    print(f"Blue cones: {len(blue_samples)}")


if __name__ == "__main__":
    main()
