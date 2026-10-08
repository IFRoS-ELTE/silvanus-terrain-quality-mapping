# Topics and record format

What each node publishes, and the format of the data saved during a run.
Once agreed, nobody renames a topic or a field without telling the others.

Everything below matches the code that already exists. The only change it asks
for is the crack node's input topic.

---

## Rules

- Standard ROS 2 message types only, so there is no shared build step.
- ROS messages use SI units: metres, seconds, m/s. Saved records use mm for
  hazard sizes, because that is how we read and report them.
- Camera topics are BEST_EFFORT. Subscribe with `qos_profile_sensor_data`, or
  the subscriber silently receives nothing.
- On the real robot `/cmd_vel` is plain `Twist`, not `TwistStamped`.

---

## 1. Robot topics

Published by the lab stack. We only read them. Names exactly as in our bags.

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 30 Hz | |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 30 Hz | 16UC1, millimetres. Not aligned to colour |
| `/camera/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | |
| `/imu/data` | `sensor_msgs/Imu` | 10 Hz | frame `xsens_link` |
| `/gnss` | `sensor_msgs/NavSatFix` | 4 Hz | outdoors only |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | wheel odometry |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | | |

---

## 2. Our topics

| Topic | Type | From | To | Status |
|---|---|---|---|---|
| `/ground/hazards` | `vision_msgs/Detection3DArray` | depth node | fusion, governor | exists |
| `/ground/hazard_markers` | `visualization_msgs/MarkerArray` | depth node | RViz, Foxglove | exists |
| `/ground/unevenness` | `std_msgs/Float32` | depth node | fusion | exists |
| `/ground/residual_cloud` | `sensor_msgs/PointCloud2` | depth node | RViz, Foxglove | exists, optional |
| `/crack_detection/detections` | `std_msgs/String` (JSON) | crack node | fusion | exists |
| `/crack_detection/debug_image` | `sensor_msgs/Image` | crack node | viewing | exists |
| `/ground/rgb_terrain` | `std_msgs/String` (JSON) | RGB terrain classifier | fusion | planned |
| `/terrain/imu_roughness` | `std_msgs/Float32` | IMU roughness node | fusion, governor | planned |
| `/terrain/imu_terrain` | `std_msgs/String` (JSON) | IMU terrain classifier | fusion | planned |
| `/terrain/speed_limit` | `std_msgs/Float32` | fusion | governor | planned |
| `/cmd_vel_teleop`, `/cmd_vel_nav` | `geometry_msgs/Twist` | teleop, Nav2 | `twist_mux` | planned |
| `/cmd_vel` | `geometry_msgs/Twist` | governor | robot base | planned |

### `/ground/hazards`

One `Detection3D` per hazard, in the frame of the depth image
(`camera_depth_optical_frame`). Use TF to bring it into `base_link` or `map`.

| Field | Meaning |
|---|---|
| `results[0].hypothesis.class_id` | `"bump"` or `"hole"` |
| `results[0].hypothesis.score` | ground plane inlier fraction, a quality measure for the frame |
| `bbox.center.position` | hazard centre, metres |
| `bbox.size.x`, `.y`, `.z` | width, length, height, metres |

Needs `ros-jazzy-vision-msgs` in the Docker image.

### `/ground/unevenness`

RMS distance of the flat ground to the fitted plane, in **millimetres**. About
4.5 mm on indoor tile, which is the sensor noise floor.

### `/crack_detection/detections`

Pixel boxes in the colour image, as JSON:

```json
{"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "camera_color_optical_frame",
 "count": 1,
 "boxes": [{"x": 120, "y": 300, "width": 40, "height": 12, "area": 380, "score": 0.13}]}
```

**Change needed:** the default input is `/camera/color/image_raw`. On the robot
and in our bags it is `/camera/camera/color/image_raw`.

When cracks are projected to 3D later, they go into `hazards.json` with type
`crack`; this topic stays as it is.

### `/terrain/imu_roughness`

RMS of vertical acceleration with gravity removed, over a 1 s window, in
m/s². Published at 5 Hz. Indoor tile was about 0.23 and garden about 0.39 in
the first bags.

### `/ground/rgb_terrain` and `/terrain/imu_terrain`

```json
{"label": "grass", "confidence": 0.87}
```

Labels are fixed: `tile`, `concrete`, `asphalt`, `grass`, `gravel`, `dirt`.

### `/terrain/speed_limit`

Maximum safe speed in m/s. The governor never drives faster than this.

---

## 3. Record format

The fusion node writes one folder per run, named after the bag:

```
runs/<bag name>/
  records.jsonl     one line per 0.25 m driven
  hazards.json      every hazard, merged
  crops/            one image per hazard, H1.png, H2.png ...
```

### `records.jsonl`

```json
{"t": 1790864150.2, "x": 4.21, "y": 1.05, "speed": 0.30, "imu_roughness": 0.23, "camera_unevenness_mm": 4.5, "terrain_camera": "tile", "terrain_imu": "tile", "hazards": ["H3"]}
```

| Field | Unit | Meaning |
|---|---|---|
| `t` | s | ROS time |
| `x`, `y` | m | robot position in `map` |
| `speed` | m/s | from `/odom` |
| `imu_roughness` | m/s² | latest `/terrain/imu_roughness` |
| `camera_unevenness_mm` | mm | latest `/ground/unevenness` |
| `terrain_camera`, `terrain_imu` | label | from the two classifiers, `null` if not running |
| `hazards` | list | IDs of hazards seen in this stretch |

### `hazards.json`

```json
[{"id": "H3", "type": "bump", "x": 4.21, "y": 1.05,
  "length_mm": 240, "width_mm": 170, "height_mm": 31,
  "imu_peak": 4.8, "crossing_speed": 0.15,
  "class": "C", "seen": 14, "crop": "crops/H3.png"}]
```

| Field | Unit | Meaning |
|---|---|---|
| `id` | | `H1`, `H2` ... in the order first seen |
| `type` | | `bump`, `hole` or `crack` |
| `x`, `y` | m | position in `map` |
| `length_mm`, `width_mm`, `height_mm` | mm | from the camera, at the closest sighting |
| `imu_peak` | m/s² | IMU spike when the wheels crossed it, `null` if not felt |
| `crossing_speed` | m/s | speed at that moment, `null` if not crossed |
| `class` | | `A` seen not felt, `B` felt not seen, `C` both |
| `seen` | | number of frames it was detected in |
| `crop` | | image file |

**Merging:** the same hazard appears in 10 to 30 frames as the robot
approaches. Detections closer than 0.3 m are the same hazard. Average the
position, and take the size from the closest frame.
