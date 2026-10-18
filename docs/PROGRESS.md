# Progress

Terrain quality and hazard mapping with Silvanus. State on 2026-10-08, against
the plan in [MILESTONES.md](MILESTONES.md). Data: [DATA.md](DATA.md).

---

## Milestones

### Week 1, 1-7 October

| Task | State |
|---|---|
| Depth pipeline running as a ROS 2 node | ✅ `ground_demo/ground_hazard_node.py`, checked live and on bags |
| `xsens_link` to `base_link` transform measured and published | ✅ [imu_mount.md](robot/imu_mount.md); IMU also raised from 10 to **100 Hz** |
| First outdoor bag, with a surface log | ✅ 5 outdoor runs; surfaces labelled from the robot camera and phone video |
| Crack model running on real frames | open; cracked asphalt is in `garden_surfaces_1548` and `_1618` |
| Robot body and gripper mask | ✅ depth mask; the colour-image mask is open |
| Topic names and record schema agreed | draft in [schema.md](schema.md), not yet agreed |

### Week 2, 8-14 October

| Task | State |
|---|---|
| Outdoor localization tested | ✅ done offline, results below |
| Indoor map with SLAM Toolbox, AMCL on it | **blocked**: the lidar scan contains the floor (below) |
| IMU roughness node | the feature that works is found (below); node not written |
| Nav2 configured for the Scout | open |
| RGB terrain classifier, first version | open; 5 labelled outdoor surfaces are ready |
| Annotation from our own bags | open |
| **Gate:** robot drives to a goal on a saved map | not yet, needs the scan fixed first |
| Week 3, started early: zigzag coverage sweep | tool written ([navigation/](../navigation/README.md)); in the lab simulator the skid-steer turns at less than half the commanded rate, so Nav2 cannot follow it yet |

---

## 1. Hazard detection from depth

![depth detection](media/crossing_detection.gif)

Left the robot camera, right the height of every point above the fitted floor
(red up, blue down). Green boxes are the tape-measured objects, labelled with the
measured height and the tape height. The clip is 14 s of the slow run; the full
37 s with all five objects: [crossing_detection.mp4](media/crossing_detection.mp4).

Each object on a typical frame, with its median height over all the frames where
it is fully in view (slow run):

![each object, measured vs tape](media/detection_objects.jpg)

Tested against objects measured with a tape, 24 passes over 3 runs:

| object | tape size, L x W x H | true height | measured height |
|---|---|---|---|
| foam block | 25.5 x 20.5 x 5 cm | 50 mm | 47 mm |
| book | 20 x 14 x 3 cm | 30 mm | 26-27 mm |
| foam strip | 94-96 x 20 x 2 cm | 20 mm | 19-21 mm |
| long wooden plank | 111 x 8.2 x 2 cm | 20 mm | 18-20 mm |
| wooden block | 31.5 x 6.5 x 1.5 cm | 15 mm | 15-19 mm |

Sizes were measured with a tape in cm; heights are compared in mm (1 cm = 10 mm)
because the errors are a few mm.

**Mean error 1.7 mm, worst 4.3 mm.** Before the fix in this update it was 4.1 mm
and 9.9 mm: thin objects read high because only the pixels above the 15 mm
threshold were measured. The same object measures the same within 1-2 mm on
every pass and at both speeds. The gripper, which sits in the bottom of every
depth frame, is now masked out; that removed the false bumps and most false
holes. Details: [BRANCH_NOTES.md](../ground_demo/BRANCH_NOTES.md).

## 2. Camera predicts, IMU confirms

![camera vs IMU](media/groundtruth_1447_camera_vs_imu.png)

The camera sees each object about 4 s before the wheels reach it. When the
front wheels and then the rear wheels reach the 50 mm block, at the times
predicted from the camera and odometry, the IMU shows the two largest jolts of
the run. That is the first confirmed hazard.

- At 0.2-0.3 m/s the IMU feels rigid objects from 15 mm (wood).
- It never feels the 20 mm foam: foam squashes under the tyre.
- Above about 0.4 m/s the robot's own shaking on tile hides every object, so the
  IMU can only confirm a hazard when the robot slows down near it.

## 3. Outdoor surfaces

![outdoor surfaces](media/outdoor_surfaces.jpg)

Top: the robot camera. Middle: what the depth camera measures on the same frame,
height above the fitted ground (white flat, red up, blue down). Asphalt and the red path
are smooth with faint stripes from the depth camera itself; wood chips and grass are
patchy. One typical frame per surface; the numbers are the medians over all runs.

| surface | camera unevenness | IMU: fast / slow shaking |
|---|---|---|
| asphalt (plain and cracked) | 3.9 mm | 2.2 |
| red rubber path | 3.6 mm | 0.51 |
| wood chips | 7.5 mm | 0.41 |
| grass / dirt | 6.8 mm | 0.29 |

Camera unevenness is the RMS height of the ground points around the fitted plane, in
each frame. For reference, the flat tile floor indoors reads 3.4-3.7 mm: that is the
depth camera's own noise. So asphalt and the red path are as flat as the camera can
tell, and grass and wood chips are clearly rougher.

Depth works in daylight: 96 % of pixels valid, floor found in every frame.

The planned roughness number, the RMS of vertical acceleration, **does not
separate surfaces**: on all of them it is about 1.0 x speed. What separates them
is which frequencies shake: power at 8-50 Hz over power at 0.5-8 Hz, at matched
speed. Hard asphalt shakes fast, soft ground slow. This needs the 100 Hz IMU; at
the old 10 Hz the 8-50 Hz band could not be seen. Asphalt vs grass from one
threshold: 97 % correct from the camera, 91 % from the IMU.

One whole run, `garden_surfaces_1628`: the path, and both roughness numbers along it.
Both change when the surface changes. The IMU number only counts at 0.25-0.50 m/s
(black dots); at other speeds it is grey.

![outdoor run 1628](media/outdoor_run_1628.png)

The same run as a video, camera and height map with detections on top:
[outdoor_run_1628.mp4](media/outdoor_run_1628.mp4) (90 s).

On grass and wood chips the 15 mm threshold also boxes tufts and leaves as 12-19 mm
bumps, so off pavement the threshold has to follow the surface's unevenness.

The robot outdoors, from a phone: [robot_outdoor_1618.mp4](media/robot_outdoor_1618.mp4).

## 4. Outdoor localization

| option | result |
|---|---|
| GNSS | a fix in 3 of 5 runs, about +-7-10 m. Too coarse for hazard positions |
| Wheel odometry | misjudges turns by 15-30 deg per run (skid steer) |
| **Wheel speed + IMU heading** | **best now**: an indoor out-and-back of 26 m ends 0.4-0.7 m from the start, against 4.7 m for wheels alone |
| Lidar SLAM (SLAM Toolbox, lab config) | **fails indoors and outdoors** with the current scan, see below |

![wheels vs IMU heading](media/heading_wheels_vs_imu.png)

The same in ROS with `robot_localization` ([config/ekf_odom_imu.yaml](../config/ekf_odom_imu.yaml):
forward speed from the wheels, turn rate from the IMU gyro): the 26 m out-and-back
ends 0.70 m from the start (wheels 4.74 m), and the 35 m outdoor loop 0.37 m
(wheels 0.97 m). To run it live, `scout_base` must stop publishing its own
`odom -> base_footprint`, which the lab launch does not allow yet; until then the
EKF publishes `/odometry/filtered` only.

**Why SLAM fails.** `/scan` is made from the 3D lidar using all points between
-0.10 and +0.60 m above the floor, so it includes the floor. The lidar's lowest
beam hits the floor 1.4 m away, and the scan keeps the closest point in each
direction, so indoors 80 % of the scan is a ring of floor around the robot and
everything behind it is lost.

![floor ring in the scan](media/scan_floor_ring.png)

The ring moves with the robot and looks the same everywhere, so SLAM decides
the robot is not moving. Removing the ring afterwards does not help, because the
real walls and trees were already dropped when the scan was made. Nav2 would see
the same ring as an obstacle.

![SLAM maps](media/slam_maps.png)

Fix: raise `min_height` in the lab bring-up to about +0.10 m indoors (+0.15 m
outdoors, for grass and tilt), and record `/rslidar_points` so scans can be
remade offline. Then repeat the SLAM test.

## 5. Everything on one map

[tools/analysis/hazard_map.py](../tools/analysis/hazard_map.py) runs the whole chain
offline on one bag and writes the map, `hazards.json` and `records.jsonl` in the
format of [schema.md](schema.md) (example: [results/hazard_map_1628](results/hazard_map_1628/)):

- **position**: wheel speed + IMU heading (section 4)
- **bumps and holes**: the depth detection of section 1, placed on the ground with the
  camera pose from the bag's TF (0.93 m ahead of the robot centre)
- **cracks**: Yamin's crack U-Net v1 on the colour image, three times a second (75 ms
  per frame on a laptop CPU); the crack pixels are put on the ground through the depth image
- **roughness**: camera unevenness along the path (grey: light smooth, dark rough)
- **class**: a hazard is **felt** when a wheel crosses it and the IMU jolt at that moment
  stands out from the shaking around it. A = seen only, B = felt only, C = seen and felt

![hazard map indoors](media/hazard_map_1447.png)

On the tape-measured run all five objects are found. The 50 mm foam block (H2) and a
15 mm wooden block (H6) are **seen and felt (C)**; the 20 mm foam strips are seen
only (A), because foam squashes under the tyre. The four marks at y = 0.8-1.0 m are
the table legs beside the corridor, never driven over. No felt-only (B) hazards.

![hazard map outdoors](media/hazard_map_1628.png)

Outdoors (1628, 28 m): the raised edges of the red rubber path (58-80 mm), the kerb
(H76, 74 mm), a few real bumps on the wood chips, and the cracks along the asphalt at
the end. Bumps and holes under 30 mm on rough ground (tufts, chips) are drawn faint.
The 35 m loop is in [hazard_map_1548.png](media/hazard_map_1548.png).

What was needed to get there:

| problem | fix |
|---|---|
| A low hazard touching a wall or door frame was dropped with it (300 mm cap) | anything over 300 mm is cut out before clustering; ground-truth detections unchanged |
| On wood chips under half the points lie on one plane, giving half-image "holes" | no hazards when the plane has under 50 % inliers (42 % of wood chip frames, 0-3 % elsewhere); unevenness is still reported |
| Fixed 15 mm threshold boxed tufts and chips | threshold = max(15 mm, 3 x the frame's unevenness): unchanged on tile and asphalt, about 21 mm on grass. 1628: holes 57 -> 21 |
| The crack model marks every tile grout line | long straight lines (100 px and more) are dropped: tile keeps 3-5 % of its "cracks", the asphalt cracks 97-100 % |
| Kerb and door edges marked as cracks | crack pixels on a height step are dropped |
| One jolt credited to every tuft next to the wheel | one jolt goes to the tallest hazard the wheel crossed |

Limits of this first version:

- The felt test is calibrated at 0.2 m/s on 1447 (plain tile up to 0.65 m/s², foam 0.63-0.69,
  wood 0.81, block 1.1-1.9). At 0.4 m/s in `crossing_slow_1652` the foam strips also come out
  as felt; the threshold has to depend on speed.
- The felt-only (B) marks on asphalt (3 in 1628, 6 in 1548) are not checked against
  anything; they may be crack edges or just the asphalt's fast shaking.
- Positions drift with the wheel + IMU path, 0.4-0.7 m over 26 m (section 4).
- `garden_surfaces_1536` (door step, drain grate) has no odometry, so it cannot be mapped.

---

## Problems found

| problem | effect | next step |
|---|---|---|
| `/scan` contains the floor | SLAM and Nav2 unusable | change `min_height` in the lab bring-up |
| Depth threshold outdoors | tufts and leaves on grass and wood chips boxed as 12-19 mm bumps | done: threshold follows the unevenness (section 5), also in the node (`rough_factor`) |
| Crack model on tile | every grout line is a crack | straight-line filter in the map; the real fix is tile frames as negatives in the fine-tune |
| Felt test | calibrated at 0.2 m/s only | speed-dependent threshold from more crossing runs |
| D435 drops off USB | 5 times on 2026-10-08, after fast or rough driving | secure the cable; report to the lab |
| Wheel odometry heading | 15-30 deg error per run | fuse IMU heading with `robot_localization` |
| GNSS | +-7-10 m, no fix in 2 of 5 runs | use for rough placement only |

## Next

1. Fix the scan, repeat SLAM indoors and outdoors, build and save the indoor map.
2. EKF live on the robot (config ready, tested offline), then Nav2 to a goal on the saved map.
3. IMU roughness node with the frequency-band feature.
4. Crack model fine-tuned on our frames with tile as negatives; terrain classifier on the outdoor bags.
5. The hazard map live on the robot: the same chain as a node, with a speed-dependent felt test.

Scripts for every figure: [tools/analysis](../tools/analysis/README.md).
