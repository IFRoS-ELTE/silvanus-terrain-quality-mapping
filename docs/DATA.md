# Recorded data

Rosbags are **not** in this repository. GitHub rejects files over 100 MB and our
bags are roughly 1.7 GB each, so they live on the shared drive.

## Shared drive

https://drive.google.com/drive/folders/1jg45KSiKPr-DoeFKEp3v48fnMJFQSeWJ?usp=drive_link

## What is there

| Bag | Length | Where | Contents |
|---|---|---|---|
| `corridor_book_1338_indoor` | 33 s | indoor, tile floor | plain floor, nothing placed |
| `corridor_book_1434_indoor_books` | 33 s | indoor, tile floor | books placed as bumps, driven over |
| `corridor_groundtruth_1447` | 40 s | indoor, tile floor | tape-measured objects, see below. IMU at 100 Hz, `/filter/free_acceleration` included |

Both recorded 2026-10-01. **Every topic at 100% of its nominal rate, no dropped
messages.**

| Topic | rate |
|---|---|
| `/camera/camera/color/image_raw` | 30 Hz |
| `/camera/camera/depth/image_rect_raw` | 30 Hz |
| `/imu/data` | 10 Hz |
| `/odom` | 50 Hz |
| `/joint_states` | 150 Hz |
| `/scan` | 10 Hz |
| `/tf`, `/tf_static` | |

An outdoor run was attempted the same day but the camera dropped off USB part
way through, so it holds IMU, odometry and GNSS but no images. Not included.

## Playing a bag

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play <bag folder> --loop
```

Your nodes then receive the topics exactly as if the robot were live: same
names, same types, same timing. Several people can replay the same bag at once,
in different places. Only the person recording needs the robot.

## Pulling frames out for annotation

```bash
python3 tools/bag_to_images.py <bag folder> --out frames/ --fps 1 --depth
```

1000 colour frames in a 33 s bag become 34 at 1 fps. Consecutive frames are
nearly identical, so annotating all of them wastes effort and inflates any
score computed on them.

`frames/index.csv` records which bag and which timestamp each image came from.
**Keep it.** The dataset must be split by recording session, never by frame:
two frames half a second apart show the same object from almost the same angle,
so splitting by frame puts the same object in train and test.

## Notes on these bags

**Tile grout lines are excellent hard negatives for crack detection.** They are
dark, thin and linear, which is exactly what a threshold detector gets wrong.
Real ones from our own floor are worth more than additional public crack images.

**The gripper and robot body appear at the bottom of every frame.** A fixed mask
is needed before any detector runs on these images.

**Depth analysis of the books bag** (see `ground_demo/BRANCH_NOTES.md`):
ground plane found in every frame, 92 to 98 % inliers, books detected at
24 to 33 mm and a taller stack at 86 to 119 mm, unevenness 4.0 to 4.9 mm RMS on
flat tile.

## Ground truth: `corridor_groundtruth_1447`

Recorded 2026-10-08. Straight run of 8 m at about 0.21 m/s. Every topic complete:
camera 30 Hz, `/imu/data` and `/filter/free_acceleration` 100 Hz, odometry 50 Hz.

| Object | Length x width x height (cm) | Wheels over it |
|---|---|---|
| small black foam block | 25.5 x 20.5 x **5.0** | right side |
| long black foam strip 1 | 94 x 20 x **2.0** | right side |
| wooden block 1 | 31.5 x 6.5 x **1.5** | right side, at the tyre edge |
| wooden block 2 | 31.5 x 6.5 x **1.5** | passed between the wheels |
| long black foam strip 2 | 96 x 20 x **2.0** | both sides |

Objects about 0.6 to 0.7 m to the left of the path are table legs, not ground
truth. Results are in `ground_demo/BRANCH_NOTES.md`.

**IMU timestamps arrive in bursts** (74 % of messages less than 2 ms apart, though
the rate is exactly 100 Hz). Use message order, not header stamps, for spacing.

## Outdoor runs, 2026-10-08

Recorded in the park beside the tennis courts. About 90 s each. Camera 30 Hz, IMU and
`/filter/free_acceleration` 100 Hz, odometry 50 Hz unless stated.

Surface labels, in seconds from the start of each bag. They were read from the robot's
camera and checked against phone videos. The phone clock runs **239 s behind the robot**.

| Bag | Surfaces (bag time, s) | Notes |
|---|---|---|
| `garden_surfaces_1536` | tile 0-5, door threshold ~33, metal drain grate 43-48, smooth concrete, parked 60-90 | driving out of the building. **No odometry**, no GNSS fix |
| `garden_surfaces_1548` | mossy concrete 2.5-6.5 (asphalt 4.5-10), grass/dirt 12-55, cracked asphalt 57-72, concrete patch ~73, asphalt 76-87 | complete. GNSS fix, about +-10 m |
| `garden_surfaces_1607` | cracked asphalt 0-6, grass/dirt 8-34, kerb 34.7-35.7, red rubber path 36-44, dirt with leaves 46-62, asphalt 63-70, grass/dirt 72-87 | **camera stops at 43.8 s**; labels after that from the phone video. Driven up to 1.2 m/s |
| `garden_surfaces_1618` | cracked asphalt 0-36 (slow), grass/dirt 38-60, red rubber path 62-75, wood chips/leaves 77-89 | complete. No GNSS fix |
| `garden_surfaces_1628` | red rubber path 0-21, kerb 22-26, wood chips/leaves 27-45, dirt with sparse grass 46-62.5, concrete patch from 62.7, asphalt 66.8-89 | complete. GNSS fix, about +-7 m |

Leave about 2 s out around each boundary; the labels are good to a few seconds.
Cracked asphalt for crack detection: `1548` 57-72 s, `1618` 0-36 s.

What these runs showed (details in `ground_demo/BRANCH_NOTES.md` once written up):

- Depth works in daylight: 96 % valid, ground plane found in every frame.
- Camera unevenness: asphalt 3.9, red path 3.6, grass/dirt 6.8, wood chips 7.5 mm.
- **IMU RMS does not separate surfaces**: it is about 1.0 x speed on all of them.
  The **ratio of 8-50 Hz to 0.5-8 Hz power does**: asphalt 2.2, red path 0.51,
  wood chips 0.41, grass/dirt 0.29, at 0.25-0.5 m/s. It needs the 100 Hz IMU.
- GNSS: a standard fix at best, 7-10 m, none in 2 of 5 runs. Not usable for hazard
  positions.

## Indoor crossing runs, 2026-10-08

Rigid and soft objects in a line, driven over and back. Objects, in order from the
start: foam block 25.5 x 20.5 x **5** cm, foam strip 94 x 20 x **2** cm, wooden block
31.5 x 6.5 x **1.5** cm, foam strip 96 x 20 x **2** cm, wooden block, book 20 x 14 x **3** cm,
long wooden plank 111 x 8.2 x **2** cm.

| Bag | Notes |
|---|---|
| `crossing_slow_1652` | 90 s, complete. About 0.26 m/s: out 0-52 s, turn, back 62-89 s |
| `crossing_fast_1703` | 59 s. About 0.6 m/s, **camera stops at 32 s**. IMU and odometry complete |

Camera heights repeat within 1-2 mm across passes and speeds. At 0.2-0.3 m/s the IMU
feels the rigid 15-20 mm wood; it does not feel the foam. Above about 0.4 m/s the
robot's own shaking on tile hides every object.

## Still to collect

- **Labelled outdoor runs** — pavement, grass, gravel, dirt, slope. Each needs a
  written log of times and surface types, otherwise the data cannot be used to
  train a terrain classifier.
- Repeat runs of the same route, for repeatability and change detection.
- A dedicated run over one stretch at three speeds, for the speed dependence
  experiment.

## Naming

```
<where>_<what>_<HHMM>
corridor_book_1434_indoor_books
garden_grass_1030
```

Add one line to this file per bag: date, route, speed, arm pose, what was on the
floor, lighting.
