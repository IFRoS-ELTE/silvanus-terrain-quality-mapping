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
