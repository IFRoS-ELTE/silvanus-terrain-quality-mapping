# Robot and depth geometry (Praveen)

What is on this branch and how far it has been checked.

## ground_demo/ - depth hazard detection

Depth image to point cloud, RANSAC ground plane, signed distance of every
point to that plane. Points standing off the plane are grouped into holes
(below) and bumps (above) and measured: length, width, height, area, and
distance ahead.

It also recovers the camera's height and tilt from the fitted plane, which is
how the arm pose was chosen and cross-checked.

### Validated against simulated ground truth

`python3 selftest.py` builds a plane at a known height and tilt with a known
object on it, and reports the error:

| quantity | error |
|---|---|
| camera height | better than 0.1 mm, across 0.60 to 0.90 m |
| camera tilt | better than 0.01 deg, across 35 to 50 deg |
| object size | 1.0 to 1.5 mm on 25 and 50 mm objects |

### Validated on the real robot

Run over the depth stream of `corridor_book_1434_indoor_books`:

```
camera height : 0.897 - 0.927 m      (TF independently says 0.846 m above the floor)
camera tilt   : 87.0 - 89.7 deg      (TF independently says 86.2 deg)
plane inliers : 92 - 98 %
unevenness    : 4.0 - 4.9 mm RMS on flat tile
books found   : 24-33 mm, and a taller stack at 86-119 mm
```

Two independent methods, TF and plane fitting, agree on the camera pose.

**Noise floor is about 4.5 mm on real tile**, so the hazard threshold is set at
15 mm. Anything smaller cannot be separated from sensor noise at this height.

### Two measurement choices worth knowing

Object size is the **90th percentile** of the absolute residual in the blob,
not the maximum. The maximum tracks the tail of the depth noise and reads 40 mm
on a 25 mm object; the 90th percentile reads 26 mm.

The residual field is **median filtered 5x5** before anything is measured.
Depth noise is independent per pixel, so this removes most of it while leaving
real steps intact. It takes the flat-ground noise floor from 4.2 mm to 1.1 mm
in simulation.

### Next

Wrap it as a ROS 2 node publishing `/ground/hazards`,
`/ground/hazard_markers` and `/ground/unevenness`.

## tools/

- `bag_to_images.py` - pull colour and depth frames out of a bag at a chosen
  rate, for annotation. Consecutive frames are nearly identical, so 1 to 2 fps
  gives genuinely different images. Writes `index.csv` so the dataset can later
  be split by recording session rather than by frame.
- `silvanus_evidence.sh`, `silvanus_diag.sh` - capture evidence for the
  hardware issues reported to the lab.

## docs/robot/

- `arm_pose.md` - joint values for the ground-facing camera pose, the measured
  transform, and what is still to be measured.
- `robot_session_guide.md` - connecting, starting the stack, recording, and the
  problems found on this robot with their workarounds.
