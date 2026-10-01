# Arm pose for the ground-facing camera

Silvanus, Person A (Praveen). Most recent pose first.

---

## Pose B — 2026-10-01, current

### Joint values

`ros2 topic echo /joint_states --once`

| Joint | radians | degrees |
|---|---|---|
| joint1 | `-0.002082378836348653` | -0.12 |
| joint2 | `0.8164670467376709` | 46.78 |
| joint3 | `-2.330569267272949` | -133.53 |
| joint4 | `-0.0074647339060902596` | -0.43 |
| joint5 | `1.5801362991333008` | 90.53 |
| joint6 | `0.0000019174760836904` | 0.00 |

```
[-0.002082378836348653, 0.8164670467376709, -2.330569267272949,
 -0.0074647339060902596, 1.5801362991333008, 0.0000019174760836904]
```

### Measured camera geometry

`ros2 run tf2_ros tf2_echo base_footprint camera_depth_optical_frame`

```
Translation: [0.938, 0.015, 0.846]
RPY (deg)  : [176.216, -0.427, -90.095]
Matrix:
 -0.002 -0.998 -0.066  0.938
 -1.000  0.002 -0.007  0.015
  0.007  0.066 -0.998  0.846
```

`base_footprint` sits at ground level, so:

| | |
|---|---|
| **Height above the floor** | **0.846 m** |
| Forward of robot centre | 0.938 m |
| Lateral offset | 0.015 m (essentially centred) |
| **Tilt below horizontal** | **86.2 degrees** |

The tilt comes from the optical Z axis (third column of the matrix),
`(-0.066, -0.007, -0.998)` in `base_footprint`. Almost straight down.

### Independently confirmed from recorded data

Running the RANSAC ground fit (`code/ground_demo/ground_probe.py`) over depth
frames from `corridor_book_1434_indoor_books` gave, across the whole run:

```
camera height : 0.897 - 0.927 m
camera tilt   : 87.0 - 89.7 deg
plane inliers : 92 - 98 %
unevenness    : 4.0 - 4.9 mm RMS on flat tile
```

Two independent methods, TF and plane fitting, agree. That is a good check that
both the transform and the fit are right.

**Noise floor: about 4.5 mm RMS at this height.** Anything below roughly 15 mm
cannot be reliably separated from sensor noise, which is why the hazard
threshold is set at 15 mm.

---

## Pose A — 2026-10-01, earlier (superseded)

```
[-0.000847917515784502, 1.656481385231018, -3.1856191158294678,
 -0.022937865927815437, 1.568184733390808, 0.022813323885202408]
```

`base_link -> camera_color_optical_frame`:
```
Translation: [0.939, 0.031, 0.626]
RPY (deg)  : [177.764, 0.169, -90.279]
```
Tilt about 88 degrees below horizontal.

---

## A note on the tilt

Both poses look nearly straight down, about 86 to 88 degrees, rather than the
40 degrees the planning document targets.

Straight down gives the best depth accuracy and no perspective distortion, and
because the camera sits 0.94 m forward of robot centre it still sees ground
ahead of the chassis. What it costs is **lookahead**: the visible strip is short,
which matters for the speed governor, since the robot needs to detect a hazard
far enough ahead to slow down before reaching it.

At 0.85 m height with a 58 degree vertical field of view, looking straight down
covers roughly 0.47 m of ground. At 0.3 m/s that is about 1.5 seconds of warning.
Tilting the wrist forward would lengthen it at some cost in depth accuracy.

Decide this deliberately and write down which you chose.

---

## How to restore a pose

1. Check today's Xsens device, the number changes every boot:
   ```bash
   ls -l /dev/serial/by-id/
   ```

2. Start the stack:
   ```bash
   ./silvanus-up.sh --arm=192.168.1.219 --no-gripper --xsens=/dev/ttyUSBn --detach
   ```

3. On the laptop:
   ```bash
   source ~/workspaces/silvanus_hw_env.bash
   source ~/workspaces/scout_xarm_ros2/install/local_setup.bash
   cd ~/workspaces/silvanus_robot && bash ./silvanus-rviz.sh
   ros2 param set /move_group trajectory_execution.allowed_start_tolerance 0.1
   ```

4. RViz: Fixed Frame `base_footprint`, Planning Group `xarm6`, Velocity and
   Accel Scaling `0.05`. Joints tab, enter the six values, then Plan & Execute.

Without the `allowed_start_tolerance` line the arm moves once and then refuses
every later trajectory.

---

## Still outstanding

- [ ] Photo of the arm from the side
- [ ] Scout 2.0 wheelbase and track width, wheel centre to wheel centre.
      Person C needs both for the wheel-track test that assigns hazard states.
- [ ] Decide and record whether to keep the near-vertical view or tilt forward
      for more lookahead.
