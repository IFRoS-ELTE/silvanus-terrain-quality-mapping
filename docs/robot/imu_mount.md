# Xsens IMU mounting transform

Silvanus, measured and verified 2026-10-08.

---

## Where it is

The Xsens MTi-670 is on an **MTi-600 DEV board** (Rev 1.4), with a u-blox
MAX-M8 GNSS receiver on a mikroBUS board beside it. It lies flat on the floor
of the white electronics box, which opens on the robot's **left** side.

![The board in the electronics box](../media/xsens_in_box.jpg)

![The open left side of the box](../media/robot_left_side.jpg)

The axes are printed on the board: **X towards the front (arm side), Y to the
left, Z up.** That is the same orientation as `base_link`, so no rotation is
needed.

![The MTi-600 DEV board with its printed axes](../media/xsens_board.jpg)

The board is held down with adhesive tape. Check it is still firm before every
recording: if it can slide, roughness measures the board moving, not the ground.

---

## The transform

`base_link` is the centre of the four wheels, 0.2348 m above the floor.

| | value | how it was found |
|---|---|---|
| x | **0.0 m** | the module sits midway between front and rear wheels |
| y | **-0.015 m** | 26 cm to the box opening (left), 23 cm to the opposite wall; the box is centred between the wheels, so (23 - 26) / 2 = -1.5 cm |
| z | **0.115 m** | 35 cm floor to module, minus 23.48 cm |
| roll, pitch, yaw | **0** | board flat, X forward, Y left |

Measured with a tape to the middle of the small Xsens module, not the edge of
the board. Accuracy is about 1 cm.

---

## Starting the robot with it

The lab stack has no transform for the Xsens until it is given one. Give it at
**every** start, on the robot:

```bash
ls -l /dev/serial/by-id/          # find the Xsens line, note ../../ttyUSBn
cd /opt/silvanus
XSENS_PARENT_FRAME=base_link XSENS_X=0.0 XSENS_Y=-0.015 XSENS_Z=0.115 \
XSENS_ROLL=0.0 XSENS_PITCH=0.0 XSENS_YAW=0.0 \
./silvanus-up.sh --no-base --xsens=/dev/ttyUSBn --detach
```

`--no-base` is for checking only. For a normal session use the same `XSENS_...`
part in front of the start command you need from the session guide.

---

## Checks, and what they gave on 2026-10-08

From the laptop, with `source ~/workspaces/silvanus_hw_env.bash`:

**Transform**

```bash
ros2 run tf2_ros tf2_echo base_link xsens_link
```
```
- Translation: [0.000, -0.015, 0.115]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
```

**Gravity, robot standing still**

```bash
ros2 topic echo /imu/data --field linear_acceleration
```
```
x: -0.06   y: 0.11   z: 9.80 to 9.84
```

z carries gravity, so Z points up. The small x and y mean the board sits tilted
by less than 1 degree (about 0.6 and 0.35 degrees). That changes vertical
acceleration by under 0.01 %, so roll and pitch are left at 0.

**Free acceleration**

```bash
ros2 topic hz /filter/free_acceleration
```
```
average rate: 9.8 to 10.4
```

---

## For the roughness node

`/filter/free_acceleration` (`geometry_msgs/Vector3Stamped`) is acceleration
with gravity already removed by the Xsens, in a world frame with z up. Vertical
roughness is the RMS of its z. It does not depend on the transform above.

Bags recorded before 2026-10-08 have only `/imu/data`. Record
`/filter/free_acceleration` from now on.

The IMU runs at **10 Hz**, so nothing above 5 Hz is visible in an FFT. Raising
it needs a baudrate change on the sensor; see the lab's `xsens_mti_node.yaml`.
