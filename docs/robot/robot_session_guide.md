# Silvanus: command reference

Everything learned up to 2026-09-30. Work top to bottom.

## Facts

| | |
|---|---|
| Wi-Fi | `HUAWEI_B361_E1D1` / `goodlife24` |
| Robot computer | `192.168.1.102` |
| SSH user / password | `agilex` / `agx` |
| xArm control box | `192.168.1.219` |
| ROS domain | `42` |
| Maintainer | Renatto Tommasi, renatto.tommasi@gmail.com |

Passwords show nothing as you type. That is normal.

**Check your prompt before every command:**

| Prompt | Where you are |
|---|---|
| `praveen@praveen-LOQ-15IRX10` | your laptop |
| `agilex@agilex-desktop` | the robot |
| `root@agilex-desktop` | inside the silvanus container |

---

## 1. Connect

```bash
ping -c 3 192.168.1.102
ssh agilex@192.168.1.102
```

If ping fails, the DHCP lease changed. Get the real address with `hostname -I` on the robot's screen, then:

```bash
export SILVANUS_ROBOT_IP=<new ip>
source ~/workspaces/silvanus_hw_env.bash
```

## 2. Preflight (on the robot)

```bash
cd /opt/silvanus
./test_robot.sh preflight
ip -details link show can0
ping -c 3 192.168.1.219
```

`can0` must be **UP** at **500000**. The arm must answer at `.219`.

## 3. Start the stack (on the robot)

Pick one. Each command **replaces** the running container, so `Ctrl-C` the old one first.

| What you want | Command |
|---|---|
| Camera + lidar only | `./silvanus-up.sh --no-base` |
| Camera + lidar + arm, no driving | `./silvanus-up.sh --no-base --arm=192.168.1.219 --no-gripper` |
| Everything, **can drive** | `./silvanus-up.sh --arm=192.168.1.219 --no-gripper` |
| Driving, no arm | `./silvanus-up.sh` |

**`--no-gripper` is always required.** The gripper's serial device is missing on this robot and `--arm` fails without it.

Working over SSH, add `--detach` so a dropped connection does not kill the stack mid-drive:

```bash
./silvanus-up.sh --arm=192.168.1.219 --no-gripper --detach
sudo docker logs -f silvanus        # watch it start; Ctrl-C only stops the log view
```

Wait for `Successfully switched controllers` and `xArm is ready to move`.

### With the IMU

Add `--xsens=/dev/ttyUSBn` and put the mounting transform in front. **Every
start, every time**: the lab stack has no Xsens transform otherwise.

```bash
ls -l /dev/serial/by-id/          # the Xsens line ends in ../../ttyUSBn; the number changes per boot
XSENS_BAUDRATE=921600 XSENS_PARENT_FRAME=base_link XSENS_X=0.0 XSENS_Y=-0.015 XSENS_Z=0.115 \
XSENS_ROLL=0.0 XSENS_PITCH=0.0 XSENS_YAW=0.0 \
./silvanus-up.sh --arm=192.168.1.219 --no-gripper --xsens=/dev/ttyUSBn --detach
```

Where these numbers come from, and how to check them: [imu_mount.md](imu_mount.md).

## 4. Check from the laptop

```bash
source ~/workspaces/silvanus_hw_env.bash
ros2 topic list --no-daemon --spin-time 5
```

Always use `--no-daemon --spin-time 5`. Plain `ros2 topic list` returns stale cached results.

Driving needs `/cmd_vel`, `/odom`, `/scout_status`:

```bash
ros2 topic info /cmd_vel --verbose
```

Must say **`geometry_msgs/msg/Twist`**, subscription count **1**.

## 5. Camera

Topic names have **camera twice**. The lab's STUDENT_GUIDE is wrong about this.

```
/camera/camera/color/image_raw          640x480 @ 30
/camera/camera/depth/image_rect_raw     848x480 @ 30
/camera/camera/color/camera_info
/camera/camera/depth/camera_info
/camera/camera/extrinsics/depth_to_color
```

There is **no aligned depth topic**. Use the extrinsics topic to align it yourself.

Real rate, measured **on the robot**: colour 29.97 Hz, depth 29.99 Hz. The cable is fine. Over Wi-Fi you see about 5 Hz, which is a bandwidth limit, not a fault.

```bash
# on the robot, inside the container: the true rate
ros2 topic hz /camera/camera/color/image_raw
```

Viewing from the laptop:

```bash
ros2 run rqt_image_view rqt_image_view
```

- **Left dropdown** → `/camera/camera/color/image_raw` (plain, **not** `/compressed`)
- **Far-right dropdown** → `raw`

If rqt starts by itself on the compressed topic and crashes, it has remembered a bad setting:

```bash
rm -f ~/.config/ros.org/rqt_gui.ini ~/.config/ros.org/rqt_gui_cpp.ini
```

## 6. Arm, from the laptop

```bash
source ~/workspaces/silvanus_hw_env.bash
source ~/workspaces/scout_xarm_ros2/install/local_setup.bash
cd ~/workspaces/silvanus_robot
bash ./silvanus-rviz.sh
```

- **Fixed Frame** → `base_footprint`
- **Planning Group** → `xarm6`

**Start RViz only after the stack is fully up.** If it starts too early it misses the latched robot description and Planning Group stays empty.

Window too big:

```bash
sudo apt install -y wmctrl
wmctrl -r RViz -e 0,50,50,1400,850
```

Moving it: someone at the arm emergency stop, **Velocity and Acceleration Scaling both 0.05**, drag the orange ball, **Plan**, watch the whole preview, **Execute**. Execute moves the real arm.

Target pose: camera about **0.75 m** above the ground, tilted about **40 degrees** down, looking 0.4 to 1.3 m ahead. Elbow bent, camera above the front bumper, not past it.

## 7. Teleop

The stack must be running **without** `--no-base`.

**Before anything moves:** clear the floor, someone at the chassis emergency stop, and put the **Scout remote into CAN / computer-control mode**.

```bash
source ~/workspaces/silvanus_hw_env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=false \
  -p speed:=0.10 \
  -p turn:=0.20 \
  -p repeat_rate:=10.0 \
  -p key_timeout:=0.5
```

```
i = forward     , = backward
j = turn left   l = turn right
k = STOP
```

First move: press `i` once, briefly, then `k`.

`stamped:=false` matters. The simulator wants `TwistStamped`; the real robot wants plain `Twist`.

Stop: `k`, then the physical emergency stop, then `Ctrl-C`.

## 8. Record a bag

**On the robot, never from the laptop.** Over Wi-Fi you would silently record 5 Hz instead of 30.

```bash
sudo docker exec -it silvanus bash
source /opt/ros/jazzy/setup.bash
source /opt/robot/setup.bash
mkdir -p /bags
ros2 bag record -s mcap -o "/bags/run-$(date +%Y%m%d-%H%M%S)" \
  /camera/camera/color/image_raw /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
  /imu/data /filter/free_acceleration /gnss \
  /odom /scan /tf /tf_static /joint_states /scout_status
```

Before leaving the robot:

```bash
ros2 bag info <the folder>
```

**Every topic must have a non-zero message count.** A bad first bag blocks both teammates for a week.

Copy it back:

```bash
scp -r agilex@192.168.1.102:/data/bags/run-* ~/bags/
```

## 9. Diagnostics

```bash
# from the laptop: can it see the nodes, not just topics?
ros2 node list --no-daemon --spin-time 5
ros2 node list --no-daemon --spin-time 5 | grep -i move

# on the robot, inside the container: is it running there?
ros2 node list
ros2 topic list | grep -E "semantic|monitored"

# robot-side logs
sudo docker logs --since 2m --tail 250 silvanus
ls -l /dev/serial/by-id/
```

If a node is visible on the robot but not on the laptop, that is a Wi-Fi discovery problem, not a crashed node.

## 10. Safe shutdown

1. Stop motion. Press `k` in teleop.
2. Engage the chassis and arm emergency stops.
3. `Ctrl-C` your own terminals.
4. `Ctrl-C` the stack last, or `sudo docker stop --time 30 silvanus`.

Stopping a process is not an emergency stop.

---

## Known problems on this robot

**Missing udev rules.** `/dev/robot/dh_ag95_gripper` and `/dev/robot/xsens_mti` do not exist, so `99-silvanus-serial.rules` was never installed. Consequence: **the Xsens IMU cannot start**, which blocks half the project. Maintainer job. Possible workaround: find the device with `ls -l /dev/serial/by-id/` and start with `--xsens=/dev/ttyUSBn`.

**Disk 95% full**, about 12 GB free. Raw colour plus depth is roughly 3.5 GB per minute, so about three minutes of recording.

**NoMachine is broken.** The robot had a licensed 7.17.1 which was uninstalled by accident on 2026-09-30 and replaced with free Personal Edition 10.1.7, which refuses connections without a subscription. The licence file is intact at `/usr/NX/etc/server.lic` and copied to `/root/nomachine-server.lic.backup-20260930`. Reported to Renatto. **NoMachine is not needed** for normal work: RViz and rqt run on the laptop.

**Duplicate node names** appear in the graph (`move_group`, `rslidar_sdk_node`). This causes flaky DDS discovery, where some nodes are visible from the laptop and others are not.

**`move_group` sometimes not discoverable from the laptop**, which leaves RViz's Planning Group empty. Check on the robot first to tell a crash from a discovery failure.

---

## Still to do

- [ ] Arm posed, joint values and photo recorded
- [ ] `base_footprint` to camera transform measured, `ground_cam` published
- [ ] Wheelbase and track width measured with a tape
- [x] Xsens mounting transform measured (six numbers): [imu_mount.md](imu_mount.md)
- [ ] Does the RC remote publish to `/cmd_vel`? Run `ros2 topic hz /cmd_vel` while someone drives with it
- [ ] First bag recorded and verified with `ros2 bag info`
- [ ] Email Renatto: NoMachine, udev rules, disk space
