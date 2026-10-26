# Navigation

`coverage_sweep.py` drives a zigzag (boustrophedon) sweep over a rectangle with
Nav2's FollowWaypoints and logs the robot's path from TF. It needs a running Nav2
with a `map` frame.

```bash
python3 coverage_sweep.py --x0 0.8 --y0 -1.6 --length 2.4 --width 3.2 --spacing 0.8
# in simulation add:  --ros-args -p use_sim_time:=true
```

Writes `coverage_path.csv` (t, x, y) and `coverage_path_waypoints.csv`.

## Speed governor

`speed_governor.py` sits between whatever drives the robot and the base, and only ever
lowers the forward speed (turning and reversing pass unchanged):

| mode | when | forward speed |
|---|---|---|
| stop | a bump or hole of 10 cm or more on the wheel track within 35 cm of the front wheels, or a lidar return within 40 cm of the front bumper | 0 |
| slow | any other hazard from the depth node on the track ahead, until the rear wheels are past it | 0.20 m/s |
| free | otherwise | up to 0.5 m/s |

0.20 m/s is where the IMU can feel a hazard: on the 2026-10-08 runs it felt rigid objects
from 15 mm at 0.2-0.3 m/s, and nothing at all above 0.4 m/s on tile. While the wheels cross
a hazard on the track, the governor measures the jolt and publishes on `/governor/felt`
whether it was felt (class C) or not (class A). Formats: `docs/schema.md`.

On the robot, with the depth node running, an operator at the e-stop, and the scan fix
from docs/PROGRESS.md running (`/scan_fixed`):

```bash
python3 navigation/speed_governor.py --ros-args -p enable:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_in
ros2 topic echo /governor/state            # free / slow / stop and why
ros2 topic echo /governor/felt             # felt or not, per hazard
```

For Nav2, remap its velocity output to `/cmd_vel_in`. Without `enable:=true` it publishes
only `/governor/state` and nothing on `/cmd_vel`. When the input stops for 0.5 s it sends
zeros instead of holding the last speed. First live test: wheels off the ground, then
`max_speed:=0.3`.

Tested offline (not yet on the robot):
- `python3 -m pytest navigation/test_speed_governor.py`: 8 cases of the stop / slow / free
  decision, including a 12 cm bump 20 cm ahead of the front wheels and a lidar obstacle.
- Replay of `corridor_groundtruth_1447` through the depth node and the governor, with a
  constant 0.4 m/s command: it slows to 0.2 m/s before the 50 mm foam block and the wooden
  blocks and goes back to 0.4 m/s after them. The block comes out felt (jolt 1.81 m/s², C), a
  15 mm wooden block felt (0.97, C), and a neighbouring hazard is not credited with the same
  jolt. Outdoors (`garden_surfaces_1628`) a 30 mm bump on wood chips is felt (2.68). The
  stop rule did not fire on the recordings: the depth node measured the kerb at 34-36 mm in
  replay, below the 10 cm stop height.

## In the lab simulator (2026-10-08)

Nav2 accepted the waypoints ("Begin navigating from (0.00, 0.00) to (0.60, -1.50)")
but the sweep did not complete in either world. The simulated skid-steer turns
far less than commanded:

| command (4 s) | simulated robot | expected |
|---|---|---|
| rotate in place 0.5 rad/s | 50 deg | 115 deg |
| arc 0.2 m/s + 0.4 rad/s | 24 deg, 0.16 m | 92 deg, 0.8 m |

Its wheel odometry and SLAM then disagree by more than 3 m, and the MPPI
controller keeps resetting ("Failed to make progress"). Driving straight works,
which is all the lab's own Nav2 check (a 0.22 m goal) tests. The training-ground
world is also almost empty, so SLAM has little to match.

The fix is in the simulator, not here: wheel friction or
`wheel_separation_multiplier` for `scout_base_controller` in
`scout_xarm_gazebo/config/ros2_controllers.yaml`, tuned until a commanded turn
gives the same turn in simulation. The real Scout misjudges turns too (15-30 deg
per run from its wheels), so on the robot run Nav2 with the IMU-fused odometry
(`config/ekf_odom_imu.yaml`).
