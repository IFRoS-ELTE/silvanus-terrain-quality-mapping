# Navigation

| file | what |
|---|---|
| `make_nav2_robot.py` | the robot's Nav2 settings, made from the lab's simulator ones |
| `coverage_sweep.py` | a zigzag sweep over a rectangle you give, on a saved map |
| `lane_planner.py`, `auto_sweep.py` | the autonomous sweep: lanes fitted into the free floor of the live map |
| `speed_governor.py`, `governor_watch.py` | slow at hazards so the IMU can feel them, stop at obstacles |
| `../tools/sim/fake_robot.py` | the robot on a saved map (wheels with a realistic error, lidar ray-cast from the map), to test all of this on a laptop |

On the robot these run through the session scripts in [tools/robot](../tools/robot):
`sweep_session.sh` (saved map) and `auto_session.sh` (no map, nobody driving).

## Nav2 on the robot

The lab's `scout_xarm_navigation/config/nav2_params.yaml` is written for Gazebo. `make_nav2_robot.py`
writes the robot's version: real time, plain Twist (what the Scout base takes), `/scan_fixed` in its
own frame in both costmaps and the collision monitor, lidar beams with no return clearing the
costmap (otherwise a person who walked past stays an obstacle), and an AMCL section.

```bash
L=~/workspaces/scout_xarm_ros2/src/scout_xarm_navigation/config/nav2_params.yaml
# goals from RViz on a saved map, Nav2 driving the base directly, 0.2 m/s
python3 make_nav2_robot.py $L nav2_robot.yaml --max-speed 0.2 --initial-pose 3.037 -3.203 66.34
# sweeps: through the speed governor, Pure Pursuit, looser goal check
python3 make_nav2_robot.py $L nav2_robot_governor.yaml --max-speed 0.35 --initial-pose 3.037 -3.203 66.34 \
    --cmd-vel-out cmd_vel_in --goal-tolerance 0.35 30 --controller rpp --arm-footprint --wait-recovery
```

The initial pose is the start tape of the `indoor_loop3_1316` map. For sweeps three things differ from
the lab's settings, each found on the first sweep on 2026-10-27, where the robot stood 190 s turning
0.23-0.5 m from a lane-change corner, and each checked in simulation:

- goal tolerance 0.35 m / 30° instead of 0.20 m: AMCL's spread in the corridor was 0.3-0.45 m
- Regulated Pure Pursuit inside the Rotation Shim instead of MPPI: the lab's forward-only MPPI circles
  a goal it cannot line up with; the shim turns on the spot to face the path and the goal heading
- every corner faces the next leg (`coverage_sweep.lanes`, `lane_planner.plan`): turn on the spot,
  drive straight, turn. Facing along the lane, the next lane start sat beside the robot.

`--wait-recovery` takes the turn on the spot (90°) and the 0.3 m reverse out of Nav2's recovery: when
stuck it only clears its costmaps and waits. Blind, with the arm out, those moves are not safe; on
2026-10-28 the robot turned that way after the governor stopped it.

`--arm-footprint` adds the arm in Pose B to the robot's outline: the camera housing is 1.05 m ahead of
the centre, so turning on the spot sweeps a circle of about 1 m, and the costmaps, the controller and the
collision monitor check it like the body. The lidar band is 0.1-0.6 m: walls and people are seen, a
table top at arm height is not.

## Sweeps

On a saved map, with a rectangle in its frame (`--yaw` turns it, e.g. along a corridor):

```bash
python3 coverage_sweep.py --x0 3.875 --y0 -0.294 --yaw 66.34 --length 5 --width 1.6 --spacing 0.8
```

Autonomous, with live SLAM and no saved map. The robot stands where the sweep should start, facing
the area; it fits lanes 0.8 m apart into the free floor it sees and sweeps them. The free floor is the
live map plus the latest lidar scan itself (`lane_planner.add_scan`): standing still, SLAM Toolbox leaves
the floor a few metres out unknown between the beams, and the planner counts unknown as a wall (14 m²
free in simulation from one spot, 53 m² with the scan added). There is no turn on the spot first: the
lidar sees all round. `--spin` (`SPIN=1`) adds one, with 1.25 m free all round. A lane keeps 0.6 m from walls and
unknown space along its length; its ends, where the robot turns with the arm out, need 1.25 m free all
round and are pulled in until they have it; lanes shorter than 1 m are dropped, and the pattern is shifted
sideways if that fits more. `sweep_session.sh` plans the same way on the saved map (`--no-spin`):

```bash
python3 auto_sweep.py --length 5 --width 5          # --dry: plan only
python3 auto_sweep.py --room                        # the whole room
python3 -m pytest test_lane_planner.py
```

With `--room` (`ROOM=1` for `auto_session.sh`) there is no area to give. The area is all the open floor
the robot can reach from where it stands without passing anywhere narrower than 1.2 m: a door is
narrower, so the sweep stays in the room even though the lidar sees out through the door. Lanes then
also lie behind the robot, and a lane that furniture cuts in two keeps both stretches. After a pass it
plans again on the map it has built meanwhile, leaving out the floor the camera has already seen
(0.4 m around the camera's track), and sweeps what is new, for example behind a cabinet the lidar could
not see past from the start. From the second pass on it also tries lanes across, whichever covers more:
they reach the floor beyond the ends of the first lanes and beside furniture. At most three passes, or
until nothing new is left.

In simulation, in a 9 x 7 m lab with a cabinet, a pillar and a door to a corridor (`tools/sim/test_lab.yaml`,
start at 2.0 2.5 0): four lanes along, then two across behind the far end, 166 s, no "failed to make
progress", and nothing planned beyond the door. The camera saw 40 % of the floor. The rest is the price of
the arm: the robot only turns where 1.25 m is free all round, so a strip about 0.85 m wide along every wall
and the floor close around furniture stay out of reach. Not for a room with tables: the lidar band ends at
0.6 m, so a table top at arm height is not seen, and the floor between two table legs looks free.

Live SLAM needs `check_min_dist_and_heading_precisely: true` and `min_pass_through: 1`
(`auto_session.sh` sets both). With the lab's `false` SLAM Toolbox waits for 9 cm of travel and a turn on
the spot maps nothing (13 m² before and after a full turn in simulation, 70 m² with `true`); with the
lab's 2 a cell is free only once two beams have crossed it, which from one spot leaves the floor beyond
about 1.5 m unknown. On the robot on 2026-10-28 that gave "no free floor for a lane" on open floor, and
the turn on the spot meant to fill it in was cut short whenever someone stood inside the arm's circle.

Tested in simulation (`fake_robot.py` on the `indoor_loop3_1316` map, 5 % turn error on the wheels):
the corridor sweep of 3 lanes x 5 m in 78 s, and the autonomous one with the arm counted: a 5 x 5 m
request gave two lanes (1.6 and 3.9 m) in this 3.3-4.3 m corridor, swept in 36 s, both without a
"failed to make progress". At the start tape it refused to turn: a wall is 1.05 m away there.

## Speed governor

`speed_governor.py` sits between whatever drives the robot and the base, and only ever
lowers the forward speed:

| mode | when | forward speed |
|---|---|---|
| stop | a bump or hole of 10 cm or more on the wheel track within 35 cm of the front wheels, or a lidar return inside the robot's own width within 15 cm of its front: the arm's camera housing in Pose B, 1.05 m ahead of the centre (`front_m`; 0.47 m, the bumper, with the arm tucked) | 0 |
| slow | any other hazard from the depth node on the track ahead, until the rear wheels are past it | 0.20 m/s |
| free | otherwise | up to `max_speed` (0.5 m/s) |

When it slows the robot it slows the turn by the same factor, so the robot keeps the curve it was
asked for; at a stop it may still turn on the spot, and reversing passes. 0.20 m/s is where the IMU
can feel a hazard: on the 2026-10-08 runs it felt rigid objects from 15 mm at 0.2-0.3 m/s, and nothing
at all above 0.4 m/s on tile. While the wheels cross a hazard on the track, the governor measures the
jolt and publishes on `/governor/felt` whether it was felt (class C) or not (class A). Pieces of one
object closer than 0.6 m that share a jolt come out as `part_of` the first one. A crossing counts only
while the robot drives forward (at least 0.1 m/s, turning under 0.3 rad/s): turning on the spot shakes
it by itself. `/governor/hazards`
lists every tracked hazard once a second (for the live inspection page). Formats: `docs/schema.md`.

On the robot `tools/robot/governor_up.sh` starts it with the depth node and `/scan_fixed`. By hand:

```bash
python3 navigation/speed_governor.py --ros-args -p enable:=true -p max_speed:=0.5
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_in   # hold the key
python3 navigation/governor_watch.py       # free / slow / stop and why, felt or not per hazard
```

For Nav2, make its collision monitor publish to `/cmd_vel_in` (`make_nav2_robot.py --cmd-vel-out
cmd_vel_in`). Without `enable:=true` it publishes only `/governor/state`. Once it has had any input it
sends zeros when the input stops for 0.5 s, so nothing else may drive `/cmd_vel` while it runs.

Live on the robot (2026-10-27, teleop up to 0.3 m/s, the stop then measured from the bumper): stopped 0.37-0.40 m before a wall, stopped
0.34-0.35 m before 123 mm and 216 mm objects, slowed from 0.30 to 0.20 m/s before 10-23 mm plates,
and felt 10 mm (jolt 0.87 m/s²), 19 mm (1.48), 18 mm (2.65), 21 mm (1.68) and 74 mm (1.28) plates.

Offline: `python3 -m pytest navigation/test_speed_governor.py` (21 cases), and on a recording:

```bash
navigation/replay_governor.sh ~/bags/corridor_groundtruth_1447 0.4
```

which slows for the 50 mm foam block and finds it felt (1.81 m/s², C).

## In the lab simulator (2026-10-08)

The lab's Gazebo skid-steer turns far less than commanded (0.5 rad/s for 4 s gives 50° instead of
115°), so its wheel odometry and SLAM disagree by more than 3 m and Nav2 cannot follow a sweep. The fix
is in the simulator (`wheel_separation_multiplier` for `scout_base_controller`). `tools/sim/fake_robot.py`
is the simpler stand-in used above.
