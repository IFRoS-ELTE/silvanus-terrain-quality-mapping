# Navigation

`coverage_sweep.py` drives a zigzag (boustrophedon) sweep over a rectangle with
Nav2's FollowWaypoints and logs the robot's path from TF. It needs a running Nav2
with a `map` frame.

```bash
python3 coverage_sweep.py --x0 0.8 --y0 -1.6 --length 2.4 --width 3.2 --spacing 0.8
# in simulation add:  --ros-args -p use_sim_time:=true
```

Writes `coverage_path.csv` (t, x, y) and `coverage_path_waypoints.csv`.

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
