#!/bin/bash
# auto_session.sh [NAME] [dry]
#
# Run on the robot. The autonomous version: no saved map, nobody driving.
#   live SLAM (lidar map built as the robot moves) + Nav2 + the speed governor
#   the robot fits lanes into the free floor the lidar sees, up to LENGTH m ahead
#   and WIDTH m across (default 5 x 5), and sweeps them; the camera finds bumps and holes, the
#   governor slows to 0.2 m/s so the IMU can feel them and stops for tall ones and obstacles.
# Everything is recorded, and the map is saved next to the recording. With "dry" it
# plans only: it prints the lanes it would drive, without driving or recording.
# ROOM=1 sweeps the whole room instead of LENGTH x WIDTH: all the open floor it can reach without
# passing a door, then again over what the map gained and the camera has not seen (up to 3 passes).
#
#   LENGTH=5 WIDTH=4 bash ~/auto_session.sh hall
#   ROOM=1 bash ~/auto_session.sh lab
#   SPIN=1 LENGTH=3 WIDTH=0 bash ~/auto_session.sh path        # one turn on the spot before it plans
#
# Needs: the stack (start_robot.sh), the arm in Pose B (arm_to_pose.py), the robot facing the area,
# ~/nav_robot/nav2_robot_governor.yaml, ~/auto_sweep.py, ~/lane_planner.py, ~/governor_up.sh,
# ~/record_session.sh, ~/governor/. Ctrl-C during the sweep cancels it; the e-stop always works.
set -u
NAME=${1:-auto_sweep}
DRY=${2:-}
LENGTH=${LENGTH:-5}
WIDTH=${WIDTH:-5}
ROOM=${ROOM:-}
if [ -n "$ROOM" ]; then
    AREA="--room"; AREA_TEXT="the whole room"; MAX_S=1800      # recording stops when the sweep ends
else
    AREA="--length $LENGTH --width $WIDTH"; AREA_TEXT="${LENGTH} m ahead x ${WIDTH} m across"; MAX_S=900
fi
# SPIN=1: turn once on the spot before planning (not needed: the lidar sees all round)
[ -n "${SPIN:-}" ] && { AREA="$AREA --spin"; AREA_TEXT="$AREA_TEXT, one turn first"; }
ROS="source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash"
D="sudo docker exec silvanus"
for f in ~/nav_robot/nav2_robot_governor.yaml ~/auto_sweep.py ~/lane_planner.py ~/governor_up.sh ~/record_session.sh \
         ~/governor/speed_governor.py; do
    [ -f "$f" ] || { echo "missing $f - copy it from the laptop first"; exit 1; }
done
sudo docker ps --format '{{.Names}}' | grep -qx silvanus || { echo "the stack is not running - run start_robot.sh first"; exit 1; }
for p in slam_toolbox nav2_regulated_pure_pursuit_controller nav2_rotation_shim_controller; do     # one at a time
    $D bash -c "$ROS && ros2 pkg prefix $p" > /dev/null 2>&1 || { echo "$p is missing in the container"; exit 1; }
done

echo "1/6 hazard node, speed governor and /scan_fixed"
$D pkill -INT -f "[b]ringup_launch|[n]avigation_launch|[n]av2_container|[o]nline_async_launch|[a]sync_slam_toolbox"
sleep 3
bash ~/governor_up.sh 0.35 || exit 1

echo "2/6 live SLAM on /scan_fixed"
$D mkdir -p /tmp/nav_robot
sudo docker cp ~/nav_robot/. silvanus:/tmp/nav_robot/ > /dev/null
for f in auto_sweep.py lane_planner.py; do sudo docker cp ~/$f silvanus:/tmp/nav_robot/$f > /dev/null; done
# the lab's SLAM settings, on the floor-free scan, taking scans while the robot turns on the spot, and
# one beam enough to call a cell free (the lab's 2 leaves the floor a few metres out unknown)
# (with check_min_dist_and_heading_precisely false it waits for 9 cm of travel and the turn maps nothing)
$D bash -c "$ROS && sed -e 's|scan_topic: /scan|scan_topic: /scan_fixed|' \
    -e 's|check_min_dist_and_heading_precisely: false|check_min_dist_and_heading_precisely: true|' \
    -e 's|use_sim_time: true|use_sim_time: false|' -e 's|min_pass_through: 2|min_pass_through: 1|' \
    \$(ros2 pkg prefix scout_xarm_navigation)/share/scout_xarm_navigation/config/slam_toolbox.yaml > /tmp/nav_robot/slam_live.yaml"
sudo docker exec -d silvanus bash -c "$ROS && ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=/tmp/nav_robot/slam_live.yaml use_sim_time:=false autostart:=true > /tmp/nav_robot/slam.log 2>&1"
sleep 5

echo "3/6 Nav2 on the live map, up to 0.35 m/s through the governor"
sudo docker exec -d silvanus bash -c "$ROS && ros2 launch nav2_bringup navigation_launch.py \
params_file:=/tmp/nav_robot/nav2_robot_governor.yaml use_sim_time:=False > /tmp/nav_robot/nav2.log 2>&1"
for i in $(seq 60); do $D grep -q 'Managed nodes are active' /tmp/nav_robot/nav2.log 2>/dev/null && break; sleep 1; done
$D grep -q 'Managed nodes are active' /tmp/nav_robot/nav2.log || { echo "Nav2 did not come up:"; $D tail -n 15 /tmp/nav_robot/nav2.log; exit 1; }
echo "    Nav2 is up"
read -r -p "4/6 In RViz the map should start to appear around the robot. Area: ${AREA_TEXT}. Enter to go on, Ctrl-C to stop: "

if [ "$DRY" = dry ]; then
    echo "5/6 dry run: plan only, no driving, no recording"
    sudo docker exec -it silvanus bash -c "$ROS && cd /tmp/nav_robot && python3 auto_sweep.py \
$AREA --dry --log /tmp/nav_robot/auto.csv"
    echo "SLAM and Nav2 keep running. Plan: /tmp/nav_robot/auto_plan.json in the container"
    exit 0
fi

echo "5/6 recording (topic check first, about 20 s)"
EXTRA_TOPICS="/map /governor/state /governor/felt /ground/hazards /ground/unevenness /cmd_vel_in /cmd_vel /plan" \
    ~/record_session.sh "$NAME" $MAX_S > /tmp/auto_record.txt 2>&1 &
REC=$!
for i in $(seq 60); do grep -q ">>> RECORDING" /tmp/auto_record.txt && break; kill -0 $REC 2>/dev/null || break; sleep 1; done
if ! grep -q ">>> RECORDING" /tmp/auto_record.txt; then cat /tmp/auto_record.txt; echo "recording did not start"; exit 1; fi
BAG=$(grep ">>> RECORDING" /tmp/auto_record.txt | sed 's/.*\/bags\/\([^ ]*\).*/\1/')
echo "    recording $BAG"
[ -f ~/live_web.sh ] && bash ~/live_web.sh
sleep 3

echo "6/6 plan and sweep: stand 1.5 m away with the remote, not in front"
sudo docker exec -it silvanus bash -c "$ROS && cd /tmp/nav_robot && python3 auto_sweep.py \
$AREA --log /bags/${BAG}_auto.csv"

$D bash -c "$ROS && timeout 30 ros2 run nav2_map_server map_saver_cli -f /bags/${BAG}_map > /dev/null 2>&1" \
    && echo "map saved: ${BAG}_map.pgm / .yaml"
sleep 2
$D pkill -INT -f "[r]os2 bag record"
kill $REC 2>/dev/null
sleep 5
$D bash -c "$ROS && ros2 bag info /bags/$BAG" | grep -E "Duration|Bag size|camera/color/image_raw|scan_fixed|imu/data"
BAGS_DIR=${BAGS_DIR:-$(mountpoint -q /mnt/ssd && echo /mnt/ssd/silvanus_bags || echo /data/bags)}
echo ""
echo "copy to the laptop:  scp -r \"agilex@192.168.1.102:$BAGS_DIR/${BAG}*\" ~/bags/"
echo "SLAM and Nav2 keep running; stop them with:"
echo "  sudo docker exec silvanus pkill -INT -f '[n]avigation_launch|[n]av2_container|[o]nline_async_launch|[a]sync_slam_toolbox'"
