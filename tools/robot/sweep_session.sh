#!/bin/bash
# sweep_session.sh [NAME] [dry]
#
# Run on the robot. Nav2 on the indoor_loop3_1316 map drives a coverage sweep of the corridor
# (3 lanes x 5 m, 0.8 m apart, 3-8 m past the tape) through the speed governor: the depth camera
# finds a bump or hole ahead, the governor slows to 0.2 m/s, the IMU feels it as the wheels cross,
# and the governor stops for tall ones and for anything the lidar sees in front of the bumper.
# Everything is recorded, for the inspection map. With "dry" it drives the same sweep without recording, to see the path first.
# Facing down the corridor from the tape, the lanes run from 3 m to 8 m ahead, 0.4, 1.2 and 2.0 m
# to the left of the straight line from the tape; the wheels run 0.29 m either side of a lane. Needs:
#   the stack (start_robot.sh), the arm in Pose B (arm_to_pose.py)
#   the robot on the tape, facing down the corridor as at the start of loop 3 (AMCL starts there)
#   ~/nav_robot (map + nav2_robot.yaml), ~/coverage_sweep.py, ~/record_session.sh
# Ctrl-C during the sweep cancels the Nav2 goal; the e-stop always works.
set -u
NAME=${1:-corridor_sweep}
DRY=${2:-}
LENGTH=${LENGTH:-5}          # lane length, m; LENGTH=3 WIDTH=0.8 for a small 2-lane test
WIDTH=${WIDTH:-1.6}
ROS="source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash"
D="sudo docker exec silvanus"
for f in ~/nav_robot/nav2_robot_governor.yaml ~/nav_robot/loop3_fixed_map.yaml ~/coverage_sweep.py ~/record_session.sh \
         ~/governor_up.sh ~/governor/speed_governor.py; do
    [ -f "$f" ] || { echo "missing $f - copy it from the laptop first"; exit 1; }
done
sudo docker ps --format '{{.Names}}' | grep -qx silvanus || { echo "the stack is not running - run start_robot.sh first"; exit 1; }

echo "1/5 hazard node and speed governor: Nav2 -> /cmd_vel_in -> governor -> /cmd_vel"
$D pkill -INT -f "[b]ringup_launch|[n]av2_container"
sleep 2
bash ~/governor_up.sh 0.35 || exit 1

echo "2/5 starting Nav2 on the loop 3 map, up to 0.35 m/s; the governor slows it to 0.2 at hazards"
$D mkdir -p /tmp/nav_robot
sudo docker cp ~/nav_robot/. silvanus:/tmp/nav_robot/ > /dev/null
sudo docker cp ~/coverage_sweep.py silvanus:/tmp/nav_robot/coverage_sweep.py > /dev/null
sudo docker exec -d silvanus bash -c "$ROS && ros2 launch nav2_bringup bringup_launch.py \
map:=/tmp/nav_robot/loop3_fixed_map.yaml params_file:=/tmp/nav_robot/nav2_robot_governor.yaml use_sim_time:=False \
> /tmp/nav_robot/nav2.log 2>&1"
active() { n=$($D grep -c 'Managed nodes are active' /tmp/nav_robot/nav2.log 2>/dev/null); echo "${n:-0}"; }
for i in $(seq 60); do
    [ "$(active)" -ge 2 ] && break
    sleep 1
done
if [ "$(active)" -lt 2 ]; then
    echo "Nav2 did not come up in 60 s:"; $D tail -n 15 /tmp/nav_robot/nav2.log; exit 1
fi
echo "    Nav2 is up"
read -r -p "3/5 In RViz the scan must lie on the map walls (2D Pose Estimate if not). Enter to go on, Ctrl-C to stop: "

if [ "$DRY" = dry ]; then
    echo "4/5 dry run: no recording"
    echo "5/5 sweep: stand at the e-stop"
    sudo docker exec -it silvanus bash -c "$ROS && python3 /tmp/nav_robot/coverage_sweep.py \
--x0 3.875 --y0 -0.294 --yaw 66.34 --length $LENGTH --width $WIDTH --spacing 0.8 --log /tmp/nav_robot/dry_coverage.csv"
    echo "Nav2 keeps running; drive back onto the tape before the real run"
    exit 0
fi

echo "4/5 recording (topic check first, about 20 s)"
EXTRA_TOPICS="/governor/state /governor/felt /ground/hazards /ground/unevenness /cmd_vel_in /cmd_vel /plan /amcl_pose" \
    ~/record_session.sh "$NAME" 420 > /tmp/sweep_record.txt 2>&1 &
REC=$!
for i in $(seq 60); do grep -q ">>> RECORDING" /tmp/sweep_record.txt && break; kill -0 $REC 2>/dev/null || break; sleep 1; done
if ! grep -q ">>> RECORDING" /tmp/sweep_record.txt; then cat /tmp/sweep_record.txt; echo "recording did not start"; exit 1; fi
BAG=$(grep ">>> RECORDING" /tmp/sweep_record.txt | sed 's/.*\/bags\/\([^ ]*\).*/\1/')
echo "    recording $BAG"
sleep 3

echo "5/5 sweep: stand at the e-stop"
sudo docker exec -it silvanus bash -c "$ROS && python3 /tmp/nav_robot/coverage_sweep.py \
--x0 3.875 --y0 -0.294 --yaw 66.34 --length $LENGTH --width $WIDTH --spacing 0.8 --log /bags/${BAG}_coverage.csv"

sleep 3
$D pkill -INT -f "[r]os2 bag record"
kill $REC 2>/dev/null
sleep 5
$D bash -c "$ROS && ros2 bag info /bags/$BAG" | grep -E "Duration|Bag size|camera/color/image_raw|scan_fixed|imu/data"
echo ""
BAGS_DIR=${BAGS_DIR:-$(mountpoint -q /mnt/ssd && echo /mnt/ssd/silvanus_bags || echo /data/bags)}
echo "copy to the laptop:  scp -r \"agilex@192.168.1.102:$BAGS_DIR/${BAG}*\" ~/bags/"
echo "Nav2 keeps running; stop it with: sudo docker exec silvanus pkill -INT -f '[b]ringup_launch|[n]av2_container'"
