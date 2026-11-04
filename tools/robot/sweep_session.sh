#!/bin/bash
# sweep_session.sh [NAME] [dry]
#
# Run on the robot. Nav2 on the indoor_loop3_1316 map drives a coverage sweep ahead of the robot through
# the speed governor. The lanes are planned on the saved map (navigation/lane_planner.py: at least 0.6 m
# from walls, 1.25 m free where the robot turns with the arm out), up to LENGTH m ahead and WIDTH m across: the depth camera
# finds a bump or hole ahead, the governor slows to 0.2 m/s, the IMU feels it as the wheels cross,
# and the governor stops for tall ones and for anything the lidar sees in front of the bumper.
# Everything is recorded, for the inspection map. With "dry" it drives the same sweep without recording, to see the path first.
# The robot starts on the tape facing down the corridor; the lanes run along that heading. Needs:
#   the stack (start_robot.sh), the arm in Pose B (arm_to_pose.py)
#   the robot on the tape, facing down the corridor as at the start of loop 3 (AMCL starts there)
#   ~/nav_robot (map + nav2_robot_governor.yaml), ~/auto_sweep.py, ~/lane_planner.py, ~/record_session.sh
# Ctrl-C during the sweep cancels the Nav2 goal; the e-stop always works.
set -u
NAME=${1:-corridor_sweep}
DRY=${2:-}
LENGTH=${LENGTH:-8}          # how far ahead of the robot the area reaches, m
WIDTH=${WIDTH:-4}            # how wide it is, centred on the robot, m
ROS="source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash"
D="sudo docker exec silvanus"
for f in ~/nav_robot/nav2_robot_governor.yaml ~/nav_robot/loop3_fixed_map.yaml ~/auto_sweep.py ~/lane_planner.py ~/record_session.sh \
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
for f in auto_sweep.py lane_planner.py; do sudo docker cp ~/$f silvanus:/tmp/nav_robot/$f > /dev/null; done
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
    sudo docker exec -it silvanus bash -c "$ROS && cd /tmp/nav_robot && python3 auto_sweep.py --no-spin \
--length $LENGTH --width $WIDTH --log /tmp/nav_robot/dry_coverage.csv"
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
sudo docker exec -it silvanus bash -c "$ROS && cd /tmp/nav_robot && python3 auto_sweep.py --no-spin \
--length $LENGTH --width $WIDTH --log /bags/${BAG}_coverage.csv"

sleep 3
$D pkill -INT -f "[r]os2 bag record"
kill $REC 2>/dev/null
sleep 5
$D bash -c "$ROS && ros2 bag info /bags/$BAG" | grep -E "Duration|Bag size|camera/color/image_raw|scan_fixed|imu/data"
echo ""
BAGS_DIR=${BAGS_DIR:-$(mountpoint -q /mnt/ssd && echo /mnt/ssd/silvanus_bags || echo /data/bags)}
echo "copy to the laptop:  scp -r \"agilex@192.168.1.102:$BAGS_DIR/${BAG}*\" ~/bags/"
echo "Nav2 keeps running; stop it with: sudo docker exec silvanus pkill -INT -f '[b]ringup_launch|[n]av2_container'"
