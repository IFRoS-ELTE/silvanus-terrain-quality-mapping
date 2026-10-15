#!/bin/bash
# run_slam.sh BAG OUTNAME SCAN_TOPIC [LO HI]   -- offline SLAM Toolbox with the lab's config, isolated from the robot network
BAG=$1; OUT=$2; SCAN=$3; LO=$4; HI=$5
export ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
ros2 daemon stop >/dev/null 2>&1
P=$(mktemp --suffix=.yaml)
sed -e "s|scan_topic: /scan|scan_topic: $SCAN|" ~/workspaces/scout_xarm_ros2/src/scout_xarm_navigation/config/slam_toolbox.yaml > $P
pids=()
[ -n "$LO" ] && { python3 clean_scan.py $LO $HI > /dev/null 2>&1 & pids+=($!); }
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$P use_sim_time:=true autostart:=true > ${OUT}_slam.log 2>&1 & pids+=($!)
python3 pose_logger.py ${OUT}_poses.csv > /dev/null 2>&1 & pids+=($!)
sleep 5
ros2 bag play $BAG --clock --topics /scan /tf /tf_static /odom > ${OUT}_play.log 2>&1
sleep 3
timeout 30 ros2 run nav2_map_server map_saver_cli -f ${OUT}_map --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0 > ${OUT}_saver.log 2>&1
for p in "${pids[@]}"; do kill -INT $p 2>/dev/null; done; sleep 3; pkill -x async_slam_tool 2>/dev/null; for p in "${pids[@]}"; do kill -9 $p 2>/dev/null; done
echo "$OUT: map $(ls ${OUT}_map.pgm 2>/dev/null || echo MISSING), poses $(wc -l < ${OUT}_poses.csv)"
