#!/bin/bash
# run_ekf.sh BAG OUT [PARAMS]  -- robot_localization EKF offline from a bag, isolated on ROS domain 87
BAG=$1; OUT=$2; PAR=${3:-$HOME/workspaces/silvanus-terrain-quality-mapping/config/ekf_odom_imu.yaml}
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
/opt/ros/jazzy/lib/robot_localization/ekf_node --ros-args --params-file $PAR -p use_sim_time:=true -r __node:=ekf_filter_node > ${OUT}_ekf.log 2>&1 & E=$!
python3 log_odom.py ${OUT}.csv > /dev/null 2>&1 & L=$!
sleep 3
ros2 bag play $BAG --clock --topics /odom /imu/data /tf /tf_static > /dev/null 2>&1
sleep 2; kill -INT $E $L 2>/dev/null; sleep 2; kill -9 $E $L 2>/dev/null; pkill -x ekf_node 2>/dev/null
echo "$OUT: $(grep -c ^ekf ${OUT}.csv) ekf rows, $(grep -c ^wheel ${OUT}.csv) wheel rows; warnings: $(grep -ciE 'warn|error' ${OUT}_ekf.log)"
