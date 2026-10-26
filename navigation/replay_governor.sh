#!/bin/bash
# replay_governor.sh [BAG] [SPEED]
#
# The speed governor on a recording, without the robot. The depth node finds the
# hazards, a stand-in operator asks for SPEED m/s (default 0.4) all the time, and
# governor_watch.py prints what the governor makes of it. Runs on ROS domain 95,
# localhost only, so nothing reaches the robot.
#
#   navigation/replay_governor.sh ~/bags/corridor_groundtruth_1447 0.4
BAG=${1:-$HOME/bags/corridor_groundtruth_1447}
SPEED=${2:-0.4}
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
PIDS=()
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; sleep 1; kill "${PIDS[@]}" 2>/dev/null' EXIT
python3 ground_demo/ground_hazard_node.py --ros-args -p use_sim_time:=true > /tmp/replay_depth.log 2>&1 & PIDS+=($!)
python3 navigation/speed_governor.py --ros-args -p use_sim_time:=true -p enable:=true -p scan_topic:=/scan \
    > /tmp/replay_governor.log 2>&1 & PIDS+=($!)
ros2 topic pub -r 20 /cmd_vel_in geometry_msgs/msg/Twist "{linear: {x: $SPEED}}" > /dev/null 2>&1 & PIDS+=($!)
python3 navigation/governor_watch.py --ros-args -p use_sim_time:=true 2>/dev/null & PIDS+=($!)
sleep 4
echo "replaying $(basename "$BAG"), operator asks for $SPEED m/s ..."
ros2 bag play "$BAG" --clock --topics /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
    /tf /tf_static /odom /filter/free_acceleration /scan > /dev/null 2>&1
sleep 4
echo "done. Governor log: /tmp/replay_governor.log"
