#!/bin/bash
# governor_up.sh [MAX_SPEED]
#
# Run on the robot after start_robot.sh. Starts, inside the running container:
#   /scan_fixed       (the lidar scan without the floor ring, same node as record_session.sh)
#   the depth hazard node and the speed governor, from ~/governor:
#       ground_hazard_node.py ground_probe.py robot_mask_d435_gripper.png speed_governor.py
# then checks the camera, the scan and both logs. Drive with teleop remapped to /cmd_vel_in;
# the governor caps the forward speed at MAX_SPEED (default 0.3 m/s).
#
# The arm must be in Pose B (tools/robot/arm_to_pose.py), otherwise the camera sees the robot itself.
set -u
SPEED=${1:-0.3}
ROS="source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash"
sudo docker ps --format '{{.Names}}' | grep -qx silvanus || { echo "the stack is not running - run start_robot.sh first"; exit 1; }
[ -f ~/governor/speed_governor.py ] || { echo "no ~/governor/speed_governor.py - copy the files from the laptop first"; exit 1; }

sudo docker exec silvanus pkill -INT -f "[g]round_hazard_node.py|[s]peed_governor.py"
sudo docker exec silvanus mkdir -p /tmp/governor
sudo docker cp ~/governor/. silvanus:/tmp/governor/ > /dev/null

if ! grep -qs "__node:=scan_fixe[d]" /proc/[0-9]*/cmdline; then
    echo "starting /scan_fixed"
    sudo docker exec -d silvanus bash -c "$ROS && ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
-r __node:=scan_fixed -r cloud_in:=/rslidar_points -r scan:=/scan_fixed \
-p target_frame:=base_footprint -p transform_tolerance:=0.05 -p min_height:=0.10 -p max_height:=0.60 \
-p angle_min:=-3.14159265 -p angle_max:=3.14159265 -p angle_increment:=0.0087 -p scan_time:=0.1 \
-p range_min:=0.20 -p range_max:=100.0 -p use_inf:=true > /tmp/governor/scan_fixed.log 2>&1"
fi
sleep 2
sudo docker exec -d silvanus bash -c "$ROS && cd /tmp/governor && python3 ground_hazard_node.py > hazard.log 2>&1"
sudo docker exec -d silvanus bash -c "$ROS && cd /tmp/governor && python3 speed_governor.py --ros-args \
-p enable:=true -p max_speed:=$SPEED -p input_timeout_s:=1.0 > governor.log 2>&1"

echo "checking (15 s) ..."
sudo docker exec silvanus bash -c "$ROS && sleep 8 && \
echo -n 'camera depth: ' && (timeout 4 ros2 topic hz /camera/camera/depth/image_rect_raw 2>/dev/null | grep -m1 'average rate' || echo 'NOTHING - replug the camera and run start_robot.sh again') && \
echo -n 'scan_fixed:   ' && (timeout 4 ros2 topic hz /scan_fixed 2>/dev/null | grep -m1 'average rate' || echo 'NOTHING') && \
echo '--- hazard node' && tail -n 2 /tmp/governor/hazard.log && echo '--- governor' && tail -n 2 /tmp/governor/governor.log"
