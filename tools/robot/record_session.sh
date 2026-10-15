#!/bin/bash
# record_session.sh NAME [SECONDS] [lidar]
#
# Run on the robot (agilex@agilex-desktop) with the stack running.
#  1. starts /scan_fixed: the lab's /scan with the floor cut out (min_height +0.10)
#  2. records every topic for 12 s and stops if any is missing or the camera is dead
#  3. records NAME_HHMM for SECONDS (default 90) inside the container, so a Wi-Fi
#     drop does not stop it, then prints ros2 bag info
# With "lidar" as the third argument the cameras are left out (about 10 MB/s instead
# of 60), for long SLAM loops.
set -u
NAME=${1:?usage: record_session.sh NAME [SECONDS]}
DUR=${2:-90}
MODE=${3:-all}
ROS="source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash"
TOPICS="/camera/camera/color/image_raw /camera/camera/color/camera_info \
/camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info \
/imu/data /filter/free_acceleration /imu/time_ref /gnss \
/odom /scan /scan_fixed /rslidar_points /tf /tf_static /joint_states /scout_status"
RATE=60                                  # MB/s with camera and 3D lidar
if [ "$MODE" = lidar ]; then
    TOPICS=$(echo $TOPICS | tr ' ' '\n' | grep -v '^/camera/' | tr '\n' ' ')
    RATE=10
fi
SCAN_FIXED="ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
-r __node:=scan_fixed -r cloud_in:=/rslidar_points -r scan:=/scan_fixed \
-p target_frame:=base_footprint -p transform_tolerance:=0.05 -p min_height:=0.10 -p max_height:=0.60 \
-p angle_min:=-3.14159265 -p angle_max:=3.14159265 -p angle_increment:=0.0087 -p scan_time:=0.1 \
-p range_min:=0.20 -p range_max:=100.0 -p use_inf:=true"

sudo docker ps --format '{{.Names}}' | grep -qx silvanus || { echo "the stack is not running - run start_robot.sh first"; exit 1; }

AVAIL=$(df --output=avail -BG /data/bags | tail -1 | tr -dc 0-9)
NEED=$(( DUR * RATE / 1000 + 2 ))
if [ "$AVAIL" -lt "$NEED" ]; then echo "only ${AVAIL} GB free, need about ${NEED} GB - copy off and delete old bags"; exit 1; fi
echo "${AVAIL} GB free"

if ! sudo docker exec silvanus bash -c 'grep -qs "__node:=scan_fixed" /proc/[0-9]*/cmdline'; then
    echo "starting /scan_fixed"
    sudo docker exec -d silvanus bash -c "$ROS && $SCAN_FIXED"
    sleep 5
fi

echo "12 s check of every topic..."
sudo docker exec silvanus bash -c "$ROS && rm -rf /bags/_check && timeout -s INT 12 ros2 bag record -s mcap -o /bags/_check $TOPICS > /dev/null 2>&1; ros2 bag info /bags/_check" > /tmp/_check.txt 2>&1
sudo docker exec silvanus rm -rf /bags/_check
BAD=0
for t in $TOPICS; do
    n=$(grep "Topic: $t |" /tmp/_check.txt | sed 's/.*Count: \([0-9]*\).*/\1/')
    n=${n:-0}
    printf "  %-42s %6s\n" "$t" "$n"
    case $t in
        /tf_static) ;;
        /camera/camera/color/image_raw|/camera/camera/depth/image_rect_raw) [ "$n" -lt 150 ] && BAD=1 ;;
        *) [ "$n" -lt 1 ] && BAD=1 ;;
    esac
done
if [ $BAD = 1 ]; then
    echo "A topic is missing or the camera is not at 30 Hz. If the camera shows ~0: replug its cable and run start_robot.sh again."
    exit 1
fi

BAG=${NAME}_$(date +%H%M)
sudo docker exec -d silvanus bash -c "$ROS && timeout -s INT $DUR ros2 bag record -s mcap -o /bags/$BAG $TOPICS > /bags/$BAG.log 2>&1"
echo ""
echo ">>> RECORDING /bags/$BAG for $DUR s - start driving now (voice memo: say each surface)"
sleep $(( DUR + 8 ))
sudo docker exec silvanus bash -c "$ROS && ros2 bag info /bags/$BAG"
echo ""
echo "copy to the laptop:  scp -r agilex@192.168.1.102:/data/bags/$BAG ~/bags/"
