#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# silvanus_evidence.sh - capture evidence for every issue found, in one file.
#
# Run ON THE ROBOT, with the stack already running:
#     ./silvanus-up.sh --arm=192.168.1.219 --no-gripper --xsens=/dev/ttyUSBn --detach
#     bash silvanus_evidence.sh
#
# Takes about 3 minutes. Does not restart anything. Writes one report file.
# ---------------------------------------------------------------------------
set -u
OUT="$HOME/silvanus-evidence-$(date +%Y%m%d-%H%M%S).txt"

say()  { printf '\n%s\n' "$*" | tee -a "$OUT"; }
head2(){ printf '\n\n=== %s ===\n' "$*" | tee -a "$OUT"; }
run()  { printf '\n$ %s\n' "$*" | tee -a "$OUT"; eval "$@" 2>&1 | tee -a "$OUT"; }
ros()  { printf '\n$ (container) %s\n' "$*" | tee -a "$OUT"
         sudo docker exec silvanus bash -c \
           "source /opt/ros/jazzy/setup.bash; source /opt/robot/setup.bash; $*" 2>&1 | tee -a "$OUT"; }

{
 echo "============================================================"
 echo " Silvanus - evidence for reported issues"
 echo " date : $(date -Is)"
 echo " host : $(hostname)   kernel $(uname -r)"
 echo "============================================================"
} | tee "$OUT"

if ! sudo docker ps --format '{{.Names}}' | grep -qx silvanus; then
  say "ERROR: the silvanus container is not running. Start the stack first."
  exit 1
fi
run "sudo docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'"

# ---------------------------------------------------------------------------
head2 "ISSUE 1 - serial device numbers are not stable between boots"
say "The udev rules in 99-silvanus-serial.rules.example were never installed,"
say "so /dev/robot/xsens_mti and /dev/robot/dh_ag95_gripper do not exist."
say "Without them the Xsens device number changes between boots, and"
say "--xsens=/dev/ttyUSBn has to be edited by hand every session."
run "ls -l /dev/serial/by-id/"
run "ls -l /dev/robot/ 2>&1 || echo '/dev/robot does not exist - udev rules not installed'"
run "ls -l /etc/udev/rules.d/99-silvanus-serial.rules 2>&1 || echo 'rule file not installed'"
say "Observed: Xsens was /dev/ttyUSB1 on 2026-09-30 and /dev/ttyUSB0 on 2026-10-01."

# ---------------------------------------------------------------------------
head2 "ISSUE 2 - the IMU publishes at 10 Hz, we need 100 Hz or more"
say "Terrain roughness is computed from vertical vibration in 1 s windows with"
say "FFT bands. At 10 Hz the usable bandwidth is 5 Hz, which is below the"
say "frequencies wheel and suspension vibration actually occupy."
ros "timeout 12 ros2 topic hz /imu/data || echo 'NO IMU DATA'"
say "The rate is stored in the sensor flash, not in software:"
run "grep -B2 -A2 'MT Manager' /opt/silvanus/bringup/xsens_mti_node.yaml"
run "grep -E 'enable_high_rate|enable_deviceConfig' /opt/silvanus/bringup/xsens_mti_node.yaml"
say "The sensor itself is healthy - gravity reads correctly:"
ros "timeout 10 ros2 topic echo /imu/data --field linear_acceleration --once || true"

# ---------------------------------------------------------------------------
head2 "ISSUE 3 - the IMU has no mounting transform"
say "A working frame, for comparison:"
ros "timeout 8 ros2 run tf2_ros tf2_echo base_link camera_link 2>&1 | head -8 || true"
say "The IMU frame - does not exist in the transform tree:"
ros "timeout 8 ros2 run tf2_ros tf2_echo base_link xsens_link 2>&1 | head -6 || true"
say "Every static transform published by the stack, for completeness:"
ros "timeout 10 ros2 topic echo /tf_static --once 2>/dev/null | grep -E 'frame_id|child_frame' | head -60 || true"

# ---------------------------------------------------------------------------
head2 "ISSUE 4 - compressed image transport is not in the image"
say "Only raw transport is available, so the camera cannot be viewed from a"
say "laptop: raw 640x480 at 30 Hz is about 220 Mbit/s and does not cross wifi."
say "RViz reports 'Some messages were lost' and displays nothing."
ros "ros2 run image_transport republish --ros-args -p in_transport:=raw -p out_transport:=compressed -r in:=/camera/camera/color/image_raw -r out/compressed:=/camera/evidence/compressed 2>&1 | head -8 || true"
ros "ros2 pkg list 2>/dev/null | grep -E 'image_transport' || true"

# ---------------------------------------------------------------------------
head2 "ISSUE 5 - storage"
say "The internal disk is nearly full. Raw colour plus depth is about 3 GB per"
say "minute, so the free space is roughly four minutes of recording."
run "df -h / /data/bags 2>/dev/null | sort -u"
run "df -h /mnt/usb 2>/dev/null || echo 'no pendrive mounted'"
say "The exFAT pendrive is also too slow to sustain raw recording: a 18.5 s run"
say "lost 1048 messages, leaving 4-5 second gaps in every topic."

# ---------------------------------------------------------------------------
head2 "ISSUE 6 - duplicate node names in the graph"
say "Duplicate names make DDS discovery unreliable; some nodes are visible from"
say "a laptop and others are not. ros2 warns about it on every call."
ros "ros2 node list 2>&1 | sort | uniq -c | sort -rn | head -12"

# ---------------------------------------------------------------------------
head2 "ISSUE 7 - MoveIt start tolerance is very tight"
say "allowed_start_tolerance defaults to 0.01 rad (0.57 deg). After the arm"
say "moves once, RViz's start state lags and every later trajectory is refused"
say "with: 'Invalid Trajectory: start point deviates from current robot state'."
say "Raising it to 0.1 makes the arm usable from RViz."
ros "ros2 param get /move_group trajectory_execution.allowed_start_tolerance 2>&1 || true"

# ---------------------------------------------------------------------------
head2 "ISSUE 8 - camera state right now"
say "The camera failed completely on 2026-09-30 (USB Protocol errors, device"
say "re-enumerated from /dev/video0 to /dev/video6) and worked all of"
say "2026-10-01. It is intermittent, not permanently broken."
run "lsusb | grep -i '8086:0b07' || echo 'NO REALSENSE ON USB'"
run "ls -l /dev/video* 2>/dev/null | head"
ros "timeout 12 ros2 topic hz /camera/camera/color/image_raw || echo 'NO CAMERA DATA'"
run "sudo docker logs --tail 3000 silvanus 2>&1 | grep -c 'Protocol error'"
run "sudo docker logs --tail 3000 silvanus 2>&1 | grep -ci 'Overrun detected'"

# ---------------------------------------------------------------------------
head2 "SUMMARY"
{
 echo "1. udev rules not installed; serial device numbers move between boots"
 echo "2. IMU publishes at 10 Hz; project needs 100 Hz or more"
 echo "3. xsens_link has no transform to base_link"
 echo "4. compressed_image_transport missing from the image"
 echo "5. internal disk 95% full; exFAT pendrive too slow for raw recording"
 echo "6. duplicate node names in the ROS graph"
 echo "7. MoveIt allowed_start_tolerance 0.01 blocks repeated arm moves"
 echo "8. camera USB link is intermittent"
 echo
 echo "Evidence file: $OUT"
} | tee -a "$OUT"

say "Done. Attach: $OUT"
