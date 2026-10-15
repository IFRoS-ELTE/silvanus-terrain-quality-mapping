#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# silvanus_diag.sh - collect evidence for three suspected hardware faults.
#
# Run ON THE ROBOT:   bash silvanus_diag.sh
# Takes about 5 minutes. Restarts the hardware stack twice.
# Writes a single report file you can attach to an email or a GitHub issue.
#
# Do not run while someone else is using the robot.
# ---------------------------------------------------------------------------
set -u

OUT="$HOME/silvanus-diag-$(date +%Y%m%d-%H%M%S).txt"
SIL=/opt/silvanus
ARM_IP=192.168.1.219
XSENS_DEV=/dev/ttyUSB1

say() { printf '\n%s\n' "$*" | tee -a "$OUT"; }
run() { printf '\n$ %s\n' "$*" | tee -a "$OUT"; eval "$@" 2>&1 | tee -a "$OUT"; }

# ROS commands have to run inside the silvanus container, with ROS sourced.
ros() {
    sudo docker exec silvanus bash -lc \
        "source /opt/ros/jazzy/setup.bash; source /opt/robot/setup.bash; $*" 2>&1
}
ros_logged() { printf '\n$ (in container) %s\n' "$*" | tee -a "$OUT"; ros "$@" | tee -a "$OUT"; }

{
  echo "============================================================"
  echo " Silvanus hardware diagnostic"
  echo " date    : $(date -Is)"
  echo " host    : $(hostname)"
  echo " kernel  : $(uname -r)"
  echo " operator: ${USER}"
  echo "============================================================"
} | tee "$OUT"

# ---------------------------------------------------------------------------
say "### 0. System state"
run "lsb_release -ds"
run "df -h / | tail -1"
run "ip -brief -details link show can0 | head -2"

# ---------------------------------------------------------------------------
say "### 1. Is the camera present on USB, and on which device nodes?"
say "Expected: an Intel 8086:0b07 device, and /dev/video0 upward."
say "If the nodes start at video6 or higher, the camera has re-enumerated"
say "after a USB dropout, which is itself a symptom."
run "lsusb | grep -i '8086:0b07' || echo 'NO INTEL REALSENSE ON USB'"
run "ls -l /dev/video* 2>/dev/null || echo 'NO /dev/video* AT ALL'"

# ---------------------------------------------------------------------------
say "============================================================"
say "### 2. TEST A - stack WITH camera"
say "============================================================"
run "sudo docker rm -f silvanus 2>/dev/null || true"
say "Starting: camera on, arm on, base off."
run "cd $SIL && ./silvanus-up.sh --no-base --arm=$ARM_IP --no-gripper --xsens=$XSENS_DEV --detach"
say "Waiting 60 s for drivers to settle..."
sleep 60

say "--- camera publish rate (10 s sample) ---"
ros_logged "timeout 12 ros2 topic hz /camera/camera/color/image_raw || echo 'NO CAMERA MESSAGES'"

say "--- RealSense USB protocol errors in the log ---"
run "sudo docker logs --tail 2000 silvanus 2>&1 | grep -c 'Protocol error' || true"
say "(a nonzero count means the driver is retrying a broken USB link)"
run "sudo docker logs --tail 2000 silvanus 2>&1 | grep -m3 'Protocol error' || echo 'none'"
run "sudo docker logs --tail 2000 silvanus 2>&1 | grep -m3 'No such device' || echo 'none'"

say "--- real-time control loop failures WITH camera ---"
OVERRUN_WITH=$(sudo docker logs --tail 2000 silvanus 2>&1 | grep -ci 'Overrun detected')
echo "Overrun count WITH camera: $OVERRUN_WITH" | tee -a "$OUT"
run "sudo docker logs --tail 2000 silvanus 2>&1 | grep -m2 'Overrun detected' || echo 'none'"

say "--- IMU rate (the project needs 100 Hz or more) ---"
ros_logged "timeout 12 ros2 topic hz /imu/data || echo 'NO IMU MESSAGES'"

say "--- IMU gravity check (robot must be level and still) ---"
ros_logged "timeout 10 ros2 topic echo /imu/data --field linear_acceleration --once || true"

say "--- is the IMU in the transform tree? ---"
say "A working frame, for comparison:"
ros_logged "timeout 8 ros2 run tf2_ros tf2_echo base_link camera_link 2>&1 | head -12 || true"
say "The IMU frame:"
ros_logged "timeout 8 ros2 run tf2_ros tf2_echo base_link xsens_link 2>&1 | head -12 || true"

say "--- a short bag, to show whether camera frames are actually captured ---"
ros_logged "cd /bags 2>/dev/null || cd /tmp; rm -rf diagbag; timeout 25 ros2 bag record -s mcap -o diagbag --topics /camera/camera/color/image_raw /camera/camera/depth/image_rect_raw /imu/data /odom /tf /tf_static >/dev/null 2>&1; echo recorded"
ros_logged "cd /bags 2>/dev/null || cd /tmp; ros2 bag info diagbag || true"

# ---------------------------------------------------------------------------
say "============================================================"
say "### 3. TEST B - identical stack, camera DISABLED"
say "============================================================"
say "Only one flag differs: --no-camera. Everything else is the same."
run "sudo docker rm -f silvanus 2>/dev/null || true"
run "cd $SIL && ./silvanus-up.sh --no-base --no-camera --arm=$ARM_IP --no-gripper --xsens=$XSENS_DEV --detach"
say "Waiting 60 s..."
sleep 60

say "--- real-time control loop failures WITHOUT camera ---"
OVERRUN_WITHOUT=$(sudo docker logs --tail 2000 silvanus 2>&1 | grep -ci 'Overrun detected')
echo "Overrun count WITHOUT camera: $OVERRUN_WITHOUT" | tee -a "$OUT"

# ---------------------------------------------------------------------------
say "============================================================"
say "### 4. SUMMARY"
say "============================================================"
{
  echo "Control-loop overruns WITH camera   : $OVERRUN_WITH"
  echo "Control-loop overruns WITHOUT camera: $OVERRUN_WITHOUT"
  echo
  echo "The controller manager must complete a cycle every 6.67 ms (150 Hz)."
  echo "Every overrun is a missed cycle. When overruns are frequent, MoveIt"
  echo "rejects arm trajectories with:"
  echo "  'Invalid Trajectory: start point deviates from current robot state'"
  echo
  echo "Report written to: $OUT"
} | tee -a "$OUT"

say "Done. Attach this file: $OUT"
