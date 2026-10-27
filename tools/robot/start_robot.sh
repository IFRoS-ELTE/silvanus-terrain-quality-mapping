#!/bin/bash
# start_robot.sh [extra silvanus-up.sh options, e.g. --no-base]
#
# Run on the robot (agilex@agilex-desktop). Does the whole start sequence:
# CAN up, find the Xsens port, check the camera is on USB, mount the recording SSD
# if it is plugged in, stop any running stack, start the lab stack with the measured
# IMU transform and 921600 baud. Recordings go to the SSD when it is mounted
# (/mnt/ssd/silvanus_bags), otherwise to /data/bags; BAGS_DIR=... overrides.
set -u
cd /opt/silvanus || { echo "no /opt/silvanus on this machine - run this on the robot"; exit 1; }

if ! ip link show can0 2>/dev/null | grep -q 'state UP'; then
    echo "can0 is down - bringing it up"
    sudo modprobe gs_usb && sudo ip link set can0 up type can bitrate 500000 || exit 1
fi
echo "can0 UP"

XS=$(ls /dev/serial/by-id/ 2>/dev/null | grep -m1 Xsens)
if [ -z "$XS" ]; then
    echo "Xsens not found in /dev/serial/by-id - check the IMU's USB cable"; exit 1
fi
DEV=$(readlink -f "/dev/serial/by-id/$XS")
echo "Xsens at $DEV"

if ! lsusb | grep -q 8086:0b07; then
    echo "WARNING: the D435 is not on USB - replug the camera cable, then run this again"
fi

SSD=/dev/disk/by-label/Robot-Data            # the professor's 1.8 TB exFAT SSD
if [ -e $SSD ] && ! mountpoint -q /mnt/ssd; then
    echo "mounting the SSD at /mnt/ssd"
    sudo mount -t exfat $SSD /mnt/ssd || echo "WARNING: could not mount the SSD"
fi
if [ -z "${BAGS_DIR:-}" ] && mountpoint -q /mnt/ssd; then
    export BAGS_DIR=/mnt/ssd/silvanus_bags
fi
echo "recordings go to ${BAGS_DIR:-/data/bags}"

if sudo docker ps --format '{{.Names}}' | grep -qx silvanus; then
    echo "stopping the running stack"
    sudo docker stop --time 30 silvanus > /dev/null
fi

# IMU mounting transform and baudrate: docs/robot/imu_mount.md
XSENS_BAUDRATE=921600 XSENS_PARENT_FRAME=base_link \
XSENS_X=0.0 XSENS_Y=-0.015 XSENS_Z=0.115 XSENS_ROLL=0.0 XSENS_PITCH=0.0 XSENS_YAW=0.0 \
    ./silvanus-up.sh --arm=192.168.1.219 --no-gripper --xsens="$DEV" --detach "$@" || exit 1

echo "waiting 30 s for the drivers"
sleep 30
if sudo docker ps --format '{{.Names}}' | grep -qx silvanus; then
    echo "stack running. Release the e-stops before driving."
else
    echo "stack NOT running - look at: sudo docker logs silvanus"; exit 1
fi
