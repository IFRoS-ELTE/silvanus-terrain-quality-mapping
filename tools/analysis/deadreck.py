#!/usr/bin/env python3
"""Dead reckoning three ways: wheels only (as /odom), wheel speed + gyro yaw, wheel speed + Xsens filter yaw."""
import sys, json, numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions, StorageFilter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
out = {}
for b in sys.argv[1:]:
    r = SequentialReader(); r.open(StorageOptions(uri=f"/home/praveen/bags/{b}", storage_id="mcap"), ConverterOptions("", ""))
    r.set_filter(StorageFilter(topics=["/odom", "/imu/data"])); O, I = [], []
    while r.has_next():
        tp, data, t = r.read_next()
        if tp == "/odom":
            m = deserialize_message(data, Odometry); q = m.pose.pose.orientation
            O.append((t*1e-9, m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x, m.twist.twist.angular.z, np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
        else:
            m = deserialize_message(data, Imu); q = m.orientation
            I.append((t*1e-9, m.angular_velocity.z, np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    O, I = np.array(O), np.array(I); t0 = O[0,0]
    still = np.abs(O[:,3]) < 0.01
    tst = O[still, 0]
    bias = np.median(I[(I[:,0] < (tst[tst < t0 + 5].max() if (tst < t0 + 5).any() else t0 + 1)), 1]) if len(tst) else 0.0
    gyaw = np.concatenate([[0], np.cumsum(np.diff(I[:,0]) * (I[1:,1] - bias))])
    fyaw = np.unwrap(I[:,2]) - I[0,2]
    oyaw = np.unwrap(O[:,5]) - O[0,5]
    def path(yaw_t, yaw):
        y = np.interp(O[:,0], yaw_t, yaw); dt = np.diff(O[:,0]); v = O[1:,3]
        x = np.concatenate([[0], np.cumsum(v * dt * np.cos(y[1:]))]); yy = np.concatenate([[0], np.cumsum(v * dt * np.sin(y[1:]))]); return x, yy
    res = {"odom": (O[:,1] - O[0,1], O[:,2] - O[0,2]), "gyro": path(I[:,0], gyaw), "xsens": path(I[:,0], fyaw)}
    # rotate the raw odom path so it starts with heading 0, like the others
    c, s = np.cos(-O[0,5]), np.sin(-O[0,5]); ox, oy = res["odom"]; res["odom"] = (c*ox - s*oy, s*ox + c*oy)
    L = np.abs(O[1:,3] * np.diff(O[:,0])).sum()
    print(f"== {b}: driven {L:.1f} m; total turn: wheels {np.degrees(oyaw[-1]):+.0f} deg, gyro {np.degrees(gyaw[-1]):+.0f} deg, Xsens {np.degrees(fyaw[-1]):+.0f} deg (gyro bias {np.degrees(bias):+.3f} deg/s)")
    for k, (x, y) in res.items(): print(f"     {k:6s} start-to-end {np.hypot(x[-1], y[-1]):5.2f} m")
    out[b] = {k: (list(map(float, x[::10])), list(map(float, y[::10]))) for k, (x, y) in res.items()}
json.dump(out, open("deadreck.json", "w"))
