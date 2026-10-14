# Analysis scripts

Offline scripts behind the figures in `docs/PROGRESS.md`. They read bags from
`~/bags/<name>` and write their output to the current folder. ROS 2 Jazzy must be
sourced.

| script | what it does |
|---|---|
| `eval_groundtruth.py BAG...` | runs the depth measurement (`ground_probe.find_hazards`, robot mask on) over every 3rd depth frame and writes `<bag>_repo.json` |
| `score_groundtruth.py` | compares those detections with the tape-measured objects of 2026-10-08 and prints old vs new height errors |
| `groundtruth_objects.py` | the tape-measured objects and when each one is in view in each bag; used by the scripts above and below |
| `render_detection.py BAG T0 T1 OUT` | full-resolution video of the detection (camera with boxes, height map) and a still of each object with measured vs tape height. `~/bags/crossing_slow_1652 5 42` made `docs/media/crossing_detection.mp4` and `detection_objects.jpg` |
| `run_slam.sh BAG OUT SCAN [LO HI]` | SLAM Toolbox with the lab's own config, offline from a bag, isolated on ROS domain 88. With `LO HI` it removes scan beams in that range first (`clean_scan.py`). Writes `OUT_map.pgm/.yaml` and `OUT_poses.csv` (`pose_logger.py`) |
| `draw_maps.py` | draws those maps with the SLAM and wheel-odometry paths |
| `deadreck.py BAG...` | dead reckoning with wheel heading, gyro-integrated heading and Xsens heading; prints how far each ends from the start |
| `run_ekf.sh BAG OUT [PARAMS]` | `robot_localization` EKF offline from a bag with `config/ekf_odom_imu.yaml`, isolated on ROS domain 87; `log_odom.py` writes `/odom` and `/odometry/filtered` to `OUT.csv` |

Robot-side scripts are in `tools/robot/` (`start_robot.sh`, `record_session.sh`); see
`docs/robot/robot_session_guide.md`.
