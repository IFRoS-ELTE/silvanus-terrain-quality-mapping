# Analysis scripts

Offline scripts behind the figures in `docs/PROGRESS.md`. They read bags from
`~/bags/<name>` and write their output to the current folder. ROS 2 Jazzy must be
sourced.

| script | what it does |
|---|---|
| `eval_groundtruth.py BAG...` | runs the depth measurement (`ground_probe.find_hazards`, robot mask on) over every 3rd depth frame and writes `<bag>_repo.json` |
| `score_groundtruth.py` | compares those detections with the tape-measured objects of 2026-10-08 and prints old vs new height errors |
| `run_slam.sh BAG OUT SCAN [LO HI]` | SLAM Toolbox with the lab's own config, offline from a bag, isolated on ROS domain 88. With `LO HI` it removes scan beams in that range first (`clean_scan.py`). Writes `OUT_map.pgm/.yaml` and `OUT_poses.csv` (`pose_logger.py`) |
| `draw_maps.py` | draws those maps with the SLAM and wheel-odometry paths |
| `deadreck.py BAG...` | dead reckoning with wheel heading, gyro-integrated heading and Xsens heading; prints how far each ends from the start |
