# Inspection viewer

A page for the output of `tools/analysis/inspection_map.py`: the ground stitched from the robot
camera, the cracks on it, every bump and hole, and a work list sorted by priority with a photo and
the position along the route for each item.

To make the runs again, for example for a new bag (each takes 1-2 minutes; needs
`vision_training/models/crack_unet16_v1.onnx`):

```bash
source /opt/ros/jazzy/setup.bash
cd tools/analysis
for b in garden_surfaces_1628 garden_surfaces_1548 corridor_groundtruth_1447; do
  python3 inspection_map.py ~/bags/$b ../inspection_viewer/runs/${b##*_}
done
```

The three runs of 2026-10-08 (1628, 1548, 1447) are in `runs/`, and the corridor sweep of
2026-10-27 (1829). That one is placed with offline SLAM instead of wheels + IMU, with the lidar map
under the photo and the route in orange where the speed governor slowed the robot:

```bash
bash tools/analysis/run_slam.sh ~/bags/corridor_sweep_1829 sweep /scan_fixed      # sweep_poses.csv, sweep_map.*
python3 tools/analysis/inspection_map.py ~/bags/corridor_sweep_1829 tools/inspection_viewer/runs/1829 \
    --poses sweep_poses.csv --map sweep_map.yaml
```

**Live.** During a run on the robot, `tools/analysis/inspection_live.py` writes a run called `live`
every 2 s and `tools/robot/live_web.sh` serves this page from the robot: open
`http://192.168.1.102:8000` on the laptop. The live run is listed first and reloads itself.

Open the page with a local web server, because it loads the run files; a double click on
index.html does not work:

```bash
cd tools/inspection_viewer && python3 -m http.server 8000
# then http://localhost:8000
```

The same data in RViz, in 3D:

```bash
export ROS_DOMAIN_ID=91          # not the robot's 42
python3 tools/analysis/inspection_rviz.py tools/inspection_viewer/runs/1628
rviz2 -d tools/analysis/inspection.rviz        # second terminal, same ROS_DOMAIN_ID
```
