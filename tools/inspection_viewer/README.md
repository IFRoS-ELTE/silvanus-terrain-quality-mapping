# Inspection viewer

A page for the output of `tools/analysis/inspection_map.py`: the ground stitched from the robot
camera, the cracks on it, every bump and hole, and a work list sorted by priority with a photo and
the position along the route for each item.

Make the runs (each takes 1-2 minutes; needs `vision_training/models/crack_unet16_v1.onnx`):

```bash
source /opt/ros/jazzy/setup.bash
cd tools/analysis
for b in garden_surfaces_1628 garden_surfaces_1548 corridor_groundtruth_1447; do
  python3 inspection_map.py ~/bags/$b ../inspection_viewer/runs/${b##*_}
done
```

Open it (the page loads the run files, so it needs a local web server, not a double click):

```bash
cd tools/inspection_viewer && python3 -m http.server 8000
# then http://localhost:8000
```

`runs/` is not in git (about 9 MB of images); make it again with the commands above.

The same data in RViz, in 3D:

```bash
export ROS_DOMAIN_ID=91          # not the robot's 42
python3 tools/analysis/inspection_rviz.py tools/inspection_viewer/runs/1628
rviz2 -d tools/analysis/inspection.rviz        # second terminal, same ROS_DOMAIN_ID
```
