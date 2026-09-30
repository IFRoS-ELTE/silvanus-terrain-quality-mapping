# silvanus-terrain-quality-mapping

Terrain quality and hazard mapping with the Silvanus robot (AgileX Scout 2.0 with a RealSense D435 and an Xsens IMU). IFRoS Robotics Lab, ELTE, autumn 2026.

The robot is driven along a route. The camera looks at the ground ahead and predicts what is coming (holes, bumps, unevenness, cracks, terrain type). The IMU measures what the robot actually felt when it drove over it. Everything is stamped with position and speed and stored on a map, and the robot slows down or stops before hazards. In short: the camera predicts, the IMU confirms.

## Repository layout

| Folder | Owner | Status |
|---|---|---|
| `vision_training/` | Person B (vision) | Crack segmentation data pipeline, training, evaluation and ONNX export. Working. |
| `terrain_vision/` | Person B (vision) | ROS 2 package: depth hazard node, crack node, RGB terrain node. Not started. |
| `terrain_bringup/` | Person A (robot) | Teleop, speed governor, `ground_cam` transform, route recorder. Not started. |
| `terrain_imu/`, `terrain_dashboard/` | Person C (IMU, fusion) | IMU roughness, fusion, hazard states, report. Not started. |
| `docs/` | everyone | Interface contracts and work logs. |

Target platform for the ROS packages: ROS 2 Jazzy on Ubuntu 24.04 (Docker, `ros:jazzy-ros-base`). `vision_training/` is plain Python and PyTorch and does not need ROS.

## Quick start for the vision training code

```bash
cd vision_training
pip install -e ".[onnx,dev]"
python scripts/download_data.py
python scripts/prepare_data.py
python scripts/train.py --config configs/smoke_cpu.yaml
```

See [vision_training/README.md](vision_training/README.md) for the full workflow and [docs/vision_log.md](docs/vision_log.md) for decisions, results and open items on the vision side.
