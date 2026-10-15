# silvanus-terrain-quality-mapping

Terrain quality and hazard mapping with the Silvanus robot (AgileX Scout 2.0 with a RealSense D435 and an Xsens IMU). IFRoS Robotics Lab, ELTE, autumn 2026.

The robot is driven along a route. The camera looks at the ground ahead and predicts what is coming (holes, bumps, unevenness, cracks, terrain type). The IMU measures what the robot actually felt when it drove over it. Everything is stamped with position and speed and stored on a map, and the robot slows down or stops before hazards. In short: the camera predicts, the IMU confirms.

## Simulation

Silvanus in Gazebo with the arm posed so the wrist camera looks at the ground, driving towards a patch of cracked pavement:

<p align="center">
  <img src="docs/images/sim_gazebo_world.jpeg" width="80%" alt="Silvanus in the Gazebo world in front of a cracked pavement patch">
</p>

The camera stream in RViz with the first crack detection node running on it. Each green box is a crack candidate, published as JSON on `/crack_detection/detections` and drawn on `/crack_detection/debug_image`:

<p align="center">
  <img src="docs/images/sim_crack_boxes_1.jpeg" width="32%" alt="Crack candidates in the camera view, frame 1">
  <img src="docs/images/sim_crack_boxes_2.jpeg" width="32%" alt="Crack candidates in the camera view, frame 2">
  <img src="docs/images/sim_crack_boxes_3.jpeg" width="32%" alt="Crack candidates in the camera view, frame 3">
</p>

The lower half of the view is the robot's own front bumper and wheels, and some boxes land there. A fixed mask for the robot body (and later the gripper on the real arm) is one of the next steps, and the learned segmentation model below replaces the dark-pixel threshold.

## Where things stand

**Progress against the milestones, with results, figures and video: [docs/PROGRESS.md](docs/PROGRESS.md).**

<p align="center">
  <img src="docs/media/crossing_detection.gif" width="80%" alt="Depth hazard detection on the robot: camera view and height above the floor with detected objects">
</p>

**Depth hazard detection** (`ground_demo/`). A ROS 2 node that fits the ground plane in every depth frame and reports bumps and holes with their size. Against tape-measured objects from 15 to 50 mm the height is within 1.7 mm on average. The IMU on the robot is now calibrated and runs at 100 Hz, and confirms a 50 mm hazard when the wheels cross it. Recorded data and surface labels: [docs/DATA.md](docs/DATA.md).

**Crack segmentation model** (`vision_training/`). A complete offline pipeline in PyTorch: it downloads the public DeepCrack and CrackForest datasets, splits them so crops of the same photo never land in both train and test, trains a small U-Net, evaluates it, and exports it to ONNX for CPU inference on the robot. Model v1 (1.9M parameters) reaches Dice 0.83 on the public test set and runs at about 5 fps on 640x480 frames on a laptop CPU. It is released for the crack node and will be fine-tuned on frames from our own bags: [docs/crack_model.md](docs/crack_model.md).

**ROS 2 crack detection node and robot container** (`src/crack_detection/`, `Dockerfile`). A ROS 2 Jazzy node that publishes crack boxes and a debug image from the colour camera; the screenshots above are its first, threshold-based version, which the U-Net replaces. The Dockerfile builds on `ros:jazzy-ros-base`, the same base as the lab's robot image, so the container runs next to the robot's hardware stack.

**Next.** The U-Net in the crack node with a colour mask for the gripper, our own annotated frames for fine-tuning, cracks placed in 3D, an RGB terrain classifier for the surface type, the IMU roughness node, and indoor SLAM once the lidar scan is fixed.

## Quick start

Crack segmentation training (plain Python, no ROS needed):

```bash
cd vision_training
pip install -e ".[onnx,dev]"
python scripts/download_data.py
python scripts/prepare_data.py
python scripts/train.py --config configs/smoke_cpu.yaml
```

See [vision_training/README.md](vision_training/README.md) for the full workflow, and [docs/vision_log.md](docs/vision_log.md) for decisions, results and open items on the vision side.

ROS packages target ROS 2 Jazzy on Ubuntu 24.04 inside Docker.
