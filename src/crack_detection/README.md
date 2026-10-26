# U-Net crack node

`crack_bbox_node` now uses the trained U-Net ONNX model instead of the dark-pixel
heuristic. A BEST_EFFORT subscription of depth 1 keeps only the newest RGB
frame, with at most 5 inferences per second. The threshold defaults to the
model JSON (v1: 0.80). Components smaller than 150 crack pixels are excluded
from boxes; the separate segmentation image preserves thin predicted pixels.

## Setup

From the repository root, with ROS 2 Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r src/crack_detection/requirements.txt
# ros2 launch uses the ROS Python interpreter, so expose this venv's runtime:
export PYTHONPATH="$VIRTUAL_ENV/lib/python3.12/site-packages:${PYTHONPATH:-}"
```

Get `crack_unet16_v1.onnx` from the team's `crack-model-v1` release or shared
model folder, following `docs/crack_model.md`. Put it in
`src/crack_detection/models/` **before building**. Its SHA-256 must be:

```text
f79243ee83f28d0842fa4ae0093a0028caa608eab2c09b296ba68d89c318e540
```

```bash
sha256sum src/crack_detection/models/crack_unet16_v1.onnx
colcon build --packages-select crack_detection
source install/setup.bash
```

The JSON metadata and mask PNGs are committed. ONNX and PyTorch weights stay
ignored by Git; download the ONNX separately after cloning. PyTorch, HRSegNet,
training data and the original workspace are not required at runtime.

The Docker image installs ONNX Runtime at build time, so the robot needs no
internet connection. Place the ONNX at the package path above, then build:

```bash
docker build --platform linux/amd64 -t silvanus-crack:v1 .
```

## Run and view

In terminal 1, after the setup commands above:

```bash
ros2 launch crack_detection crack_bbox.launch.py use_sim_time:=true run_rviz:=true
```

In terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play /path/to/garden_surfaces_1536 --clock
```

Both terminals must use the same `ROS_DOMAIN_ID`. For the live camera, omit
`use_sim_time:=true`. Default RGB input: `/camera/camera/color/image_raw`.
Override it with `image_topic:=...` if needed. Add
`publish_cloud:=false publish_markers:=false` to run without depth.

For a custom model, pass `model_path:=/absolute/model.onnx`; the adjacent JSON
is used unless `config_path:=...` is supplied. Other launch arguments include
`threshold:=0.8`, `min_area:=150.0`, `min_length:=1`, `max_rate_hz:=5.0`,
`num_threads:=2`, `drop_straight_lines:=true` and `drop_step_edges:=true`. Inference time and frame counts are logged every 5 seconds.

## Outputs

| Topic | Type | Contents |
|---|---|---|
| `/crack_detection/detections` | `std_msgs/String` | Existing JSON: `stamp`, `frame_id`, `count`, `boxes` with `x`, `y`, `width`, `height`, `area`, `score` |
| `/crack_detection/debug_image` | `sensor_msgs/Image` | RGB with yellow boxes only |
| `/crack_detection/mask` | `sensor_msgs/Image` | Full-size mono8 segmentation, 0/255, after the false-crack filters |
| `/crack_detection/camera_info` | `sensor_msgs/CameraInfo` | Colour calibration with the processed RGB header |
| `/crack_detection/points` | `sensor_msgs/PointCloud2` | Yellow measured surface points associated with cracks |
| `/crack_detection/markers` | `visualization_msgs/MarkerArray` | Short-lived RViz points |

The 2D JSON contract stays compatible with `docs/schema.md`. Box scores are
mean component probabilities; `area` is the component pixel count. RGB and mask
outputs retain the RGB stamp and optical frame. Segmentation is published
separately and is never tinted onto the RGB debug image.

Depth is not aligned to colour. Projection pairs depth within 50 ms of RGB,
uses both CameraInfo topics and depth-to-colour TF (including colour
distortion), and rejects invalid depths and robot pixels. Defaults: 0.30–1.60 m
depth, stride 2, at most 5,000 points. Clouds retain the depth stamp and optical
frame. The RViz config uses `camera_depth_optical_frame` as the fixed frame.

`target_frame:=map` transforms points when the required TF exists at the depth
stamp. Persistent map storage, tracking and the future
`/crack_detection/hazards` Detection3DArray interface from the team guide still
require fusion work. If depth, calibration or TF is unavailable, 2D outputs
continue; 3D outputs are empty with a throttled diagnostic.

## False-crack filters

The U-Net also marks tile grout, painted lines and the edges of kerbs and door steps.
Two filters remove them from the mask before boxes, 3D points and the published mask
are made. Both are on by default:

| Parameter | Default | What it removes |
|---|---|---|
| `drop_straight_lines` | `true` | straight runs of `straight_min_length_px` (100) or more: tile grout, painted lines |
| `drop_step_edges` | `true` | crack pixels on a height step in the depth image (Sobel of the 5x5 median depth above `step_edge_threshold_m`, 0.03): kerb and door edges. Needs the paired depth frame and TF; without them the frame is not filtered and a warning is logged |

Measured with this node on the 2026-10-08 recordings (`ros2 bag play`, 5 Hz):

| Recording | Filters off | Filters on |
|---|---|---|
| cracked asphalt, `garden_surfaces_1618` 0-20 s | crack in 100 % of frames, median 2,569 px, 268 3D points | 100 % of frames, 2,470 px (96 % kept), 298 3D points |
| indoor tile, `corridor_book_1338_indoor` 0-25 s | 100 % of frames, median 12,708 px of grout, 1,537 3D points | median 423 px (97 % removed), 1 3D point |
| tile and the door step, `garden_surfaces_1536` 0-28 s | median 1,792 px | median 71 px |

About 60 % of tile frames still keep a short piece of grout (around 400 px). Training
with our own tile frames as "no crack" examples is the real fix (`docs/crack_model.md`, step 6).

## Robot masks

`models/robot_mask_color.png` is for the **640x480 colour camera**: white means
ground, black means robot. Probabilities are zeroed under this mask before
extracting boxes, without cropping the image. Dimensions must match RGB.
Use `color_robot_mask:=/path/mask.png` for another camera configuration.

`models/robot_mask_depth.png` is the existing **848x480 depth-camera** mask,
used only to filter 3D points. It is never resized into colour space. Replace
it with `robot_mask:=/path/depth_mask.png`, or pass `robot_mask:=''` to disable
only that depth filter. Recheck both masks if the arm pose changes.
Recording sources and colour-mask validation are in `models/README.md`.

## Tests

After sourcing ROS, the built package and the Python environment:

```bash
python -m pytest src/crack_detection/test
```

Tests cover the existing JSON contract, padded and endian-aware image decoding,
colour conversion, component confidence, masking and calibrated 3D projection.
