# RGB terrain classifier

Tells which surface the robot camera is looking at: asphalt, red rubber path, wood chips, grass/dirt or indoor tile. It does not measure how rough the ground is; depth unevenness and the IMU already do that. Together they give "grass, rough" or "asphalt, smooth", and the comparison of vision-only, IMU-only and combined terrain classification from the evaluation plan.

Code: `vision_training/src/terrainclf/`, scripts `vision_training/scripts/terrain_*.py`, ROS node `src/rgb_terrain/`.

## How it works

1. **Labels.** The surface time ranges from `docs/DATA.md` are copied into `vision_training/configs/terrain_labels.yaml`. Frames within 2 s of a boundary are dropped: the labels are good to a few seconds and the camera sees the next surface about a metre before the robot reaches it.
2. **Frames.** Colour frames are read straight from the `.mcap` bags with the pure-Python `rosbags` package (no ROS needed, so it runs on the GPU server), 2 per second, timed from the first colour frame like `tools/bag_to_images.py`.
3. **Patches.** Each 640x480 frame gives six 200x200 ground patches in a 2x3 grid that ends where the colour robot mask starts (row 400), so the gripper is never in a patch.
4. **Model.** MobileNetV3-small pretrained on ImageNet (1.5M parameters), fine-tuned on the patches. Classes are sampled equally often, because wood chips has far fewer frames than grass. Augmentation: random crops, flips, brightness/contrast and a little saturation; hue is kept almost fixed because colour is a real cue (the red path is red).
5. **One answer per frame.** The six patch probabilities are averaged, so a leaf or a stone in one patch does not flip the label.
6. **Testing on unseen runs.** Split by bag, never by frame. Each fold trains on every labelled bag except its test bags:

| Fold | Test bags | Why |
|---|---|---|
| `test_1628_1447` (reported) | `garden_surfaces_1628`, `corridor_groundtruth_1447` | 1628 has all four outdoor surfaces; 1447 is the indoor tile test bag, the same one the crack model uses |
| `test_1618_1338` | `garden_surfaces_1618`, `corridor_book_1338_indoor` | Second held-out run, to show how much the result depends on which run is held out |

The number of epochs (12) is fixed in advance. There is no extra validation run to tune on, so nothing is tuned on the test runs.

## Frames available (2 fps, after the 2 s margins)

Extracted on 2026-11-09 from the bags on the shared drive:

| Bag | asphalt | red path | wood chips | grass/dirt | tile |
|---|---|---|---|---|---|
| garden_surfaces_1548 | 39 | | | 78 | |
| garden_surfaces_1607 | 4 | 6 | | 44 | |
| garden_surfaces_1618 | 64 | 18 | 16 | 36 | |
| garden_surfaces_1628 | 36 | 34 | 28 | 25 | |
| corridor_book_1338_indoor | | | | | 58 |
| corridor_book_1434_indoor_books | | | | | 58 |
| corridor_groundtruth_1447 | | | | | 72 |
| crossing_slow_1652 | | | | | 172 |
| **total** | **143** | **58** | **44** | **183** | **360** |

788 frames, 271 MB of PNG. Wood chips is the weak spot: in each fold the model trains on wood chips from a single run.

**Label check.** Every segment was checked on its first, middle and last frame, and the suspicious boundaries frame by frame. Three ranges in `docs/DATA.md` are off by a few seconds and would have put wrong frames into training, so `terrain_labels.yaml` uses corrected times:

| Bag | DATA.md | What the frames show | Used |
|---|---|---|---|
| 1548 | asphalt 0-10 | 2.5-6.5 s is a mossy concrete patch | asphalt 4.5-10 |
| 1607 | grass 8-30, red path 31-44 | grass to 34.2 s, kerb 34.7-35.7 s, red path from 36 s | grass 8-34, red path 34-41 |
| 1628 | grass/dirt 46-65 | concrete patch from 62.7 s, asphalt from 66.8 s | grass/dirt 46-62.5 |

**Bags from the Drive.** Downloading the `bags` folder as one zip gives the large `.mcap` files as loose files named `<bag>_0-00N.mcap` and the rest in several zips. Put each bag back into its own folder (`<bag>/<bag>_0.mcap` plus `metadata.yaml`, the names listed in `metadata.yaml`) before running the extractor.

## Running it on the GPU server

```bash
cd /scratch2/yamin/silvanus-terrain-quality-mapping
git switch main && git pull
cd vision_training && source .venv/bin/activate
pip install torchvision --index-url https://download.pytorch.org/whl/cu121   # same CUDA build as torch
pip install -e ".[onnx,dev]"
python -m pytest
```

Either copy the extracted frames (`data/terrain/`, 271 MB) to the server and skip the extraction, or put these bag folders from the shared drive in one folder, for example `/scratch2/yamin/bags/`: `garden_surfaces_1548`, `garden_surfaces_1607`, `garden_surfaces_1618`, `garden_surfaces_1628`, `corridor_book_1338_indoor`, `corridor_book_1434_indoor_books`, `corridor_groundtruth_1447`, `crossing_slow_1652`. Each is a folder with the `.mcap` and `metadata.yaml`.

```bash
python scripts/terrain_extract.py --bags /scratch2/yamin/bags          # prints frames per surface and bag
python scripts/terrain_train.py --config configs/terrain_mobilenet.yaml  # both folds, a few minutes on the GPU
```

Results land in `runs/terrain_mnv3s_<time>/`: `summary.md` (the per-surface tables), and per fold `confusion_matrix.png`, `metrics.json`, `model.pt`.

Model for the robot, trained on all bags with the same recipe, then exported:

```bash
python scripts/terrain_train.py --config configs/terrain_mobilenet.yaml --final
python scripts/terrain_export_onnx.py --checkpoint runs/terrain_mnv3s_final_<time>/model.pt
```

Copy `rgb_terrain_v1.onnx` and `rgb_terrain_v1.json` to `src/rgb_terrain/models/`.

## On the robot

```bash
ros2 launch rgb_terrain rgb_terrain.launch.py              # live camera
ros2 launch rgb_terrain rgb_terrain.launch.py use_sim_time:=true   # with ros2 bag play --clock
```

Publishes `{"label": "grass_dirt", "confidence": 0.87}` on `/ground/rgb_terrain` at 2 Hz. One frame takes about 8 ms on a laptop CPU. Needs `onnxruntime`, which the Dockerfile already installs for the crack node.

## Results

To be filled in after the first run on the server: per-surface accuracy for both folds, the pooled confusion matrix, and the CPU time per frame.
