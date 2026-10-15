# Crack model: v1 handoff

The crack segmentation model for the ROS crack node, how to run it, what it can and cannot do yet, and how to fine-tune it on our own frames. Training code: `vision_training/`. Decisions and history: `docs/vision_log.md`.

## 1. The model

| | |
|---|---|
| Name | `crack_unet16_v1` |
| Architecture | U-Net, 4 levels, base width 16, 1.94M parameters (`vision_training/src/crackseg/models/unet.py`) |
| Trained on | DeepCrack + CrackForest, 342 training images, from scratch, no robot frames |
| Source run | `runs/unet16_public_20261006_125835` on the GPU server, best epoch 108 of 138 (early stop) |
| Input | RGB uint8 image of any size, normalised with ImageNet mean/std; height and width padded to a multiple of 16 |
| Output | one logit per pixel; sigmoid gives the crack probability |
| Threshold | 0.80, chosen on validation (stored in the JSON) |
| Speed, ONNX Runtime, CPU | 205 ms per 640x480 frame (4.9 fps) on a laptop with 8 threads, no GPU |

### Files

The JSON is in git. The two binary files are not; download them from the `crack-model-v1` release on GitHub (or the shared drive) into `vision_training/models/`.

| File | Size | Use | sha256 |
|---|---|---|---|
| `crack_unet16_v1.onnx` | 7.8 MB | the ROS node | `f79243ee83f28d0842fa4ae0093a0028caa608eab2c09b296ba68d89c318e540` |
| `crack_unet16_v1.pt` | 7.8 MB | PyTorch weights + config, starting point for fine-tuning | `cf4b60381f7ea9fcd7850a587c14fe1a8965fb447f01adb79aa18c0b5ba4f608` |
| `crack_unet16_v1.json` | 1 KB | threshold, normalisation, size multiple; the node reads it | in git |

Check a download with `sha256sum vision_training/models/*.onnx vision_training/models/*.pt`.

### Test results (public test images, threshold 0.80)

| Subset | Images | Dice | IoU | Precision | Recall | F1, 2 px tolerance |
|---|---|---|---|---|---|---|
| All | 255 | 0.833 | 0.714 | 0.908 | 0.770 | 0.912 |
| DeepCrack | 237 | 0.836 | 0.719 | | | 0.911 |
| CrackForest | 18 | 0.719 | 0.561 | | | 0.937 |

These numbers are on close-up photos of cracks. **They say nothing yet about our robot frames.** See section 3.

## 2. Using it in the ROS node

Copy `vision_training/deploy/crack_onnx.py` into the node package. It only needs `numpy` and `onnxruntime` (add `python3-numpy` and `pip install onnxruntime` to the Dockerfile; the robot has no internet, so it has to be in the image).

```python
from crack_onnx import CrackSegmenter, load_ground_mask

seg = CrackSegmenter("/ws/models/crack_unet16_v1.onnx", num_threads=2)
ground = load_ground_mask("/ws/models/robot_mask_color.png", 640, 480)   # white = ground

frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
prob, mask = seg.predict(frame, bgr=True, ground_mask=ground)
```

Things to get right:

- **Colour order.** cv_bridge gives BGR, the model was trained on RGB. Pass `bgr=True`.
- **Rate.** Do not try to run on every frame at 30 Hz. Process only the newest frame when the previous inference is done (subscription queue depth 1, drop the rest). At 4 to 5 Hz and 0.3 m/s the robot moves 6 to 8 cm between inferences, and Praveen measured that the camera sees each spot for about 4 s before the wheels reach it, so every crack is seen many times.
- **Robot body mask.** The depth mask `ground_demo/robot_mask_d435_gripper.png` (848x480) **cannot** be reused for colour: the D435 colour and depth sensors have different fields of view. Draw a separate mask on one colour frame (640x480), same convention (white = ground, black = robot).
- **Boxes for `/crack_detection/detections`.** Keep the existing JSON format. Boxes come from connected components of the mask:

```python
n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
boxes = []
for i in range(1, n):
    x, y, w, h, area = stats[i]
    if area < 150:          # same min size as scripts/predict.py; tune on our bags
        continue
    score = float(prob[labels == i].mean())
    boxes.append({"x": int(x), "y": int(y), "width": int(w), "height": int(h),
                  "area": float(area), "score": round(score, 3)})
```

- **Cracks in 3D.** For each component, read the depth of its pixels (aligned depth if available, otherwise project through `camera_info` of both streams), take the median 3D point as the crack position and the extent of the pixels as its length and width. Publish as type `crack` in the format of `docs/schema.md`.

## 3. What to expect on the robot (known limits)

The model has never seen a robot frame. The main differences:

| Public training photos | Robot camera |
|---|---|
| taken close, roughly straight down | 0.7 to 0.9 m high, tilted, looking 0.8 to 1.5 m ahead |
| cracks 5 to 30 px wide | cracks a few px wide, thinner further away (perspective) |
| concrete and asphalt only | also tile with grout lines, painted lines, drains, leaves, shadows |
| up to 544x384 | 640x480, rolling shutter, motion blur when driving fast |

Expect: wide asphalt cracks near the bottom of the ground area found; hairline cracks and far cracks missed (the worst public test images are already faint hairline cracks); false alarms on tile grout lines, dark seams and sharp shadows. The first job is to measure this (section 4) before deciding how much annotation is needed.

## 4. Step 1: measure the base model on our frames (no training)

This gives the table in the milestone list: detection rate on cracked asphalt, false alarms on indoor tile.

```bash
# frames from the bags (needs ROS 2 Jazzy for rosbag2_py), 2 fps
python3 tools/bag_to_images.py <bags>/garden_surfaces_1618 --out frames/garden_1618 --fps 2
python3 tools/bag_to_images.py <bags>/garden_surfaces_1548 --out frames/garden_1548 --fps 2
python3 tools/bag_to_images.py <bags>/corridor_book_1338_indoor --out frames/corridor_1338 --fps 2
```

Move the frames inside the cracked asphalt time ranges (`1618` 0-36 s, `1548` 57-72 s, the time is in the file name) into `frames/cracked/`, keep the tile frames in `frames/tile/`, then:

```bash
cd vision_training
python scripts/predict.py --checkpoint models/crack_unet16_v1.pt --images ../frames/cracked --mask <colour mask>
python scripts/predict.py --checkpoint models/crack_unet16_v1.pt --images ../frames/tile    --mask <colour mask>
```

Each run prints how many frames have a crack found, and writes `summary.csv` plus an overlay per frame. Cracked folder: share of frames with a crack found = detection rate. Tile folder: every frame with a crack found is a false alarm. Push a few overlays and the two numbers to `docs/`.

## 5. Step 2: annotate and fine-tune

**What to label.** Frames from `bag_to_images.py`, one folder per bag. For each frame with a crack, a mask PNG of the same size, white (255) on crack pixels, black elsewhere, named `<frame>_mask.png` next to the frame. Frames without cracks need **no** mask file: a frame without a mask is treated as a negative. Put tile, grout, painted lines and shadows in on purpose; they teach the model what is not a crack. Targets from the plan: about 120 frames with cracks, about 150 negatives.

**Tool.** CVAT or Label Studio, one class `crack`, brush. Export as segmentation masks and rename to `<frame>_mask.png` if the tool uses other names. Label the crack as you see it, a few px wide; do not paint the whole dark area around it.

**Layout.**

```
vision_training/data/raw/robot/
  garden_surfaces_1618/   garden_surfaces_1618_0012_6.0.png, garden_surfaces_1618_0012_6.0_mask.png, ...
  garden_surfaces_1548/   ...
  corridor_book_1338/     ...      (mostly negatives, no masks)
  splits.yaml
```

**Split by bag, never by frame.** Frames half a second apart show the same crack. `splits.yaml` fixes which bag goes where, for example:

```yaml
garden_surfaces_1618: train      # most cracked asphalt, 36 s
corridor_book_1434_indoor_books: train
garden_surfaces_1607: val
corridor_book_1338_indoor: val
garden_surfaces_1548: test       # cracked asphalt the model never trained on
corridor_groundtruth_1447: test  # tile false alarms
```

Then:

```bash
cd vision_training
python scripts/prepare_data.py           # picks up data/raw/robot automatically, prints the split table
python scripts/train.py --config configs/finetune_robot.yaml
python scripts/evaluate.py --checkpoint runs/<run>/best.pt --split test --sources robot
python scripts/export_onnx.py --checkpoint runs/<run>/best.pt --name crack_unet16_v2
```

`finetune_robot.yaml` starts from `models/crack_unet16_v1.pt`, keeps the public images in the training mix so the model does not forget them, and validates and tests on robot frames only. Report the robot test numbers next to the step 1 numbers; the difference is what fine-tuning bought.
