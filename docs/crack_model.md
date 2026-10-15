# Crack model handoff (for Aritra)

Everything you need to take over the crack model, in the order you will need it. You own the crack model from here: measuring it on our bags, the gripper mask, annotation, fine-tuning, and running it in the ROS node. Yamin trained the first version (v1) and can answer questions about the training code; the history of how v1 was made is in `docs/vision_log.md`, but you do not need it to follow this page.

**Goal** (from the milestone list):

| Task | Done when |
|---|---|
| Model on our frames | Overlays from the cracked asphalt (`garden_surfaces_1548` 57-72 s, `_1618` 0-36 s) pushed, plus a short table: on how many frames it finds the cracks, and how many false alarms it gives on the indoor tile |
| Colour gripper mask | A mask PNG in the same style as `ground_demo/robot_mask_d435_gripper.png` (white = ground, black = robot), applied in the node; no boxes on the gripper in our bags |
| Annotation | A first set of our own frames labelled with crack masks, split by recording run |
| Fine-tune and export | The model retrained on our frames, exported to ONNX |
| Use the model in the node | The dark-pixel threshold replaced by the ONNX U-Net |
| Cracks in 3D | Each crack placed in 3D using the depth image, published as type `crack` in the format of `docs/schema.md` (step 8) |

---

## Step 0. Get the latest code and make your branch

Everything is merged into `main`, including your old branch. Start from a fresh `main`:

```bash
cd silvanus-terrain-quality-mapping
git switch main
git pull
git switch -c aritra-crack-model
```

Work and commit on `aritra-crack-model`, push it with `git push -u origin aritra-crack-model` (after the first time just `git push`), and open a pull request into `main` when a step is done. If `main` moves while you work and you want the new changes: `git fetch origin && git merge origin/main`. Never `--force`.

Your old branch `aritra-crack-detection` is merged and can be deleted on GitHub; do not continue on it.

## Step 1. Two environments

You need two separate setups, because ROS and the training code have different dependencies.

**A. ROS 2 Jazzy** (what you already use for the node). Needed for extracting frames from bags and for running the node.

**B. Python for the model** (`vision_training/`, plain Python + PyTorch, no ROS):

```bash
cd vision_training
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[onnx,dev]"
python -m pytest            # should end with "17 passed"
```

Measuring the model (step 4) and making the mask (step 3) run fine on a laptop CPU. **Fine-tuning (step 7) needs a GPU**: on the GPU server it takes about 5 minutes, on a laptop CPU many hours. Ask Yamin to run it on the server, or use Google Colab.

## Step 2. Get the model files and the frames

**Model v1.** Download from the GitHub release `crack-model-v1` (repository page, Releases on the right; if it is not there yet, ask Yamin) and put the files in `vision_training/models/`:

| File | Use | sha256 |
|---|---|---|
| `crack_unet16_v1.onnx` | the ROS node | `f79243ee83f28d0842fa4ae0093a0028caa608eab2c09b296ba68d89c318e540` |
| `crack_unet16_v1.pt` | PyTorch weights, for measuring and fine-tuning | `cf4b60381f7ea9fcd7850a587c14fe1a8965fb447f01adb79aa18c0b5ba4f608` |
| `crack_unet16_v1.json` | threshold and input normalisation; already in git | |

Check with `sha256sum vision_training/models/*.onnx vision_training/models/*.pt`.

**Frames.** Bags are on the shared drive (link in `docs/DATA.md`). Extract colour frames in your ROS environment, 2 per second, into `frames/` at the repo root (ignored by git):

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/bag_to_images.py <bags>/garden_surfaces_1618 --out frames/garden_surfaces_1618 --fps 2
python3 tools/bag_to_images.py <bags>/garden_surfaces_1548 --out frames/garden_surfaces_1548 --fps 2
python3 tools/bag_to_images.py <bags>/garden_surfaces_1628 --out frames/garden_surfaces_1628 --fps 2
python3 tools/bag_to_images.py <bags>/corridor_book_1338_indoor --out frames/corridor_book_1338_indoor --fps 2
```

File names are `<bag>_<index>_<seconds>.png`, so the bag time of every frame is in its name. Keep `index.csv` from each folder.

## Step 3. The gripper mask

A bit of the gripper is visible at the bottom of every colour frame. **Mask it, do not crop it.**

- Cropping changes pixel coordinates, so the 3D step (step 8) would need corrected camera intrinsics, and the gripper is not a full-width band, so a crop throws away ground that is fine.
- With a mask the model runs on the full 640x480 frame and the crack probability is set to zero under the mask. No box can ever appear on the gripper, and pixel coordinates stay valid for 3D.

The existing `ground_demo/robot_mask_d435_gripper.png` is for the **depth** image (848x480). It cannot be resized for colour: the D435 colour and depth sensors have different fields of view, so the gripper sits in a different place. Make a colour mask with:

```bash
cd vision_training
python scripts/make_robot_mask.py \
    --frames ../frames/garden_surfaces_1618 ../frames/garden_surfaces_1628 ../frames/corridor_book_1338_indoor \
    --out models/robot_mask_color.png
```

It finds the pixels that stay the same across many frames from different bags (the gripper does not move in the image, the ground does), keeps only the regions touching the bottom edge, and adds an 8 px safety margin because the arm shakes while driving. **Look at `models/robot_mask_color_preview.png`**: the red area must cover all of the gripper and not much more. Use frames from at least three bags with different ground; frames that all show plain asphalt make the mask too big.

If the automatic result looks wrong, block a strip instead, for example the bottom 60 rows:

```bash
python scripts/make_robot_mask.py --frames ../frames/garden_surfaces_1618 --out models/robot_mask_color.png --bottom-rows 60
```

The mask is a normal PNG; touch it up in any paint program if needed (white = ground, black = robot). Commit it (`git add vision_training/models/robot_mask_color.png`, it is small). If the arm pose ever changes, the mask has to be made again.

## Step 4. Measure v1 on our frames (no training)

This answers "how good is the public-data model on our robot?" and gives the first table.

Copy the frames inside the cracked asphalt time ranges (`1618` 0-36 s, `1548` 57-72 s) into `frames/cracked/`, and plain tile frames from `corridor_book_1338_indoor` into `frames/tile/`. Then:

```bash
cd vision_training
python scripts/predict.py --checkpoint models/crack_unet16_v1.pt --images ../frames/cracked --mask models/robot_mask_color.png
python scripts/predict.py --checkpoint models/crack_unet16_v1.pt --images ../frames/tile    --mask models/robot_mask_color.png
```

Each run prints `N frames, crack found in M`, and writes `frames/<folder>_pred/` with an overlay per frame (crack pixels in red, robot area darkened) and `summary.csv`. A frame counts as "crack found" with at least 150 crack pixels (`--min-pixels` to change).

- Cracked folder: M / N = **detection rate**.
- Tile folder: every frame with a crack found is a **false alarm**.

Put the two numbers and 4 to 6 overlays (good ones and bad ones) into a short section in `docs/PROGRESS.md`.

What to expect: v1 was trained only on close-up public photos of cracks. On our camera (0.7 to 0.9 m high, tilted, 640x480) wide asphalt cracks near the robot should be found; thin and far cracks will be missed, and tile grout lines, painted lines and sharp shadows may give false alarms. These failures are exactly what the annotation in step 6 fixes.

## Step 5. Put the model in the ROS node

Changes in `src/crack_detection/`:

1. **Wrapper.** Copy `vision_training/deploy/crack_onnx.py` to `src/crack_detection/crack_detection/crack_onnx.py`. It only needs numpy and onnxruntime, and already handles the three things that are easy to get wrong: BGR input from cv_bridge (the model wants RGB), the input normalisation, and padding the size to a multiple of 16.
2. **Model files in the package.** The Dockerfile only copies `src/`, so the model must live inside the package. Copy `crack_unet16_v1.onnx`, `crack_unet16_v1.json` and `robot_mask_color.png` to `src/crack_detection/models/`, and install them in `setup.py`:
   ```python
   ('share/' + package_name + '/models', glob('models/*')),
   ```
   The `.onnx` is ignored by git, which is fine: it is in the Docker build context as long as the file is on disk when you build.
3. **Dependency.** In the Dockerfile, after the apt install: `RUN pip3 install --break-system-packages onnxruntime` (Ubuntu 24.04 needs that flag; the robot has no internet, so it must be in the image).
4. **Topic.** On the robot the colour topic is `/camera/camera/color/image_raw`, not `/camera/color/image_raw`. Change the default in the node and the launch file.
5. **Inference in the callback**, replacing `detect_cracks`:
   ```python
   from ament_index_python.packages import get_package_share_directory
   from crack_detection.crack_onnx import CrackSegmenter, load_ground_mask

   models = Path(get_package_share_directory('crack_detection')) / 'models'
   self.seg = CrackSegmenter(models / 'crack_unet16_v1.onnx', num_threads=2)
   self.ground = load_ground_mask(models / 'robot_mask_color.png', 640, 480)

   # in on_image:
   frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
   prob, mask = self.seg.predict(frame, bgr=True, ground_mask=self.ground)

   n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
   boxes = []
   for i in range(1, n):
       x, y, w, h, area = stats[i]
       if area < 150:      # same minimum as predict.py; tune on the bags
           continue
       boxes.append({'x': int(x), 'y': int(y), 'width': int(w), 'height': int(h),
                     'area': float(area), 'score': round(float(prob[labels == i].mean()), 3)})
   ```
   Keep the JSON on `/crack_detection/detections` and the debug image exactly as they are now (`docs/schema.md`).
6. **Rate.** One inference takes about 0.2 s on a laptop CPU, so the node cannot keep up with 30 Hz. Subscribe with queue depth 1 so old frames are dropped, and process whatever frame is newest. At 0.3 m/s the robot moves about 7 cm between inferences; the camera sees each spot for about 4 s before the wheels reach it, so every crack is still seen many times. Add the measured inference time to the log output once per few seconds.

Test with `ros2 bag play <bags>/garden_surfaces_1618` and RViz on `/crack_detection/debug_image`: boxes on the cracks, none on the gripper.

## Step 6. Annotate

**Which frames.** From the extracted folders: frames with cracks (the cracked asphalt ranges, plus any other cracks you see), and on purpose frames that look like cracks but are not (tile grout, painted lines, drains, kerb edges, leaves, shadows). Target: about 120 frames with cracks and about 150 without. The step 4 overlays show where v1 fails; label those first.

**How.** CVAT or Label Studio, one class `crack`, brush. Paint the crack as you see it, a few pixels wide; do not paint the whole dark area around it. Never paint anything on the gripper. Export as segmentation masks: one PNG per frame, same size as the frame, white (255) on crack pixels, black elsewhere, named `<frame>_mask.png`. Rename if the tool uses other names. Frames without cracks need **no** mask file; a frame without a mask is counted as "no crack".

**Where.** One folder per bag:

```
vision_training/data/raw/robot/
  garden_surfaces_1618/        garden_surfaces_1618_0012_6.0.png
                               garden_surfaces_1618_0012_6.0_mask.png
                               garden_surfaces_1618_0013_6.5.png      (no mask = no crack)
  garden_surfaces_1548/        ...
  corridor_book_1338_indoor/   ...
  splits.yaml
```

`data/` is not in git, so also upload the annotated folders to the shared drive.

**Split by bag, never by frame.** Two frames half a second apart show the same crack, so a frame-level split puts the same crack in train and test and the score comes out far too high. `splits.yaml` says which bag goes where:

```yaml
garden_surfaces_1618: train
garden_surfaces_1628: train
corridor_book_1434_indoor_books: train
garden_surfaces_1607: val
corridor_book_1338_indoor: val
garden_surfaces_1548: test       # cracked asphalt the model never trains on
corridor_groundtruth_1447: test  # tile, for false alarms
```

## Step 7. Fine-tune, evaluate, export (on the GPU)

```bash
cd vision_training
python scripts/download_data.py && python scripts/prepare_data.py   # public data + data/raw/robot, prints a split table
python scripts/train.py --config configs/finetune_robot.yaml
python scripts/evaluate.py --checkpoint runs/<run>/best.pt --split test --sources robot
python scripts/export_onnx.py --checkpoint runs/<run>/best.pt --name crack_unet16_v2
```

Check the split table that `prepare_data.py` prints: every bag must be in the split you put it in, and the test split must have both crack frames and tile frames.

`configs/finetune_robot.yaml` starts from `models/crack_unet16_v1.pt`, keeps the public images in the training mix so the model does not forget them, and validates and tests on robot frames only.

Then rerun step 4 with `runs/<run>/best.pt` on the test bag frames, and put the v1 and v2 numbers side by side in `docs/PROGRESS.md`. The difference is what our annotation bought. Swap `crack_unet16_v2.onnx` and its `.json` into the node (step 5.2) and share both files the same way as v1.

## Step 8. Cracks in 3D

The bags have **no aligned depth** topic, so a colour pixel cannot simply be looked up in the depth image. For each crack component:

1. Take its pixels in the colour image.
2. Map them to the depth image: back-project candidate depths through the colour `camera_info`, transform with the colour-to-depth transform from `/tf_static`, project with the depth `camera_info`. Simpler alternative: project every valid depth pixel into the colour image once per frame (depth → 3D → colour frame → colour pixel) and keep, for each colour pixel, the 3D point that lands there.
3. Use the median 3D point of the crack pixels as its position and the extent of the points as length and width.
4. Publish the same way the depth node publishes bumps and holes, so the fusion side can treat all hazards alike: a `vision_msgs/Detection3DArray`, one `Detection3D` per crack, `class_id` `"crack"`, centre and size in metres, header frame = the camera frame the points are in, stamp = the image stamp. Fusion transforms it to `map` and writes it to `hazards.json` with type `crack`. Use a new topic `/crack_detection/hazards`, keep `/crack_detection/detections` (2D boxes) as it is, and add the new topic to the table in `docs/schema.md` (tell Praveen, the fusion side reads that file).

Praveen's `ground_demo/ground_hazard_node.py` already turns depth pixels into 3D points with `camera_info` and fills `Detection3DArray`; copy its approach. Note from it: camera topics are BEST_EFFORT, subscribe with `qos_profile_sensor_data` or you silently receive nothing.

If the lab can enable `align_depth` in the camera driver, step 2 becomes a plain lookup. Ask.

## Model facts (v1)

| | |
|---|---|
| Architecture | U-Net, 4 levels, base width 16, 1.94M parameters |
| Trained on | DeepCrack + CrackForest (public close-up photos), 342 training images, no robot frames |
| Input | RGB uint8, any size; the wrapper normalises and pads |
| Output | crack probability per pixel; threshold 0.80 (in the JSON, chosen on validation) |
| Public test set (255 images) | Dice 0.833, IoU 0.714, precision 0.908, recall 0.770, F1 with 2 px tolerance 0.912 |
| Speed | 205 ms per 640x480 frame, ONNX Runtime, laptop CPU (robot CPU not measured) |

The public test numbers say nothing about our robot frames; step 4 is the first real measurement.

## If something is unclear

Training code, configs, `predict.py`, `make_robot_mask.py`: ask Yamin. Bags, topics, arm pose, depth node: ask Praveen. Write decisions and results in `docs/PROGRESS.md` so nothing lives only in chat.
