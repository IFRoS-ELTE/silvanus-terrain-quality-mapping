# Vision work log

Running log of the vision part: what was built, why, what was measured, and what comes next. It is written so the work can be picked up again from this file alone, without any chat history. Newest entries go at the bottom of section 3.

Project context is in `PROJECT_HANDOVER.md` (shared by the team, not in this repo). The parts that matter for vision: sections 4.1 (sensing layers), 4.8 (cracks), 8 (interface contracts), 9.2 and 9.4 (data collection), 10 (evaluation plan), 12 (work split).

## 1. Scope of the vision work

- Depth hazard node: depth image to point cloud, RANSAC ground plane, residuals, holes and bumps with size, unevenness score. Publishes `/ground/hazards`, `/ground/hazard_markers`, `/ground/unevenness`.
- Crack node: colour image to crack mask (ONNX model), crack pixels to 3D through the aligned depth, onto the map.
- RGB terrain classifier on image patches (grass, asphalt, gravel, tiles). Publishes `/ground/rgb_terrain`.
- Spare D435 home experiments: best height, tilt, camera modes, walking speed. Output: recommended arm pose for the robot side of the team.

Topic and JSON formats are fixed in handover section 8. Hazard JSON on `/ground/hazards`: `{id, type: "hole"|"bump"|"crack", x, y, z, frame, length, width, depth, confidence, stamp}`.

## 2. Environment

| Machine | What | Notes |
|---|---|---|
| Own laptop | Windows 11 + WSL, no GPU | Python 3.14, PyTorch 2.12 CPU. Enough for data prep, tests, smoke runs, ONNX export and CPU timing. |
| Private GPU server | Real training runs | Install CUDA PyTorch first, then `pip install -e ".[onnx,dev]"`. Copy `data/raw/` or just rerun the download script there. |
| Robot | GPU unknown (handover section 13, item 8) | Plan is CPU inference with ONNX Runtime unless a GPU shows up. |

ROS 2 Jazzy is not needed for `vision_training/`. It will be needed for the `terrain_vision` nodes (WSL2 with Ubuntu 24.04, handover section 12, week 1).

## 3. Log entries

### 2026-09-30: repository created, crack segmentation pipeline working

**What was built.** `vision_training/`, a Python package (`crackseg`) plus six scripts:

1. `download_data.py`: fetches DeepCrack and CrackForest from GitHub into `data/raw/`. Crack500 is Google Drive only; `--list` prints how to get it by hand.
2. `prepare_data.py`: converts every dataset to `images/` + `masks/` (0/255 PNG), assigns splits, writes `data/processed/index.csv`, prints a table with image count and crack pixel share per split.
3. `train.py`: trains from a YAML config. Every run folder has the config, log, per-epoch CSV, curve plot, `best.pt` and `last.pt`.
4. `evaluate.py`: test set metrics overall and per dataset, per-image Dice, overlays of the worst and best images.
5. `export_onnx.py`: ONNX model plus a JSON with threshold and normalisation for the ROS node; checks ONNX against PyTorch and times CPU inference.
6. `predict.py`: runs a model on a folder of unlabelled frames (our own D435 photos), for choosing what to annotate first.

14 unit tests in `tests/` (metrics, splits, transforms, converter round trip, model shapes, loss). They run without any downloaded data.

**Datasets as downloaded** (checked by opening the files, not taken from the papers):

| Dataset | Source | Images | Size | Mask format | Split |
|---|---|---|---|---|---|
| DeepCrack | github.com/yhlleo/DeepCrack, `dataset/DeepCrack.zip` (68 MB) | 537 | 544x384 | PNG 0/255 | official 300 train / 237 test |
| CrackForest (CFD) | github.com/cuilimeng/CrackForest-dataset (10 MB) | 118 with labels (155 jpg, 37 have no .mat) | 480x320 | `.mat`, `groundTruth.Segmentation`, 1 = background, 2 = crack | none, we split 70/15/15 by seed |
| Crack500 | Google Drive via github.com/fyangneil/pavement-crack-detection | not downloaded yet | 640x360 crops | `name_mask.png` next to `name.jpg` | official train/val/test crop folders |

The expected Crack500 layout (`traincrop/`, `valcrop/`, `testcrop/` with `_mask.png` files) comes from the dataset description. Check it when the download is done; the converter prints a warning if a folder is missing.

Current split table (`python scripts/prepare_data.py`):

```
dataset       split    images  crack %
crackforest   test         18     1.75
crackforest   train        82     1.58
crackforest   val          18     1.63
deepcrack     test        237     4.33
deepcrack     train       260     2.98
deepcrack     unused       12     3.30
deepcrack     val          28     2.34
```

**Decisions and why.**

- *Small U-Net written in the repo, trained from scratch.* Handover allows YOLOv8-seg or a small U-Net. U-Net gives a pixel mask directly, which is what we need to project crack pixels to 3D through depth. No pretrained weights to download, no extra library, and we know the architecture well from medical segmentation. `base_channels` 8 / 16 / 32 gives 0.49M / 1.94M / 7.76M parameters. Default for the real run is 16, small enough for the robot CPU. A pretrained encoder is a possible later experiment; the data code already uses ImageNet normalisation so that swap does not touch it.
- *BCE (pos_weight 5) + soft Dice loss.* Cracks are only 1.5 to 4.5 percent of pixels (table above). Plain BCE learns to predict "no crack".
- *Random 256 px crops, half of them centred near a crack, instead of resizing.* Cracks are a few pixels wide and vanish when a whole image is shrunk. Crack-centred crops fight the class imbalance.
- *Augmentations: flips, 90 degree rotations, brightness/contrast, light blur.* Cracks have no direction; the robot drives through shade; the D435 colour sensor is rolling shutter so frames smear.
- *Validation and test on full images, batch size 1.* Images differ in size between datasets. Padding to a multiple of 16 happens inside the model call and is cropped away after.
- *Two metric families.* Strict pixel Dice/IoU/precision/recall (handover section 10), plus precision/recall/F1 with a 2 px tolerance as in the crack literature, because a one-pixel annotation shift can take strict Dice to zero on a thin crack (a unit test shows exactly this).
- *Threshold chosen on validation, stored in the checkpoint, reused on test and in the ONNX JSON.* Test numbers never use a threshold tuned on test.
- *Splits by group, never by single image.* Same rule as patient-level splits. For our own data the group will be the recording session.
- *Numpy augmentations, no albumentations.* Fewer dependencies, easy to read, enough for this job.
- *Legacy ONNX exporter (`dynamo=False`).* It prints a deprecation warning on torch 2.9+ but is still supported and does not need `onnxscript`.

**Problem found and fixed: data leakage in DeepCrack.** DeepCrack file names like `11134`, `11134-3`, `11134-4` are crops of the same photo. With a per-image split, crops of one photo landed in both train and val. After grouping by the part of the name before the first `-`:
- train/val overlap: gone.
- DeepCrack's own official train/test split still shares 3 photos (`11142`, `11289`, `IMG_6536`). We keep the official test set unchanged (so numbers stay comparable with papers) and mark the 12 train/val crops of those photos as `unused`.
- Effect on the smoke run: validation Dice went from 0.81 (leaky split) to 0.57 (clean split), same config. So the leak inflated the score by a lot. Worth mentioning in the final report.

**Smoke run results** (`configs/smoke_cpu.yaml`: base_channels 8, 128 px crops, 2 epochs of 20 batches, DeepCrack only, laptop CPU, about 6 s per epoch). This only proves the pipeline works; it is not a model result.

```
val (epoch 1, best):  Dice 0.565  IoU 0.394  tolerant F1 0.630  threshold 0.7
test, 237 images:     Dice 0.649  IoU 0.481  P 0.938  R 0.496
                      tolerant (2 px) P 0.946  R 0.658  F1 0.776
ONNX vs PyTorch max difference: 5.4e-07
ONNX Runtime CPU, laptop, 640x480, base_channels 8: 88 ms per frame (11 fps)
```

Low recall and high precision are expected this early: the model finds the obvious wide cracks and misses thin ones. The worst overlays are thin hairline cracks and a few `IMG*` photos that look different from the rest.

### 2026-09-30 (later): README, simulation screenshots, git history

- README rewritten: project summary, Gazebo and RViz screenshots from the simulation (`docs/images/`), and a short "where things stand" section instead of the old per-folder table.
- The screenshots come from the threshold-based crack node on branch `aritra-crack-detection` (ROS 2 Jazzy, subscribes to `/camera/color/image_raw`, publishes JSON boxes on `/crack_detection/detections` and a debug image on `/crack_detection/debug_image`). Several boxes fall on the robot's own bumper and wheels, which confirms that the camera needs a fixed body mask.
- The first commit was amended once to remove an unwanted co-author line; `main` was force-pushed a single time and the `aritra-crack-detection` branch was rebased onto it. Nothing else rewrites history. Normal workflow from now on: one branch per piece of work, `git pull` (with `pull.rebase true`) before pushing, merge into `main` through a pull request, never force-push.

### 2026-09-30: first full training run (baseline)

Setup: GPU server (`/scratch2/yamin/silvanus-terrain-quality-mapping`), own venv inside `vision_training/.venv` with CUDA PyTorch. Config `configs/unet_public.yaml` unchanged: U-Net with base_channels 16 (1.94M parameters), DeepCrack + CrackForest, 256 px crops, batch 16, AdamW lr 1e-3 with cosine schedule, 150 epochs. 2 to 3 s per epoch, about 7 minutes for the whole run.

Run folder on the server: `vision_training/runs/unet16_public_20260930_102903/` (best.pt, eval_test/). Not in git.

Training: best validation Dice 0.797 at epoch 146, threshold 0.7. Early stopping never triggered; the cosine schedule brought the learning rate to zero and the curve flattened by itself. Final train loss about 0.15, val loss about 0.177, so no real overfitting.

Test set, 255 images, threshold 0.7 (chosen on validation):

| Subset | Images | Dice | IoU | Precision | Recall | Tolerant F1 (2 px) |
|---|---|---|---|---|---|---|
| All | 255 | 0.833 | 0.713 | 0.893 | 0.780 | 0.909 (P 0.958, R 0.865) |
| DeepCrack | 237 | 0.836 | 0.718 | | | 0.908 |
| CrackForest | 18 | 0.718 | 0.560 | | | 0.934 |

What the numbers say:
- The chosen threshold was the top of the sweep (0.3 to 0.7), so the real optimum may be higher. The sweep in `unet_public.yaml` now goes to 0.9. The BCE pos_weight pushes probabilities up, which explains a best cut above 0.5.
- CrackForest: strict Dice 0.72 but tolerant F1 0.93. Its cracks are 1 to 2 px wide, so small border disagreements with the annotator cost a lot of strict Dice. The model finds the cracks; it does not match the hand-drawn border to the pixel. This is the reason we report both metric families.
- Test Dice (0.83) is higher than validation Dice (0.80). Validation has only 46 images, so its numbers are noisy. Report test numbers, use validation only for choosing the checkpoint and threshold.
- Recall (0.78) is lower than precision (0.89): the model misses more than it hallucinates. For the robot that is the safer side (fewer false alarms on tile joints), but check the worst overlays in `eval_test/overlays/` for the kind of cracks that are missed.
- Not directly comparable with published DeepCrack numbers: papers usually report ODS/OIS F-scores on thinned predictions with their own tolerance. Ours is a pixel metric with a 2 px dilation.

### 2026-10-14: model choice, v1 handoff, plan review

**Runs compared** (all on the GPU server, same data, `unet_public.yaml`; test set 255 images, threshold chosen on validation):

| Run | Params | Best epoch | Threshold | Test Dice | Test IoU | P | R | Tolerant F1 | CPU, 640x480 ONNX |
|---|---|---|---|---|---|---|---|---|---|
| unet16, 2026-09-30 (sweep to 0.7) | 1.94M | 146 | 0.7 | 0.833 | 0.713 | 0.893 | 0.780 | 0.909 | 205 ms |
| **unet16, 2026-10-06 (sweep to 0.9)** | 1.94M | 108 (early stop 138) | 0.8 | **0.833** | **0.714** | 0.908 | 0.770 | **0.912** | **205 ms (4.9 fps)** |
| unet32, 2026-10-14 | 7.76M | 58 (early stop 88) | 0.7 | 0.831 | 0.711 | 0.891 | 0.779 | 0.913 | 690 ms (1.4 fps) |

CPU timing: laptop, 8 threads, ONNX Runtime. Robot CPU not measured yet.

- The 4x bigger model is not better. The limit is the data (342 training images), not model capacity. **Chosen: unet16 from 2026-10-06**, released as `crack_unet16_v1`.
- The wider threshold sweep picked 0.8 (old range stopped at 0.7): precision up 1.5 points, recall down 1, tolerant F1 up 0.3. Small gain, but the threshold is no longer at the edge of the sweep.
- Worst test images are faint hairline cracks on low-contrast concrete (`IMG24-*`, `11247-*`): found in pieces, gaps where the crack fades.

**v1 handoff** (branch `vision-crack-model-handoff`):
- `vision_training/models/crack_unet16_v1.{pt,onnx,json}`, optimizer state removed from the `.pt` (23 MB to 7.8 MB, CPU tensors). Checksums in `docs/crack_model.md`. Not in git; to be shared as a GitHub release or on the shared drive.
- `vision_training/deploy/crack_onnx.py`: ONNX Runtime wrapper for the ROS node, numpy + onnxruntime only. Handles BGR input from cv_bridge, normalisation, padding to 16, and an optional robot body mask. A test checks it against PyTorch.
- `model.init_from` in the training config and `configs/finetune_robot.yaml`: fine-tune from v1 with public + robot frames in training, robot frames only in val/test, lr 3e-4.
- `robot` dataset converter: `data/raw/robot/<bag>/`, group = bag, frames without a mask are negatives, optional `splits.yaml` to fix bags to splits.
- `predict.py`: optional colour mask, per-frame `summary.csv` with "crack found" (at least 150 crack pixels). Produces the detection-rate and tile false-alarm table from the milestone list.
- `docs/crack_model.md`: model card, how to use it in the node, known limits, measure / annotate / fine-tune steps.

**Facts from the robot branch (`praveen-robot-and-depth`) that change the vision plan:**
- Robot colour stream is 640x480 at 30 Hz (`/camera/camera/color/image_raw`), depth 848x480. Topic prefix is `/camera/camera/`.
- The depth hazard node already exists (`ground_demo/ground_hazard_node.py`), tested against tape-measured objects (mean error 1.7 mm). The depth node item is removed from this list.
- The gripper/body mask exists for **depth only** (848x480, 28 % of the frame). It cannot be resized for colour because the two sensors have different fields of view. A colour mask still has to be drawn.
- The bags have no aligned depth topic. Placing crack pixels in 3D needs the colour and depth `camera_info` plus the colour-to-depth transform from `/tf_static`.
- Camera unevenness (depth) and the IMU 8-50 Hz / 0.5-8 Hz band ratio already separate smooth from rough surfaces (asphalt vs grass 97 % from the camera, 91 % from the IMU). The RGB terrain classifier only has to say what the surface **is**, not how bumpy it is.

**Plan review: HrSegNet** ([repo](https://github.com/CHDyshli/HrSegNet4CrackSegmentation), [paper](https://arxiv.org/abs/2307.00270), Automation in Construction 2023). Decision: **keep our U-Net as the base, do not switch now.**
- It is PaddlePaddle / PaddleSeg, not PyTorch. Fine-tuning it on our frames means a second training framework; deployment needs Paddle2ONNX or their TensorRT engines (built for TensorRT 8.6 on a GPU; the robot has no known GPU).
- Its reported numbers (78-80 % mIoU on CrackSeg9k) are mean IoU over two classes, background and crack. Background IoU is close to 99 %, so the crack IoU behind it is roughly 57-61 %. Our 0.71 is crack-only IoU on different test data. The numbers cannot be compared directly, and neither says anything about our robot frames.
- What it does have that we do not: training on CrackSeg9k (about 9,000 images merged from 10 datasets). That is the useful part. If v1 struggles on our frames even after fine-tuning, the next step is to train our U-Net on CrackSeg9k (Harvard Dataverse; a later paper reports matching problems in the upload, so check counts), not to switch frameworks.
- Fair comparison if needed later: run their released CrackSeg9k weights and our v1 on the same annotated robot test bag.

**Plan review: terrain.** Agreed with the team idea, with two corrections:
- Terrain does **not** go into the crack model. It is a separate small classifier (different output: one label per region, not a pixel mask). Combining them in one network would make both harder to train and to debug.
- "Plain vs bumpy" does **not** come from RGB. Depth unevenness and the IMU band ratio already measure it. RGB adds the surface type (asphalt, red rubber path, wood chips, grass/dirt, tile), which is what lets us say "grass, rough" vs "asphalt, smooth" and compare vision-only, IMU-only and combined classification (evaluation plan).
- First version trains on our own labelled outdoor runs (5 bags, labels in `docs/DATA.md`), not on an online dataset: our surfaces, our camera, our tilt. ImageNet-pretrained small CNN (MobileNetV3-small or ResNet18) on ground-region crops, split by bag (test on a run it never saw). Public terrain datasets (GTOS-mobile, RUGD) only if our data is too small.

### 2026-10-15: crack model handed to Aritra, gripper mask tool

- All branches are merged into `main` (training pipeline, crack node, robot and depth work).
- **Ownership decided:** Aritra owns the crack model from here (measuring v1 on bags, gripper mask, annotation, fine-tuning, the model in the node, cracks in 3D). Yamin keeps the training code and moves to the RGB terrain classifier.
- `docs/crack_model.md` rewritten as one step-by-step handoff: git branch, environments, model files, frames, mask, measurement, node integration (model files inside the ROS package because the Dockerfile only copies `src/`, `onnxruntime` in the image, topic `/camera/camera/color/image_raw`, queue depth 1), annotation, fine-tuning, 3D.
- **Gripper in the colour frames** (small part at the bottom of every frame, see the team screenshot of 2026-10-15). Decision: **mask, do not crop.** Cropping shifts pixel coordinates (the 3D step would need corrected intrinsics) and the gripper is not a full-width band, so a crop wastes good ground. With a mask the model sees the full frame and the probability under the mask is set to zero, so no detection can land on the gripper. The depth mask (848x480) cannot be reused for colour (different field of view).
- New `scripts/make_robot_mask.py`: builds the colour mask from frames of several bags. The gripper is static in the image while the ground changes, so pixels with low variance over time are the robot; only regions touching the bottom edge are kept, then grown by 8 px for arm shake. Writes a preview for checking; `--bottom-rows N` is the simple fallback. One new test (17 in total).

## 4. Next steps

Crack model (Aritra, steps in `docs/crack_model.md`):
1. Download v1, extract frames, make the colour gripper mask.
2. Measure v1 on our frames: detection rate on cracked asphalt (`1618` 0-36 s, `1548` 57-72 s), false alarms on indoor tile.
3. U-Net in the crack node.
4. Annotate (about 120 crack frames, 150 negatives), fine-tune on the GPU server with `finetune_robot.yaml`, export v2. Test bags: `garden_surfaces_1548` + `corridor_groundtruth_1447`.
5. Cracks in 3D as `Detection3DArray` on `/crack_detection/hazards`.
6. Only if v2 is still weak: CrackSeg9k pretraining, Crack500, or an HrSegNet comparison on the robot test bag.

Yamin:
- Publish the `crack-model-v1` release (onnx + pt) on GitHub.
- Run Aritra's fine-tuning on the GPU server when the annotations are ready.

Terrain (Yamin):
7. RGB terrain classifier v1: frames from the 5 outdoor bags at the labelled time ranges (2 s margin at each boundary), ground-region crops below the horizon and outside the robot mask, classes asphalt / red path / wood chips / grass-dirt (+ tile from indoor bags). Pretrained small CNN, split by bag, accuracy per surface on an unseen run. Publishes `{"label", "confidence"}` on `/ground/rgb_terrain`.

Dropped: the spare D435 home experiments (real robot bags exist and the arm pose is chosen, `docs/robot/arm_pose.md`), and our own depth hazard node (exists in `ground_demo/`).

## 5. Open questions

- Does the robot computer have a GPU? Decides whether the crack node runs on the robot CPU (about 5 fps measured on a laptop) or on a laptop on the robot network.
- No aligned depth in the bags. Either ask for `align_depth` in the lab bring-up or project colour pixels to depth ourselves (both `camera_info` + `/tf_static`).
- Licences of the public datasets for a public repo: we do not redistribute them, the scripts download them from the original source. Cite them in the report (citations are in `src/crackseg/data/sources.py`).

## 6. How to resume from scratch

```bash
git clone https://github.com/IFRoS-ELTE/silvanus-terrain-quality-mapping.git
cd silvanus-terrain-quality-mapping/vision_training
pip install -e ".[onnx,dev]"
python -m pytest
python scripts/download_data.py && python scripts/prepare_data.py
python scripts/train.py --config configs/smoke_cpu.yaml
```

Data and runs are not in git. Everything under `data/` is recreated by the two data scripts. Trained models should be copied to the shared drive; write the path here next to the results.
