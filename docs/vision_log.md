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

## 4. Next steps

Vision training:
1. ~~Real training run on the GPU server~~ Done, see the baseline entry above. Still to do: rerun with the wider threshold sweep, look at the worst overlays, export ONNX and time it on CPU.
2. Download Crack500 by hand, check the layout, rerun `prepare_data.py`, retrain. Crack500 is phone photos of pavement, the closest public match to the robot view.
3. Compare base_channels 16 vs 32 (accuracy vs CPU fps on the robot).
4. Run `predict.py` on spare D435 frames of campus pavement and corridor floors. Collect false positives (tile joints, painted lines, shadows, drains) as hard negatives.
5. Annotate about 120 crack frames and 150 negatives from our own data (CVAT or Label Studio, class `crack`), add an `own` converter grouped by session, fine tune.
6. RGB terrain classifier on patches (grass, asphalt, gravel, tiles). Same package structure; will need patch labels from the terrain segments labelled in the bags on the IMU side.

Rest of the vision work (not started):
7. Spare D435 experiments (handover section 9.4): height 0.4 to 0.9 m, tilt 30 to 60 degrees, walking 0.3 / 0.6 / 1.0 m/s, depth 848x480 at 30 fps, colour 1280x720, align on, fixed exposure. Output: plane fit noise vs distance and speed, recommended arm pose for the robot side of the team.
8. `terrain_vision` ROS 2 package: depth hazard node (RANSAC plane, residual clusters, sizes, unevenness), crack node using the exported ONNX and its JSON, mock publisher for `/ground/*` topics so the IMU/fusion and robot-control sides can test their subscribers.

## 5. Open questions

- Does the robot computer have a GPU? Decides base_channels and whether the crack node runs on the robot or a laptop.
- Is aligned depth (`/camera/aligned_depth_to_color/image_raw`) published? Needed to put crack pixels in 3D.
- Where exactly the gripper appears in the image at the chosen arm pose (fixed mask).
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
