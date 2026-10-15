# vision_training

Offline code for the crack segmentation model: download public datasets, convert them to one format, split them, train a U-Net, evaluate it, and export it to ONNX for the ROS node on the robot.

No ROS in here. It runs the same on Windows, WSL, a laptop CPU and the GPU server.

## Setup

```bash
cd vision_training
python -m venv .venv
source .venv/bin/activate          # Windows PowerShell: .venv\Scripts\Activate.ps1
pip install -e ".[onnx,dev]"
python -m pytest                   # 16 tests, about 20 seconds
```

On the GPU server, install the CUDA build of PyTorch first (see the comment at the top of `requirements.txt`), then the same `pip install -e ".[onnx,dev]"`.

## Workflow

```bash
# 1. Raw data into data/raw/  (DeepCrack and CrackForest download directly)
python scripts/download_data.py
python scripts/download_data.py --list        # also shows how to get Crack500 by hand

# 2. Common layout and splits into data/processed/ (+ index.csv)
python scripts/prepare_data.py

# 3. Train. Every run gets its own folder in runs/
python scripts/train.py --config configs/smoke_cpu.yaml          # 1 minute on a CPU, pipeline check only
python scripts/train.py --config configs/unet_public.yaml        # real run, use the GPU server
python scripts/train.py --config configs/unet_public.yaml train.epochs=50 model.base_channels=32

# 4. Test set numbers, per dataset, plus worst/best overlays
python scripts/evaluate.py --checkpoint runs/<run>/best.pt

# 5. ONNX for the robot, plus a JSON with threshold and normalisation
python scripts/export_onnx.py --checkpoint runs/<run>/best.pt

# 6. Try the model on our own unlabelled frames (per-frame summary.csv, optional robot mask)
python scripts/predict.py --checkpoint runs/<run>/best.pt --images path/to/frames --mask colour_mask.png

# 7. Fine-tune the released model on annotated robot frames in data/raw/robot/
python scripts/train.py --config configs/finetune_robot.yaml
```

The released model, how the ROS node uses it, and the annotation and fine-tuning steps are in [docs/crack_model.md](../docs/crack_model.md).

## Code layout

```
configs/               experiment settings (YAML), any key can be overridden on the command line
scripts/               the six entry points above, thin wrappers around the package
src/crackseg/
  data/sources.py      where each dataset comes from, download and unzip
  data/converters.py   each dataset format -> images/ + masks/ (0/255 PNG)
  data/splits.py       train/val/test by group, index.csv read/write
  data/transforms.py   crops biased towards cracks, flips, rotations, brightness, blur
  data/dataset.py      PyTorch Dataset over index.csv
  models/unet.py       small U-Net, width configurable
  losses.py            BCE + Dice
  metrics.py           Dice, IoU, precision, recall, and the same with a 2 px tolerance
  engine.py            train and eval loops
  inference.py         full image prediction with padding, checkpoint loading
  utils.py             config, seeding, device, logging
deploy/crack_onnx.py   ONNX Runtime wrapper for the ROS node, numpy + onnxruntime only
models/                released model files (downloaded, not in git)
tests/                 pytest, no dataset download needed
data/, runs/           created by the scripts, ignored by git
```

## Adding our own annotated frames

Put frames in `data/raw/robot/<bag name>/`, each crack frame with a `<frame>_mask.png` next to it (frames without a mask count as negatives), and optionally a `data/raw/robot/splits.yaml` that assigns each bag to train, val or test. `prepare_data.py` picks the folder up automatically. All frames of one bag stay in one split; see `docs/crack_model.md` section 5.
