"""Export a terrain classifier to ONNX for the /ground/rgb_terrain node.

    python scripts/terrain_export_onnx.py --checkpoint runs/<run>_final/model.pt

Writes rgb_terrain_v1.onnx and rgb_terrain_v1.json next to the checkpoint.
The JSON holds everything the node needs besides the weights: class names,
the patch boxes in the 640x480 frame, input size and normalisation.
"""

import argparse
import json
import logging
import time
from pathlib import Path

import numpy as np
import torch

from crackseg.utils import setup_logging
from terrainclf.data import MEAN, STD
from terrainclf.model import build_model

log = logging.getLogger("terrain_export")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--name", default="rgb_terrain_v1")
    args = parser.parse_args()
    setup_logging()

    ckpt_path = Path(args.checkpoint)
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    cfg, classes = ckpt["config"], ckpt["classes"]
    model = build_model(cfg["model"]["backbone"], len(classes), pretrained=False)
    model.load_state_dict(ckpt["model"])
    model.eval()

    size = cfg["data"]["input_size"]
    onnx_path = ckpt_path.parent / f"{args.name}.onnx"
    torch.onnx.export(model, torch.randn(1, 3, size, size), onnx_path,
                      input_names=["patches"], output_names=["logits"],
                      dynamic_axes={"patches": {0: "n"}, "logits": {0: "n"}},
                      opset_version=17, dynamo=False)

    meta = {
        "model_file": onnx_path.name,
        "classes": classes,
        "patch_boxes_xywh": [list(map(int, b)) for b in ckpt["boxes"]],
        "frame_size_wh": [640, 480],
        "input_size": size,
        "mean": MEAN,
        "std": STD,
        "input": "RGB patches cut at patch_boxes, resized to input_size, /255, (x - mean) / std, NCHW",
        "output": "logits per patch; softmax, average over the patches of a frame",
        "backbone": cfg["model"]["backbone"],
        "trained_on": ckpt.get("trained_on") or f"all bags except {ckpt.get('test_bags')}",
    }
    (ckpt_path.parent / f"{args.name}.json").write_text(json.dumps(meta, indent=2))
    log.info("Wrote %s and its JSON", onnx_path)

    try:
        import onnxruntime as ort
    except ImportError:
        log.warning("onnxruntime not installed, skipping the check")
        return
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    x = torch.randn(len(meta["patch_boxes_xywh"]), 3, size, size)
    with torch.no_grad():
        expected = model(x).numpy()
    got = session.run(None, {"patches": x.numpy()})[0]
    log.info("Max difference PyTorch vs ONNX: %.2e", float(np.abs(expected - got).max()))
    start = time.perf_counter()
    for _ in range(10):
        session.run(None, {"patches": x.numpy()})
    log.info("ONNX Runtime CPU: %.0f ms per frame (%d patches)",
             (time.perf_counter() - start) * 100, len(x))


if __name__ == "__main__":
    main()
