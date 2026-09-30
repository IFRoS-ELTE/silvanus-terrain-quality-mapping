"""Export a checkpoint to ONNX for the ROS crack node.

    python scripts/export_onnx.py --checkpoint runs/<run>/best.pt

Writes next to the checkpoint:
    crack_unet.onnx        the model, dynamic height and width
    crack_unet.json        what the ROS node needs to use it correctly:
                           threshold, normalisation, size multiple

The robot may not have a GPU, so the node will most likely run the model
with ONNX Runtime on the CPU. If onnxruntime is installed, this script
checks that ONNX and PyTorch give the same output and prints a rough CPU
timing for one 640x480 frame.
"""

import argparse
import json
import logging
import time
from pathlib import Path

import numpy as np
import torch

from crackseg.inference import load_checkpoint
from crackseg.utils import setup_logging

log = logging.getLogger("export")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--name", default="crack_unet")
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()
    setup_logging()

    ckpt_path = Path(args.checkpoint)
    model, ckpt = load_checkpoint(ckpt_path, torch.device("cpu"))
    multiple = 2 ** model.depth

    onnx_path = ckpt_path.parent / f"{args.name}.onnx"
    dummy = torch.randn(1, 3, 480, 640)
    torch.onnx.export(
        model, dummy, onnx_path,
        input_names=["image"], output_names=["logits"],
        dynamic_axes={"image": {0: "batch", 2: "height", 3: "width"},
                      "logits": {0: "batch", 2: "height", 3: "width"}},
        opset_version=args.opset,
        # The older TorchScript exporter prints a deprecation warning on
        # torch 2.9+, but it is still supported, handles a plain conv net
        # fine and does not need the extra onnxscript package.
        dynamo=False,
    )
    log.info("Wrote %s", onnx_path)

    meta = {
        "model_file": onnx_path.name,
        "input": "RGB uint8 image -> float32, divided by 255, then (x - mean) / std, NCHW",
        "mean": ckpt["normalization"]["mean"],
        "std": ckpt["normalization"]["std"],
        "size_multiple": multiple,
        "output": "logits, apply sigmoid to get crack probability",
        "threshold": ckpt["threshold"],
        "trained_epoch": ckpt["epoch"],
        "val_metrics": ckpt["val_metrics"],
        "model_config": ckpt["config"]["model"],
    }
    with open(ckpt_path.parent / f"{args.name}.json", "w") as f:
        json.dump(meta, f, indent=2)

    try:
        import onnxruntime as ort
    except ImportError:
        log.warning("onnxruntime is not installed, skipping the output check")
        return

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    x = torch.randn(1, 3, 480, 640)
    with torch.no_grad():
        expected = model(x).numpy()
    got = session.run(None, {"image": x.numpy()})[0]
    diff = float(np.abs(expected - got).max())
    log.info("Max difference PyTorch vs ONNX: %.2e", diff)
    if diff > 1e-3:
        log.warning("Outputs differ more than expected, check the export before using it")

    runs = 10
    start = time.perf_counter()
    for _ in range(runs):
        session.run(None, {"image": x.numpy()})
    ms = (time.perf_counter() - start) / runs * 1000
    log.info("ONNX Runtime CPU: %.0f ms per 640x480 frame (%.1f fps)", ms, 1000 / ms)


if __name__ == "__main__":
    main()
