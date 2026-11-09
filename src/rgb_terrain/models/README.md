# Model files

The node loads `rgb_terrain_v1.onnx` and `rgb_terrain_v1.json` from here. The JSON is small and goes into git; the `.onnx` is ignored by git, so copy it here before `colcon build` or `docker build`.

Both come from `vision_training`:

```bash
python scripts/terrain_train.py --config configs/terrain_mobilenet.yaml --final
python scripts/terrain_export_onnx.py --checkpoint runs/<run>_final/model.pt
```

How the model was trained and tested: `docs/terrain_classifier.md`.
