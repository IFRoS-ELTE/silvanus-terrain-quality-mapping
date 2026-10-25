# Runtime assets

`crack_unet16_v1.json` is the metadata accompanying Aritra's local U-Net v1.
The ONNX SHA-256 is
`f79243ee83f28d0842fa4ae0093a0028caa608eab2c09b296ba68d89c318e540`.
Keep `crack_unet16_v1.onnx` here on disk before colcon or Docker builds.
It is intentionally ignored by Git; `.pt` weights are not needed for this node.
The wrapper is copied unchanged from `vision_training/deploy/crack_onnx.py`.

## Colour mask provenance

`robot_mask_color.png`: binary 640x480, white ground, black robot.
Generated and visually checked on 2026-10-25 using 88 RGB frames, sampled
every 2 seconds, from the available recordings:

- `garden_surfaces_1536`: 45 frames, 0–88 s, tile and asphalt.
- `garden_surfaces_1548`: 43 frames, 0–84 s, asphalt and vegetation.

The static-pixel method from the team guide missed reflective parts of the
gripper. Only two bags were available (the guide recommends three for that
method). The committed mask therefore uses the guide's conservative strip
fallback: rows 400–479 are black, all other rows white. The bottom 80 rows
cover the visible robot in all 88 inspected samples, with a margin above the
highest visible part. This also excludes ground in that strip: 16.67% of the
image. It preserves the full 640x480 pixel coordinates and camera intrinsics.
This is a checked mask for these recordings, not validation of every arm pose.

To reproduce the PNG from the repository root:

```bash
python3 - <<'PY'
import cv2
import numpy as np
mask = np.full((480, 640), 255, dtype=np.uint8)
mask[400:] = 0
assert cv2.imwrite('src/crack_detection/models/robot_mask_color.png', mask)
PY
```

`robot_mask_depth.png`: the original 848x480 depth-camera gripper mask from
the local `terrain_vision` package. Used only in depth projection; it is not
the colour mask. Replace or recheck both masks when the arm/camera pose changes.
