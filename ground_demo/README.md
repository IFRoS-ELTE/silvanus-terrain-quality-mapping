# ground_probe

Terrain geometry from a single RGB-D camera, with no robot and no ROS.

Fits the ground plane with RANSAC, measures the signed distance of every point
to that plane, and reports holes and bumps with their real size in millimetres.
It also reports the camera height and tilt it infers from the plane, which is
how we choose the arm pose on Silvanus.

This is the depth geometry layer of the terrain mapping project, built early on
hand-held data so it can be developed and demonstrated before robot time.

## Install

    pip install -r requirements.txt

If `pyrealsense2` will not install on Python 3.12, either make a Python 3.11
virtualenv for it, or record with `realsense-viewer` and replay the `.bag`
with `--bag`. Everything except live capture works without it.

## Check it works, with no camera

    python3 selftest.py

Simulates a ground plane at a known height and tilt with a known object on it,
and reports the error in what the pipeline recovers. Expect camera height and
tilt correct to better than 0.1 mm and 0.01 degrees, and object size within
about 1.5 mm. **Run this first.** It is the evidence that the numbers on screen
mean something.

Note what it does *not* prove: the simulated depth noise is independent per
pixel, while a real D435's is spatially correlated and has edge artifacts. So
the simulated false positive rate (zero) is a lower bound. The real noise
limits have to be measured with the real camera, which is what the logging
below is for.

## Run it

    python3 ground_probe.py --synthetic     # no camera needed
    python3 ground_probe.py --live          # attached RealSense
    python3 ground_probe.py --bag FILE.bag  # replay a recording

Four panels: colour with detected hazards boxed and labelled, depth, the
signed residual map (blue is below the plane, red is above), and the live
numbers.

| key | action |
|---|---|
| `q` | quit |
| `s` | start / stop logging to `out/` |
| `p` | save a screenshot of the panel |
| `[` `]` | lower / raise the hazard threshold |
| `1`-`9` | label the surface being logged |

Label `1` is `flat_calib` and matters most: log flat ground at a range of
distances, because that measures the sensor noise floor, and the noise floor is
what sets the usable band, which sets the arm pose.

## Make the figures

    python3 make_plots.py

- `fig1_noise_vs_range.png` noise floor against range. **The plot that justifies
  the arm pose.**
- `fig2_surface.png` unevenness per surface. Shows the metric separates real
  surfaces above that noise floor.
- `fig3_detection.png` measured size against distance ahead. Shows how far out
  a hazard is still measurable.

## What the numbers mean

**unevenness** is the RMS residual to the fitted plane over the ground that is
*not* flagged as a hazard. It includes sensor noise, so it is only meaningful
relative to the flat-ground baseline at the same range. That is why `flat_calib`
is logged first.

**size** for a hazard is its height: the median absolute residual over the
hazard's outline grown down to half the threshold. The hazard is still detected
at the full threshold. Measuring only the pixels above the threshold keeps the
top of the noise and reads thin objects 4 to 10 mm high; on 24 tape-measured
passes the grown median is off by 1.7 mm on average (see `BRANCH_NOTES.md`).
`peak_mm`, the 90th percentile of the pixels above the threshold, is kept
alongside it.

The residual field is median filtered 5x5 before anything is measured. Depth
noise is independent per pixel, so this removes most of it while leaving real
steps intact, and it is what takes the flat-ground noise floor from roughly
4 mm to roughly 1 mm in simulation.

A hazard must also cover at least 25 cm^2 of real ground. Pixel noise does not
form connected blobs that large, so this is what suppresses false positives.

## Limits, stated up front

- Nothing subtler than the threshold is detectable, by design. At a 15 mm
  threshold a 10 mm object is invisible. Lower the threshold and false
  positives rise; the honest trade has to be measured on real ground.
- Cracks are far below the depth noise floor at any usable range. They come
  from the colour image, not from depth. This tool is for holes and bumps.
- The RANSAC fit assumes one dominant ground plane in view. On a slope change
  or a kerb it will pick one of the two surfaces and report the other as a
  hazard. That is often the behaviour you want, but it is worth knowing.
