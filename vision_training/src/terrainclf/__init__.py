"""RGB terrain classifier: which surface the robot camera is looking at.

Separate from the crack model on purpose. Cracks need a pixel mask, the
terrain needs one label per region. How bumpy the ground is comes from depth
unevenness and the IMU; this package only answers what the surface is.
"""
