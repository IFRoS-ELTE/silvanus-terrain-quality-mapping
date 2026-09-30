"""Crack segmentation training code for the Silvanus terrain mapping project.

This package is plain Python and PyTorch. It does not import ROS, so it
runs the same on Windows, WSL and the GPU server. The ROS node that uses
the exported ONNX model lives in the `terrain_vision` package later on.
"""

__version__ = "0.1.0"
