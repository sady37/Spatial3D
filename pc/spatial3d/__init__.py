"""Spatial3D PC-layer: reconstruct a 3D static spatial model from the radar.

Interim pipeline (TI out-of-box demo streams a point cloud):
    TI TLV frames (UART) -> accumulate points into voxel grid -> planes/furniture
Long-term (custom DSP firmware, see project brief): DSP emits the voxel map directly.

See docs/GET-DATA-FROM-TI.md for how the radar is made to stream in the first place.
"""

from .voxel import Voxel, VoxelGrid, GRID_DIMS, VOXEL_SIZE_M

__all__ = ["Voxel", "VoxelGrid", "GRID_DIMS", "VOXEL_SIZE_M"]
