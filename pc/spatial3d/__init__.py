"""Spatial3D PC-layer: reconstruct a 3D static spatial model from the DSP
voxel map synchronized over UART.

Pipeline (see project brief):
    UART voxel map -> occupancy grid -> point cloud -> plane/furniture fit
"""

from .voxel import Voxel, VoxelGrid, GRID_DIMS, VOXEL_SIZE_M

__all__ = ["Voxel", "VoxelGrid", "GRID_DIMS", "VOXEL_SIZE_M"]
