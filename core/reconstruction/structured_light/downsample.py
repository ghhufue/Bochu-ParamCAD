"""Voxel downsampling shared by the reconstruction pipeline and its callers.

The same rule is used for the in-memory triangulated points produced by a
structured-light reconstruction and for any point cloud that is downsampled
later; keeping it in one module is what makes both paths comparable.
"""

from __future__ import annotations

import math

import numpy as np

from . import StructuredLightError


def voxel_downsample(points: np.ndarray, weights: np.ndarray,
                     voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    """Merge points that fall into the same voxel into one weighted centroid.

    ``weights`` must have one entry per point. The returned counts are the
    number of observations merged into each output point.
    """
    if not math.isfinite(voxel_size) or voxel_size <= 0:
        raise StructuredLightError("voxel_size_mm must be finite and positive")
    if len(points) == 0:
        return np.empty((0, 3)), np.empty(0, np.int64)
    if len(weights) != len(points):
        raise StructuredLightError("voxel downsampling needs one weight per point")
    scaled = np.floor(points / voxel_size)
    if not np.isfinite(scaled).all() or np.abs(scaled).max() >= 2**62:
        raise StructuredLightError("voxel coordinates overflow; check scale and units")
    clipped = np.maximum(weights, 1e-12)
    _, indices, counts = np.unique(scaled.astype(np.int64), axis=0,
                                   return_inverse=True, return_counts=True)
    sums = np.bincount(indices, weights=clipped)
    merged = np.column_stack([np.bincount(indices, weights=points[:, axis] * clipped)
                              / sums for axis in range(3)])
    return merged, counts
