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
                     voxel_size: float, normals: np.ndarray | None = None
                     ) -> tuple[np.ndarray, np.ndarray] | tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Merge points that fall into the same voxel into one weighted centroid.

    ``weights`` must have one entry per point. The returned counts are the
    number of observations merged into each output point. With normals, return
    (points, counts, normals) and split incompatible surfaces inside each voxel.
    """
    if not math.isfinite(voxel_size) or voxel_size <= 0:
        raise StructuredLightError("voxel_size_mm must be finite and positive")
    points = np.asarray(points, dtype=np.float64)
    weights = np.asarray(weights, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise StructuredLightError("voxel downsampling needs finite Nx3 positions")
    if len(weights) != len(points):
        raise StructuredLightError("voxel downsampling needs one weight per point")
    if weights.ndim != 1 or not np.isfinite(weights).all():
        raise StructuredLightError("voxel downsampling needs finite scalar weights")
    if normals is not None:
        normals = np.asarray(normals, dtype=np.float64)
        if normals.shape != points.shape or not np.isfinite(normals).all():
            raise StructuredLightError("voxel downsampling needs aligned finite Nx3 normals")
    if len(points) == 0:
        result = (np.empty((0, 3)), np.empty(0, np.int64))
        return result if normals is None else (*result, np.empty((0, 3)))
    scaled = np.floor(points / voxel_size)
    if not np.isfinite(scaled).all() or np.abs(scaled).max() >= 2**62:
        raise StructuredLightError("voxel coordinates overflow; check scale and units")
    clipped = np.maximum(weights, 1e-12)
    _, indices, counts = np.unique(scaled.astype(np.int64), axis=0,
                                   return_inverse=True, return_counts=True)
    if normals is not None:
        lengths = np.linalg.norm(normals, axis=1)
        known = lengths > 1e-12
        normals = np.divide(normals, lengths[:, None], out=np.zeros_like(normals),
                            where=known[:, None])
        # Process one representative per occupied voxel at a time, in parallel.
        # Testing against fixed seeds prevents transitive smoothing across edges.
        remaining = np.arange(len(points))
        groups = np.empty(len(points), np.int64)
        group_count = 0
        while len(remaining):
            seeds = np.full(len(counts), len(points), np.int64)
            np.minimum.at(seeds, indices[remaining], remaining)
            seed = seeds[indices[remaining]]
            compatible = known[remaining] == known[seed]
            alignment = np.einsum("ij,ij->i", normals[remaining], normals[seed])
            displacement = points[remaining] - points[seed]
            plane_distance = np.maximum(
                np.abs(np.einsum("ij,ij->i", displacement, normals[seed])),
                np.abs(np.einsum("ij,ij->i", displacement, normals[remaining])))
            compatible &= (~known[remaining] | ((alignment >= np.cos(np.deg2rad(30)))
                                                 & (plane_distance <= voxel_size * .25)))
            selected = remaining[compatible]
            _, local = np.unique(indices[selected], return_inverse=True)
            groups[selected] = group_count + local
            group_count += int(local.max()) + 1
            remaining = remaining[~compatible]
        indices = groups
        counts = np.bincount(indices)
    sums = np.bincount(indices, weights=clipped)
    merged = np.column_stack([np.bincount(indices, weights=points[:, axis] * clipped)
                              / sums for axis in range(3)])
    if normals is None:
        return merged, counts
    merged_normals = np.column_stack([
        np.bincount(indices, weights=normals[:, axis] * clipped) / sums for axis in range(3)])
    lengths = np.linalg.norm(merged_normals, axis=1)
    merged_normals = np.divide(merged_normals, lengths[:, None], out=np.zeros_like(merged_normals),
                               where=lengths[:, None] > 1e-12)
    return merged, counts, merged_normals
