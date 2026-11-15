"""Surface normals from organized measured points and the observing camera."""

from __future__ import annotations

import numpy as np


def oriented_normals(points: np.ndarray, camera_position: np.ndarray) -> np.ndarray:
    """Return unit normals facing the camera; zero means insufficient evidence.

    Fit a local plane to a 5x5 image window. Missing pixels, depth jumps,
    grazing views and neighborhoods without a stable planar direction do not
    supply signed normal seeds. No scene geometry is consulted.
    """
    points = np.asarray(points, dtype=np.float64)
    camera_position = np.asarray(camera_position, dtype=np.float64)
    if points.ndim != 3 or points.shape[2] != 3 or camera_position.shape != (3,):
        raise ValueError("normal estimation needs an HxWx3 point image and a camera position")
    if not np.isfinite(camera_position).all():
        raise ValueError("camera position must be finite")
    result = np.zeros_like(points)
    if min(points.shape[:2]) < 3:
        return result
    yy, xx = np.nonzero(np.isfinite(points).all(axis=-1))
    interior = (yy > 0) & (yy < points.shape[0] - 1) & (xx > 0) & (xx < points.shape[1] - 1)
    yy, xx = yy[interior], xx[interior]
    if not len(xx):
        return result
    center = points[yy, xx]
    adjacent = np.stack([points[yy + dy, xx + dx] - center
                         for dy, dx in ((0, 1), (1, 0), (0, -1), (-1, 0))], axis=1)
    lengths = np.linalg.norm(adjacent, axis=-1)
    shortest = np.min(np.where(np.isfinite(lengths) & (lengths > 1e-12), lengths, np.inf), axis=1)
    padded = np.pad(points, ((2, 2), (2, 2), (0, 0)), constant_values=np.nan)
    offsets = [(dy, dx) for dy in range(-2, 3) for dx in range(-2, 3)]
    delta = np.stack([padded[yy + dy + 2, xx + dx + 2] - center for dy, dx in offsets], axis=1)
    radius = np.array([max(1, np.hypot(dy, dx)) for dy, dx in offsets])
    valid = np.isfinite(delta).all(axis=-1) & (np.linalg.norm(delta, axis=-1) <= shortest[:, None] * radius * 4)
    delta = np.where(valid[..., None], delta, 0)
    count = np.maximum(valid.sum(axis=1), 1)
    mean = delta.sum(axis=1) / count[:, None]
    covariance = np.einsum("nki,nkj->nij", delta, delta) / count[:, None, None]
    covariance -= np.einsum("ni,nj->nij", mean, mean)
    values, vectors = np.linalg.eigh(covariance)
    normal = vectors[:, :, 0]
    view = camera_position - center
    facing = np.einsum("ni,ni->n", normal, view)
    normal *= np.where(facing < 0, -1, 1)[:, None]
    reliable = ((count >= 9) & np.isfinite(shortest)
                & (values[:, 1] > np.maximum(1e-18, values[:, 2] * .01))
                & (values[:, 0] <= values[:, 1] * .05)
                & (np.abs(facing) > np.linalg.norm(view, axis=1) * .05))
    result[yy[reliable], xx[reliable]] = normal[reliable]
    return result
