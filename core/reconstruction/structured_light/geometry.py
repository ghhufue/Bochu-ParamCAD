"""Calibrated pinhole triangulation, using only decoded image correspondences."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from . import StructuredLightError


def rigid_transform(value, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise StructuredLightError(f"{name}: expected finite 4x4 transform")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise StructuredLightError(f"{name}: invalid homogeneous row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(rotation), 1, atol=1e-6):
        raise StructuredLightError(f"{name}: rotation must be right-handed and orthonormal")
    return matrix


@dataclass(frozen=True)
class Device:
    width: int
    height: int
    K: np.ndarray
    to_world: np.ndarray

    @classmethod
    def from_dict(cls, source: dict, transform, name: str) -> "Device":
        if source.get("model") != "PINHOLE":
            raise StructuredLightError(f"{name}: only distortion-free PINHOLE is supported")
        width, height = source.get("width"), source.get("height")
        if type(width) is not int or type(height) is not int or min(width, height) < 2:
            raise StructuredLightError(f"{name}: invalid device dimensions")
        intrinsic = np.asarray(source.get("K"), dtype=np.float64)
        if intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all() or not np.allclose(
                intrinsic[2], [0, 0, 1]) or intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0:
            raise StructuredLightError(f"{name}: invalid pinhole intrinsics")
        if abs(intrinsic[0, 1]) > 1e-8 or abs(intrinsic[1, 0]) > 1e-8:
            raise StructuredLightError(f"{name}: skew is not supported")
        return cls(width, height, intrinsic, rigid_transform(transform, name))

    def rays(self, uv: np.ndarray) -> np.ndarray:
        local = np.column_stack((uv, np.ones(len(uv)))) @ np.linalg.inv(self.K).T
        rays = local @ self.to_world[:3, :3].T
        return rays / np.linalg.norm(rays, axis=1, keepdims=True)

    def project(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        local = (points - self.to_world[:3, 3]) @ self.to_world[:3, :3]
        projected = local @ self.K.T
        with np.errstate(divide="ignore", invalid="ignore"):
            uv = projected[:, :2] / projected[:, 2, None]
        return uv, local[:, 2]


def triangulate(camera: Device, projector: Device, camera_uv: np.ndarray,
                projector_uv: np.ndarray, *, min_angle_deg: float = .5,
                max_reprojection_px: float = 1.5) -> tuple[np.ndarray, np.ndarray, dict]:
    if not (0 < min_angle_deg < 90) or not math.isfinite(max_reprojection_px) or max_reprojection_px <= 0:
        raise StructuredLightError("invalid triangulation quality thresholds")
    center_a, center_b = camera.to_world[:3, 3], projector.to_world[:3, 3]
    if np.linalg.norm(center_a - center_b) < 1e-8:
        raise StructuredLightError("camera/projector baseline is zero")
    a, b = camera.rays(camera_uv), projector.rays(projector_uv)
    dot = np.einsum("ij,ij->i", a, b)
    denominator = 1 - dot * dot
    delta = center_a - center_b
    d, e = a @ delta, b @ delta
    with np.errstate(divide="ignore", invalid="ignore"):
        distance_a = (dot * e - d) / denominator
        distance_b = (e - dot * d) / denominator
        point_a = center_a + distance_a[:, None] * a
        point_b = center_b + distance_b[:, None] * b
    points = (point_a + point_b) / 2
    uv_a, depth_a = camera.project(points)
    uv_b, depth_b = projector.project(points)
    error = np.maximum(np.linalg.norm(uv_a - camera_uv, axis=1),
                       np.linalg.norm(uv_b - projector_uv, axis=1))
    angle_valid = denominator >= math.sin(math.radians(min_angle_deg)) ** 2
    valid = (angle_valid & (distance_a > 0) & (distance_b > 0) & (depth_a > 0)
             & (depth_b > 0) & np.isfinite(points).all(axis=1) & (error <= max_reprojection_px))
    return points, valid, dict(depth=depth_a, reprojection_error=error,
                              ray_gap=np.linalg.norm(point_a - point_b, axis=1),
                              angle_weight=np.clip(denominator, 0, 1))


def fuse(points: np.ndarray, weights: np.ndarray, voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    """Backward-compatible entry point; the rule lives in ``downsample``."""
    from .downsample import voxel_downsample
    return voxel_downsample(points, weights, voxel_size)
