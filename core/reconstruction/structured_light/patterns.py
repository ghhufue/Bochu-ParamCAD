"""Deterministic monochrome projection patterns and radiometric decoding."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from . import StructuredLightError


@dataclass(frozen=True)
class Pattern:
    id: str
    kind: str
    axis: str = ""
    bit: int = -1
    inverted: bool = False
    phase: float = 0.0

    def as_dict(self) -> dict:
        return dict(id=self.id, kind=self.kind, axis=self.axis, bit=self.bit,
                    inverted=self.inverted, phase=self.phase)


def sequence(width: int, height: int, phase_steps: int = 0) -> list[Pattern]:
    if not (2 <= width <= 16384 and 2 <= height <= 16384):
        raise StructuredLightError("projector dimensions must be in [2, 16384]")
    if phase_steps not in (0, 4):
        raise StructuredLightError("phase_steps must be 0 or 4")
    result = [Pattern("black", "black"), Pattern("white", "white")]
    for axis, size in (("x", width), ("y", height)):
        for bit in reversed(range((size - 1).bit_length())):
            for inverted in (False, True):
                result.append(Pattern(f"{axis}_{bit}_{int(inverted)}", "gray", axis,
                                      bit, inverted))
    for axis in ("x", "y"):
        for step in range(phase_steps):
            result.append(Pattern(f"{axis}_phase_{step}", "phase", axis,
                                  phase=step * 2 * math.pi / phase_steps))
    return result


def image(pattern: Pattern, width: int, height: int, period: float = 16.0) -> np.ndarray:
    """Projector raster: top-left origin, pixel centers at (i+.5,j+.5)."""
    if pattern.kind in ("black", "white"):
        return np.full((height, width), 255 if pattern.kind == "white" else 0, np.uint8)
    size = width if pattern.axis == "x" else height
    coordinate = np.arange(size, dtype=np.uint32)
    if pattern.kind == "gray":
        row = (((coordinate ^ (coordinate >> 1)) >> pattern.bit) & 1) * 255
        if pattern.inverted:
            row = 255 - row
    elif pattern.kind == "phase" and period > 2:
        row = np.rint(127.5 * (1 + np.cos(2 * math.pi * (coordinate + .5) / period
                                        + pattern.phase)))
    else:
        raise StructuredLightError("invalid pattern kind or phase period")
    return np.broadcast_to(row[None, :] if pattern.axis == "x" else row[:, None],
                           (height, width)).astype(np.uint8).copy()


def decode(images: dict[str, np.ndarray], width: int, height: int, *,
           min_contrast: float = 8.0, saturation: float = 254.0,
           phase_steps: int = 0, period: float = 16.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return projector coordinates, validity and contrast; no geometry input."""
    expected = sequence(width, height, phase_steps)
    if set(images) != {p.id for p in expected}:
        raise StructuredLightError("pattern sequence has missing or unexpected images")
    if not (0 < min_contrast < saturation <= 255):
        raise StructuredLightError("invalid contrast/saturation thresholds")
    shape = images["black"].shape
    if len(shape) != 2 or any(a.shape != shape for a in images.values()):
        raise StructuredLightError("all pattern images must have the same 2D dimensions")
    black, white = (images[key].astype(np.float32) for key in ("black", "white"))
    contrast = white - black
    valid = np.isfinite(contrast) & (contrast >= min_contrast) & (white < saturation)
    confidence = np.clip(contrast / 255, 0, 1)
    coordinates = np.zeros((*shape, 2), np.float32)
    for dimension, (axis, size) in enumerate((("x", width), ("y", height))):
        binary = np.zeros(shape, np.uint32)
        previous = np.zeros(shape, np.uint32)
        for bit in reversed(range((size - 1).bit_length())):
            lit = images[f"{axis}_{bit}_0"].astype(np.float32)
            dark = images[f"{axis}_{bit}_1"].astype(np.float32)
            difference = lit - dark
            valid &= np.isfinite(difference) & (np.abs(difference) >= min_contrast)
            valid &= (lit < saturation) & (dark < saturation)
            confidence = np.minimum(confidence, np.abs(difference) / 255)
            previous ^= (difference > 0).astype(np.uint32)
            binary |= previous << bit
        valid &= binary < size
        coordinate = binary.astype(np.float32) + .5
        if phase_steps:
            if not math.isfinite(period) or period <= 2:
                raise StructuredLightError("phase period must be finite and greater than 2")
            cosine = np.zeros(shape, np.float32)
            sine = np.zeros(shape, np.float32)
            for step in range(phase_steps):
                sample = images[f"{axis}_phase_{step}"].astype(np.float32)
                phase = step * 2 * math.pi / phase_steps
                cosine += sample * math.cos(phase)
                sine -= sample * math.sin(phase)
                valid &= np.isfinite(sample) & (sample < saturation)
            fractional = np.mod(np.arctan2(sine, cosine), 2 * math.pi) / (2 * math.pi)
            refined = period * (fractional + np.rint(coordinate / period - fractional))
            amplitude = np.hypot(cosine, sine) * (2 / phase_steps)
            valid &= (amplitude >= min_contrast) & (np.abs(refined - coordinate) <= 1.0)
            valid &= (refined >= 0) & (refined < size)
            confidence = np.minimum(confidence, np.clip(amplitude / 127.5, 0, 1))
            coordinate = refined
        coordinates[..., dimension] = coordinate
    coordinates[~valid] = np.nan
    confidence[~valid] = 0
    return coordinates, valid, confidence
