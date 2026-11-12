"""Versioned acquisition contract shared by all capture rendering backends."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re

import numpy as np

from . import StructuredLightError
from .geometry import Device, rigid_transform
from .patterns import Pattern, sequence

PIXEL_CONVENTION = "top-left; centers=(i+0.5,j+0.5)"


def read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as exc:
        raise StructuredLightError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise StructuredLightError(f"expected JSON object: {path}")
    return value


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def asset(root: Path, relative: str, checksum: str | None = None) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise StructuredLightError("asset paths must be nonempty and relative to the capture")
    resolved = (root / relative).resolve()
    if not resolved.is_relative_to(root) or not resolved.is_file():
        raise StructuredLightError(f"missing asset or path outside capture: {relative}")
    if not isinstance(checksum, str) or not re.fullmatch(r"[a-f0-9]{64}", checksum):
        raise StructuredLightError(f"missing or invalid asset checksum: {relative}")
    if sha256(resolved) != checksum:
        raise StructuredLightError(f"asset checksum mismatch: {relative}")
    return resolved


def schema(source: dict, expected: str) -> None:
    if source.get("schema") != expected or source.get("version") != 1:
        raise StructuredLightError(f"unsupported schema; expected {expected} v1")


@dataclass(frozen=True)
class Station:
    id: str
    camera: Device
    projector: Device
    # One complete pattern sequence per exposure. Different exposures are not
    # interleaved during bit decoding; valid results are selected afterwards.
    exposures: dict[str, dict[str, Path]]


@dataclass(frozen=True)
class Capture:
    path: Path
    manifest: dict
    patterns: list[Pattern]
    width: int
    height: int
    phase_steps: int
    period: float
    stations: list[Station]


def load_capture(path: str | Path) -> Capture:
    manifest_path = Path(path).resolve()
    if manifest_path.is_dir():
        manifest_path /= "capture.json"
    root = manifest_path.parent
    source = read_json(manifest_path)
    schema(source, "paramcad.structured-light.capture")
    if source.get("status") != "complete":
        raise StructuredLightError("capture is incomplete, failed or cancelled")
    if source.get("unit") != "mm" or source.get("pixel_convention") != PIXEL_CONVENTION:
        raise StructuredLightError("capture must declare mm and the supported pixel convention")
    if source.get("intensity_encoding") != "linear_uint8":
        raise StructuredLightError("only fixed linear_uint8 captures are currently supported")
    asset(root, source.get("scene", ""), source.get("scene_sha256"))
    calibration_path = asset(root, source.get("calibration", ""), source.get("calibration_sha256"))
    calibration = read_json(calibration_path)
    schema(calibration, "paramcad.structured-light.calibration")
    if calibration.get("unit") != "mm":
        raise StructuredLightError("calibration unit must be mm")
    definitions = calibration.get("devices", {})
    settings = source.get("pattern_set", {})
    if not isinstance(definitions, dict) or not isinstance(settings, dict):
        raise StructuredLightError("device definitions and pattern settings must be objects")
    width, height = settings.get("width"), settings.get("height")
    phase_steps, period = settings.get("phase_steps", 0), settings.get("period", 16)
    if type(width) is not int or type(height) is not int:
        raise StructuredLightError("invalid projector pattern dimensions")
    if type(phase_steps) is not int or not isinstance(period, (int, float)) or not np.isfinite(period) or period <= 2:
        raise StructuredLightError("invalid phase settings")
    expected = sequence(width, height, phase_steps)
    declarations = settings.get("patterns", [])
    if not isinstance(declarations, list) or len(declarations) != len(expected) or any(
            not isinstance(record, dict) for record in declarations):
        raise StructuredLightError("pattern declarations are incomplete")
    for actual, required in zip(declarations, expected):
        for key, value in required.as_dict().items():
            if actual.get(key) != value:
                raise StructuredLightError(f"pattern declaration mismatch: {required.id}/{key}")
        asset(root, actual.get("image", ""), actual.get("sha256"))
    expected_ids = {p.id for p in expected}
    result = []
    seen = set()
    stations = source.get("stations", [])
    if not isinstance(stations, list) or any(not isinstance(record, dict) for record in stations):
        raise StructuredLightError("stations must be a list of station objects")
    for station in stations:
        identifier = station.get("id", "")
        if not isinstance(identifier, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", identifier) or identifier in seen:
            raise StructuredLightError("station IDs must be unique safe identifiers")
        seen.add(identifier)
        rig = rigid_transform(station.get("rig_to_world"), f"{identifier}/rig")
        devices = []
        for key in ("camera", "projector"):
            device_id = station.get(f"{key}_id")
            if device_id not in definitions:
                raise StructuredLightError(f"{identifier}: missing {key} calibration")
            definition = definitions[device_id]
            if not isinstance(definition, dict):
                raise StructuredLightError(f"{identifier}: invalid {key} calibration object")
            transform = rig @ rigid_transform(definition.get("device_to_rig"), device_id)
            devices.append(Device.from_dict(definition, transform, device_id))
        camera, projector = devices
        if (projector.width, projector.height) != (width, height):
            raise StructuredLightError(f"{identifier}: pattern/projector dimensions differ")
        if np.linalg.norm(camera.to_world[:3, 3] - projector.to_world[:3, 3]) < 1e-8:
            raise StructuredLightError(f"{identifier}: camera/projector baseline is zero")
        exposures: dict[str, dict[str, Path]] = {}
        frames = station.get("frames", [])
        if not isinstance(frames, list) or any(not isinstance(record, dict) for record in frames):
            raise StructuredLightError(f"{identifier}: frames must be a list of objects")
        for frame in frames:
            exposure_id, pattern_id = frame.get("exposure_id"), frame.get("pattern_id")
            if not isinstance(exposure_id, str) or not exposure_id or pattern_id not in expected_ids or frame.get("status") != "complete":
                raise StructuredLightError(f"{identifier}: invalid frame identity or state")
            group = exposures.setdefault(exposure_id, {})
            if pattern_id in group:
                raise StructuredLightError(f"{identifier}: duplicate pattern frame")
            checksum = frame.get("sha256")
            if not isinstance(checksum, str):
                raise StructuredLightError(f"{identifier}: missing frame checksum")
            group[pattern_id] = asset(root, frame.get("image", ""), checksum)
        if not exposures or any(set(group) != expected_ids for group in exposures.values()):
            raise StructuredLightError(f"{identifier}: missing pattern frames in exposure sequence")
        result.append(Station(identifier, camera, projector, exposures))
    if not result:
        raise StructuredLightError("capture has no stations")
    return Capture(manifest_path, source, expected, width, height, phase_steps, float(period), result)
