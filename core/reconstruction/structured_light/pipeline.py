"""Image decoding, calibrated triangulation and multi-station point fusion."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import sys
import time
from typing import Callable

import cv2
import numpy as np

from . import StructuredLightError, __version__
from .capture import load_capture, sha256
from .downsample import voxel_downsample
from .geometry import triangulate
from .normals import oriented_normals
from .patterns import decode


class Cancelled(StructuredLightError):
    pass


def peak_process_memory_bytes() -> int:
    """OS process high-water mark, including native NumPy/OpenCV allocations."""
    if sys.platform == "win32":
        import ctypes
        from ctypes import wintypes

        class Counters(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD)] + [
                (name, ctypes.c_size_t) for name in (
                    "PeakWorkingSetSize", "WorkingSetSize", "QuotaPeakPagedPoolUsage",
                    "QuotaPagedPoolUsage", "QuotaPeakNonPagedPoolUsage", "QuotaNonPagedPoolUsage",
                    "PagefileUsage", "PeakPagefileUsage")]

        counters = Counters()
        counters.cb = ctypes.sizeof(counters)
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.GetCurrentProcess.restype = wintypes.HANDLE
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD]
        if not psapi.GetProcessMemoryInfo(kernel.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
            raise OSError(ctypes.get_last_error(), "Cannot query process peak working set")
        return counters.PeakWorkingSetSize
    import resource
    peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(peak if sys.platform == "darwin" else peak * 1024)


@dataclass(frozen=True)
class Options:
    min_contrast: float = 8.0
    saturation: float = 254.0
    min_angle_deg: float = .5
    max_reprojection_px: float = 1.5
    voxel_size_mm: float = .2


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                         encoding="utf-8")
    os.replace(temporary, path)


def write_ply(path: Path, points: np.ndarray, normals: np.ndarray | None = None) -> None:
    if normals is not None and (normals.shape != points.shape or not np.isfinite(normals).all()):
        raise StructuredLightError("PLY normals must be finite and align with positions")
    temporary = path.with_suffix(".ply.tmp")
    with temporary.open("wb") as stream:
        stream.write(("ply\nformat binary_little_endian 1.0\ncomment unit mm\n"
                      f"element vertex {len(points)}\nproperty double x\nproperty double y\n"
                      "property double z\n" +
                      ("property double nx\nproperty double ny\nproperty double nz\n"
                       if normals is not None else "") + "end_header\n").encode("ascii"))
        rows = points if normals is None else np.column_stack((points, normals))
        stream.write(np.asarray(rows, dtype="<f8").tobytes())
    os.replace(temporary, path)


def read_image(path: Path, shape: tuple[int, int]) -> np.ndarray:
    raw = np.fromfile(path, np.uint8)
    image = cv2.imdecode(raw, cv2.IMREAD_UNCHANGED)
    if image is None or image.dtype != np.uint8 or image.shape[:2] != shape:
        raise StructuredLightError(f"invalid image dimensions/format: {path}")
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY if image.shape[2] == 4 else cv2.COLOR_BGR2GRAY)
    return image


def reconstruct(capture_path: str | Path, output: str | Path, *, options: Options | None = None,
                event: Callable[[dict], None] | None = None, cancel_file: Path | None = None) -> dict:
    settings = options or Options()
    root = Path(output).resolve()
    source = Path(capture_path).resolve()
    acquisition_root = source if source.is_dir() else source.parent
    if root.is_relative_to(acquisition_root):
        raise StructuredLightError("reconstruction output must be outside the acquisition directory")
    if root.exists() and any(root.iterdir()):
        raise StructuredLightError("reconstruction run directory must be empty")
    root.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    timings = dict(validate=0.0, decode=0.0, triangulate=0.0, fuse=0.0, export=0.0)
    state = dict(schema="paramcad.structured-light.result", version=1, status="running",
                 engine="structured_light", engine_version=__version__, unit="mm",
                 coordinate_system="right-handed; capture world frame",
                 options=asdict(settings), point_cloud="cloud.ply", point_count=0,
                 camera_mode="provided", density="dense", stages=[])
    write_json(root / "result.json", state)
    sequence_number = 0

    def publish(kind: str, **values) -> None:
        nonlocal sequence_number
        sequence_number += 1
        message = dict(schema="paramcad.reconstruction.event", version=1, type=kind,
                       job_id=root.name, sequence=sequence_number, **values)
        with (root / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(message, ensure_ascii=False, allow_nan=False) + "\n")
        if event:
            event(message)

    def check_cancel() -> None:
        if cancel_file is not None and cancel_file.exists():
            raise Cancelled("reconstruction cancelled")

    try:
        check_cancel()
        publish("progress", stage="validate", completed=0, total=1)
        capture = load_capture(capture_path)
        # Validate all thresholds before decoding a potentially large acquisition.
        values = np.array(list(asdict(settings).values()))
        if not np.isfinite(values).all() or not (0 < settings.min_contrast < settings.saturation <= 255
                and 0 < settings.min_angle_deg < 90 and settings.max_reprojection_px > 0
                and settings.voxel_size_mm > 0):
            raise StructuredLightError("invalid reconstruction options")
        state.update(capture=str(capture.path), capture_sha256=sha256(capture.path),
                     rendering=capture.manifest.get("rendering", {}))
        timings["validate"] = time.monotonic() - started
        publish("progress", stage="validate", completed=1, total=1)
        station_reports, all_points, all_weights, all_normals = [], [], [], []
        total = len(capture.stations)
        for index, station in enumerate(capture.stations):
            check_cancel()
            stage_started = time.monotonic()
            publish("progress", stage="decode", station=station.id, completed=index, total=total)
            shape = station.camera.height, station.camera.width
            coordinates = np.full((*shape, 2), np.nan, np.float32)
            confidence = np.zeros(shape, np.float32)
            chosen = np.full(shape, -1, np.int16)
            for exposure_index, (_, group) in enumerate(station.exposures.items()):
                images = {}
                for pattern_id, path in group.items():
                    check_cancel()
                    images[pattern_id] = read_image(path, shape)
                decoded, valid, score = decode(images, capture.width, capture.height,
                    min_contrast=settings.min_contrast, saturation=settings.saturation,
                    phase_steps=capture.phase_steps, period=capture.period)
                better = valid & (score > confidence)
                coordinates[better], confidence[better] = decoded[better], score[better]
                chosen[better] = exposure_index
                del images
            timings["decode"] += time.monotonic() - stage_started
            stage_started = time.monotonic()
            station_root = root / "stations" / station.id
            station_root.mkdir(parents=True)
            np.save(station_root / "projector_coordinates.npy", coordinates)
            np.save(station_root / "decode_confidence.npy", confidence)
            np.save(station_root / "exposure_index.npy", chosen)
            yy, xx = np.nonzero(confidence > 0)
            camera_uv = np.column_stack((xx + .5, yy + .5))
            projector_uv = coordinates[yy, xx]
            publish("progress", stage="triangulate", station=station.id, completed=index, total=total)
            depth = np.full(shape, np.nan, np.float32)
            mask = np.zeros(shape, np.uint8)
            point_image = np.full((*shape, 3), np.nan, np.float64)
            station_points, station_weights, errors = [], [], []
            for offset in range(0, len(xx), 100000):
                check_cancel()
                limit = slice(offset, offset + 100000)
                points, valid, quality = triangulate(station.camera, station.projector,
                    camera_uv[limit], projector_uv[limit], min_angle_deg=settings.min_angle_deg,
                    max_reprojection_px=settings.max_reprojection_px)
                xs, ys = xx[limit][valid], yy[limit][valid]
                depth[ys, xs] = quality["depth"][valid]
                mask[ys, xs] = 1
                point_image[ys, xs] = points[valid]
                station_points.append(points[valid])
                station_weights.append(confidence[ys, xs] * quality["angle_weight"][valid])
                errors.extend(quality["reprojection_error"][valid].tolist())
            points = np.concatenate(station_points) if station_points else np.empty((0, 3))
            weights = np.concatenate(station_weights) if station_weights else np.empty(0)
            normal_image = oriented_normals(point_image, station.camera.to_world[:3, 3])
            # Triangulation chunks preserve the row-major decoded pixel order.
            normals = normal_image[mask.astype(bool)]
            write_ply(station_root / "cloud.ply", points, normals)
            np.save(station_root / "depth_camera_z_mm.npy", depth)
            np.save(station_root / "valid_mask.npy", mask)
            report = dict(station=station.id, pixels=shape[0] * shape[1], decoded_pixels=len(xx),
                point_count=len(points), valid_ratio=float(mask.mean()),
                oriented_normals=int(np.count_nonzero(np.linalg.norm(normals, axis=1) > 0)),
                reprojection_p50_px=float(np.median(errors)) if errors else None,
                reprojection_p95_px=float(np.percentile(errors, 95)) if errors else None,
                exposure_ids=list(station.exposures),
                status="complete" if len(points) else "no_valid_points")
            station_reports.append(report)
            all_points.append(points)
            all_weights.append(weights)
            all_normals.append(normals)
            publish("log", message=f"{station.id}: {len(points)} valid points / {len(xx)} decoded pixels")
            publish("progress", stage="triangulate", station=station.id, completed=index + 1, total=total)
            timings["triangulate"] += time.monotonic() - stage_started
        check_cancel()
        stage_started = time.monotonic()
        publish("progress", stage="fuse", completed=0, total=1)
        points, counts, normals = voxel_downsample(np.concatenate(all_points), np.concatenate(all_weights),
                                                   settings.voxel_size_mm, np.concatenate(all_normals))
        if len(points) == 0:
            raise StructuredLightError("no valid points; inspect contrast, exposure, visibility and calibration")
        timings["fuse"] = time.monotonic() - stage_started
        publish("progress", stage="fuse", completed=1, total=1)
        stage_started = time.monotonic()
        write_ply(root / "cloud.ply", points, normals)
        np.save(root / "observation_counts.npy", counts)
        timings["export"] = time.monotonic() - stage_started
        image_paths = [path for station in capture.stations for group in station.exposures.values()
                       for path in group.values()]
        quality = dict(stations=station_reports, input_points=sum(s["point_count"] for s in station_reports),
            output_points=len(points), valid_stations=sum(s["point_count"] > 0 for s in station_reports),
            oriented_normals=int(np.count_nonzero(np.linalg.norm(normals, axis=1) > 0)),
            normal_definition="unit surface normals facing each observing camera; zero is unknown",
            elapsed_seconds=time.monotonic() - started,
            bounds_mm=[points.min(axis=0).tolist(), points.max(axis=0).tolist()],
            cost=dict(station_count=total, frame_count=len(image_paths),
                      image_bytes=sum(path.stat().st_size for path in image_paths),
                      capture_elapsed_ms=capture.manifest.get("elapsed_ms"),
                      stage_seconds=timings, peak_process_memory_bytes=peak_process_memory_bytes(),
                      memory_definition="OS process peak resident working set, including interpreter and native libraries"),
            evaluation="image-only diagnostics; CAD ground truth not used")
        write_json(root / "quality.json", quality)
        state.update(status="complete", point_count=len(points), valid_stations=quality["valid_stations"],
                     oriented_normals=quality["oriented_normals"],
                     elapsed_seconds=quality["elapsed_seconds"], quality="quality.json",
                     depth_definition="camera optical-axis Z in mm; invalid=NaN")
        check_cancel()
        write_json(root / "result.json", state)
        publish("progress", stage="export", completed=1, total=1)
        publish("result", point_cloud=str(root / "cloud.ply"), point_count=len(points),
                manifest=str(root / "result.json"), camera_mode="provided", unit="mm")
        return state
    except (StructuredLightError, OSError, KeyError, TypeError, ValueError) as exc:
        state.update(status="cancelled" if isinstance(exc, Cancelled) else "failed", error=str(exc),
                     elapsed_seconds=time.monotonic() - started)
        write_json(root / "result.json", state)
        publish("error", message=str(exc), status=state["status"])
        raise StructuredLightError(str(exc)) from exc
