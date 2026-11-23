#!/usr/bin/env python3
"""Run Studio cloud analysis/reconstruction and capture each successful stage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PCAD = ROOT / "apps" / "ParamCAD Studio" / "build" / "msvc-debug" / "Release" / "pcad.exe"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--cloud", type=Path, help="Existing PLY input cloud")
    source.add_argument("--model", type=Path, help="STEP/STP model to acquire with structured light")
    parser.add_argument("--output", required=True, type=Path, help="Case output directory")
    parser.add_argument("--pcad", type=Path, default=DEFAULT_PCAD)
    parser.add_argument("--session", help="Studio session ID when more than one Studio is running")
    parser.add_argument("--elevation", type=float, default=45)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--frames", type=int, default=96, help="Number of camera poses per revolution")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--allow-missing-solid", action="store_true",
        help="Keep acquisition and surface videos when CAD reconstruction fails")
    parser.add_argument("--ffmpeg", default="ffmpeg")
    return parser.parse_args()


def run_cli(pcad: Path, arguments: list[str], session: str | None = None, report: Path | None = None) -> dict:
    command = [str(pcad), "--online", "--timeout", "600"]
    if session:
        command.extend(("--session", session))
    if report:
        report.parent.mkdir(parents=True, exist_ok=True)
        command.extend(("--report", str(report)))
    command.extend(arguments)
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if completed.returncode and report is None:
        raise RuntimeError(completed.stdout + completed.stderr)
    if report and report.is_file():
        result = json.loads(report.read_text(encoding="utf-8"))
        return result
    try:
        result = json.loads(completed.stdout)
    except json.JSONDecodeError:
        result = {"exit_code": completed.returncode, "stdout": completed.stdout, "stderr": completed.stderr}
    result.setdefault("exit_code", completed.returncode)
    return result


def run_cli_when_idle(pcad: Path, arguments: list[str], session: str | None = None) -> dict:
    for attempt in range(10):
        completed = subprocess.run(
            [str(pcad), "--online", "--session", session, "status"] if session else
            [str(pcad), "--online", "status"], cwd=ROOT, text=True, capture_output=True)
        if completed.returncode:
            raise RuntimeError(completed.stdout + completed.stderr)
        if not json.loads(completed.stdout).get("busy"):
            return run_cli(pcad, arguments, session)
        time.sleep(0.5)
    raise RuntimeError("Studio remained busy after CAD reconstruction")


def capture_spin(pcad: Path, directory: Path, target: str, elevation: float, session: str | None,
                 width: int, height: int, fps: int, frame_count: int, ffmpeg: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    frames = directory / "frames"
    frames.mkdir(exist_ok=True)
    for index in range(frame_count):
        image = frames / f"frame_{index:03}.png"
        run_cli(pcad, [
            "capture", "frame", "--output", str(image), "--target", target,
            "--azimuth", str(index * 360 / frame_count), "--elevation", str(elevation),
            "--width", str(width), "--height", str(height),
        ], session)
    video = directory / "spin.mp4"
    subprocess.run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(fps),
        "-i", str(frames / "frame_%03d.png"), "-frames:v", str(frame_count),
        "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(video),
    ], check=True)
    return video


def main() -> int:
    args = parse_args()
    pcad = args.pcad.resolve()
    cloud = args.cloud.resolve() if args.cloud else None
    model = args.model.resolve() if args.model else None
    output = args.output.resolve()
    if not pcad.is_file() or (cloud is not None and not cloud.is_file()) or (model is not None and not model.is_file()):
        raise SystemExit("--pcad and the selected --cloud/--model input must point to existing files")
    if args.frames < 1 or args.fps < 1 or args.width < 32 or args.height < 32:
        raise SystemExit("Use positive --frames and --fps and --width/--height values of at least 32")

    if model is not None:
        acquisition = output / "structured-light"
        acquisition_result = run_cli(pcad, [
            "capture", "structured-light", "--in", str(model), "--output", str(acquisition),
            "--scan-reconstruct",
        ], args.session)
        (output / "structured-light-command.json").write_text(
            json.dumps(acquisition_result, indent=2) + "\n", encoding="utf-8")
        point_cloud = acquisition_result.get("point_cloud")
        if point_cloud:
            cloud = Path(point_cloud).resolve()
        else:
            candidates = sorted(acquisition.glob("reconstructions/**/cloud.ply"))
            if not candidates:
                raise SystemExit("Structured-light capture completed without a reconstructed cloud.ply")
            cloud = candidates[-1].resolve()
        if not cloud.is_file():
            raise SystemExit(f"Structured-light output cloud does not exist: {cloud}")
    run_cli(pcad, ["cloud", "open", "--in", str(cloud)], args.session)
    capture_spin(pcad, output / "point-cloud", "cloud", args.elevation, args.session, args.width, args.height, args.fps, args.frames, args.ffmpeg)

    detect_report = output / "surfaces" / "detect-command.json"
    detection = run_cli(pcad, ["detect", "--in", str(cloud)], args.session, detect_report)
    if detection.get("status") != "complete" or detection.get("exit_code") != 0:
        print(f"Surface detection failed. See {detect_report}", file=sys.stderr)
        return 1
    capture_spin(pcad, output / "surfaces", "surfaces", args.elevation, args.session, args.width, args.height, args.fps, args.frames, args.ffmpeg)

    reconstruct_report = output / "reconstructed" / "reconstruct-command.json"
    reconstruction = run_cli(pcad, ["reconstruct", "cad", "--in", str(cloud)], args.session, reconstruct_report)
    if reconstruction.get("status") != "complete" or reconstruction.get("exit_code") != 0:
        print(f"CAD reconstruction did not produce an entity. See {reconstruct_report}", file=sys.stderr)
        return 0 if args.allow_missing_solid else 2
    step_path = output / "reconstructed" / "reconstructed.step"
    exported = run_cli_when_idle(pcad, ["export", "step", "--out", str(step_path)], args.session)
    if exported.get("status") != "complete" or not step_path.is_file():
        print("CAD reconstruction completed but STEP export failed", file=sys.stderr)
        return 3
    capture_spin(pcad, output / "reconstructed", "cad", args.elevation, args.session,
                 args.width, args.height, args.fps, args.frames, args.ffmpeg)
    print(f"Saved reconstructed STEP to {step_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
