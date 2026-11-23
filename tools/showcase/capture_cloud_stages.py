#!/usr/bin/env python3
"""Run Studio cloud analysis/reconstruction and capture each successful stage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PCAD = ROOT / "apps" / "ParamCAD Studio" / "build" / "msvc-debug" / "Release" / "pcad.exe"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud", required=True, type=Path, help="PLY input cloud")
    parser.add_argument("--output", required=True, type=Path, help="Case output directory")
    parser.add_argument("--pcad", type=Path, default=DEFAULT_PCAD)
    parser.add_argument("--session", help="Studio session ID when more than one Studio is running")
    parser.add_argument("--elevation", type=float, default=-30)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
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
        return json.loads(report.read_text(encoding="utf-8"))
    return {"exit_code": completed.returncode, "stdout": completed.stdout, "stderr": completed.stderr}


def capture_spin(pcad: Path, directory: Path, target: str, elevation: float, session: str | None,
                 width: int, height: int, fps: int, ffmpeg: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    frames = directory / "frames"
    frames.mkdir(exist_ok=True)
    image_paths = []
    for index in range(12):
        image = frames / f"frame_{index:03}.png"
        run_cli(pcad, [
            "capture", "frame", "--output", str(image), "--target", target,
            "--azimuth", str(index * 30), "--elevation", str(elevation),
            "--width", str(width), "--height", str(height),
        ], session)
        image_paths.append(image)
    frame_list = directory / "frames.txt"
    duration = 1 / fps
    entries = [line for image in image_paths for line in (f"file '{image.as_posix()}'", f"duration {duration:.9f}")]
    entries.append(f"file '{image_paths[-1].as_posix()}'")
    frame_list.write_text("\n".join(entries) + "\n", encoding="utf-8")
    video = directory / "spin.mp4"
    subprocess.run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(frame_list),
        "-vf", f"fps={fps},scale=trunc(iw/2)*2:trunc(ih/2)*2", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(video),
    ], check=True)
    return video


def main() -> int:
    args = parse_args()
    pcad = args.pcad.resolve()
    cloud = args.cloud.resolve()
    output = args.output.resolve()
    if not pcad.is_file() or not cloud.is_file():
        raise SystemExit("Both --pcad and --cloud must point to existing files")
    if args.fps < 1 or args.width < 32 or args.height < 32:
        raise SystemExit("Use positive --fps and --width/--height values of at least 32")

    run_cli(pcad, ["cloud", "open", "--in", str(cloud)], args.session)
    capture_spin(pcad, output / "point-cloud", "cloud", args.elevation, args.session, args.width, args.height, args.fps, args.ffmpeg)

    detect_report = output / "surfaces" / "detect-command.json"
    detection = run_cli(pcad, ["detect", "--in", str(cloud)], args.session, detect_report)
    if detection.get("status") != "complete" or detection.get("exit_code") != 0:
        print(f"Surface detection failed. See {detect_report}", file=sys.stderr)
        return 1
    capture_spin(pcad, output / "surfaces", "surfaces", args.elevation, args.session, args.width, args.height, args.fps, args.ffmpeg)

    reconstruct_report = output / "reconstructed" / "reconstruct-command.json"
    reconstruction = run_cli(pcad, ["reconstruct", "cad", "--in", str(cloud)], args.session, reconstruct_report)
    if reconstruction.get("status") != "complete" or reconstruction.get("exit_code") != 0:
        print(f"CAD reconstruction did not produce an entity. See {reconstruct_report}", file=sys.stderr)
        return 2
    step_path = output / "reconstructed" / "reconstructed.step"
    exported = run_cli(pcad, ["export", "step", "--out", str(step_path)], args.session)
    if exported.get("status") != "complete" or not step_path.is_file():
        print("CAD reconstruction completed but STEP export failed", file=sys.stderr)
        return 3
    capture_spin(pcad, output / "reconstructed", "cad", args.elevation, args.session,
                 args.width, args.height, args.fps, args.ffmpeg)
    print(f"Saved reconstructed STEP to {step_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
