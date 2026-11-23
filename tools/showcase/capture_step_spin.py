#!/usr/bin/env python3
"""Capture one fixed-elevation STEP orbit in ParamCAD Studio and encode it."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PCAD = ROOT / "apps" / "ParamCAD Studio" / "build" / "msvc-debug" / "Release" / "pcad.exe"


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path, help="STEP/STP source model")
    parser.add_argument("--output", required=True, type=Path, help="Output directory, which must not exist")
    parser.add_argument("--pcad", type=Path, default=DEFAULT_PCAD, help="Path to pcad.exe")
    parser.add_argument("--session", help="Studio session ID when more than one Studio is running")
    parser.add_argument("--elevation", type=float, default=-30)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--ffmpeg", default="ffmpeg", help="ffmpeg executable")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    model = args.model.resolve()
    output = args.output.resolve()
    pcad = args.pcad.resolve()
    ffmpeg = args.ffmpeg
    if not model.is_file():
        raise SystemExit(f"Model not found: {model}")
    if not pcad.is_file():
        raise SystemExit(f"pcad.exe not found: {pcad}")
    if output.exists():
        raise SystemExit(f"Output already exists: {output}")
    if args.fps < 1:
        raise SystemExit("--fps must be positive")
    output.mkdir(parents=True)
    capture_directory = output / "capture"
    prefix = [str(pcad), "--online"]
    if args.session:
        prefix.extend(("--session", args.session))
    command = [*prefix, "capture", "views", "--in", str(model), "--output", str(capture_directory)]
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if completed.returncode:
        shutil.rmtree(output)
        sys.stderr.write(completed.stdout + completed.stderr)
        raise SystemExit(completed.returncode)
    frames_directory = output / "frames"
    frames_directory.mkdir()
    frames = []
    for index in range(12):
        frame = frames_directory / f"frame_{index:03}.png"
        subprocess.run([
            *prefix, "capture", "frame", "--output", str(frame), "--target", "model",
            "--azimuth", str(index * 30), "--elevation", str(args.elevation),
            "--width", str(args.width), "--height", str(args.height),
        ], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
        frames.append(frame)
    frame_list = output / "frames.txt"
    frame_duration = 1 / args.fps
    lines = [line for path in frames for line in (f"file '{path.as_posix()}'", f"duration {frame_duration:.9f}")]
    lines.append(f"file '{frames[-1].as_posix()}'")
    frame_list.write_text("\n".join(lines) + "\n", encoding="utf-8")
    video = output / "spin.mp4"
    subprocess.run(
        [ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(frame_list),
         "-vf", f"fps={args.fps},scale=trunc(iw/2)*2:trunc(ih/2)*2", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(video)],
        check=True,
    )
    print(f"Saved {video} from {len(frames)} frames at elevation {args.elevation:g} degrees")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
