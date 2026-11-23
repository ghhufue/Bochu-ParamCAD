#!/usr/bin/env python3
"""Capture fixed-elevation STEP spins for every source model in a level."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
CAPTURE_SCRIPT = Path(__file__).with_name("capture_step_spin.py")
DATA_ROOT = ROOT / "data" / "performance-validation"
OUTPUT_ROOT = ROOT / "outputs" / "performance-showcase"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--level", required=True, help="Level directory name, for example level-01")
    parser.add_argument("--data", type=Path, default=DATA_ROOT)
    parser.add_argument("--output", type=Path, default=OUTPUT_ROOT)
    parser.add_argument("--pcad", type=Path)
    parser.add_argument("--session", help="Studio session ID when more than one Studio is running")
    parser.add_argument("--elevation", type=float, default=-30)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    return parser.parse_args()


def source_models(level: Path) -> list[Path]:
    models = []
    for entry in level.iterdir():
        if not entry.is_dir():
            continue
        direct = [path for path in entry.iterdir() if path.is_file() and path.suffix.casefold() in {".step", ".stp"}]
        if direct:
            models.extend(direct)
        else:
            for child in entry.iterdir():
                if child.is_dir():
                    models.extend(path for path in child.iterdir() if path.is_file() and path.suffix.casefold() in {".step", ".stp"})
    return sorted(models, key=lambda path: path.relative_to(level).as_posix().casefold())


def main() -> int:
    args = parse_args()
    data_root = args.data.resolve()
    level = data_root / args.level
    if not level.is_dir():
        raise SystemExit(f"Level directory not found: {level}")
    models = source_models(level)
    if not models:
        raise SystemExit(f"No STEP/STP source models found in {level}")
    for model in models:
        part = model.parent.relative_to(level)
        destination = args.output.resolve() / args.level / part / "step"
        command = [
            sys.executable, str(CAPTURE_SCRIPT), "--model", str(model), "--output", str(destination),
            "--elevation", str(args.elevation), "--fps", str(args.fps), "--ffmpeg", args.ffmpeg,
            "--width", str(args.width), "--height", str(args.height),
        ]
        if args.pcad:
            command.extend(("--pcad", str(args.pcad.resolve())))
        if args.session:
            command.extend(("--session", args.session))
        subprocess.run(command, cwd=ROOT, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
