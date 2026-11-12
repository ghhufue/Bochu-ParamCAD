"""Headless compute entry point used by the existing desktop client."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from . import StructuredLightError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Reconstruct structured-light acquisitions from images and calibration.")
    parser.add_argument("--capture", required=True, help="Capture directory or capture.json")
    parser.add_argument("--output", required=True, help="New empty reconstruction run directory")
    parser.add_argument("--cancel-file", type=Path, help="Cooperative cancellation marker")
    parser.add_argument("--min-contrast", type=float, default=8)
    parser.add_argument("--saturation", type=float, default=254)
    parser.add_argument("--min-angle-deg", type=float, default=.5)
    parser.add_argument("--max-reprojection-px", type=float, default=1.5)
    parser.add_argument("--voxel-size-mm", type=float, default=.2)
    args = parser.parse_args(argv)
    try:
        from .pipeline import Options, reconstruct
        options = Options(args.min_contrast, args.saturation, args.min_angle_deg,
                          args.max_reprojection_px, args.voxel_size_mm)
        reconstruct(args.capture, args.output, options=options, cancel_file=args.cancel_file,
                    event=lambda value: print(json.dumps(value, ensure_ascii=False), flush=True))
    except ImportError as exc:
        print(json.dumps(dict(type="error", message=f"Bundled structured-light dependency missing: {exc}")), flush=True)
        return 2
    except (StructuredLightError, OSError) as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 2
    return 0
