#!/usr/bin/env python3
"""Capture one fixed-elevation STEP orbit in ParamCAD Studio and encode it."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PCAD = ROOT / "apps" / "ParamCAD Studio" / "build" / "msvc-debug" / "Release" / "pcad.exe"


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path, help="STEP/STP source model")
    parser.add_argument("--output", required=True, type=Path, help="Output directory, which must not exist")
    parser.add_argument("--pcad", type=Path, default=DEFAULT_PCAD, help="Path to pcad.exe")
    parser.add_argument("--session", help="Studio session ID when more than one Studio is running")
    parser.add_argument("--elevation", type=float, default=45)
    parser.add_argument("--up-axis", choices=("z", "y"), default="z",
                         help="Vertical model axis; y keeps the XZ plane as the base")
    parser.add_argument("--label", default="", help="Optional English title over the video")
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--frames", type=int, default=96, help="Number of camera poses per revolution")
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
    if args.frames < 1 or args.fps < 1:
        raise SystemExit("--frames and --fps must be positive")
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
    background = Image.new("RGB", (args.width, args.height), (35, 39, 46))
    frames_directory = output / "frames"
    frames_directory.mkdir()
    frames = []
    for index in range(args.frames):
        frame = frames_directory / f"frame_{index:03}.png"
        phase = math.radians(index * 360 / args.frames)
        if args.up_axis == "y":
            elevation = math.radians(args.elevation)
            x = math.cos(elevation) * math.cos(phase)
            y = math.sin(elevation)
            z = math.cos(elevation) * math.sin(phase)
            azimuth = math.degrees(math.atan2(y, x))
            camera_elevation = math.degrees(math.asin(z))
        else:
            azimuth = index * 360 / args.frames
            camera_elevation = args.elevation
        subprocess.run([
            *prefix, "capture", "frame", "--output", str(frame), "--target", "model",
            "--azimuth", str(azimuth), "--elevation", str(camera_elevation),
            "--width", str(args.width), "--height", str(args.height),
        ], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
        if args.up_axis == "y" or args.label:
            with Image.open(frame) as captured:
                image = captured.convert("RGB")
            if args.up_axis == "y":
                app_azimuth = math.radians(azimuth)
                app_elevation = math.radians(camera_elevation)
                forward = (
                    -math.cos(app_elevation) * math.cos(app_azimuth),
                    -math.cos(app_elevation) * math.sin(app_azimuth),
                    -math.sin(app_elevation),
                )
                up = (
                    -math.sin(app_elevation) * math.cos(app_azimuth),
                    -math.sin(app_elevation) * math.sin(app_azimuth),
                    math.cos(app_elevation),
                )
                right = (
                    forward[1] * up[2] - forward[2] * up[1],
                    forward[2] * up[0] - forward[0] * up[2],
                    forward[0] * up[1] - forward[1] * up[0],
                )
                projected_x = right[1]
                projected_y = up[1]
                roll = 90 - math.degrees(math.atan2(projected_y, projected_x))
                # Rotate the rendered object only. The viewport background is a
                # fixed studio backdrop and must stay in the same screen frame.
                red, _, blue = image.split()
                mask = ImageChops.subtract(blue, red).point(
                    lambda value: 255 if value >= 18 else 0
                ).filter(ImageFilter.GaussianBlur(0.55))
                foreground = Image.new("RGBA", image.size, (0, 0, 0, 0))
                foreground.paste(image, mask=mask)
                rotated = foreground.rotate(roll, resample=Image.Resampling.BICUBIC,
                                            expand=False, fillcolor=(0, 0, 0, 0))
                image = Image.alpha_composite(background.convert("RGBA"), rotated).convert("RGB")
            if args.label:
                draw = ImageDraw.Draw(image, "RGBA")
                font_path = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / "bahnschrift.ttf"
                title_font = ImageFont.truetype(str(font_path), max(24, args.height // 25))
                bounds = draw.textbbox((0, 0), args.label, font=title_font)
                x, y = 44, 36
                draw.rounded_rectangle(
                    (x - 20, y - 14, x + bounds[2] + 20, y + bounds[3] + 14),
                    radius=7, fill=(12, 20, 30, 205),
                )
                draw.text((x, y), args.label, font=title_font, fill=(240, 246, 252, 255))
            image.save(frame, "PNG")
        frames.append(frame)
    video = output / "spin.mp4"
    subprocess.run(
        [ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-framerate", str(args.fps),
         "-i", str(frames_directory / "frame_%03d.png"), "-frames:v", str(args.frames),
         "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(video)],
        check=True,
    )
    print(f"Saved {video} from {len(frames)} frames at elevation {args.elevation:g} degrees")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
