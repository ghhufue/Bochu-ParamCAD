#!/usr/bin/env python3
"""Compose per-level, four-stage ParamCAD showcase videos and HTML pages."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory


ROOT = Path(__file__).resolve().parents[2]
DATA_ROOT = ROOT / "data" / "performance-validation"
STAGES = (
    ("step", "STEP model"),
    ("point-cloud", "Point cloud"),
    ("surfaces", "Surface recognition"),
    ("reconstructed", "Reconstructed solid"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DATA_ROOT)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "performance-showcase")
    parser.add_argument("--level", action="append", help="Level to build. Repeat to set playback order.")
    parser.add_argument("--combined", type=Path, help="Write the selected levels as one MP4")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--duration", type=float, default=8)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    return parser.parse_args()


def part_directories(level: Path) -> list[Path]:
    parts = []
    for entry in level.iterdir():
        if not entry.is_dir():
            continue
        source_models = [path for path in entry.iterdir() if path.is_file() and path.suffix.casefold() in {".step", ".stp"}]
        if source_models:
            parts.append(entry)
            continue
        parts.extend(
            child for child in entry.iterdir()
            if child.is_dir() and any(path.is_file() and path.suffix.casefold() in {".step", ".stp"} for path in child.iterdir())
        )
    return sorted(parts, key=lambda part: part.relative_to(level).as_posix().casefold())


def part_key(level: Path, part: Path) -> str:
    return part.relative_to(level).as_posix()


def make_page(level: Path, parts: list[Path], video_paths: dict[tuple[str, str], Path], output: Path) -> None:
    level_name = level.name
    count = len(parts)
    rows = []
    for part in parts:
        item_key = part_key(level, part)
        cells = []
        for stage_key, title in STAGES:
            clip = video_paths.get((item_key, stage_key))
            cell_content = (
                f'<video src="{html.escape(clip.relative_to(output / level_name).as_posix())}" autoplay muted loop playsinline></video>'
                if clip else '<div class="missing">Video not supplied</div>'
            )
            cells.append(f'<section class="cell"><header>{html.escape(title)}</header>{cell_content}</section>')
        rows.append(f'<article class="part"><h2>{html.escape(item_key)}</h2><div class="grid">{"".join(cells)}</div></article>')
    document = f'''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ParamCAD {html.escape(level_name)} showcase</title>
<style>
*{{box-sizing:border-box}}body{{margin:0;background:#101619;color:#e8efec;font:14px "Segoe UI",sans-serif}}
main{{--rows:{count};padding:18px;max-width:2200px;margin:auto}}h1{{font-size:22px;margin:0 0 14px}}
.part{{border-top:1px solid #35413f;padding:10px 0 14px}}h2{{font-size:14px;font-weight:600;margin:0 0 8px;color:#bad1c9}}
.grid{{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px}}
.cell{{min-width:0;background:#1a2325;border:1px solid #334143;border-radius:4px;overflow:hidden}}
.cell header{{padding:5px 8px;color:#adc0bb;font-size:12px}}video,.missing{{display:block;width:100%;height:max(150px,calc((100vh - 100px)/var(--rows)));object-fit:contain;background:#080b0c}}
.missing{{display:grid;place-items:center;color:#72817e}}@media(max-width:800px){{main{{padding:10px}}.grid{{grid-template-columns:repeat(2,minmax(0,1fr))}}video,.missing{{height:180px}}}}
</style><main><h1>{html.escape(level_name)}</h1>{''.join(rows)}</main></html>'''
    (output / level_name / "index.html").write_text(document, encoding="utf-8")


def compose_level(level: Path, parts: list[Path], output: Path, ffmpeg: str, fps: int, duration: float) -> Path:
    level_name = level.name
    level_output = output / level_name
    level_output.mkdir(parents=True, exist_ok=True)
    frame_height = 1080
    row_height = (frame_height // len(parts)) // 2 * 2
    cell_width = 480
    inputs: list[str] = []
    filters: list[str] = []
    video_paths: dict[tuple[str, str], Path] = {}
    input_index = 0
    for row_index, part in enumerate(parts):
        row_labels = []
        for stage_key, _ in STAGES:
            key = part_key(level, part)
            candidate = output / level_name / key / stage_key / "spin.mp4"
            if candidate.is_file():
                inputs.extend(["-stream_loop", "-1", "-i", str(candidate)])
                video_paths[(key, stage_key)] = candidate
                filters.append(
                    f"[{input_index}:v]fps={fps},scale={cell_width}:{row_height}:"
                    "force_original_aspect_ratio=decrease:force_divisible_by=2,"
                    f"pad={cell_width}:{row_height}:(ow-iw)/2:(oh-ih)/2:color=0x080b0c,setsar=1[v{input_index}]"
                )
            else:
                inputs.extend(["-f", "lavfi", "-i", f"color=c=0x080b0c:s={cell_width}x{row_height}:r={fps}"])
                filters.append(f"[{input_index}:v]trim=duration={duration},setpts=PTS-STARTPTS,setsar=1[v{input_index}]")
            row_labels.append(f"[v{input_index}]")
            input_index += 1
        filters.append("".join(row_labels) + f"hstack=inputs=4[row{row_index}]")
    grid = "[row0]"
    if len(parts) > 1:
        rows = "".join(f"[row{index}]" for index in range(len(parts)))
        filters.append(f"{rows}vstack=inputs={len(parts)}[grid]")
        grid = "[grid]"
    filters.append(f"{grid}pad=1920:{frame_height}:0:(oh-ih)/2:color=0x080b0c[outv]")
    output_video = level_output / "showcase.mp4"
    command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y", *inputs, "-filter_complex", ";".join(filters), "-map", "[outv]", "-t", str(duration),
               "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output_video)]
    subprocess.run(command, check=True)
    make_page(level, parts, video_paths, output)
    (level_output / "showcase.json").write_text(json.dumps({
        "level": level_name,
        "parts": [part_key(level, part) for part in parts],
        "columns": [key for key, _ in STAGES],
        "video": output_video.name,
        "available_videos": {f"{part}/{stage}": path.relative_to(output).as_posix() for (part, stage), path in video_paths.items()},
    }, indent=2), encoding="utf-8")
    print(f"Wrote {output_video} and {level_output / 'index.html'}")
    return output_video


def combine_levels(ffmpeg: str, videos: list[Path], destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory() as temporary:
        playlist = Path(temporary) / "levels.ffconcat"
        paths = [video.as_posix().replace("'", "'\\''") for video in videos]
        playlist.write_text("".join(f"file '{path}'\n" for path in paths), encoding="utf-8")
        subprocess.run([
            ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0",
            "-i", str(playlist), "-c", "copy", "-movflags", "+faststart", str(destination),
        ], check=True)
    print(f"Wrote {destination}")


def main() -> int:
    args = parse_args()
    data_root = args.data.resolve()
    output = args.output.resolve()
    ffmpeg = shutil.which(args.ffmpeg)
    if not data_root.is_dir():
        raise SystemExit(f"Dataset directory not found: {data_root}")
    if ffmpeg is None:
        raise SystemExit(f"ffmpeg not found: {args.ffmpeg}")
    if args.fps < 1 or args.duration <= 0:
        raise SystemExit("--fps and --duration must be positive")
    levels = sorted(path for path in data_root.iterdir() if path.is_dir() and path.name.startswith("level-"))
    if args.level:
        available = {path.name.casefold(): path for path in levels}
        levels = []
        for name in args.level:
            if name.casefold() not in available:
                raise SystemExit(f"Level not found: {name}")
            levels.append(available[name.casefold()])
    videos = []
    combined = args.combined.resolve() if args.combined else None
    if combined in [output / level.name / "showcase.mp4" for level in levels]:
        raise SystemExit("--combined must be different from the per-level MP4s")
    for level in levels:
        parts = part_directories(level)
        if parts:
            videos.append(compose_level(level, parts, output, ffmpeg, args.fps, args.duration))
    if combined:
        if not videos:
            raise SystemExit("No levels to combine")
        combine_levels(ffmpeg, videos, combined)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
