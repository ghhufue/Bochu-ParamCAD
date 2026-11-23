"""Render a split-screen showcase from ParamCAD Studio's structured-light capture."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps


WIDTH, HEIGHT, FPS = 1920, 1080, 30
BACKGROUND = "#0c141e"
PANEL = "#111e2c"
OUTLINE = "#283c50"
TEXT = "#eef5fc"
MUTED = "#9cb1c6"
ACCENT = "#65ead6"
IMAGE_BOX = (864, 220, 1824, 940)
CREATE_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def capture_model(client: Path, model: Path, output: Path) -> Path:
    scan = output / "scan"
    if scan.exists():
        raise FileExistsError(f"{scan} already exists. Use --capture to reuse a capture.")
    log = output / "capture.log"
    command = [
        str(client), "--structured-light-capture", str(model), "--output", str(scan),
        "--scan-stations", "16", "--scan-size", "960x720", "--scan-renderer", "vtk",
        "--scan-metallic", "0.15", "--scan-roughness", "0.65", "--scan-light-intensity", "3",
    ]
    with log.open("wb") as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                creationflags=CREATE_NO_WINDOW)
    if result.returncode:
        raise RuntimeError(f"Studio capture exited with code {result.returncode}. See {log}")
    return scan / "capture" / "capture.json"


def font(size: int, filename: str = "msyh.ttc") -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / filename), size)


def describe_pattern(pattern: dict) -> str:
    match pattern["kind"]:
        case "black":
            return "Black reference"
        case "white":
            return "White reference"
        case "gray":
            polarity = "Inverted" if pattern["inverted"] else "Normal"
            return f"Gray code · {pattern['axis'].upper()} axis · Bit {pattern['bit']} · {polarity}"
        case "phase":
            return f"Phase-shift fringes · {pattern['axis'].upper()} axis · {math.degrees(pattern['phase']):g}°"
        case _:
            return pattern["id"]


def make_layout(title: str) -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    heading = f"{title}  /  Structured-Light Capture" if title else "Structured-Light Capture"
    draw.text((64, 47), heading, fill=TEXT, font=font(49, "msyhbd.ttc"))
    draw.text((1856, 77), "ParamCAD Studio", fill=MUTED, font=font(25), anchor="rm")
    draw.rounded_rectangle((64, 145, 808, 1016), radius=18, fill=PANEL, outline=OUTLINE, width=2)
    draw.rounded_rectangle((832, 145, 1856, 1016), radius=18, fill=PANEL, outline=OUTLINE, width=2)
    draw.text((96, 166), "Camera Pose", fill=TEXT, font=font(30, "msyhbd.ttc"))
    draw.text((864, 166), "Rendered Camera Frame", fill=TEXT, font=font(30, "msyhbd.ttc"))
    draw.rectangle(IMAGE_BOX, fill="black", outline=OUTLINE)
    return image


def pose_diagrams(layout: Image.Image, geometry: dict, poses: list, intrinsics: dict):
    vertices = np.asarray(geometry["positions"])
    triangles = np.asarray(geometry["triangles"])
    center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
    cameras = np.stack(poses)[:, :3, 3]
    radius = np.linalg.norm(cameras - center, axis=1).max()

    azimuth, elevation = np.radians([37, 22])
    right = np.array([-np.sin(azimuth), np.cos(azimuth), 0])
    near = np.array([np.cos(azimuth) * np.cos(elevation), np.sin(azimuth) * np.cos(elevation),
                     np.sin(elevation)])
    up = np.cross(near, right)
    basis = np.stack([right, -up], axis=1)

    def project(points):
        return (points - center) @ basis * (272 / radius) + [436, 554]

    positions = project(cameras)
    diagram = layout.copy()
    draw = ImageDraw.Draw(diagram)
    draw.rounded_rectangle((96, 220, 776, 864), radius=12, fill="#0c1723", outline=OUTLINE)
    radians = np.linspace(0, np.pi * 2, 181)
    for z in (-0.75, -0.4, 0, 0.4, 0.75):
        ring_radius = np.sqrt(1 - z * z)
        ring = np.stack([ring_radius * np.cos(radians), ring_radius * np.sin(radians),
                         np.full_like(radians, z)], axis=1)
        draw.line(list(map(tuple, project(center + radius * ring))), fill="#25394c", width=2)
    for angle in np.radians([0, 45, 90, 135]):
        meridian = np.stack([np.cos(angle) * np.cos(radians), np.sin(angle) * np.cos(radians),
                             np.sin(radians)], axis=1)
        draw.line(list(map(tuple, project(center + radius * meridian))), fill="#203247", width=2)

    screens = project(vertices)[triangles]
    depth = (vertices @ near)[triangles].mean(axis=1)
    normals = np.asarray(geometry["normals"])[triangles].mean(axis=1)
    light = np.clip(normals @ np.array([0.32, 0.47, 0.81]), 0, 1)
    for index in depth.argsort():
        brightness = light[index]
        fill = (int(48 + 131 * brightness), int(76 + 126 * brightness), int(103 + 119 * brightness))
        draw.polygon(list(map(tuple, screens[index])), fill=fill)

    for x, y in positions:
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill="#7a9dbb", outline="#c0d8ef")
    draw.text((116, 239), "Scan Stations", fill=MUTED, font=font(22))

    focal_x, focal_y = intrinsics["K"][0][0], intrinsics["K"][1][1]
    frustum_length = 0.24 * radius
    half_width = frustum_length * intrinsics["width"] / (2 * focal_x)
    half_height = frustum_length * intrinsics["height"] / (2 * focal_y)
    corners = np.array([[-half_width, -half_height, frustum_length],
                        [half_width, -half_height, frustum_length],
                        [half_width, half_height, frustum_length],
                        [-half_width, half_height, frustum_length]])
    label_font = font(24)
    position_font = font(29, "consola.ttf")
    for index, (pose, position) in enumerate(zip(poses, positions, strict=True)):
        frame = diagram.copy()
        draw = ImageDraw.Draw(frame)
        eye = tuple(position)
        draw.line([eye, tuple(project(center))], fill="#377c7e", width=2)
        frustum = list(map(tuple, project(corners @ pose[:3, :3].T + pose[:3, 3])))
        draw.line(frustum + frustum[:1], fill=ACCENT, width=2)
        for corner in frustum:
            draw.line([eye, corner], fill=ACCENT, width=2)
        x, y = eye
        draw.ellipse((x - 15, y - 15, x + 15, y + 15), outline=ACCENT, width=2)
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill=ACCENT)
        draw.text((776, 187), f"{index + 1:02d} / {len(poses):02d}", fill=ACCENT,
                  font=position_font, anchor="rm")
        draw.text((96, 887), "Camera Position (mm)", fill=MUTED, font=label_font)
        for axis, value, left in zip("XYZ", pose[:3, 3], (96, 327, 558), strict=True):
            draw.text((left, 935), f"{axis} {value:+8.1f}", fill=TEXT, font=position_font)
        yield frame


def build_video(manifest: Path, output: Path, title: str) -> None:
    root = manifest.parent
    capture = read_json(manifest)
    stations = capture["stations"]
    if capture["status"] != "complete" or not stations:
        raise ValueError(f"Expected a complete structured-light capture in {manifest}")
    geometry = read_json(root / capture["scene"])["geometry"]
    devices = read_json(root / capture["calibration"])["devices"]
    poses = [
        np.asarray(station["rig_to_world"]) @ np.asarray(devices[station["camera_id"]]["device_to_rig"])
        for station in stations
    ]
    patterns = {pattern["id"]: pattern for pattern in capture["pattern_set"]["patterns"]}
    # The capture stores linear intensity. Encode it for display without changing the source PNGs.
    display_lut = None
    if capture["intensity_encoding"] == "linear_uint8":
        linear = np.arange(256) / 255
        srgb = np.where(linear <= 0.0031308, 12.92 * linear, 1.055 * linear ** (1 / 2.4) - 0.055)
        display_lut = np.rint(srgb * 255).astype(int).tolist() * 3
    capture_width = IMAGE_BOX[2] - IMAGE_BOX[0]
    capture_height = IMAGE_BOX[3] - IMAGE_BOX[1]
    subtitle_font = font(26)
    counter_font = font(25, "consola.ttf")
    layout = make_layout(title)
    diagrams = pose_diagrams(layout, geometry, poses, devices[stations[0]["camera_id"]])
    temporary = output / "showcase.encoding.mp4"
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS),
        "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "17",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary),
    ]
    count = 0
    preview_station = len(stations) // 2
    with subprocess.Popen(command, stdin=subprocess.PIPE, creationflags=CREATE_NO_WINDOW) as encoder:
        try:
            for index, (station, base) in enumerate(zip(stations, diagrams, strict=True)):
                frames = station["frames"]
                intrinsics = devices[station["camera_id"]]
                header = ImageDraw.Draw(base)
                header.text((1824, 187), f"{intrinsics['width']} × {intrinsics['height']}  ·  {station['id'].upper()}",
                            fill=MUTED, font=counter_font, anchor="rm")
                for number, record in enumerate(frames, start=1):
                    if record["status"] != "complete":
                        raise ValueError(f"Incomplete frame in {station['id']}: {record['image']}")
                    pattern = patterns[record["pattern_id"]]
                    composed = base.copy()
                    with Image.open(root / record["image"]) as source:
                        rgb = source.convert("RGB")
                        if display_lut is not None:
                            rgb = rgb.point(display_lut)
                        photo = ImageOps.contain(rgb, (capture_width, capture_height),
                                                 Image.Resampling.LANCZOS)
                    x = IMAGE_BOX[0] + (capture_width - photo.width) // 2
                    y = IMAGE_BOX[1] + (capture_height - photo.height) // 2
                    composed.paste(photo, (x, y))
                    draw = ImageDraw.Draw(composed)
                    draw.text((864, 965), describe_pattern(pattern), fill=TEXT, font=subtitle_font)
                    draw.text((1824, 982), f"{number:02d} / {len(frames):02d}",
                              fill=ACCENT, font=counter_font, anchor="rm")
                    if (index, number) == (0, 2) or (index == preview_station and pattern["id"] == "x_phase_1"):
                        composed.save(output / "preview.jpg", quality=94)
                    pixels = composed.tobytes()
                    repeats = 6 if pattern["kind"] == "white" else 1
                    for _ in range(repeats):
                        encoder.stdin.write(pixels)
                    count += 1
                print(f"{station['id']}  {len(frames)} captured frames", flush=True)
            encoder.stdin.close()
            if encoder.wait() or count != capture["captured_frames"]:
                raise RuntimeError("Encoding failed or the capture manifest frame count did not match.")
        except BaseException:
            encoder.kill()
            encoder.wait()
            temporary.unlink(missing_ok=True)
            raise
    temporary.replace(output / "showcase.mp4")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--capture", type=Path, help="Existing Studio capture.json")
    source.add_argument("--model", type=Path, help="STEP file to capture in a separate hidden Studio process")
    parser.add_argument("--client", type=Path, help="Path to ParamCAD Studio.exe when using --model")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="", help="Optional name to display before the heading")
    args = parser.parse_args()
    if args.model and (not args.model.is_file() or not args.client or not args.client.is_file()):
        parser.error("--model requires an existing STEP file and --client executable")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = (
        capture_model(args.client.resolve(), args.model.resolve(), output)
        if args.model else args.capture.resolve()
    )
    build_video(manifest, output, args.title)
    (output / "index.html").write_text(
        """<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ParamCAD Studio · Structured-Light Capture</title>
<style>
  body { margin: 0; min-height: 100dvh; display: grid; place-items: center; background: #0c141e; }
  video { display: block; width: min(100vw, 177.78dvh); max-height: 100dvh; }
</style>
<video controls playsinline preload="metadata" poster="preview.jpg" aria-label="Structured-Light Capture">
  <source src="showcase.mp4" type="video/mp4">
</video>
</html>
""",
        encoding="utf-8",
    )
    print(output / "showcase.mp4")


if __name__ == "__main__":
    main()
