"""Private rendering worker, invoked and scheduled by the existing Qt client.

Scene geometry is consumed here for image synthesis only. The reconstruction
pipeline remains independent and reads only the committed images/calibration.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np

from . import StructuredLightError
from .capture import read_json, sha256


def emit(kind: str, **fields) -> None:
    print(json.dumps(dict(type=kind, **fields), ensure_ascii=False, allow_nan=False), flush=True)


def validate_device(device: dict) -> None:
    k = np.asarray(device["K"], dtype=float)
    width, height = device["width"], device["height"]
    if k.shape != (3, 3) or not np.isfinite(k).all() or not np.allclose(
            k, [[k[0, 0], 0, width / 2], [0, k[0, 0], height / 2], [0, 0, 1]], atol=1e-7):
        raise StructuredLightError("Mitsuba adapter currently requires centered, zero-skew, square-pixel pinhole devices")


def read_rgb(path: Path) -> np.ndarray:
    value = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if value is None:
        raise StructuredLightError(f"Cannot read material/pattern image: {path}")
    return value[..., ::-1].astype(np.float32) / 255


def linear(value: np.ndarray) -> np.ndarray:
    return np.where(value <= .04045, value / 12.92, ((value + .055) / 1.055) ** 2.4)


class Renderer:
    def __init__(self, root: Path, *, samples: int, depth: int, seed: int):
        import mitsuba as mi
        if mi.__version__ != "3.9.1":
            raise StructuredLightError(f"Expected bundled Mitsuba 3.9.1, found {mi.__version__}")
        mi.set_variant("scalar_rgb")  # Works without LLVM, CUDA SDK or JIT setup.
        self.mi, self.root = mi, root
        self.samples, self.depth, self.seed = samples, depth, seed
        self.capture = read_json(root / "capture.json")
        if self.capture.get("status") != "capturing":
            raise StructuredLightError("Worker only accepts a live client-owned capture")
        for key in ("scene", "calibration"):
            if sha256(root / self.capture[key]) != self.capture[key + "_sha256"]:
                raise StructuredLightError(f"Changed acquisition {key}")
        self.scene_description = read_json(root / self.capture["scene"])
        self.calibration = read_json(root / self.capture["calibration"])
        for device in self.calibration["devices"].values():
            validate_device(device)
        geometry = self.scene_description["geometry"]
        material = self.scene_description["material"]
        properties = mi.Properties()
        properties["bsdf"] = mi.load_dict(dict(type="principled",
            base_color=self.material_map(material, "base_color", np.asarray(material["base_color"], np.float32), True),
            metallic=self.material_map(material, "metallic", material["metallic"], False),
            roughness=self.material_map(material, "roughness", material["roughness"], False)))
        self.mesh = mi.Mesh("frozen_capture_mesh", len(geometry["positions"]), len(geometry["triangles"]),
                            properties, has_vertex_normals=True, has_vertex_texcoords=True)
        parameters = mi.traverse(self.mesh)
        for destination, source, dtype in (("vertex_positions", "positions", np.float32),
                ("faces", "triangles", np.uint32), ("vertex_normals", "normals", np.float32),
                ("vertex_texcoords", "uv", np.float32)):
            values = np.asarray(geometry[source], dtype=dtype)
            # Material UV is bottom-left, bitmap data uses top-left.
            if source == "uv":
                values = values.copy()
                values[:, 1] = 1 - values[:, 1]
            parameters[destination] = values.ravel()
        parameters.update()
        self.scenes = {}

    def material_map(self, material: dict, key: str, factor, color: bool):
        path = material.get(key + "_map")
        if not path:
            return dict(type="rgb", value=np.asarray(factor).tolist()) if color else float(factor)
        full = self.root / path
        if sha256(full) != material[key + "_map_sha256"]:
            raise StructuredLightError(f"Changed material map: {key}")
        data = read_rgb(full)
        data = linear(data) * factor if color else data[..., :1] * factor
        return dict(type="bitmap", bitmap=self.mi.Bitmap(np.asarray(data, np.float32)), raw=True,
                    filter_type="bilinear", wrap_mode="clamp")

    def transform(self, definition: dict, rig: np.ndarray):
        # Mitsuba +X left, +Y up, +Z forward -> common +X right, +Y down, +Z forward.
        return self.mi.ScalarTransform4f(rig @ np.asarray(definition["device_to_rig"]) @ np.diag([-1, -1, 1, 1]))

    def scene(self, camera: dict, projector: dict, filter_type: str):
        if filter_type in self.scenes:
            return self.scenes[filter_type]
        mi = self.mi
        source = dict(type="scene", integrator=dict(type="path", max_depth=self.depth), surface=self.mesh,
            sensor=dict(type="perspective", fov=math.degrees(2 * math.atan(camera["width"] / (2 * camera["K"][0][0]))),
                fov_axis="x", near_clip=.001, far_clip=1e9,
                film=dict(type="hdrfilm", width=camera["width"], height=camera["height"], pixel_format="rgb",
                          component_format="float32", rfilter=dict(type="box")),
                sampler=dict(type="independent", sample_count=self.samples)),
            projection=dict(type="projector", fov=math.degrees(2 * math.atan(projector["width"] / (2 * projector["K"][0][0]))),
                fov_axis="x", irradiance=dict(type="bitmap", raw=True, filter_type=filter_type, wrap_mode="clamp",
                    bitmap=mi.Bitmap(np.ones((projector["height"], projector["width"], 3), np.float32))),
                scale=self.scene_description["lighting"]["intensity"] * self.scene_description["lighting"]["reference_distance_mm"] ** 2))
        ambient = self.scene_description["lighting"]["ambient"]
        if ambient:
            source["environment"] = dict(type="constant", radiance=dict(type="rgb", value=[ambient] * 3))
        scene = mi.load_dict(source)
        self.scenes[filter_type] = scene
        return scene

    def render(self):
        mi = self.mi
        count = 0
        group = 0
        for station_index, station in enumerate(self.capture["stations"]):
            devices = self.calibration["devices"]
            camera, projector = devices[station["camera_id"]], devices[station["projector_id"]]
            rig = np.asarray(station["rig_to_world"])
            for exposure_index, exposure in enumerate(self.capture["exposures"]):
                # Every pattern frame of one station/exposure reuses its sampled
                # paths: only the projector image changes, which scales the
                # radiance of those paths. Correlated Monte Carlo noise then
                # cancels in Gray/phase differences instead of being decoded as
                # a signal. This is a deliberately noise-free synthetic sensor.
                group_seed = (self.seed + group) % (2**32)
                group += 1
                for pattern_index, pattern in enumerate(self.capture["pattern_set"]["patterns"]):
                    if (self.root / ".render-cancel").exists():
                        emit("cancelled")
                        return 3
                    pattern_path = self.root / pattern["image"]
                    if sha256(pattern_path) != pattern["sha256"]:
                        raise StructuredLightError(f"Changed pattern: {pattern['id']}")
                    scene = self.scene(camera, projector, "bilinear" if pattern["kind"] == "phase" else "nearest")
                    parameters = mi.traverse(scene)
                    parameters["sensor.to_world"] = self.transform(camera, rig)
                    parameters["projection.to_world"] = self.transform(projector, rig)
                    parameters["projection.irradiance.data"] = mi.TensorXf(read_rgb(pattern_path))
                    parameters.update()
                    # Fixed seed per station/exposure; no adaptive exposure, denoising or tone mapping.
                    radiance = np.asarray(mi.render(scene, spp=self.samples, seed=group_seed), np.float32)
                    radiance = radiance * float(exposure)
                    if not np.isfinite(radiance).all():
                        raise StructuredLightError("Renderer produced non-finite radiance")
                    relative = f"stations/{station['id']}/e{exposure_index}/{pattern['id']}.png"
                    path = self.root / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    # Retain linear float measurements independently of fixed uint8 decoder input.
                    mi.Bitmap(radiance).write(str(path.with_suffix(".exr")))
                    encoded = np.rint(np.clip(radiance, 0, 1) * 255).astype(np.uint8)[..., ::-1]
                    ok, png = cv2.imencode(".png", encoded)
                    if not ok:
                        raise StructuredLightError(f"Cannot encode frame {relative}")
                    png.tofile(path)
                    count += 1
                    emit("frame", station=station_index, exposure=exposure_index, pattern=pattern_index,
                         image=relative, completed=count)
        emit("complete", frames=count)
        return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=32)
    parser.add_argument("--max-depth", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    try:
        if not (1 <= args.samples <= 4096 and 2 <= args.max_depth <= 16 and 0 <= args.seed < 2**32):
            raise StructuredLightError("Invalid sampling or path settings")
        return Renderer(args.capture.resolve(), samples=args.samples, depth=args.max_depth, seed=args.seed).render()
    except (ImportError, OSError, KeyError, TypeError, ValueError, RuntimeError, StructuredLightError) as exc:
        emit("error", message=str(exc))
        print(str(exc), file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
