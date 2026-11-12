"""Unit and integration checks for the optional Mitsuba capture worker.

The module-level helpers run everywhere. Renderer checks require the bundled
Mitsuba runtime and are skipped when it is not importable.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from structured_light import StructuredLightError
from structured_light import mitsuba_renderer
from structured_light.patterns import image, sequence

HAS_MITSUBA = importlib.util.find_spec("mitsuba") is not None


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plane_capture(root: Path) -> None:
    root.mkdir(parents=True, exist_ok=True)
    width, height, projector = 32, 24, 32
    camera = dict(model="PINHOLE", width=width, height=height,
                  K=[[24, 0, 16], [0, 24, 12], [0, 0, 1]],
                  device_to_rig=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    device = dict(model="PINHOLE", width=projector, height=projector,
                  K=[[24, 0, 16], [0, 24, 16], [0, 0, 1]],
                  device_to_rig=[[1, 0, 0, 5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    scene = dict(schema="paramcad.capture.scene", version=1, unit="mm",
                 geometry=dict(positions=[[-40, -30, 100], [40, -30, 100], [40, 30, 100], [-40, 30, 100]],
                               normals=[[0, 0, -1]] * 4,
                               uv=[[0, 0], [1, 0], [1, 1], [0, 1]],
                               triangles=[[0, 2, 1], [0, 3, 2]]),
                 material=dict(base_color=[.8, .8, .8], metallic=0.0, roughness=0.7,
                               workflow="metallic-roughness", map_channels="separate images; scalar data in red",
                               mapping="frozen mesh UV", base_color_space="linear constant; sRGB image",
                               data_map_space="linear", base_color_map="", metallic_map="", roughness_map=""),
                 lighting=dict(intensity=2.0, reference_distance_mm=100.0, ambient=0.015))
    calibration = dict(schema="paramcad.structured-light.calibration", version=1, unit="mm",
                       devices=dict(camera=camera, projector=device))
    (root / "scene.json").write_text(json.dumps(scene), encoding="utf-8")
    (root / "calibration.json").write_text(json.dumps(calibration), encoding="utf-8")
    records = []
    for pattern in sequence(projector, projector, 0):
        relative = f"patterns/{pattern.id}.png"
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), image(pattern, projector, projector))
        record = pattern.as_dict()
        record.update(image=relative, sha256=sha256(path))
        records.append(record)
    capture = dict(schema="paramcad.structured-light.capture", version=1, status="capturing",
                   job_id="mitsuba-unit", unit="mm", pixel_convention="top-left; centers=(i+0.5,j+0.5)",
                   intensity_encoding="linear_uint8", calibration="calibration.json",
                   calibration_sha256=sha256(root / "calibration.json"),
                   scene="scene.json", scene_sha256=sha256(root / "scene.json"),
                   rendering=dict(backend="mitsuba", linear_float_images=True),
                   exposures=[1.0],
                   stations=[dict(id="s0", rig_to_world=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                                  camera_id="camera", projector_id="projector", frames=[])],
                   pattern_set=dict(width=projector, height=projector, phase_steps=0, period=16.0,
                                    patterns=records))
    (root / "capture.json").write_text(json.dumps(capture), encoding="utf-8")


class DeviceValidationTest(unittest.TestCase):
    def test_centered_square_pixel_pinhole_is_accepted(self):
        mitsuba_renderer.validate_device(dict(width=128, height=96, K=[[120, 0, 64], [0, 120, 48], [0, 0, 1]]))

    def test_off_center_skewed_or_anisotropic_devices_are_rejected(self):
        for K in (
            [[120, 0, 60], [0, 120, 48], [0, 0, 1]],
            [[120, 0, 64], [0, 120, 50], [0, 0, 1]],
            [[120, 2, 64], [0, 120, 48], [0, 0, 1]],
            [[120, 0, 64], [0, 140, 48], [0, 0, 1]],
        ):
            with self.assertRaises(StructuredLightError):
                mitsuba_renderer.validate_device(dict(width=128, height=96, K=K))

    def test_srgb_to_linear_curve(self):
        values = mitsuba_renderer.linear(np.asarray([0.0, 0.04045, 0.5, 1.0], np.float32))
        np.testing.assert_allclose(values, [0.0, 0.04045 / 12.92, ((0.5 + 0.055) / 1.055) ** 2.4, 1.0], atol=1e-6)

    def test_rgb_images_are_read_back_in_display_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.png"
            cv2.imwrite(str(path), np.asarray([[[0, 0, 255]]], np.uint8))
            np.testing.assert_allclose(mitsuba_renderer.read_rgb(path), [[[1.0, 0.0, 0.0]]], atol=1e-6)


@unittest.skipUnless(HAS_MITSUBA, "Mitsuba runtime is not installed")
class RendererIntegrationTest(unittest.TestCase):
    def test_plane_capture_projects_signal_and_keeps_linear_measurements(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plane_capture(root)
            mitsuba_renderer.Renderer(root, samples=2, depth=2, seed=0).render()
            base = root / "stations" / "s0" / "e0"
            white = cv2.imread(str(base / "white.png"), cv2.IMREAD_GRAYSCALE)
            black = cv2.imread(str(base / "black.png"), cv2.IMREAD_GRAYSCALE)
            self.assertIsNotNone(white)
            self.assertIsNotNone(black)
            signal = int(white[12, 16]) - int(black[12, 16])
            self.assertGreater(signal, 20)
            self.assertLess(signal, 255)
            import mitsuba as mi
            exr = np.array(mi.Bitmap(str(base / "white.exr")))
            np.testing.assert_array_equal(np.rint(np.clip(exr, 0, 1) * 255)[..., 0], white)
            first = white.copy()
            mitsuba_renderer.Renderer(root, samples=2, depth=2, seed=0).render()
            np.testing.assert_array_equal(cv2.imread(str(base / "white.png"), cv2.IMREAD_GRAYSCALE), first)

    def test_changed_scene_asset_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plane_capture(root)
            scene = json.loads((root / "scene.json").read_text(encoding="utf-8"))
            scene["lighting"]["ambient"] = 0.5
            (root / "scene.json").write_text(json.dumps(scene), encoding="utf-8")
            with self.assertRaises(StructuredLightError):
                mitsuba_renderer.Renderer(root, samples=1, depth=2, seed=0)


if __name__ == "__main__":
    unittest.main()
