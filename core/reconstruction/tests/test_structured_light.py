"""Independent image fixtures and geometric checks for the production backend."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np

from structured_light import StructuredLightError
from structured_light.capture import PIXEL_CONVENTION, load_capture, sha256
from structured_light.geometry import Device, fuse, triangulate
from structured_light.patterns import decode, image, sequence
from structured_light.pipeline import Options, reconstruct, write_json


def save_image(path: Path, data: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(".png", data)
    if not ok:
        raise AssertionError("cannot encode fixture")
    encoded.tofile(path)


def planar_capture(root: Path, *, phase: bool = False, stations: int = 1,
                   hdr: bool = False) -> dict:
    """Analytic pinhole plane images; no geometry is passed to reconstruction.

    The image generator evaluates radiance at known ray-plane intersections.
    Plane geometry is a test oracle only, not an input/depth file for the backend.
    """
    root.mkdir(parents=True)
    width, height, pw, ph = 64, 48, 128, 96
    steps = 4 if phase else 0
    patterns = sequence(pw, ph, steps)
    descriptors = []
    for pattern in patterns:
        path = root / "patterns" / f"{pattern.id}.png"
        save_image(path, image(pattern, pw, ph))
        descriptors.append(dict(**pattern.as_dict(), image=path.relative_to(root).as_posix(), sha256=sha256(path)))
    transform = np.eye(4)
    transform[0, 3] = 15
    camera = dict(model="PINHOLE", width=width, height=height,
                  K=[[80, 0, 32], [0, 80, 24], [0, 0, 1]], device_to_rig=np.eye(4).tolist())
    projector = dict(model="PINHOLE", width=pw, height=ph,
                     K=[[120, 0, 64], [0, 120, 48], [0, 0, 1]], device_to_rig=transform.tolist())
    write_json(root / "calibration.json", dict(schema="paramcad.structured-light.calibration", version=1,
               unit="mm", devices={"camera": camera, "projector": projector}))
    write_json(root / "scene.json", {"description": "analytic fixture; no geometry or depth supplied"})
    yy, xx = np.mgrid[:height, :width]
    # Projection on z=100: x=(u-cx)*z/f. Projector translated 15 mm to camera right.
    up = 120 * ((xx + .5 - 32) / 80 - .15) + 64
    vp = 120 * (yy + .5 - 24) / 80 + 48
    coordinate = {"x": up, "y": vp}
    records = []
    for station in range(stations):
        rig = np.eye(4)
        angle = station * .35
        rig[:3, :3] = [[math.cos(angle), -math.sin(angle), 0],
                       [math.sin(angle), math.cos(angle), 0], [0, 0, 1]]
        rig[0, 3] = station * 8
        frames = []
        for exposure_id, gain in (("0", 3.0), ("1", .6)) if hdr else (("0", .6),):
            for pattern in patterns:
                if pattern.kind == "white":
                    signal = np.ones((height, width))
                elif pattern.kind == "black":
                    signal = np.zeros((height, width))
                elif pattern.kind == "gray":
                    indices = coordinate[pattern.axis].astype(np.uint32)
                    signal = (((indices ^ (indices >> 1)) >> pattern.bit) & 1).astype(float)
                    if pattern.inverted:
                        signal = 1 - signal
                else:
                    signal = .5 * (1 + np.cos(2 * math.pi * coordinate[pattern.axis] / 16 + pattern.phase))
                radiance = np.rint(np.clip(255 * (.04 + gain * signal), 0, 255)).astype(np.uint8)
                radiance[(up < 0) | (up >= pw) | (vp < 0) | (vp >= ph)] = 0
                path = root / f"stations/s{station}/e{exposure_id}/{pattern.id}.png"
                save_image(path, radiance)
                frames.append(dict(pattern_id=pattern.id, exposure_id=exposure_id, status="complete",
                                   image=path.relative_to(root).as_posix(), sha256=sha256(path)))
        records.append(dict(id=f"s{station}", camera_id="camera", projector_id="projector",
                            rig_to_world=rig.tolist(), frames=frames))
    manifest = dict(schema="paramcad.structured-light.capture", version=1, status="complete", unit="mm",
        pixel_convention=PIXEL_CONVENTION, intensity_encoding="linear_uint8",
        calibration="calibration.json", calibration_sha256=sha256(root / "calibration.json"),
        scene="scene.json", scene_sha256=sha256(root / "scene.json"),
        pattern_set=dict(width=pw, height=ph, phase_steps=steps, period=16, patterns=descriptors),
        rendering=dict(backend="analytic_test_fixture", illumination="direct"), stations=records)
    write_json(root / "capture.json", manifest)
    return manifest


class PatternTests(unittest.TestCase):
    def test_gray_sequence_matches_independent_opencv_implementation(self):
        generator = cv2.structured_light.GrayCodePattern_create(128, 96)
        success, expected = generator.generate()
        self.assertTrue(success)
        actual = [image(p, 128, 96) for p in sequence(128, 96)[2:]]
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            np.testing.assert_array_equal(a, b)

    def test_every_projector_pixel_decodes_and_non_power_of_two_size(self):
        width, height = 71, 53
        images = {p.id: (image(p, width, height).astype(float) * .6 + 10).astype(np.uint8)
                  for p in sequence(width, height)}
        coordinates, valid, _ = decode(images, width, height)
        self.assertTrue(valid.all())
        yy, xx = np.mgrid[:height, :width]
        np.testing.assert_array_equal(coordinates[..., 0], xx + .5)
        np.testing.assert_array_equal(coordinates[..., 1], yy + .5)

    def test_dark_saturated_and_missing_patterns_are_not_decoded(self):
        width, height = 16, 12
        images = {p.id: np.rint(image(p, width, height) * .6 + 10).astype(np.uint8)
                  for p in sequence(width, height)}
        for sample in images.values():
            sample[0, 0] = 2
            sample[0, 1] = 255
        _, valid, _ = decode(images, width, height)
        self.assertFalse(valid[0, 0])
        self.assertFalse(valid[0, 1])
        del images["x_0_1"]
        with self.assertRaises(StructuredLightError):
            decode(images, width, height)


class GeometryTests(unittest.TestCase):
    def test_general_world_pose_and_optical_axis_depth(self):
        angle = .47
        rotation = np.array([[math.cos(angle), 0, math.sin(angle)], [0, 1, 0],
                             [-math.sin(angle), 0, math.cos(angle)]])
        a, b = np.eye(4), np.eye(4)
        a[:3, :3] = b[:3, :3] = rotation
        a[:3, 3] = [30, -10, 7]
        b[:3, 3] = a[:3, 3] + rotation @ [15, 0, 0]
        k = np.array([[300, 0, 160], [0, 310, 120], [0, 0, 1]])
        camera, projector = Device(320, 240, k, a), Device(320, 240, k, b)
        expected = np.array([[-5, 7, 100], [10, -4, 120], [0, 0, 90]]) @ rotation.T + a[:3, 3]
        uv_a, _ = camera.project(expected)
        uv_b, _ = projector.project(expected)
        actual, valid, quality = triangulate(camera, projector, uv_a, uv_b)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(actual, expected, atol=1e-9)
        np.testing.assert_allclose(quality["depth"], [100, 120, 90], atol=1e-9)
        with self.assertRaisesRegex(StructuredLightError, "baseline"):
            triangulate(camera, camera, uv_a, uv_a)

    def test_negative_depth_and_wrong_correspondences_rejected(self):
        a, b = np.eye(4), np.eye(4)
        b[0, 3] = 10
        k = np.array([[100, 0, 50], [0, 100, 50], [0, 0, 1]])
        camera, projector = Device(100, 100, k, a), Device(100, 100, k, b)
        _, valid, _ = triangulate(camera, projector, np.array([[50, 50], [50, 50]]),
                                  np.array([[60, 50], [40, 70]]))
        self.assertFalse(valid.any())

    def test_voxel_fusion_uses_weighted_positions(self):
        points, counts = fuse(np.array([[.1, .1, .1], [.3, .1, .1], [2, 2, 2]]),
                               np.array([1, 3, 1]), 1)
        np.testing.assert_allclose(points, [[.25, .1, .1], [2, 2, 2]])
        np.testing.assert_array_equal(counts, [2, 1])

    def test_shared_voxel_downsample_module(self):
        from structured_light.downsample import voxel_downsample
        points, counts = voxel_downsample(np.array([[.1, .1, .1], [.3, .1, .1]]),
                                          np.array([1, 3]), 1)
        np.testing.assert_allclose(points, [[.25, .1, .1]])
        np.testing.assert_array_equal(counts, [2])
        with self.assertRaises(StructuredLightError):
            voxel_downsample(points, np.ones(len(points)), 0)
        with self.assertRaises(StructuredLightError):
            voxel_downsample(points, np.ones(3), 1)


class PipelineTests(unittest.TestCase):
    def test_multistation_phase_hdr_pipeline_preserves_mm_world_plane(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            planar_capture(root / "采集 数据", phase=True, stations=2, hdr=True)
            events = []
            state = reconstruct(root / "采集 数据", root / "run", options=Options(voxel_size_mm=.5), event=events.append)
            self.assertEqual(state["status"], "complete")
            self.assertGreater(state["point_count"], 1000)
            self.assertEqual(state["valid_stations"], 2)
            self.assertEqual(events[-1]["type"], "result")
            for station in ("s0", "s1"):
                depth = np.load(root / f"run/stations/{station}/depth_camera_z_mm.npy")
                chosen = np.load(root / f"run/stations/{station}/exposure_index.npy")
                self.assertGreater(np.isfinite(depth).mean(), .98)
                self.assertLess(float(np.nanmax(np.abs(depth - 100))), .1)
                self.assertTrue((chosen[np.isfinite(depth)] == 1).all())
            self.assertEqual(json.loads((root / "run/result.json").read_text())["unit"], "mm")
            cost = json.loads((root / "run/quality.json").read_text())["cost"]
            self.assertGreater(cost["peak_process_memory_bytes"], 0)
            self.assertEqual(cost["frame_count"], 2 * 2 * len(sequence(128, 96, 4)))
            self.assertGreater(cost["stage_seconds"]["decode"], 0)
            with self.assertRaisesRegex(StructuredLightError, "empty"):
                reconstruct(root / "采集 数据", root / "run")

    def test_gray_only_has_expected_projector_quantization_error(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            planar_capture(root / "capture")
            reconstruct(root / "capture", root / "run")
            depth = np.load(root / "run/stations/s0/depth_camera_z_mm.npy")
            self.assertGreater(np.isfinite(depth).mean(), .98)
            self.assertLess(float(np.nanmax(np.abs(depth - 100))), 3)

    def test_output_cannot_modify_raw_acquisition_and_malformed_objects_fail(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            original = planar_capture(root / "capture")
            before = sorted(path.relative_to(root / "capture") for path in (root / "capture").rglob("*"))
            with self.assertRaisesRegex(StructuredLightError, "outside"):
                reconstruct(root / "capture", root / "capture/run")
            self.assertEqual(before, sorted(path.relative_to(root / "capture")
                             for path in (root / "capture").rglob("*")))
            for name, replace in (("stations", [None]), ("pattern_set", [])):
                malformed = copy.deepcopy(original)
                malformed[name] = replace
                write_json(root / "capture/capture.json", malformed)
                with self.assertRaises(StructuredLightError):
                    reconstruct(root / "capture", root / name)
                self.assertEqual(json.loads((root / name / "result.json").read_text())["status"], "failed")

    def test_corrupt_missing_frame_incomplete_capture_and_zero_baseline_fail(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            original = planar_capture(root / "capture")
            for case in ("missing", "incomplete", "baseline", "hash", "outside", "no_scene_hash", "no_pattern_hash"):
                manifest = copy.deepcopy(original)
                if case == "missing":
                    manifest["stations"][0]["frames"].pop()
                elif case == "incomplete":
                    manifest["status"] = "cancelled"
                elif case == "hash":
                    manifest["stations"][0]["frames"][0]["sha256"] = "0" * 64
                elif case == "outside":
                    manifest["stations"][0]["frames"][0]["image"] = "../outside.png"
                elif case == "no_scene_hash":
                    del manifest["scene_sha256"]
                elif case == "no_pattern_hash":
                    del manifest["pattern_set"]["patterns"][0]["sha256"]
                else:
                    calibration = json.loads((root / "capture/calibration.json").read_text())
                    calibration["devices"]["projector"]["device_to_rig"] = np.eye(4).tolist()
                    write_json(root / "capture/zero.json", calibration)
                    manifest["calibration"] = "zero.json"
                    manifest["calibration_sha256"] = sha256(root / "capture/zero.json")
                write_json(root / "capture/capture.json", manifest)
                with self.subTest(case=case), self.assertRaises(StructuredLightError):
                    reconstruct(root / "capture", root / case)
                report = json.loads((root / case / "result.json").read_text())
                self.assertEqual(report["status"], "failed")
                self.assertFalse((root / case / "cloud.ply").exists())

    def test_cancel_writes_terminal_state_and_cli_emits_json_events(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            planar_capture(root / "capture")
            marker = root / "cancel"
            marker.touch()
            with self.assertRaisesRegex(StructuredLightError, "cancelled"):
                reconstruct(root / "capture", root / "cancelled", cancel_file=marker)
            self.assertEqual(json.loads((root / "cancelled/result.json").read_text())["status"], "cancelled")
            script = Path(__file__).parents[1] / "reconstruct_structured_light.py"
            result = subprocess.run([sys.executable, str(script), "--capture", str(root / "capture"),
                                     "--output", str(root / "cli")], capture_output=True, text=True, check=True)
            messages = [json.loads(line) for line in result.stdout.splitlines()]
            self.assertEqual(messages[-1]["type"], "result")
            self.assertTrue(Path(messages[-1]["point_cloud"]).is_file())


if __name__ == "__main__":
    unittest.main()
