"""Black-box contracts for the built native CLI, including C++/Python parity."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from tools.analysis_bundle.inspect_bundle import Bundle
from tools.pcad_cli.native import locate_cli


class NativeCliTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.exe = locate_cli()
        except RuntimeError as error:
            raise unittest.SkipTest(str(error))

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="pcad-native-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.env = {**os.environ, "PCAD_SESSION_DIR": str(self.root / "sessions")}

    def run_cli(self, *args, expected=0):
        result = subprocess.run([str(self.exe), *map(str, args)], capture_output=True,
                                encoding="utf-8", errors="replace", env=self.env, timeout=60)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return json.loads(result.stdout)

    def test_capabilities_and_mode_rejection(self):
        rows = {r["command"]: r for r in self.run_cli("capabilities")["commands"]}
        self.assertTrue(rows["reconstruct cad"]["online"])
        self.assertTrue(rows["reconstruct cad"]["offline"])
        self.assertFalse(rows["inspect"]["online"])
        self.run_cli("status", expected=2)
        self.run_cli("detect", "--online", "--synthetic", "box", expected=2)
        self.run_cli("detect", "--offline", "--online", expected=2)

    def test_preprocess_carries_unit_and_full_cad_round_trip(self):
        cloud = self.root / "cloud with spaces.ply"
        self.run_cli("preprocess", "--synthetic", "box", "--out", cloud)
        unit = json.loads(cloud.with_suffix(".json").read_text())
        self.assertEqual(unit["unit"], "mm")
        self.assertEqual(self.run_cli("cloud", "info", "--in", cloud)["unit"], "mm")
        out = self.root / "cad"
        result = self.run_cli("reconstruct", "cad", "--offline", "--in", cloud, "--output", out)
        self.assertTrue(result["cad_constructed"])
        self.assertTrue(result["project_round_trip_verified"])
        self.assertTrue(result["step_round_trip_verified"])
        for name in ("model.pcad", "model.step", "model.features.json", "report.json"):
            self.assertTrue((out / name).is_file())
        self.run_cli("reconstruct", "cad", "--offline", "--in", cloud, "--output", out, expected=2)

    def test_all_module_dispatch_and_report_contract(self):
        for name in ("detect", "analyze", "recognize", "assemble", "project"):
            result = self.run_cli(name, "--offline", "--synthetic", "box")
            self.assertEqual(result["schema"], "paramcad.cli.result")
            self.assertEqual(result["mode"], "offline")
        self.run_cli("detect", "--offline", "--synthetic", "box", "--step", self.root/"ignored.step", expected=2)
        exported = self.run_cli("project", "--offline", "--synthetic", "box", "--step", self.root/"projection.step")
        self.assertTrue(exported["accepted"])
        self.assertTrue((self.root/"projection.step").is_file())

    def test_native_inspector_matches_exporter_and_python(self):
        exporter = self.exe.parent / "analysis_bundle_export_test.exe"
        if not exporter.is_file():
            self.skipTest("Build analysis_bundle_export_test for native cross-language validation")
        bundle = self.root / "fixture.pcad-analysis"
        generated = subprocess.run([str(exporter), str(bundle)], capture_output=True, timeout=30)
        self.assertEqual(generated.returncode, 0, generated.stderr)
        self.assertFalse(list(bundle.glob("*.py")))
        python_bundle = Bundle(bundle)
        grouped = self.run_cli("inspect", "--bundle", bundle, "show", "--id", 182, "--group")
        self.assertEqual(grouped["support_validation_report"], python_bundle.support_validation(python_bundle.surface(surface_id=182)))
        actual = self.run_cli("inspect", "--bundle", bundle, "analyze", "--id", 182)
        expected = python_bundle.analyze(python_bundle.surface(surface_id=182))
        self.assertEqual(actual["observations_read"], expected["observations_read"])
        self.assertEqual(actual["mesh"], expected["mesh"])
        self.assertEqual(actual["measured_plane_residual"], expected["measured_plane_residual"])
        self.run_cli("inspect", "--bundle", bundle, "verify", "--full")
        point_file = bundle / "points.bin"
        raw = point_file.read_bytes()
        damaged = bytearray(raw)
        damaged[24] ^= 1
        point_file.write_bytes(damaged)
        self.run_cli("inspect", "--bundle", bundle, "analyze", "--id", 182, expected=2)
        point_file.write_bytes(raw)
        for name in ("points.bin", "point-map.bin", "meshes.bin"):
            (bundle / name).unlink()
        self.run_cli("inspect", "--bundle", bundle, "show", "--id", 182, "--group")
        self.run_cli("inspect", "--bundle", bundle, "points", "--id", 182, "--limit", 101, expected=2)


if __name__ == "__main__":
    unittest.main()
