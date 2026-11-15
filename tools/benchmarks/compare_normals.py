"""Isolated before/after timings and peak memory; compare exact output hashes.

Run from a checkout: python tools/benchmarks/compare_normals.py --output report.json
The baseline is read from git, without replacing working tree files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def peak_bytes():
    if sys.platform == "win32":
        import ctypes
        from ctypes import wintypes
        class Counters(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD)] + [
                (name, ctypes.c_size_t) for name in ("PeakWorkingSetSize", "WorkingSetSize",
                "QuotaPeakPagedPoolUsage", "QuotaPagedPoolUsage", "QuotaPeakNonPagedPoolUsage",
                "QuotaNonPagedPoolUsage", "PagefileUsage", "PeakPagefileUsage")]
        counters = Counters(); counters.cb = ctypes.sizeof(counters)
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.GetCurrentProcess.restype = wintypes.HANDLE
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD]
        if not psapi.GetProcessMemoryInfo(kernel.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
            raise ctypes.WinError(ctypes.get_last_error())
        return counters.PeakWorkingSetSize
    import resource
    value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return value if sys.platform == "darwin" else value * 1024


def measure(args):
    import numpy as np
    if args.mode == "baseline":
        source = subprocess.check_output(["git", "show", f"{args.baseline_ref}:core/reconstruction/structured_light/normals.py"], cwd=ROOT)
        namespace = {"__name__": "baseline_normals"}
        exec(compile(source, "baseline_normals.py", "exec"), namespace)
        calculate = namespace["oriented_normals"]
    else:
        sys.path.insert(0, str(ROOT / "core" / "reconstruction"))
        from structured_light.normals import oriented_normals
        calculate = lambda p, c: oriented_normals(p, c, workers=args.workers)
    yy, xx = np.mgrid[:args.height, :args.width]
    points = np.stack((xx * .05, yy * .05, 10 + xx * .001 + yy * .002), axis=-1)
    points[args.height // 3:args.height // 3 + 3, args.width // 3:args.width // 2] = np.nan
    points[:, args.width // 2:, 2] += 1
    camera = np.array([args.width * .025, args.height * .025, 100.])
    calculate(points, camera)
    times = []
    for _ in range(3):
        started = time.perf_counter(); normals = calculate(points, camera)
        times.append(time.perf_counter() - started)
    return dict(size=[args.width, args.height], mode=args.mode, workers=args.workers,
                median_seconds=statistics.median(times), peak_process_bytes=peak_bytes(),
                sha256=hashlib.sha256(normals.tobytes()).hexdigest(), numpy=np.__version__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baseline-ref", default="HEAD")
    parser.add_argument("--mode", choices=("baseline", "optimized"))
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    args = parser.parse_args()
    if args.mode:
        print(json.dumps(measure(args)), flush=True)
        return
    records = []
    for width, height in ((512, 384), (1280, 960)):
        hashes = set()
        for mode, workers in (("baseline", 0), ("optimized", 1), ("optimized", 0)):
            command = [sys.executable, str(Path(__file__).resolve()), "--mode", mode,
                       "--workers", str(workers), "--width", str(width), "--height", str(height),
                       "--baseline-ref", args.baseline_ref]
            record = json.loads(subprocess.check_output(command, cwd=ROOT, text=True))
            print(json.dumps(record), flush=True); records.append(record); hashes.add(record["sha256"])
        if len(hashes) != 1:
            raise AssertionError("normal outputs differ from the baseline")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        baseline = subprocess.check_output(["git", "rev-parse", args.baseline_ref], cwd=ROOT, text=True).strip()
        args.output.write_text(json.dumps(dict(baseline_commit=baseline, python=sys.version, cases=records), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
