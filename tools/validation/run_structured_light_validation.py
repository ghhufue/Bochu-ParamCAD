"""Developer acceptance runner: all acquisition/reconstruction uses the actual client.

This is a test harness, not a second capture application. CAD geometry is used by
the client renderer and its post-reconstruction evaluator, never by the decoder.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8-sig"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--client", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Fresh test output directory")
    parser.add_argument("--models", type=Path, default=Path(__file__).resolve().parents[2] / "data/performance-validation")
    parser.add_argument("--camera-size", default="256x192")
    parser.add_argument("--projector-size", default="256x256")
    parser.add_argument("--stations", type=int, default=8)
    parser.add_argument("--renderer", choices=("vtk", "mitsuba"), default="vtk",
                        help="Capture renderer; the same client and reconstruction are used.")
    parser.add_argument("--samples", type=int, default=32, help="Mitsuba samples per pixel")
    parser.add_argument("--max-depth", type=int, default=4, help="Mitsuba maximum path depth; 2 = direct only")
    parser.add_argument("--cases", default="", help="Comma-separated case names; empty runs all")
    parser.add_argument("--timeout", type=float, default=600.0, help="Per-case client timeout in seconds")
    args = parser.parse_args()
    client, root = args.client.resolve(), args.output.resolve()
    if not client.is_file() or root.exists():
        parser.error("client must exist and output must be a fresh directory")
    if not 1 <= args.samples <= 4096 or not 2 <= args.max_depth <= 16 or args.timeout <= 0:
        parser.error("invalid sampling, path depth or timeout")
    root.mkdir(parents=True)
    # Freeze the comparison settings and acceptance gate before acquisition.
    cases = [
        ("cylinder_gray", "cylinder", .9, .4, False, False),
        ("cylinder_phase", "cylinder", .9, .4, True, False),
        ("cylinder_phase_hdr", "cylinder", .9, .4, True, True),
        ("cylinder_bright_gray", "cylinder", 1., .15, False, False),
        ("cylinder_bright_phase_hdr", "cylinder", 1., .15, True, True),
        ("hole_phase_hdr", "box-through-hole", .9, .4, True, True),
        ("bracket_phase_hdr", "l-bracket-gusset", .9, .4, True, True),
    ]
    if args.cases:
        wanted = {name.strip() for name in args.cases.split(",") if name.strip()}
        known = {case[0] for case in cases}
        if not wanted <= known:
            parser.error(f"unknown cases: {sorted(wanted - known)}")
        cases = [case for case in cases if case[0] in wanted]
    illumination = ("VTK direct only; no interreflection" if args.renderer == "vtk"
                    else f"Mitsuba path tracer; max depth {args.max_depth}"
                         + (" (direct only)" if args.max_depth <= 2 else " (indirect reflection enabled)"))
    gate = (dict(case="cylinder_phase_hdr", max_p95_mm=.5, min_coverage=.8)
            if args.renderer == "vtk" else None)
    settings = dict(camera_size=args.camera_size, projector_size=args.projector_size, stations=args.stations,
                    tolerance_mm=.5, sampling_seed=617, material="uniform; no material maps",
                    renderer=args.renderer, samples_per_pixel=args.samples if args.renderer == "mitsuba" else 1,
                    max_path_depth=args.max_depth if args.renderer == "mitsuba" else 1,
                    timeout_seconds=args.timeout, illumination=illumination, gate=gate)
    (root / "settings.json").write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
    reports = []
    for name, model, metallic, roughness, phase, hdr in cases:
        output = root / name
        level = "level-02" if model == "l-bracket-gusset" else "level-01"
        model_path = args.models / level / model / "model.step"
        if not model_path.is_file():
            model_path = args.models / model / "model.step"
        command = [str(client), "--structured-light-capture", str(model_path.resolve()),
                   "--output", str(output), "--scan-stations", str(args.stations), "--scan-size", args.camera_size,
                   "--projector-size", args.projector_size, "--scan-metallic", str(metallic),
                   "--scan-roughness", str(roughness), "--scan-renderer", args.renderer,
                   "--scan-samples", str(args.samples), "--scan-max-depth", str(args.max_depth),
                   "--scan-reconstruct"]
        if phase:
            command.append("--scan-phase-shift")
        if hdr:
            command.append("--scan-multi-exposure")
        started = time.monotonic()
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=args.timeout,
                                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        (root / f"{name}.log").write_bytes(result.stdout)
        record = dict(case=name, model=model, metallic=metallic, roughness=roughness, phase=phase, hdr=hdr,
                      client_exit_code=result.returncode, wall_seconds=time.monotonic() - started)
        evaluation = output / "evaluation.json"
        if result.returncode == 0 and evaluation.is_file():
            record["evaluation"] = read_json(evaluation)
            runs = list((output / "reconstructions").glob("*/quality.json"))
            if len(runs) != 1:
                raise RuntimeError(f"Unexpected reconstruction run count: {name}")
            record["quality"] = read_json(runs[0])
            record["capture"] = {k: read_json(output / "capture/capture.json").get(k)
                                 for k in ("elapsed_ms", "captured_frames", "rendering")}
        else:
            record["failure_log"] = f"{name}.log"
        reports.append(record)
        (root / "report.json").write_text(json.dumps(dict(settings=settings, cases=reports), indent=2) + "\n", encoding="utf-8")
        print(f"{name}: exit={result.returncode}, seconds={record['wall_seconds']:.2f}", flush=True)
    complete = all("evaluation" in case for case in reports)
    if settings["gate"]:
        baseline = next(case for case in reports if case["case"] == settings["gate"]["case"])
        passed = complete and baseline["evaluation"]["point_error_p95_mm"] <= .5 \
            and baseline["evaluation"]["full_surface_coverage"] >= .8
    else:
        passed = complete
    report = dict(settings=settings, cases=reports, gate_passed=passed,
                  interpretation="Gate covers this simulated cylinder fixture only; bright steel and concavities are reported, not promised"
                  if settings["gate"] else
                  "Cross-backend measurement run: completion requires every case to evaluate; no VTK gate is applied")
    (root / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
