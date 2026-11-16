"""Extract only a selected surface family for native meshing replay.

Raw coordinates are written to an excerpt file, never to stdout. The console
contains metadata and aggregate statistics only. No original project is read.
Run from the repository root with `python -m tools.diagnostics.trace_bundle_surface`.
"""
import argparse
import json
from pathlib import Path
import sys

from tools.analysis_bundle.inspect_bundle import Bundle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path)
    selector = parser.add_mutually_exclusive_group(required=True)
    selector.add_argument("--id", type=int)
    selector.add_argument("--name")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    bundle = Bundle(args.bundle)
    target = bundle.surface(surface_id=args.id, name=args.name)
    # Finite extents can differ between patches of one analytic equation.
    equations = {"plane": ("normal", "offset"), "cone": ("center", "axis", "angle"),
                 "cylinder": ("center", "axis", "radius"), "sphere": ("center", "radius"),
                 "ring": ("center", "axis", "major_radius", "minor_radius")}
    keys = equations.get(target["geometry"], ())
    if not keys:
        raise ValueError("Surface has no supported analytic geometry")
    family, points, summary = [], [], []
    for row in bundle.surfaces():
        if row["geometry"] != target["geometry"]:
            continue
        face = bundle.surface(surface_id=row["id"])
        if any(face["parameters"].get(key) != target["parameters"].get(key) for key in keys):
            continue
        family.append({"id": face["id"], "name": face["name"], "geometry": face["geometry"],
                       "parameters": face["parameters"], "residual": face["fit_residual"],
                       "confidence": face["confidence"], "fitStale": face["fit_stale"]})
        points.extend({**point, "owner": face["id"]} for point in bundle.points(face))
        summary.append({"id": face["id"], "name": face["name"], "point_count": face["point_count"]})
    args.output.mkdir(parents=True, exist_ok=True)
    excerpt = args.output / "surface-excerpt.json"
    excerpt.write_text(json.dumps({"bundle_id": str(bundle.bundle_id),
                                  "target": {"id": target["id"], "name": target["name"]},
                                  "family": family, "points": points}, ensure_ascii=False, allow_nan=False),
                       encoding="utf-8")
    result = {"bundle_id": str(bundle.bundle_id), "target": bundle.analyze(target),
              "same_equation_collections": summary, "extracted_observations": len(points),
              "private_native_replay_excerpt": str(excerpt)}
    (args.output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
