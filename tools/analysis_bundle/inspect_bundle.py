"""Inspect one exported surface without loading unrelated point data.

No third-party packages are required. `list` and `show` read metadata only;
`analyze` reads the selected surface's binary observations and display mesh;
`points` is the explicit, bounded raw-coordinate output command.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import uuid

HEADER = struct.Struct("<8sIIQ16sQ")
POINT = struct.Struct("<6f4BI")
VERTEX = struct.Struct("<3dQ")
TRIANGLE = struct.Struct("<3Q")
INDEX = struct.Struct("<Q")


class Bundle:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        self.manifest = self._json("manifest.json")
        if self.manifest.get("schema") != "paramcad.analysis" or self.manifest.get("version") != 1:
            raise ValueError("Unsupported analysis bundle schema/version")
        self.bundle_id = uuid.UUID(self.manifest["bundle_id"])

    def _path(self, name):
        path = (self.directory / name).resolve()
        if not path.is_relative_to(self.directory):
            raise ValueError("Bundle path escapes its directory")
        return path

    def _json(self, name):
        with self._path(name).open(encoding="utf-8") as stream:
            return json.load(stream)

    def surfaces(self):
        if not hasattr(self, "_surfaces"):
            with self._path("surfaces.jsonl").open(encoding="utf-8") as stream:
                self._surfaces = [json.loads(line) for line in stream if line.strip()]
            if len({row["id"] for row in self._surfaces}) != len(self._surfaces):
                raise ValueError("Duplicate surface ids")
        return iter(self._surfaces)

    def surface(self, surface_id=None, name=None):
        if (surface_id is None) == (name is None):
            raise ValueError("Specify exactly one surface id or name")
        matches = [row for row in self.surfaces()
                   if (row["id"] == surface_id if surface_id is not None else row["name"] == name)]
        if not matches:
            raise ValueError("Surface not found")
        if len(matches) != 1:
            raise ValueError("Ambiguous surface name; use --id: " + ", ".join(str(x["id"]) for x in matches[:20]))
        index = matches[0]
        data = self._path(index["metadata"]).read_bytes()
        if hashlib.sha256(data).hexdigest() != index["sha256"]:
            raise ValueError("Surface metadata checksum mismatch")
        result = json.loads(data)
        if result["bundle_id"] != str(self.bundle_id) or result["id"] != index["id"]:
            raise ValueError("Surface metadata belongs to another snapshot")
        return result

    def plane_group(self, surface):
        reference = surface.get("shared_plane")
        if not reference:
            return None
        raw = self._path(reference["metadata"]).read_bytes()
        if hashlib.sha256(raw).hexdigest() != reference["sha256"]:
            raise ValueError("Plane group checksum mismatch")
        result = json.loads(raw)
        if result["bundle_id"] != str(self.bundle_id) or result["id"] != reference["id"]:
            raise ValueError("Plane group belongs to another snapshot")
        return result

    def support_validation(self, surface=None):
        reference = self.manifest.get("support_validation")
        if not reference:
            return None
        raw = self._path(reference["metadata"]).read_bytes()
        if hashlib.sha256(raw).hexdigest() != reference["sha256"]:
            raise ValueError("Support validation checksum mismatch")
        result = json.loads(raw)
        if result.get("bundle_id") != str(self.bundle_id) or result.get("version") != 1:
            raise ValueError("Support validation belongs to a different snapshot/version")
        if surface is None:
            return result
        stage = result.get("after_ownership", result)
        selected = surface.get("support_validation", {})
        matches = [d for d in stage.get("decisions", [])
                   if "proposal_id" in selected and d.get("proposal_id") == selected["proposal_id"]]
        summary = {"version": result["version"], "selected_decisions": matches}
        if "scope" in result:
            summary["scope"] = result["scope"]
        for key in ("mode", "reference", "options"):
            if key in stage:
                summary[key] = stage[key]
        return summary

    def selected_validation(self, surface):
        report = self.support_validation()
        if report is None:
            return None
        selected = surface.get("support_validation", {})
        result = {"version": report["version"], "scope": report.get("scope")}
        if "proposal_id" in selected:
            result["proposal_id"] = selected["proposal_id"]
        if "patch_id" in selected:
            result["patch_id"] = selected["patch_id"]
        recovery_links = [link for link in report.get("before_ownership", {}).get("source_recovery_proposal_links", [])
                          if link.get("source_recovery_proposal_id") == selected.get("source_recovery_proposal_id")]
        before_proposal_ids = {link.get("proposal_id") for link in recovery_links}
        for stage_name in ("before_ownership", "after_ownership"):
            stage = report.get(stage_name)
            if stage is None:
                continue
            before_ownership = stage_name == "before_ownership"
            decisions = [decision for decision in stage.get("decisions", [])
                         if (decision.get("proposal_id") in before_proposal_ids if before_ownership else
                             decision.get("proposal_id") == selected.get("proposal_id") and
                             decision.get("patch_id") == selected.get("patch_id"))]
            if decisions:
                result[stage_name] = {"mode": stage.get("mode"), "reference": stage.get("reference"),
                                      "decisions": decisions}
                if before_ownership:
                    result[stage_name]["source_recovery_proposal_links"] = recovery_links
        trace = report.get("plane_merge_trace")
        if trace is not None and selected.get("source_recovery_proposal_id") is not None:
            proposal_id = selected["source_recovery_proposal_id"]
            attempts = [attempt for attempt in trace.get("attempts", [])
                        if proposal_id in [*attempt.get("first_result_proposal_ids", []),
                                           *attempt.get("second_result_proposal_ids", [])]]
            result["plane_merge_trace"] = {"version": trace.get("version"), "source": trace.get("source"),
                "options": trace.get("options"), "truncated": trace.get("truncated"),
                "event_limit": trace.get("event_limit"), "attempts": attempts}
        return result

    def relations(self):
        if not hasattr(self, "_relations"):
            with self._path("relations.jsonl").open(encoding="utf-8") as stream:
                self._relations = [json.loads(line) for line in stream if line.strip()]
        return self._relations

    @staticmethod
    def _plane_equation(surface):
        if surface.get("geometry") != "plane":
            raise ValueError("compare currently requires two plane surfaces")
        normal = list(map(float, surface["parameters"]["normal"]))
        offset = float(surface["parameters"]["offset"])
        length = math.sqrt(sum(value*value for value in normal))
        if not math.isfinite(length) or length <= 1e-12:
            raise ValueError("Invalid plane equation")
        equation = [value/length for value in (*normal, offset)]
        major = max(range(3), key=lambda axis: abs(equation[axis]))
        if equation[major] < 0:
            equation = [-value for value in equation]
        return equation

    def diagnose(self, surface):
        selected = surface.get("support_validation", {})
        validation = self.selected_validation(surface)
        merge_trace_available = bool(validation and "plane_merge_trace" in validation)
        source_candidate_linked = selected.get("source_recovery_proposal_id") is not None
        trace = (validation or {}).get("plane_merge_trace", {})
        trace_present = bool((self.support_validation() or {}).get("plane_merge_trace"))
        mesh = surface.get("render_mesh")
        mesh_summary = None if not mesh else {
            "vertices": mesh["vertices"]["count"], "triangles": mesh["triangles"]["count"],
            "foreign_source_vertices": mesh["foreign_source_vertices"],
            "covered_observations": mesh["covered_observations"]}
        incident_all = [relation for relation in self.relations()
                        if relation["first"] == surface["id"] or relation["second"] == surface["id"]]
        incident = incident_all[:100]
        result = self.analyze(surface)
        summary = surface.get("summary", {})
        return {
            "schema": "paramcad.face_diagnostics", "version": 1,
            "bundle_id": str(self.bundle_id), "source": self.manifest["source"],
            "target": {key: surface.get(key) for key in ("id", "name", "geometry", "source", "point_count",
                "parameters", "fit_residual", "confidence", "fit_stale")}
                | {key: summary.get(key) for key in ("bounds", "centroid")},
            "lineage": {"status": "source_recovery_candidate_link" if source_candidate_linked else
                ("final_validation_link" if "proposal_id" in selected else "surface_only"),
                **({"proposal_id": selected["proposal_id"]} if "proposal_id" in selected else {}),
                **({"patch_id": selected["patch_id"]} if "patch_id" in selected else {}),
                **({"source_recovery_proposal_id": selected["source_recovery_proposal_id"]}
                   if source_candidate_linked else {}),
                "trace_available": trace_present,
                "candidate_linked": source_candidate_linked,
                "matching_attempt_found": merge_trace_available and bool(trace.get("attempts")),
                "trace_truncated": trace.get("truncated")},
            "support_validation": validation,
            "relations": incident,
            "relation_count": len(incident_all),
            "relations_truncated": len(incident_all) > len(incident),
            "display_family": self.plane_group(surface),
            "display_mesh": mesh_summary,
            "measured_geometry": result,
        }

    def compare_planes(self, first, second):
        a, b = self._plane_equation(first), self._plane_equation(second)
        alignment = min(1., abs(sum(a[i]*b[i] for i in range(3))))
        angle = math.degrees(math.acos(alignment))
        offset_difference = abs(a[3]-b[3])
        first_ids, second_ids = set(self.point_indices(first)), set(self.point_indices(second))
        overlap = len(first_ids & second_ids)
        first_bounds = first.get("summary", {}).get("bounds")
        second_bounds = second.get("summary", {}).get("bounds")
        bounds_gap = None
        if first_bounds and second_bounds:
            gaps = [max(0., first_bounds[0][axis]-second_bounds[1][axis],
                        second_bounds[0][axis]-first_bounds[1][axis]) for axis in range(3)]
            bounds_gap = math.sqrt(sum(gap*gap for gap in gaps))

        common = [(a[i]+b[i])*.5 for i in range(4)]
        length = math.sqrt(sum(value*value for value in common[:3]))
        if length <= 1e-12:
            raise ValueError("Cannot form a diagnostic midpoint plane")
        common = [value/length for value in common]

        def residual_summary(surface):
            residuals = sorted(abs(sum(common[axis]*point["position"][axis] for axis in range(3)) + common[3])
                               for point in self.points(surface))
            if not residuals:
                return {"count": 0}
            def percentile(q):
                return residuals[min(len(residuals)-1, math.ceil(q*len(residuals))-1)]
            return {"count": len(residuals), "p50": percentile(.5), "p95": percentile(.95),
                    "p99": percentile(.99), "max": residuals[-1]}

        recorded = [relation for relation in self.relations()
                    if {relation["first"], relation["second"]} == {first["id"], second["id"]}]
        first_group, second_group = self.plane_group(first), self.plane_group(second)
        same_group = bool(first_group and second_group and first_group["id"] == second_group["id"])
        report = self.support_validation()
        trace = report.get("plane_merge_trace", {}) if report else {}
        first_proposal = first.get("support_validation", {}).get("source_recovery_proposal_id")
        second_proposal = second.get("support_validation", {}).get("source_recovery_proposal_id")
        def contains(values, value):
            return value in list(values)
        attempts = [attempt for attempt in trace.get("attempts", [])
                    if (contains(attempt.get("first_result_proposal_ids", []), first_proposal) and
                        contains(attempt.get("second_result_proposal_ids", []), second_proposal)) or
                        (contains(attempt.get("first_result_proposal_ids", []), second_proposal) and
                         contains(attempt.get("second_result_proposal_ids", []), first_proposal))]
        trace_available = bool(trace and first_proposal is not None and second_proposal is not None)
        candidate_links_available = first_proposal is not None and second_proposal is not None
        return {
            "schema": "paramcad.face_diagnostics", "version": 1, "bundle_id": str(self.bundle_id),
            "comparison": {"first": first["id"], "second": second["id"], "angle_degrees": angle,
                "offset_difference": offset_difference,
                "same_equation_within_numeric_tolerance": angle < 1e-5 and offset_difference < 1e-5,
                "bounds_distance_lower_bound": bounds_gap, "support_overlap_points": overlap,
                "support_overlap_fraction_of_smaller": overlap/max(1, min(len(first_ids), len(second_ids))),
                "diagnostic_midpoint_plane_residuals": {"first": residual_summary(first),
                                                         "second": residual_summary(second)}},
            "recorded_relations": recorded, "same_display_group": same_group,
            "merge_trace_available": bool(trace), "candidate_links_available": candidate_links_available,
            "same_recovery_proposal": candidate_links_available and first_proposal == second_proposal,
            "matching_attempt_found": bool(attempts),
            "trace_truncated": trace.get("truncated"), "merge_attempts": attempts,
            "interpretation": ("Geometric comparisons are measurements on the exported supports. Midpoint-plane residuals are diagnostic probes, not the detector's merge fit. Merge attempts are linked through each surface's source proposal ID."
                if trace_available else
                "Geometric comparisons are measurements on the exported supports. Midpoint-plane residuals are diagnostic probes, not the detector's merge fit. This bundle has no merge trace linked to both selected source proposals."),
        }

    def _binary(self, name, magic, record_size):
        stream = self._path(name).open("rb")
        try:
            raw = stream.read(HEADER.size)
            if len(raw) != HEADER.size:
                raise ValueError("Truncated binary header")
            actual, version, stride, count, identity, reserved = HEADER.unpack(raw)
            if (actual, version, stride, identity, reserved) != (magic, 1, record_size, self.bundle_id.bytes, 0):
                raise ValueError("Binary data belongs to a different snapshot or format")
            size = self._path(name).stat().st_size
            if record_size and size != HEADER.size + count * record_size:
                raise ValueError("Truncated or extended binary records")
            return stream, count, size
        except Exception:
            stream.close()
            raise

    @staticmethod
    def _range(reference, stride, size):
        offset, count = reference["offset_bytes"], reference["count"]
        if not isinstance(offset, int) or not isinstance(count, int) or offset < HEADER.size or count < 0:
            raise ValueError("Invalid binary range")
        if offset + count * stride > size:
            raise ValueError("Binary range is out of bounds")
        return offset, count

    def point_indices(self, surface):
        stream, _, size = self._binary("point-map.bin", b"PCADMAP1", INDEX.size)
        with stream:
            offset, remaining = self._range(surface["point_map"], INDEX.size, size)
            if (offset - HEADER.size) % INDEX.size or remaining != surface["point_count"]:
                raise ValueError("Mapping count or alignment differs from surface metadata")
            stream.seek(offset)
            while remaining:
                count = min(8192, remaining)
                raw = stream.read(count * INDEX.size)
                if len(raw) != count * INDEX.size:
                    raise ValueError("Truncated point mapping")
                for (index,) in struct.iter_unpack("<Q", raw):
                    if index >= self.manifest["source"]["point_count"]:
                        raise ValueError("Point index is out of range")
                    yield index
                remaining -= count

    def points(self, surface):
        stream, count, _ = self._binary("points.bin", b"PCADPTS1", POINT.size)
        if count != self.manifest["source"]["point_count"]:
            stream.close()
            raise ValueError("Point count differs from manifest")
        with stream:
            for index in self.point_indices(surface):
                stream.seek(HEADER.size + index * POINT.size)
                values = POINT.unpack(stream.read(POINT.size))
                if values[9] or values[10] != 0:
                    raise ValueError("Active surface mapping references a deleted or invalid record")
                yield {"source_index": index, "position": values[:3], "normal": values[3:6], "color": values[6:9]}

    def mesh_summary(self, surface):
        mesh = surface.get("render_mesh")
        if not mesh:
            return None
        stream, _, size = self._binary("meshes.bin", b"PCADMSH1", 0)
        with stream:
            offset, count = self._range(mesh["vertices"], VERTEX.size, size)
            stream.seek(offset)
            vertices = [VERTEX.unpack(stream.read(VERTEX.size)) for _ in range(count)]
            offset, count = self._range(mesh["triangles"], TRIANGLE.size, size)
            stream.seek(offset)
            members = set(self.point_indices(surface))
            area = foreign_area = maximum_edge = 0.
            foreign_triangles = 0
            largest = []
            for _ in range(count):
                indices = TRIANGLE.unpack(stream.read(TRIANGLE.size))
                if any(i >= len(vertices) for i in indices):
                    raise ValueError("Invalid mesh vertex index")
                points = [vertices[i] for i in indices]
                a, b, c = points
                u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
                cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
                triangle_area = math.sqrt(sum(x*x for x in cross))*.5
                area += triangle_area
                foreign = any(p[3] not in members for p in points)
                if foreign:
                    foreign_triangles += 1
                    foreign_area += triangle_area
                maximum_edge = max(maximum_edge, *(math.dist(points[i][:3], points[(i+1)%3][:3]) for i in range(3)))
                largest.append({"area": triangle_area, "source_indices": [p[3] for p in points], "uses_other_collection": foreign})
                largest.sort(key=lambda item: item["area"], reverse=True)
                del largest[3:]
            return {"area": area, "triangle_count": count, "maximum_edge": maximum_edge,
                    "foreign_source_vertices": sum(v[3] not in members for v in vertices),
                    "foreign_triangles": foreign_triangles, "foreign_triangle_area": foreign_area,
                    "largest_triangles": largest}

    def analyze(self, surface):
        count = valid_normals = 0
        low, high, total = [math.inf]*3, [-math.inf]*3, [0.]*3
        residual_sum = residual_max = residual_squared = 0.
        plane = surface["parameters"] if surface["geometry"] == "plane" and surface["parameters"] else None
        normal_length = math.sqrt(sum(x*x for x in plane["normal"])) if plane else 0.
        for point in self.points(surface):
            p = point["position"]
            if not all(math.isfinite(x) for x in p):
                raise ValueError("Active mapping contains non-finite coordinates")
            count += 1
            valid_normals += math.sqrt(sum(x*x for x in point["normal"])) > 1e-12
            for i in range(3):
                low[i], high[i] = min(low[i], p[i]), max(high[i], p[i])
                total[i] += p[i]
            if plane and normal_length > 0:
                residual = abs(sum(x*y for x, y in zip(plane["normal"], p)) + plane["offset"])/normal_length
                residual_sum += residual
                residual_squared += residual*residual
                residual_max = max(residual_max, residual)
        result = {"id": surface["id"], "name": surface["name"], "observations_read": count,
                  "bounds": [low, high] if count else None,
                  "centroid": [x/count for x in total] if count else None,
                  "nonzero_normals": valid_normals,
                  "mesh": self.mesh_summary(surface)}
        if plane and count and normal_length > 0:
            result["measured_plane_residual"] = {"mean": residual_sum/count, "max": residual_max,
                                                "rms": math.sqrt(residual_squared/count)}
        if "support_validation" in surface:
            result["support_validation"] = surface["support_validation"]
        return result

    def verify(self, full=False):
        surfaces = list(self.surfaces())
        self.support_validation()
        for row in surfaces:
            self.plane_group(self.surface(surface_id=row["id"]))
        if full:
            for asset in self.manifest["assets"]:
                path = self._path(asset["path"])
                if path.stat().st_size != asset["bytes"]:
                    raise ValueError("Asset size mismatch: " + asset["path"])
                digest = hashlib.sha256()
                with path.open("rb") as stream:
                    while chunk := stream.read(1024*1024):
                        digest.update(chunk)
                if digest.hexdigest() != asset["sha256"]:
                    raise ValueError("Asset checksum mismatch: " + asset["path"])
        return {"bundle_id": str(self.bundle_id), "surface_records": len(surfaces),
                "verification": "all_assets" if full else "metadata_only"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=Path(__file__).resolve().parent)
    commands = parser.add_subparsers(dest="command", required=True)
    listing = commands.add_parser("list", help="List bounded surface metadata; never reads points")
    listing.add_argument("--filter", default="")
    listing.add_argument("--kind")
    listing.add_argument("--limit", type=int, default=20)
    listing.add_argument("--offset", type=int, default=0)
    for command in ("show", "analyze", "explain", "points"):
        sub = commands.add_parser(command)
        group = sub.add_mutually_exclusive_group(required=True)
        group.add_argument("--id", type=int)
        group.add_argument("--name")
        if command == "show":
            sub.add_argument("--group", action="store_true", help="Also read the shared-plane metadata, not observations")
        if command == "points":
            sub.add_argument("--limit", type=int, default=10)
    verify = commands.add_parser("verify")
    verify.add_argument("--full", action="store_true", help="Explicitly hash binary assets as well")
    compare = commands.add_parser("compare", help="Compare two fitted planes using their exported observations")
    compare.add_argument("--ids", required=True, help="Exactly two comma-separated surface IDs")
    args = parser.parse_args(argv)
    bundle = Bundle(args.bundle)
    if args.command == "list":
        if not 1 <= args.limit <= 100 or args.offset < 0:
            parser.error("Use limit 1..100 and a nonnegative offset")
        rows = [row for row in bundle.surfaces() if args.filter.casefold() in row["name"].casefold()
                and (not args.kind or row["geometry"] == args.kind)]
        result = {"total_matches": len(rows), "surfaces": rows[args.offset:args.offset+args.limit]}
    elif args.command == "verify":
        result = bundle.verify(args.full)
    elif args.command == "compare":
        try:
            ids = [int(value.strip()) for value in args.ids.split(",")]
        except ValueError as error:
            parser.error("--ids must contain two comma-separated integers")
        if len(ids) != 2 or ids[0] == ids[1] or any(value < 0 for value in ids):
            parser.error("--ids must contain exactly two distinct nonnegative IDs")
        result = bundle.compare_planes(*(bundle.surface(surface_id=value) for value in ids))
    else:
        surface = bundle.surface(surface_id=args.id, name=args.name)
        if args.command == "show":
            result = surface
            if args.group:
                result = {"surface": surface, "shared_plane": bundle.plane_group(surface),
                          "support_validation_report": bundle.support_validation(surface)}
        elif args.command == "analyze":
            result = bundle.analyze(surface)
        elif args.command == "explain":
            result = bundle.diagnose(surface)
        else:
            if not 1 <= args.limit <= 100:
                parser.error("Raw coordinate output requires limit 1..100")
            import itertools
            result = {"id": surface["id"], "raw_points": list(itertools.islice(bundle.points(surface), args.limit)),
                      "total_points": surface["point_count"]}
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, struct.error) as error:
        print("Analysis bundle error: " + str(error), file=sys.stderr)
        raise SystemExit(1)
