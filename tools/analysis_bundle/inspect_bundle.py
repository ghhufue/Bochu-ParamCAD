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
        return result

    def verify(self, full=False):
        surfaces = list(self.surfaces())
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
    for command in ("show", "analyze", "points"):
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
    else:
        surface = bundle.surface(surface_id=args.id, name=args.name)
        if args.command == "show":
            result = surface
            if args.group:
                result = {"surface": surface, "shared_plane": bundle.plane_group(surface)}
        elif args.command == "analyze":
            result = bundle.analyze(surface)
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
