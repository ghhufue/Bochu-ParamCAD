"""Extract one surface and its coplanar family without loading a whole .pcad.

The project manifest is read once. Annotation objects are decoded one at a
time; coordinates are accessed by a read-only memory map at the selected rows.
"""
from __future__ import annotations

import argparse
import codecs
import json
from pathlib import Path
import re
import struct

import numpy as np
from scipy.spatial import ConvexHull, cKDTree


def manifest(path):
    with path.open("rb") as stream:
        if stream.read(8) != b"PCADPRJ1":
            raise ValueError("Unsupported project signature")
        size = struct.unpack(">I", stream.read(4))[0]
        if not 0 < size <= 16 * 1024 * 1024:
            raise ValueError("Invalid manifest size")
        metadata = json.loads(stream.read(size))
        offset = stream.tell()
    assets = {}
    for asset in metadata["assets"]:
        length = int(asset["bytes"])
        if length < 0 or offset + length > path.stat().st_size:
            raise ValueError("Invalid project asset bounds")
        assets[asset["name"]] = (offset, length)
        offset += length
    return metadata, assets


def regions(path, asset):
    offset, remaining = asset
    decoder = json.JSONDecoder()
    utf8 = codecs.getincrementaldecoder("utf-8")()
    with path.open("rb") as stream:
        stream.seek(offset)
        buffer = ""

        def read():
            nonlocal remaining
            if not remaining:
                raise ValueError("Truncated annotation regions")
            data = stream.read(min(65536, remaining))
            remaining -= len(data)
            return utf8.decode(data, final=not remaining)

        while True:
            match = re.search(r'"regions"\s*:\s*\[', buffer)
            if match:
                buffer = buffer[match.end():]
                break
            buffer = buffer[-256:] + read()
        while True:
            buffer = buffer.lstrip(" \r\n\t,")
            if not buffer:
                buffer += read()
                continue
            if buffer.startswith("]"):
                return
            try:
                value, end = decoder.raw_decode(buffer)
            except json.JSONDecodeError:
                buffer += read()
                continue
            yield value
            buffer = buffer[end:]


def plane_key(region):
    if region.get("geometry") != "plane" or region.get("fitStale"):
        return None
    parameters = region["parameters"]
    normal = np.asarray(parameters["normal"], dtype=float)
    length = np.linalg.norm(normal)
    equation = np.append(normal, parameters["offset"]) / length
    if normal[np.argmax(np.abs(normal))] < 0:
        equation *= -1
    return tuple(equation)


def vertices(path, asset):
    offset, length = asset
    with path.open("rb") as stream:
        stream.seek(offset)
        header = []
        while stream.tell() - offset < min(length, 32768):
            line = stream.readline().decode("ascii").strip()
            header.append(line)
            if line == "end_header":
                break
        else:
            raise ValueError("Invalid PLY header")
        payload = stream.tell()
    if "format binary_little_endian 1.0" not in header:
        raise ValueError("Expected project binary PLY")
    count = int(next(line.split()[2] for line in header if line.startswith("element vertex ")))
    types = {"float": "<f4", "double": "<f8", "uchar": "u1", "uint": "<u4", "int": "<i4"}
    properties = [line.split()[1:] for line in header if line.startswith("property ")]
    dtype = np.dtype([(name, types[kind]) for kind, name in properties])
    if payload + count * dtype.itemsize > offset + length:
        raise ValueError("Truncated PLY payload")
    return np.memmap(path, mode="r", dtype=dtype, offset=payload, shape=(count,))


def summarize(points, normal, offset):
    reference = np.array([1., 0., 0.]) if abs(normal[0]) < .8 else np.array([0., 1., 0.])
    x = reference - normal * np.dot(reference, normal)
    x /= np.linalg.norm(x)
    y = np.cross(normal, x)
    uv = np.column_stack(((points - points[0]) @ x, (points - points[0]) @ y))
    center = uv.mean(axis=0)
    covariance = np.cov(uv.T, bias=True)
    eigenvalues, axes = np.linalg.eigh(covariance)
    principal = (uv-center) @ axes
    distance, neighbor = cKDTree(uv).query(uv, k=min(len(points), 25))
    hull = ConvexHull(uv)
    distances = distance[:, 1:]
    short_spacing = np.sort(distances, axis=1)[:, 1]
    # This measures observed neighborhoods, not an area-coverage probability.
    return {
        "point_count": len(points),
        "bounds": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
        "plane_residual_mean": float(np.abs(points @ normal + offset).mean()),
        "plane_residual_max": float(np.abs(points @ normal + offset).max()),
        "projected_hull_area": float(hull.volume),
        "projected_covariance_eigenvalues": eigenvalues.tolist(),
        "principal_spans": np.ptp(principal, axis=0).tolist(),
        "second_neighbor_spacing_median": float(np.median(short_spacing)),
        "nearest_distance_max": float(distance[:, 1].max()),
        "farthest_from_neighbors_local_index": int(np.argmax(distance[:, 1])),
        "uv": uv.tolist(),
        "neighbor_distances": distance.tolist(),
        "neighbor_indices": neighbor.tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project", type=Path)
    parser.add_argument("--name", default="平面 406")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-assets", action="store_true",
                        help="Stream the original PLY and annotation assets to the output directory for an exact client replay")
    args = parser.parse_args()
    metadata, assets = manifest(args.project)
    target = None
    region_count = 0
    for region in regions(args.project, assets["annotations.json"]):
        region_count += 1
        if region["name"] == args.name:
            target = region
    if target is None:
        raise ValueError(f"Surface not found: {args.name}")
    if plane_key(target) is None:
        raise ValueError("This diagnostic requires a current fitted plane")
    family = [region for region in regions(args.project, assets["annotations.json"])
              if plane_key(region) == plane_key(target)]
    observations = vertices(args.project, assets["cloud.ply"])
    selected = set(target["pointIndices"])
    family_ids = sorted({row for region in family for row in region["pointIndices"]})
    owner = {row: region["id"] for region in family for row in region["pointIndices"]}
    rows = observations[family_ids]
    positions = np.column_stack([rows[axis] for axis in ("x", "y", "z")]).astype(float)
    normals = (np.column_stack([rows[axis] for axis in ("nx", "ny", "nz")]).astype(float)
               if all(axis in rows.dtype.names for axis in ("nx", "ny", "nz")) else np.zeros_like(positions))
    parameters = target["parameters"]
    normal = np.asarray(parameters["normal"], dtype=float)
    length = np.linalg.norm(normal)
    normal /= length
    metrics = summarize(positions[[i in selected for i in family_ids]], normal, parameters["offset"]/length)
    args.output.mkdir(parents=True, exist_ok=True)
    if args.replay_assets:
        with args.project.open("rb") as stream:
            for name in ("cloud.ply", "annotations.json"):
                offset, remaining = assets[name]
                stream.seek(offset)
                with (args.output / name).open("wb") as output:
                    while remaining:
                        data = stream.read(min(65536, remaining))
                        if not data:
                            raise ValueError("Truncated replay asset")
                        output.write(data)
                        remaining -= len(data)
        (args.output / "cloud.json").write_text(json.dumps({"unit": metadata["cloudUnit"]}), encoding="utf-8")
    excerpt = {
        "project": str(args.project.resolve()), "region_count": region_count,
        "target": target, "family": family, "metrics": metrics,
        "points": [{"source_index": row, "owner": owner[row], "position": position.tolist(),
                    "normal": direction.tolist()} for row, position, direction in zip(family_ids, positions, normals)],
    }
    (args.output / "surface-excerpt.json").write_text(json.dumps(excerpt, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {"project_bytes": args.project.stat().st_size, "manifest_cloud_unit": metadata["cloudUnit"],
               "region_count": region_count, "target_id": target["id"], "target_name": target["name"],
               "target_residual": target["residual"], "target_confidence": target["confidence"],
               "family": [{"id": r["id"], "name": r["name"], "points": len(r["pointIndices"])} for r in family],
               "metrics": {key: value for key, value in metrics.items() if key not in ("uv", "neighbor_distances", "neighbor_indices")}}
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
