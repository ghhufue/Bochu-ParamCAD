#pragma once

#include "reconstruction/data/SurfaceEvidence.h"

#include <array>
#include <functional>
#include <vector>

namespace reconstruction {

using PlanarSurfaceTriangle = std::array<std::size_t, 3>;
using PlanarIntersectionSegment = std::array<modeling::Vec3, 2>;

struct PlanarSurfaceInput {
    int id = 0;
    PlaneEquation plane;
    double residual = 0;
    std::vector<modeling::Vec3> vertices;
    std::vector<PlanarSurfaceTriangle> triangles;
};

struct PlanarSurfaceBoundary {
    std::vector<modeling::Vec3> vertices;
    bool hole = false;
};

struct FittedPlanarSurface {
    int id = 0;
    std::vector<modeling::Vec3> vertices;
    std::vector<PlanarSurfaceTriangle> triangles;
    std::vector<PlanarSurfaceBoundary> boundaries;
    bool boundaryAdjusted = false;
};

enum class PlanarJunctionKind { NearlyParallel, Disjoint, Intersecting };

struct PlanarSurfaceJunction {
    int first = 0, second = 0;
    PlanarJunctionKind kind = PlanarJunctionKind::Disjoint;
    std::vector<PlanarIntersectionSegment> segments;
};

struct PlanarSurfaceJunctionResult {
    std::vector<FittedPlanarSurface> surfaces;
    std::vector<PlanarSurfaceJunction> junctions;
};

struct PlanarContourCorrectionOptions {
    double simplificationSpacingMultiplier = .45;
    bool simplifyWithoutJunction = false;
};

// Fits local joins in the supplied observation units. Each successful face
// remains an independent open surface and the input snapshot stays intact.
PlanarSurfaceJunctionResult fitPlanarSurfaceJunctions(std::vector<PlanarSurfaceInput> inputs,
    const std::function<bool()>& cancelled = {});

PlanarSurfaceJunctionResult fitPlanarSurfaceJunctions(std::vector<PlanarSurfaceInput> inputs,
    const PlanarContourCorrectionOptions& options, const std::function<bool()>& cancelled);

} // namespace reconstruction
