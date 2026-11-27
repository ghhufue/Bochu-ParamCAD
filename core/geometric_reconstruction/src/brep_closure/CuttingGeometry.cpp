#include "ClosureInternal.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax3.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <unordered_set>

namespace reconstruction::detail::closure {
namespace {
constexpr double twoPi = 6.283185307179586;

struct Extents {
    double u0 = INFINITY, u1 = -INFINITY, v0 = INFINITY, v1 = -INFINITY;
    void add(double u, double v) {
        u0 = std::min(u0, u); u1 = std::max(u1, u);
        v0 = std::min(v0, v); v1 = std::max(v1, v);
    }
};

struct Revolution {
    const SurfacePatch* patch;
    Vec3 origin, axis;
    double radius, angle, low = INFINITY, high = -INFINITY;
    double signedSupport = 0;
    bool fullCircle = false;
    double radiusAt(double height) const { return radius + height * std::tan(angle); }
};

struct End {
    const SurfacePatch* patch = nullptr;
    double height = 0;
    enum Kind { Unknown, Mouth, Floor, Shoulder } kind = Unknown;
};

std::pair<Vec3, Vec3> frame(Vec3 n) {
    const auto x = normalized(cross(std::abs(n.z) < .8 ? Vec3{0, 0, 1} : Vec3{0, 1, 0}, n));
    return {x, cross(n, x)};
}

TopoDS_Face planeFace(const PlaneEvidence& plane, const SurfacePatch& patch,
                      const std::vector<const SurfacePatch*>& neighbors, const PointIndex& samples,
                      double margin, double spacing, double tolerance) {
    const auto n = plane.plane.normal;
    const auto o = subtract(patch.centroid, scale(n, dot(n, patch.centroid) + plane.plane.offset));
    const auto [x, y] = frame(n);
    const auto project = [&](const SurfacePatch& source) {
        Extents bounds;
        for (const auto id : source.supportPointIds) {
            const auto p = subtract(samples.at(id)->position, o);
            bounds.add(dot(p, x), dot(p, y));
        }
        return bounds;
    };
    auto b = project(patch);
    const double width = b.u1 - b.u0, height = b.v1 - b.v0;
    if (std::min(width, height) <= Precision::Confusion()) return {};
    const double minPad = std::max(4 * tolerance, 1.5 * spacing);
    const double uPad = std::max(minPad, width * margin);
    const double vPad = std::max(minPad, height * margin);
    const auto local = b;
    b = {b.u0 - uPad, b.u1 + uPad, b.v0 - vPad, b.v1 + vPad};
    for (const auto* adjacent : neighbors) {
        const auto projected = project(*adjacent);
        b.u0 = std::min(b.u0, std::max(local.u0 - 2 * uPad, projected.u0 - minPad));
        b.u1 = std::max(b.u1, std::min(local.u1 + 2 * uPad, projected.u1 + minPad));
        b.v0 = std::min(b.v0, std::max(local.v0 - 2 * vPad, projected.v0 - minPad));
        b.v1 = std::max(b.v1, std::min(local.v1 + 2 * vPad, projected.v1 + minPad));
    }
    BRepBuilderAPI_MakeFace make(gp_Pln(gp_Ax3(point(o), direction(n), direction(x))), b.u0, b.u1, b.v0, b.v1);
    return make.IsDone() ? make.Face() : TopoDS_Face{};
}

double supportAngle(Vec3 origin, Vec3 axis, const std::vector<PointId>& ids, const PointIndex& samples) {
    const auto [x, y] = frame(axis);
    std::vector<double> angles;
    angles.reserve(ids.size());
    for (const auto id : ids) {
        const auto delta = subtract(samples.at(id)->position, origin);
        angles.push_back(std::atan2(dot(delta, y), dot(delta, x)));
    }
    if (angles.empty()) return 0;
    std::sort(angles.begin(), angles.end());
    double maximumGap = angles.front() + twoPi - angles.back();
    for (std::size_t i = 1; i < angles.size(); ++i) maximumGap = std::max(maximumGap, angles[i] - angles[i - 1]);
    return twoPi - maximumGap;
}

bool planarNormals(Vec3 normal, const std::vector<PointId>& ids, const PointIndex& samples) {
    const auto [u, v] = frame(normal);
    const auto origin = samples.at(ids.front())->position;
    struct Observation { double x, y, a, b; };
    std::vector<Observation> observations;
    Observation mean{};
    for (const auto id : ids) {
        const auto& sample = *samples.at(id);
        const auto n = sampleNormal(sample);
        if (!n || sample.confidence < .5 || std::abs(dot(*n, normal)) < .94) continue;
        const auto p = subtract(sample.position, origin);
        const double sign = std::copysign(1., dot(*n, normal));
        const Observation row{dot(p, u), dot(p, v), sign * dot(*n, u), sign * dot(*n, v)};
        observations.push_back(row);
        mean.x += row.x; mean.y += row.y; mean.a += row.a; mean.b += row.b;
    }
    const double count = static_cast<double>(observations.size());
    if (count < std::max(12., .65 * ids.size())) return true;
    mean = {mean.x / count, mean.y / count, mean.a / count, mean.b / count};
    double xx = 0, xy = 0, yy = 0, xa = 0, xb = 0, ya = 0, yb = 0, variation = 0;
    for (const auto& row : observations) {
        const double x = row.x - mean.x, y = row.y - mean.y, a = row.a - mean.a, b = row.b - mean.b;
        xx += x * x; xy += x * y; yy += y * y;
        xa += x * a; xb += x * b; ya += y * a; yb += y * b;
        variation += a * a + b * b;
    }
    const double determinant = xx * yy - xy * xy;
    if (determinant <= 1e-12) return true;
    const double explained = (yy * (xa * xa + xb * xb) - 2 * xy * (xa * ya + xb * yb) +
                              xx * (ya * ya + yb * yb)) / determinant;
    // A coherent change of direction across the patch is evidence of a
    // curved surface that RANSAC has approximated by a local tangent plane.
    return explained <= .000225 * count || explained <= .55 * variation;
}

Vec3 seamAxis(const Revolution& wall, const std::vector<Vec3>& neighborDirections) {
    const auto [x, y] = frame(wall.axis);
    Vec3 axis = x;
    double best = -1;
    for (int i = 0; i < 32; ++i) {
        const auto theta = twoPi * (i + .37) / 32;
        const auto candidate = add(scale(x, std::cos(theta)), scale(y, std::sin(theta)));
        double score = 1;
        for (const auto adjacent : neighborDirections) {
            const auto across = cross(wall.axis, adjacent);
            if (length(across) < 1e-4) continue;
            score = std::min(score, std::min(std::abs(dot(candidate, normalized(across))),
                                            std::abs(dot(candidate, normalized(cross(across, wall.axis))))));
        }
        if (score > best) { best = score; axis = candidate; }
    }
    return axis;
}

End findEnd(const Revolution& wall, double expected, const std::vector<const SurfacePatch*>& planes,
            const SurfaceAnalysisResult& analysis, const PointIndex& samples, double spacing, double tolerance) {
    End best;
    const auto matchTolerance = std::max(4 * tolerance, 2.5 * spacing);
    double gap = matchTolerance;
    for (const auto* patch : planes) {
        const auto& plane = std::get<PlaneEvidence>(analysis.surfaces[patch->surfaceId - 1]).plane;
        const double alignment = dot(plane.normal, wall.axis);
        if (std::abs(alignment) < 1 - 1e-6) continue;
        const double height = -(dot(plane.normal, wall.origin) + plane.offset) / alignment;
        if (std::abs(height - expected) >= gap) continue;
        const double radius = wall.radiusAt(height);
        if (radius < 0) continue;
        const auto center = add(wall.origin, scale(wall.axis, height));
        const auto [x, y] = frame(wall.axis);
        Extents extents;
        unsigned sectors = 0;
        bool inner = false;
        double farthest = 0;
        for (const auto id : patch->supportPointIds) {
            const auto p = subtract(samples.at(id)->position, center);
            const double u = dot(p, x), v = dot(p, y);
            const double r = std::hypot(u, v);
            extents.add(u, v);
            farthest = std::max(farthest, r);
            inner |= r < radius * .55;
            if (std::abs(r - radius) <= matchTolerance) {
                const auto angle = std::atan2(v, u) + twoPi;
                sectors |= 1u << (static_cast<unsigned>(angle * 8 / twoPi) % 8);
            }
        }
        int occupied = 0;
        for (auto mask = sectors; mask; mask >>= 1) occupied += mask & 1;
        if (extents.u0 > 0 || extents.u1 < 0 || extents.v0 > 0 || extents.v1 < 0 || occupied < 6) continue;
        const bool local = farthest <= radius + std::max(.2 * radius, 2 * tolerance);
        const auto kind = local ? (inner ? End::Floor : End::Shoulder) : (inner ? End::Unknown : End::Mouth);
        if (kind == End::Unknown) continue;
        if (kind == End::Mouth) {
            // An opening whose circle reaches the edge of the adjacent
            // surface needs to participate in the partition.
            const double rim = radius - tolerance;
            if (extents.u0 > -rim || extents.u1 < rim || extents.v0 > -rim || extents.v1 < rim) continue;
        }
        best = {patch, height, kind};
        gap = std::abs(height - expected);
    }
    return best;
}

double adjacentStep(const Revolution& wall, const End& end, int outward, bool larger,
                    const std::vector<Revolution>& walls, double tolerance, double spacing) {
    for (const auto& other : walls) {
        if (&wall == &other || !other.fullCircle || other.signedSupport > .25 ||
            std::abs(dot(wall.axis, other.axis)) < 1 - 1e-6) continue;
        const auto delta = subtract(other.origin, wall.origin);
        if (length(cross(delta, wall.axis)) > tolerance) continue;
        for (const double height : {other.low, other.high}) {
            const auto toward = dot(wall.axis, add(delta, scale(other.axis, height)));
            const auto depth = dot(wall.axis, other.axis) * (height == other.low ? other.high - height : other.low - height);
            const auto radiusDifference = other.radiusAt(height) - wall.radiusAt(end.height);
            if (depth * outward > tolerance && std::abs(toward - end.height) <= std::max(3 * tolerance, 2 * spacing) &&
                (larger ? radiusDifference : -radiusDifference) > tolerance) return std::abs(depth);
        }
    }
    return 0;
}

TopoDS_Shape regularHoleTool(const Revolution& wall, const End& low, const End& high,
    const std::vector<Revolution>& walls, Vec3 x, double spacing, double tolerance,
    bool& lowStep, bool& highStep) {
    if (!wall.fullCircle || wall.signedSupport > .25 || low.kind == End::Unknown ||
        high.kind == End::Unknown || low.patch == high.patch) return {};
    const auto connects = [&](const End& end, int outward) {
        return end.kind != End::Shoulder || adjacentStep(wall, end, outward, false, walls, tolerance, spacing) > 0;
    };
    if (!connects(low, -1) || !connects(high, 1) || (low.kind == End::Floor && high.kind == End::Floor)) return {};
    const double overshoot = std::clamp(4 * tolerance, 2., 5.);
    const auto extend = [&](const End& end, int outward, bool& step) {
        const auto adjacentDepth = adjacentStep(wall, end, outward, true, walls, tolerance, spacing);
        step = adjacentDepth > 0;
        if (step) return std::min(overshoot, adjacentDepth * .25);
        return end.kind == End::Mouth ? overshoot : 0.;
    };
    double lower = low.height - extend(low, -1, lowStep);
    if (wall.angle > 0) lower = std::max(0., lower);
    const double upper = high.height + extend(high, 1, highStep);
    if (upper - lower <= tolerance) return {};
    const gp_Ax2 axis(point(add(wall.origin, scale(wall.axis, lower))), direction(wall.axis), direction(x));
    if (wall.angle == 0) return BRepPrimAPI_MakeCylinder(axis, wall.radius, upper - lower).Solid();
    return BRepPrimAPI_MakeCone(axis, wall.radiusAt(lower), wall.radiusAt(upper), upper - lower).Solid();
}

TopoDS_Face revolutionFace(const Revolution& wall, Vec3 x, double spacing, double tolerance) {
    const double pad = std::max({.18 * (wall.high - wall.low), 4 * tolerance, 2 * spacing,
                                 .3 * wall.radiusAt(wall.high)});
    const gp_Ax3 placement(point(wall.origin), direction(wall.axis), direction(x));
    if (wall.angle > 0) {
        const double cosine = std::cos(wall.angle);
        BRepBuilderAPI_MakeFace make(gp_Cone(placement, wall.angle, 0),
            0, twoPi, std::max(0., wall.low - pad) / cosine, (wall.high + pad) / cosine);
        return make.IsDone() ? make.Face() : TopoDS_Face{};
    }
    BRepBuilderAPI_MakeFace make(gp_Cylinder(placement, wall.radius), 0, twoPi, wall.low - pad, wall.high + pad);
    return make.IsDone() ? make.Face() : TopoDS_Face{};
}
} // namespace

bool supportedAnalyticPatch(const PrimitiveEvidence& evidence, const std::vector<PointId>& ids,
                            const PointIndex& samples) {
    if (ids.size() < 6) return false;
    if (const auto* plane = std::get_if<PlaneEvidence>(&evidence))
        return planarNormals(plane->plane.normal, ids, samples);
    if (const auto* sphere = std::get_if<SphereEvidence>(&evidence)) {
        Vec3 mean;
        for (const auto id : ids)
            mean = add(mean, scale(subtract(samples.at(id)->position, sphere->sphere.center), 1 / sphere->sphere.radius));
        return length(mean) < .79 * ids.size();
    }
    const auto kind = primitiveKind(evidence);
    if (kind == PrimitiveKind::Torus) return true;
    const auto axis = primitiveAxis(evidence), origin = primitiveCenter(evidence);
    if (supportAngle(origin, axis, ids, samples) < .24 * twoPi) return false;
    const auto anchor = samples.at(ids.front())->position;
    std::size_t normals = 0, inliers = 0;
    double z = 0, slope = 0, zz = 0, ss = 0, zs = 0;
    for (const auto id : ids) {
        const auto& sample = *samples.at(id);
        if (const auto n = sampleNormal(sample)) {
            ++normals;
            inliers += surfaceNormalAlignment(evidence, sample.position, *n) >= .993;
            const double height = dot(subtract(sample.position, anchor), axis);
            const double inclination = std::abs(dot(*n, axis));
            z += height;
            slope += inclination;
            zz += height * height;
            ss += inclination * inclination;
            zs += height * inclination;
        }
    }
    if (normals < 12) return true;
    if (inliers < .65 * normals) return false;
    zz -= z * z / normals;
    ss -= slope * slope / normals;
    zs -= z * slope / normals;
    // Axial inclination is constant on a circular cylinder or cone. A
    // systematic slope change along the axis leaves the patch in the residual.
    return zz <= 1e-12 || zs * zs <= zz * std::max(.0001 * normals, .6 * ss);
}

CuttingGeometry makeCuttingGeometry(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const PointIndex& samples, const KdTree& tree, double spacing, double tolerance,
    const BrepClosureOptions& options, ClosedBrepResult& result, const std::function<bool()>& cancelled) {
    CuttingGeometry output;
    std::vector<const SurfacePatch*> planes;
    std::vector<Revolution> walls;
    std::unordered_map<PatchId, const SurfacePatch*> patches;
    std::unordered_set<SurfaceId> completeSurfaces;
    for (const auto& patch : analysis.patches) {
        if (cancelled && cancelled()) return {};
        patches.emplace(patch.id, &patch);
        if (!patch.surfaceId || patch.surfaceId > analysis.surfaces.size() || patch.supportPointIds.size() < 6) continue;
        const auto& surface = analysis.surfaces[patch.surfaceId - 1];
        if (std::holds_alternative<PlaneEvidence>(surface)) {
            planes.push_back(&patch);
            continue;
        }
        Revolution wall{&patch, {}, {}, 0, 0};
        if (const auto* cylinder = std::get_if<CylinderEvidence>(&surface)) {
            wall.origin = cylinder->cylinder.axisPoint;
            wall.axis = cylinder->cylinder.axisDirection;
            wall.radius = cylinder->cylinder.radius;
        } else if (const auto* cone = std::get_if<ConeEvidence>(&surface)) {
            wall.origin = cone->cone.apex;
            wall.axis = cone->cone.axisDirection;
            wall.angle = cone->cone.openingAngleRadians;
        } else if (completeSurfaces.insert(patch.surfaceId).second) {
            TopoDS_Shape shape;
            if (const auto* sphere = std::get_if<SphereEvidence>(&surface))
                shape = BRepPrimAPI_MakeSphere(point(sphere->sphere.center), sphere->sphere.radius).Shape();
            if (const auto* torus = std::get_if<TorusEvidence>(&surface))
                shape = BRepPrimAPI_MakeTorus(gp_Ax2(point(torus->torus.center), direction(torus->torus.axisDirection)),
                    torus->torus.majorRadius, torus->torus.minorRadius).Shape();
            for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next())
                output.faces.push_back(TopoDS::Face(face.Current()));
            continue;
        } else continue;
        std::size_t signedCount = 0;
        for (const auto id : patch.supportPointIds) {
            const auto& p = *samples.at(id);
            const auto d = subtract(p.position, wall.origin);
            const double height = dot(d, wall.axis);
            wall.low = std::min(wall.low, height);
            wall.high = std::max(wall.high, height);
            const auto n = sampleNormal(p);
            const auto radial = subtract(d, scale(wall.axis, height));
            if (n && p.normalOrientationKnown && length(radial) > Precision::Confusion()) {
                wall.signedSupport += dot(*n, normalized(radial));
                ++signedCount;
            }
        }
        if (signedCount) wall.signedSupport /= signedCount;
        wall.fullCircle = supportAngle(wall.origin, wall.axis, patch.supportPointIds, samples) > .86 * twoPi;
        if (wall.high - wall.low > Precision::Confusion()) walls.push_back(wall);
    }
    std::unordered_map<PatchId, std::vector<const SurfacePatch*>> adjacent;
    for (const auto& relation : analysis.relations) {
        if (relation.kind != SurfaceRelationKind::Adjacent && relation.kind != SurfaceRelationKind::Intersect) continue;
        const auto a = patches.find(relation.first), b = patches.find(relation.second);
        if (a != patches.end() && b != patches.end()) {
            adjacent[a->first].push_back(b->second);
            adjacent[b->first].push_back(a->second);
        }
    }
    std::unordered_set<PatchId> delayedCaps;
    for (const auto& wall : walls) {
        if (cancelled && cancelled()) return {};
        std::vector<Vec3> directions;
        for (const auto* neighbor : adjacent[wall.patch->id]) {
            const auto& primitive = analysis.surfaces[neighbor->surfaceId - 1];
            if (const auto* plane = std::get_if<PlaneEvidence>(&primitive)) directions.push_back(plane->plane.normal);
            else if (const auto* cylinder = std::get_if<CylinderEvidence>(&primitive)) directions.push_back(cylinder->cylinder.axisDirection);
            else if (const auto* cone = std::get_if<ConeEvidence>(&primitive)) directions.push_back(cone->cone.axisDirection);
        }
        const auto x = seamAxis(wall, directions);
        auto low = findEnd(wall, wall.low, planes, analysis, samples, spacing, tolerance);
        auto high = findEnd(wall, wall.high, planes, analysis, samples, spacing, tolerance);
        if (options.separateRegularHoles) {
            bool lowStep = false, highStep = false;
            auto tool = regularHoleTool(wall, low, high, walls, x, spacing, tolerance, lowStep, highStep);
            if (!tool.IsNull()) {
                output.holeTools.push_back(tool);
                if (lowStep || low.kind != End::Mouth) delayedCaps.insert(low.patch->id);
                if (highStep || high.kind != End::Mouth) delayedCaps.insert(high.patch->id);
                continue;
            }
        }
        const auto face = revolutionFace(wall, x, spacing, tolerance);
        if (!face.IsNull()) output.faces.push_back(face);
    }
    for (const auto* patch : planes) {
        if (delayedCaps.count(patch->id)) continue;
        const auto& evidence = std::get<PlaneEvidence>(analysis.surfaces[patch->surfaceId - 1]);
        const auto face = planeFace(evidence, *patch, adjacent[patch->id], samples,
                                    options.planeMarginFraction, spacing, tolerance);
        if (!face.IsNull()) output.faces.push_back(face);
    }
    auto freeform = fitFreeformFaces(points, analysis, samples, tree, spacing, tolerance,
                                    options.freeformExtendFraction, result.diagnostics, cancelled);
    result.quality.freeformFaceCount = freeform.size();
    result.diagnostics.push_back("Cutting geometry: " + std::to_string(planes.size() - delayedCaps.size()) +
        " planar patches, " + std::to_string(walls.size() - output.holeTools.size()) +
        " cylinder or cone patches, " + std::to_string(output.holeTools.size()) + " deferred hole tools");
    output.faces.insert(output.faces.end(), freeform.begin(), freeform.end());
    return output;
}

} // namespace reconstruction::detail::closure
