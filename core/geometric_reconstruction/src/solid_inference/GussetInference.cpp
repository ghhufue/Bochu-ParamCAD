#include "solid_inference/GussetInference.h"

#include "common/Math3.h"

namespace reconstruction {
namespace {
using namespace math3;

// Points in the stock sketch frame use x/y for the section and z for depth.
struct StockFrame {
    const modeling::Plane3d& plane;
    Vec3 y;

    Vec3 project(Vec3 world) const {
        const auto delta = subtract(world, plane.origin);
        return {dot(delta, plane.xDirection), dot(delta, y), dot(delta, plane.normal)};
    }

    Vec3 world(Vec3 local) const {
        return add(plane.origin, add(scale(plane.xDirection, local.x),
            add(scale(y, local.y), scale(plane.normal, local.z))));
    }
};

struct Corner {
    Vec3 vertex;
    std::array<Vec3, 2> neighbors;
};

std::optional<Corner> reentrantCorner(const SolidFeatureOperation& stock, double tolerance,
    double perpendicularLimit) {
    const auto& entities = stock.sketch.entities;
    if (entities.size() != 6) return std::nullopt;
    std::array<Vec3, 6> starts, ends;
    double area = 0;
    for (std::size_t i = 0; i < starts.size(); ++i) {
        const auto* line = std::get_if<modeling::Line2D>(&entities[i].geometry);
        if (!line) return std::nullopt;
        starts[i] = {line->x1, line->y1, 0};
        ends[i] = {line->x2, line->y2, 0};
        area += cross(starts[i], ends[i]).z;
    }
    if (std::abs(area) <= tolerance * tolerance) return std::nullopt;
    std::optional<Corner> corner;
    for (std::size_t i = 0; i < starts.size(); ++i) {
        const auto previous = (i + starts.size() - 1) % starts.size();
        const auto next = (i + 1) % starts.size();
        if (length(subtract(starts[i], ends[previous])) > tolerance) return std::nullopt;
        const auto in = subtract(starts[i], starts[previous]);
        const auto out = subtract(starts[next], starts[i]);
        const double inLength = length(in), outLength = length(out);
        if (inLength <= tolerance || outLength <= tolerance ||
            std::abs(dot(in, out)) > inLength * outLength * perpendicularLimit) return std::nullopt;
        if (cross(in, out).z * area < 0) {
            if (corner) return std::nullopt;
            corner = Corner{starts[i], {starts[previous], starts[next]}};
        }
    }
    return corner;
}

const BoundaryLoop3d* outerLoop(const SurfaceScene& scene, const ScenePatch& patch,
    std::size_t vertexCount) {
    const double normalLength = length(patch.materialNormal);
    if (!patch.orientationKnown || patch.boundaryStatus != SceneBoundaryStatus::ObservedClosed ||
        normalLength <= 1e-12 || !std::isfinite(normalLength) || patch.loops.size() != 1) return nullptr;
    const auto* evidence = std::get_if<PlaneEvidence>(&scene.surfaces[patch.surfaceId - 1]);
    if (!evidence) return nullptr;
    const auto& loop = patch.loops.front();
    if (!loop.closed || loop.role != SceneLoopRole::Outer || loop.vertices.size() != vertexCount)
        return nullptr;
    const double fittedLength = length(evidence->plane.normal);
    if (!std::isfinite(fittedLength) || fittedLength <= 1e-12 || !std::isfinite(evidence->plane.offset))
        return nullptr;
    const double tolerance = scene.header.distanceToleranceMm * fittedLength;
    if (!std::all_of(loop.vertices.begin(), loop.vertices.end(), [&](Vec3 point) {
            return std::abs(dot(evidence->plane.normal, point) + evidence->plane.offset) <= tolerance;
        })) return nullptr;
    return &loop;
}

struct RibSide {
    PatchId patch;
    double depth;
    bool forward;
    std::array<Vec3, 2> feet;
};

std::optional<RibSide> ribSide(const ScenePatch& patch, const BoundaryLoop3d& loop,
    const StockFrame& frame, const Corner& corner, double stockDepth, double tolerance, double cosine) {
    const double facing = dot(normalized(patch.materialNormal), frame.plane.normal);
    if (std::abs(facing) < cosine) return std::nullopt;
    std::array<Vec3, 3> section;
    std::transform(loop.vertices.begin(), loop.vertices.end(), section.begin(),
        [&](Vec3 world) { return frame.project(world); });
    const double depth = (section[0].z + section[1].z + section[2].z) / 3;
    if (depth <= tolerance || depth >= stockDepth - tolerance) return std::nullopt;
    for (auto& vertex : section) {
        if (std::abs(vertex.z - depth) > tolerance) return std::nullopt;
        vertex.z = 0;
    }

    const auto tip = std::find_if(section.begin(), section.end(),
        [&](Vec3 point) { return length(subtract(point, corner.vertex)) <= tolerance; });
    if (tip == section.end()) return std::nullopt;
    const auto tipIndex = static_cast<std::size_t>(tip - section.begin());
    std::array<Vec3, 2> feet{section[(tipIndex + 1) % 3], section[(tipIndex + 2) % 3]};
    for (const bool swap : {false, true}) {
        std::array<Vec3, 2> snapped;
        bool supported = true;
        for (std::size_t side = 0; side < 2; ++side) {
            const auto edge = subtract(corner.neighbors[side], corner.vertex);
            const auto foot = feet[swap ? 1 - side : side];
            const double fraction = dot(subtract(foot, corner.vertex), edge) / dot(edge, edge);
            snapped[side] = add(corner.vertex, scale(edge, std::clamp(fraction, 0.0, 1.0)));
            supported &= length(subtract(snapped[side], corner.vertex)) > tolerance &&
                length(subtract(foot, snapped[side])) <= tolerance;
        }
        if (supported) return RibSide{patch.id, depth, facing > 0, snapped};
    }
    return std::nullopt;
}

const ScenePatch* slopePatch(const SurfaceScene& scene, const StockFrame& frame,
    const Corner& corner, const RibSide& first, const RibSide& second, double tolerance, double cosine) {
    const auto atDepth = [&](Vec3 point, double depth) {
        point.z = depth;
        return frame.world(point);
    };
    const std::array<Vec3, 4> corners{atDepth(first.feet[0], first.depth),
        atDepth(first.feet[1], first.depth), atDepth(second.feet[0], second.depth),
        atDepth(second.feet[1], second.depth)};
    auto normal = normalized(cross(subtract(corners[1], corners[0]), frame.plane.normal));
    if (dot(normal, subtract(atDepth(corner.vertex, first.depth), corners[0])) > 0)
        normal = scale(normal, -1);
    for (const auto& patch : scene.patches) {
        const auto* loop = outerLoop(scene, patch, corners.size());
        if (!loop || dot(normalized(patch.materialNormal), normal) < cosine) continue;
        std::array<bool, 4> matched{};
        for (const auto cornerPoint : corners) {
            for (std::size_t i = 0; i < matched.size(); ++i) {
                if (matched[i] || length(subtract(loop->vertices[i], cornerPoint)) > tolerance) continue;
                matched[i] = true;
                break;
            }
        }
        if (std::all_of(matched.begin(), matched.end(), [](bool found) { return found; })) return &patch;
    }
    return nullptr;
}
} // namespace

void proposeTriangularGussets(const SurfaceScene& scene, const SolidCandidate& base,
    std::vector<SolidCandidate>& output, std::size_t maximumProposals) {
    if (base.featureProgram.size() != 1) return;
    const auto& stock = base.featureProgram.front();
    const auto* plane = std::get_if<modeling::Plane3d>(&stock.sketch.plane);
    if (!plane || stock.reverse || stock.operation != modeling::ExtrudeOperation::NewBody) return;
    const double tolerance = scene.header.distanceToleranceMm;
    const double cosine = std::cos(scene.header.angularToleranceRad);
    const auto corner = reentrantCorner(stock, tolerance, std::sin(scene.header.angularToleranceRad));
    if (!corner) return;

    const StockFrame frame{*plane, cross(plane->normal, plane->xDirection)};
    std::vector<RibSide> sides;
    for (const auto& patch : scene.patches) {
        const auto* loop = outerLoop(scene, patch, 3);
        if (!loop) continue;
        if (const auto side = ribSide(patch, *loop, frame, *corner, stock.depth, tolerance, cosine))
            sides.push_back(*side);
    }
    for (const auto& first : sides) {
        if (first.forward) continue;
        for (const auto& second : sides) {
            if (output.size() >= maximumProposals) return;
            if (!second.forward || second.depth - first.depth <= tolerance ||
                length(subtract(first.feet[0], second.feet[0])) > tolerance ||
                length(subtract(first.feet[1], second.feet[1])) > tolerance) continue;
            const auto* slope = slopePatch(scene, frame, *corner, first, second, tolerance, cosine);
            if (!slope) continue;

            SolidFeatureOperation join;
            join.sketchFace = ProgramFaceReference{0, modeling::FaceRole::EndFace, first.depth - stock.depth};
            join.depth = second.depth - first.depth;
            join.operation = modeling::ExtrudeOperation::Join;
            const std::array<Vec3, 3> section{corner->vertex,
                scale(add(first.feet[0], second.feet[0]), .5),
                scale(add(first.feet[1], second.feet[1]), .5)};
            for (std::size_t i = 0; i < section.size(); ++i) {
                const auto& from = section[i];
                const auto& to = section[(i + 1) % section.size()];
                join.sketch.entities.push_back({static_cast<modeling::SketchEntityId>(i + 1),
                    modeling::Line2D{from.x, from.y, to.x, to.y}});
            }
            auto candidate = base;
            candidate.featureProgram.push_back(std::move(join));
            candidate.boundaryBody.reset();
            const std::vector<PatchId> ribPatches{first.patch, second.patch, slope->id};
            candidate.supportingPatches.insert(candidate.supportingPatches.end(), ribPatches.begin(), ribPatches.end());
            const double confidence = std::min(slope->supportConfidence, slope->loops.front().confidence);
            for (std::size_t i = 1; i < section.size(); ++i) {
                const auto& foot = section[i];
                const auto& tip = section[0];
                candidate.inferredSurfaces.push_back({PrimitiveKind::Plane, candidate.supportingPatches,
                    {frame.world({tip.x, tip.y, first.depth}), frame.world({foot.x, foot.y, first.depth}),
                     frame.world({foot.x, foot.y, second.depth}), frame.world({tip.x, tip.y, second.depth})},
                    confidence, "Gusset contact inferred along the reentrant stock corner"});
            }
            candidate.diagnostics.push_back("Observed triangular sides and slope support a gusset attached to the stock");
            output.push_back(std::move(candidate));
        }
    }
}

} // namespace reconstruction
