#include "solid_inference/SideHoleInference.h"

#include "common/Math3.h"
#include "modeling/SemanticFaces.h"

namespace reconstruction {
namespace {
using namespace math3;

struct StockSide {
    modeling::SketchEntityId edge;
    modeling::Plane3d plane;
    Vec3 across;
    double minAcross;
    double maxAcross;
};

std::vector<StockSide> stockSides(const SolidFeatureOperation& stock, const modeling::Plane3d& frame) {
    std::vector<StockSide> sides;
    const auto y = cross(frame.normal, frame.xDirection);
    for (const auto& entity : stock.sketch.entities) {
        const auto* line = std::get_if<modeling::Line2D>(&entity.geometry);
        modeling::Plane3d side;
        std::string error;
        if (!line || !modeling::extrusionSidePlane(stock.sketch, frame, stock.reverse, entity.id, side, error))
            return {};
        const auto across = cross(side.normal, side.xDirection);
        const auto tangent = add(scale(frame.xDirection, line->x2-line->x1), scale(y, line->y2-line->y1));
        const double end = dot(tangent, across);
        sides.push_back({entity.id, side, across, std::min(0.0, end), std::max(0.0, end)});
    }
    return sides;
}

struct Opening {
    const StockSide* side;
    double axial;
    double x;
    double y;
};

std::vector<Opening> crossings(const std::vector<StockSide>& sides, Vec3 axisPoint, Vec3 axis,
    double depth, double tolerance) {
    std::vector<Opening> result;
    for (const auto& side : sides) {
        const double facing = dot(axis, side.plane.normal);
        if (std::abs(facing) < 1e-9) continue;
        const double axial = dot(subtract(side.plane.origin, axisPoint), side.plane.normal) / facing;
        const auto delta = subtract(add(axisPoint, scale(axis, axial)), side.plane.origin);
        const double x = dot(delta, side.plane.xDirection), y = dot(delta, side.across);
        if (x >= -tolerance && x <= depth + tolerance &&
            y >= side.minAcross - tolerance && y <= side.maxAcross + tolerance)
            result.push_back({&side, axial, x, y});
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.axial < b.axial; });
    return result;
}

bool matchesRim(const BoundaryLoop3d& rim, const Opening& opening, double radius, double tolerance) {
    if (rim.role != SceneLoopRole::Hole || !rim.closed || rim.vertices.size() < 8) return false;
    const auto& side = *opening.side;
    std::vector<double> angles;
    angles.reserve(rim.vertices.size());
    for (const auto vertex : rim.vertices) {
        const auto delta = subtract(vertex, side.plane.origin);
        const double x = dot(delta, side.plane.xDirection) - opening.x;
        const double y = dot(delta, side.across) - opening.y;
        if (std::abs(dot(delta, side.plane.normal)) > tolerance ||
            std::abs(std::hypot(x, y) - radius) > tolerance) return false;
        angles.push_back(std::atan2(y, x));
    }
    constexpr double pi = 3.14159265358979323846;
    std::sort(angles.begin(), angles.end());
    if (angles.front() + 2*pi - angles.back() > pi/2) return false;
    return std::adjacent_find(angles.begin(), angles.end(), [](double a, double b) {
        return b-a > pi/2;
    }) == angles.end();
}

std::optional<PatchId> openingPatch(const SurfaceScene& scene, const Opening& opening,
    double radius, double tolerance, double cosine) {
    const auto& side = *opening.side;
    std::optional<PatchId> match;
    for (const auto& patch : scene.patches) {
        if (!patch.orientationKnown || patch.boundaryStatus != SceneBoundaryStatus::ObservedClosed) continue;
        const auto* plane = std::get_if<PlaneEvidence>(&scene.surfaces[patch.surfaceId - 1]);
        if (!plane) continue;
        const double fittedLength = length(plane->plane.normal);
        const double materialLength = length(patch.materialNormal);
        if (!(fittedLength > 1e-9) || !(materialLength > 1e-9) ||
            !std::isfinite(fittedLength) || !std::isfinite(materialLength) ||
            dot(patch.materialNormal, side.plane.normal) < materialLength * cosine ||
            std::abs(dot(plane->plane.normal, side.plane.normal)) < fittedLength * cosine) continue;
        const auto center = add(add(side.plane.origin, scale(side.plane.xDirection, opening.x)),
            scale(side.across, opening.y));
        if (!(std::abs(dot(plane->plane.normal, center) + plane->plane.offset) <= tolerance * fittedLength))
            continue;
        if (!std::any_of(patch.loops.begin(), patch.loops.end(), [&](const auto& rim) {
                return matchesRim(rim, opening, radius, tolerance);
            })) continue;
        if (match) return std::nullopt;
        match = patch.id;
    }
    return match;
}
} // namespace

std::optional<SolidCandidate> proposeSideThroughHoles(const SurfaceScene& scene, const SolidCandidate& base,
    std::size_t maximumOperations) {
    if (base.featureProgram.empty() || base.featureProgram.size() >= maximumOperations ||
        base.featureProgram.front().operation != modeling::ExtrudeOperation::NewBody ||
        std::any_of(base.featureProgram.begin() + 1, base.featureProgram.end(),
            [](const auto& step) { return step.operation != modeling::ExtrudeOperation::Cut; })) return {};
    if (std::none_of(scene.surfaces.begin(), scene.surfaces.end(), [](const auto& surface) {
            return std::holds_alternative<CylinderEvidence>(surface);
        })) return {};
    const auto& stock = base.featureProgram.front();
    const auto* frame = std::get_if<modeling::Plane3d>(&stock.sketch.plane);
    if (!frame) return {};
    const auto sides = stockSides(stock, *frame);
    if (sides.empty()) return {};
    const double tolerance = scene.header.distanceToleranceMm;
    const double cosine = std::cos(scene.header.angularToleranceRad);
    const double perpendicular = std::sin(scene.header.angularToleranceRad);
    auto candidate = base;
    for (const auto& patch : scene.patches) {
        if (candidate.featureProgram.size() >= maximumOperations) break;
        if (patch.supportPointIds.empty() || !patch.supportConfidence ||
            std::find(base.supportingPatches.begin(), base.supportingPatches.end(), patch.id) != base.supportingPatches.end())
            continue;
        const auto* wall = std::get_if<CylinderEvidence>(&scene.surfaces[patch.surfaceId - 1]);
        if (!wall || !(wall->angularCoverage >= .7) || !std::isfinite(wall->axialMinimum) ||
            !std::isfinite(wall->axialMaximum)) continue;
        const auto& cylinder = wall->cylinder;
        const double axisLength = length(cylinder.axisDirection);
        const double radius = cylinder.radius;
        if (!(axisLength > 1e-9) || !std::isfinite(axisLength) ||
            !(radius > tolerance) || !std::isfinite(radius)) continue;
        const auto axis = scale(cylinder.axisDirection, 1 / axisLength);
        if (std::abs(dot(axis, frame->normal)) > perpendicular) continue;
        const auto ends = crossings(sides, cylinder.axisPoint, axis, stock.depth, tolerance);
        if (ends.size() != 2) continue;
        const auto& entry = ends.front();
        const auto& exit = ends.back();
        const auto fits = [&](const Opening& opening) {
            const auto& side = *opening.side;
            return opening.x - radius > tolerance && opening.x + radius < stock.depth - tolerance &&
                opening.y - radius > side.minAcross + tolerance && opening.y + radius < side.maxAcross - tolerance;
        };
        if (dot(axis, entry.side->plane.normal) > -cosine || dot(axis, exit.side->plane.normal) < cosine ||
            exit.axial - entry.axial <= 2*tolerance || !fits(entry) || !fits(exit) ||
            std::abs(wall->axialMinimum - entry.axial) > 2*tolerance ||
            std::abs(wall->axialMaximum - exit.axial) > 2*tolerance) continue;
        const auto entryPatch = openingPatch(scene, entry, radius, tolerance, cosine);
        const auto exitPatch = openingPatch(scene, exit, radius, tolerance, cosine);
        if (!entryPatch || !exitPatch || entryPatch == exitPatch) continue;

        SolidFeatureOperation cut;
        cut.sketchFace = ProgramFaceReference{0, modeling::FaceRole::ExtrusionSide, 0, entry.side->edge};
        cut.sketch.entities.push_back({1, modeling::Circle2D{entry.x, entry.y, radius}});
        cut.operation = modeling::ExtrudeOperation::Cut;
        cut.reverse = true;
        cut.throughAll = true;
        cut.depth = exit.axial - entry.axial;
        cut.supportPointIds = patch.supportPointIds;
        candidate.featureProgram.push_back(std::move(cut));
        candidate.supportingPatches.insert(candidate.supportingPatches.end(), {patch.id, *entryPatch, *exitPatch});
    }
    if (candidate.featureProgram.size() == base.featureProgram.size()) return {};
    candidate.boundaryBody.reset();
    candidate.diagnostics.push_back("Observed cylinder walls and both side openings support host-relative through-all cuts");
    return candidate;
}

} // namespace reconstruction
