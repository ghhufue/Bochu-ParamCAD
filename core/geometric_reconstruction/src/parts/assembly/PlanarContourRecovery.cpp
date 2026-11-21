#include "AssemblyInternal.h"
#include "BoundaryGeometry.h"
#include "parts/projection/ProjectionInternal.h"

#include <contour_fitting/ContourParameterization.h>
#include <modeling/ModelingCore.h>

#include <BRepBndLib.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace reconstruction::detail::assembly {
namespace {
using projection::Frame;

constexpr double parallelCosine = .9986;
constexpr double maximumGapRatio = .5;
constexpr std::size_t maximumContourEdges = 128;

struct Section {
    const FiniteSurfacePatch* patch;
    std::size_t part;
    double extent; // Square root of the outer area.
};

struct Recess {
    std::size_t floor;
    double tolerance;
};

struct Proposal {
    std::vector<std::size_t> parts;
    std::vector<ProjectionOperation> operations;
    double tolerance;
    bool orientationKnown;
};

void checkCancelled(const std::function<bool()>& cancelled) {
    if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
}

BoundaryLoop projectLoop(const Frame& frame, const std::vector<Vec3>& loop) {
    BoundaryLoop projected;
    projected.reserve(loop.size());
    for (const auto point : loop) projected.push_back(frame.project(point));
    return projected;
}

double matchTolerance(const Section& a, const Section& b, double extent, double noise) {
    // Allow one sample pitch for independently traced boundaries, capped by
    // feature size so that sparse samples cannot match displaced outlines.
    return std::min(.08 * extent, std::max({4 * noise, a.patch->spacing, b.patch->spacing}));
}

bool onSection(const Frame& frame, const FiniteSurfacePatch& patch, double depth, double tolerance) {
    const auto& contour = patch.loops.front();
    return std::abs(dot(frame.axis, patch.normal)) >= parallelCosine &&
        std::all_of(contour.begin(), contour.end(), [&](Vec3 point) {
            return std::abs(frame.depth(point) - depth) <= tolerance;
        });
}

bool matches(const BoundaryLoop& first, const BoundaryLoop& second, double tolerance, double areaRatio) {
    const auto a = std::abs(boundaryArea(first));
    const auto b = std::abs(boundaryArea(second));
    if (std::min(a, b) < areaRatio * std::max(a, b)) return false;

    const auto follows = [&](const BoundaryLoop& from, const BoundaryLoop& to) {
        for (std::size_t i = 0; i < from.size(); ++i) {
            const auto start = from[i];
            const auto end = from[(i + 1) % from.size()];
            for (const double t : {0., .25, .5, .75}) {
                if (boundaryDistance(to, {start.x + t * (end.x - start.x),
                                          start.y + t * (end.y - start.y)}) > tolerance)
                    return false;
            }
        }
        return true;
    };
    return follows(first, second) && follows(second, first);
}

std::vector<Section> planarSections(const AssemblyReconstructionResult& result) {
    // Local recovery can share original patch IDs between a closed part and
    // a residual part. A contour used here must have a single current owner.
    std::vector<std::size_t> owner(result.patches.size() + 1, result.parts.size());
    std::vector<bool> shared(owner.size());
    for (std::size_t i = 0; i < result.parts.size(); ++i) {
        for (const auto id : result.parts[i].patchIds) {
            shared[id] = shared[id] || owner[id] != result.parts.size();
            owner[id] = i;
        }
    }

    std::vector<Section> sections;
    for (const auto& patch : result.patches) {
        const auto i = owner[patch.id];
        if (shared[patch.id] || i == result.parts.size() || result.parts[i].closed ||
            !patch.planar || patch.loops.empty() || patch.loops.size() > 2 ||
            std::any_of(patch.loops.begin(), patch.loops.end(), [](const auto& loop) {
                return loop.size() < 3 || loop.size() > maximumContourEdges;
            }))
            continue;

        const Frame frame{patch.origin, patch.normal, patch.xDirection, cross(patch.normal, patch.xDirection)};
        const auto outer = projectLoop(frame, patch.loops.front());
        const auto area = boundaryArea(outer);
        if (area <= 0 || !simpleBoundary(outer)) continue;
        if (patch.loops.size() == 2) {
            const auto hole = projectLoop(frame, patch.loops.back());
            if (boundaryArea(hole) >= 0 || !simpleBoundary(hole) ||
                !boundaryContainsLoop(outer, hole))
                continue;
        }
        sections.push_back({&patch, i, std::sqrt(area)});
    }
    return sections;
}

std::optional<Recess> matchRecess(const Section& opening, const Section& floor,
                                 std::size_t floorIndex, double noise) {
    const auto& a = *opening.patch;
    const auto& b = *floor.patch;
    const Frame frame{a.origin, a.normal, a.xDirection, cross(a.normal, a.xDirection)};
    const double offset = frame.depth(b.origin);
    if (b.loops.size() != 1 || std::abs(offset) <= 4 * noise ||
        std::abs(offset) > maximumGapRatio * opening.extent ||
        (a.orientationKnown && offset > 0) ||
        (b.orientationKnown && dot(b.normal, scale(frame.axis, offset)) >= 0))
        return {};

    const auto hole = projectLoop(frame, a.loops.back());
    const double extent = std::min(floor.extent, std::sqrt(std::abs(boundaryArea(hole))));
    const double tolerance = matchTolerance(opening, floor, extent, noise);
    if (!onSection(frame, b, offset, tolerance) ||
        !matches(hole, projectLoop(frame, b.loops.front()), tolerance, .78))
        return {};
    return Recess{floorIndex, tolerance};
}

std::vector<modeling::SketchEntity> sketchEntities(const BoundaryLoop& loop, double tolerance,
                                                   const std::function<bool()>& cancelled) {
    contour_fitting::Contour contour;
    for (const auto point : loop) contour.outer.push_back({point.x, point.y});
    const auto fit = contour_fitting::parameterizeContours(
        {contour}, {tolerance, maximumContourEdges, true}, cancelled);
    checkCancelled(cancelled);
    if (fit.failure != contour_fitting::FitFailure::None || fit.contours.size() != 1) return {};

    const auto& ring = fit.contours.front().outer;
    if (ring.circle) {
        const auto& circle = *ring.circle;
        return {{1, modeling::Circle2D{circle.center.x, circle.center.y, circle.radius}}};
    }
    std::vector<modeling::SketchEntity> entities;
    for (std::size_t i = 0; i < ring.polyline.size(); ++i) {
        const auto a = ring.polyline[i];
        const auto b = ring.polyline[(i + 1) % ring.polyline.size()];
        entities.push_back({static_cast<modeling::SketchEntityId>(i + 1),
                            modeling::Line2D{a.x, a.y, b.x, b.y}});
    }
    return entities;
}

std::optional<Proposal> matchCaps(const Section& front, const Section& back, const Section* floor,
                                 double recessTolerance, double noise,
                                 const std::function<bool()>& cancelled) {
    const auto& a = *front.patch;
    const auto& b = *back.patch;
    const Vec3 axis = dot(subtract(b.origin, a.origin), a.normal) < 0 ? scale(a.normal, -1) : a.normal;
    const Frame frame{a.origin, axis, a.xDirection, cross(axis, a.xDirection)};
    const double depth = frame.depth(b.origin);
    const double extent = std::min(front.extent, back.extent);
    const double tolerance = matchTolerance(front, back, extent, noise);
    if (depth <= std::max(4 * noise, .5 * tolerance) || depth > maximumGapRatio * extent ||
        (a.orientationKnown && dot(axis, a.normal) > 0) ||
        (b.orientationKnown && dot(axis, b.normal) < 0) || !onSection(frame, b, depth, tolerance))
        return {};

    const auto outer = projectLoop(frame, a.loops.front());
    if (!matches(outer, projectLoop(frame, b.loops.front()), tolerance, .88)) return {};

    Proposal proposal{{front.part, back.part}, {}, tolerance, a.orientationKnown && b.orientationKnown};
    if (floor) {
        const auto& bottom = *floor->patch;
        const double cutDepth = frame.depth(bottom.origin);
        if (cutDepth <= recessTolerance || depth - cutDepth <= recessTolerance ||
            !onSection(frame, bottom, cutDepth, recessTolerance) ||
            (bottom.orientationKnown && dot(axis, bottom.normal) > 0))
            return {};
        proposal.parts.push_back(floor->part);
        proposal.orientationKnown &= bottom.orientationKnown;
        proposal.tolerance = std::max(tolerance, recessTolerance);
    }
    std::sort(proposal.parts.begin(), proposal.parts.end());
    proposal.parts.erase(std::unique(proposal.parts.begin(), proposal.parts.end()), proposal.parts.end());

    ProjectionOperation base;
    base.sketch.plane = frame.plane(0);
    base.sketch.entities = sketchEntities(outer, tolerance * .5, cancelled);
    if (base.sketch.entities.empty()) return {};
    base.depth = depth;
    base.supportPointIds = a.pointIds;
    base.supportPointIds.insert(base.supportPointIds.end(), b.pointIds.begin(), b.pointIds.end());
    proposal.operations.push_back(std::move(base));

    if (floor) {
        ProjectionOperation cut;
        cut.sketch.plane = frame.plane(0);
        cut.sketch.entities = sketchEntities(projectLoop(frame, a.loops.back()), recessTolerance, cancelled);
        if (cut.sketch.entities.empty()) return {};
        cut.operation = modeling::ExtrudeOperation::Cut;
        cut.depth = frame.depth(floor->patch->origin);
        cut.supportPointIds = floor->patch->pointIds;
        proposal.operations.push_back(std::move(cut));
    }
    return proposal;
}

std::optional<double> validate(const Proposal& proposal, const WorkingData& work,
    const std::vector<AssemblyPart>& parts, const std::unordered_map<PointId, const PointSample*>& points,
    const std::function<bool()>& cancelled) {
    modeling::ModelingCore model;
    std::string error;
    if (!buildProjectionProgram(proposal.operations, model, error)) return {};
    const auto& shape = model.bodyShape();

    // Check the participating parts' observations, including any visible side
    // walls, without imposing a surface area quota on inferred walls.
    std::vector<PointSample> samples;
    for (const auto part : proposal.parts) {
        const auto& ids = parts[part].workingPointIds;
        const auto stride = std::max<std::size_t>(1, (ids.size() + 511) / 512);
        for (std::size_t i = 0; i < ids.size(); i += stride) samples.push_back(*points.at(ids[i]));
    }
    const auto fit = projection::scoreShape(shape, samples, proposal.tolerance, proposal.operations.size(),
                                            0, cancelled, {});
    checkCancelled(cancelled);
    if (!fit || fit->quality.p95Distance > proposal.tolerance ||
        fit->quality.meanNormalErrorDegrees > 20)
        return {};

    // Observations inside the proposed solid contradict material inferred from
    // the caps, including observations that belong to another open group.
    Bnd_Box bounds;
    BRepBndLib::Add(shape, bounds, false);
    BRepClass3d_SolidClassifier classifier(shape);
    std::size_t inspected = 0;
    for (const auto& point : work.points) {
        if (inspected++ % 256 == 0) checkCancelled(cancelled);
        const gp_Pnt position(point.position.x, point.position.y, point.position.z);
        if (bounds.IsOut(position)) continue;
        classifier.Perform(position, proposal.tolerance);
        if (classifier.State() == TopAbs_IN) return {};
    }
    return fit->quality.surfaceInlierFraction;
}

void publish(AssemblyReconstructionResult& result, Proposal& proposal, double confidence,
             std::vector<bool>& consumed) {
    auto& part = result.parts[proposal.parts.front()];
    for (std::size_t i = 1; i < proposal.parts.size(); ++i) {
        const auto index = proposal.parts[i];
        const auto& donor = result.parts[index];
        part.patchIds.insert(part.patchIds.end(), donor.patchIds.begin(), donor.patchIds.end());
        part.pointIds.insert(part.pointIds.end(), donor.pointIds.begin(), donor.pointIds.end());
        part.workingPointIds.insert(part.workingPointIds.end(), donor.workingPointIds.begin(), donor.workingPointIds.end());
        part.lower = {std::min(part.lower.x, donor.lower.x), std::min(part.lower.y, donor.lower.y),
                      std::min(part.lower.z, donor.lower.z)};
        part.upper = {std::max(part.upper.x, donor.upper.x), std::max(part.upper.y, donor.upper.y),
                      std::max(part.upper.z, donor.upper.z)};
        consumed[index] = true;
    }
    const bool recess = proposal.operations.size() > 1;
    part.closed = true;
    part.openEdges = part.nonmanifoldEdges = 0;
    part.orientationKnown = proposal.orientationKnown;
    part.normalOrientationInferred = !proposal.orientationKnown;
    part.confidence = confidence;
    part.status = recess ? "closed_inferred_recess" : "closed_inferred_extrusion";
    part.reason = recess ? "Matched the opening and floor contours within a paired host"
                         : "Matched opposing planar contours and inferred the side walls";
    result.candidate.projectionPrograms.push_back(std::move(proposal.operations));
}
} // namespace

void recoverPlanarContours(const WorkingData& work, const AssemblyOptions& options,
                           AssemblyReconstructionResult& result, const std::function<bool()>& cancelled) {
    if (!options.buildSolids) return;
    const auto sections = planarSections(result);
    if (sections.size() < 2 || sections.size() > options.maximumClosurePatches) return;

    std::vector<std::vector<Recess>> recesses(sections.size());
    std::vector<std::size_t> floorUses(sections.size());
    for (std::size_t i = 0; i < sections.size(); ++i) {
        checkCancelled(cancelled);
        if (sections[i].patch->loops.size() != 2) continue;
        for (std::size_t j = 0; j < sections.size(); ++j) {
            if (i == j) continue;
            if (auto match = matchRecess(sections[i], sections[j], j, work.tolerance)) {
                recesses[i].push_back(*match);
                ++floorUses[j];
            }
        }
    }

    std::vector<Proposal> proposals;
    std::vector<std::size_t> uses(result.parts.size());
    for (std::size_t i = 0; i < sections.size(); ++i) {
        checkCancelled(cancelled);
        const auto& front = sections[i];
        const bool hasHole = front.patch->loops.size() == 2;
        const Section* floor = nullptr;
        double recessTolerance = 0;
        if (hasHole) {
            if (recesses[i].size() != 1 || floorUses[recesses[i].front().floor] != 1) continue;
            const auto match = recesses[i].front();
            floor = &sections[match.floor];
            recessTolerance = match.tolerance;
            for (const auto index : {front.part, floor->part}) {
                auto& part = result.parts[index];
                part.status = "open_matched_recess";
                part.reason = "Matched hole patch " + std::to_string(front.patch->id) +
                    " to floor patch " + std::to_string(floor->patch->id) +
                    "; a consistent closed host has yet to be verified";
            }
        } else if (floorUses[i]) continue;

        for (std::size_t j = hasHole ? 0 : i + 1; j < sections.size(); ++j) {
            const auto& back = sections[j];
            if (i == j || floorUses[j] || back.patch->loops.size() != 1) continue;
            if (auto proposal = matchCaps(front, back, floor, recessTolerance, work.tolerance, cancelled)) {
                for (const auto part : proposal->parts) ++uses[part];
                proposals.push_back(std::move(*proposal));
            }
        }
    }
    if (proposals.empty()) return;

    std::unordered_map<PointId, const PointSample*> points;
    points.reserve(work.points.size());
    for (const auto& point : work.points) points.emplace(point.id, &point);

    std::vector<bool> consumed(result.parts.size());
    bool published = false;
    for (auto& proposal : proposals) {
        checkCancelled(cancelled);
        if (std::any_of(proposal.parts.begin(), proposal.parts.end(), [&](std::size_t part) {
                return uses[part] != 1;
            }))
            continue;
        if (const auto confidence = validate(proposal, work, result.parts, points, cancelled)) {
            publish(result, proposal, *confidence, consumed);
            published = true;
        }
    }
    if (!published) return;
    std::size_t next = 0;
    std::size_t covered = 0;
    for (std::size_t i = 0; i < result.parts.size(); ++i) {
        if (consumed[i]) continue;
        if (result.parts[i].closed) covered += result.parts[i].pointIds.size();
        if (next != i) result.parts[next] = std::move(result.parts[i]);
        ++next;
    }
    result.parts.resize(next);
    result.candidate.surfaceCoverage = result.candidate.confidence =
        double(covered) / std::max<std::size_t>(1, result.inputPoints);
}
} // namespace reconstruction::detail::assembly
