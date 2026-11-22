#include "detail/AnalysisInternal.h"
#include "common/spatial/KdTree.h"
#include "reconstruction/data/StageProfile.h"
#include "reconstruction/surface/SurfaceSupportValidation.h"
#include <iterator>

namespace reconstruction::detail::analysis {
namespace {

constexpr std::size_t maximumRegionPoints = 8192;

struct PlaneFragment {
    Vec3 normal, lower, upper;
    std::vector<const PointSample*> points;
};

bool adjacent(const PlaneFragment& a, const PlaneFragment& b, const Grid& grid, double reach,
              double parallelCosine) {
    const double alignment = std::abs(dot(a.normal, b.normal));
    if (alignment < .5 || alignment >= parallelCosine) return false;
    if (a.lower.x > b.upper.x + reach || b.lower.x > a.upper.x + reach ||
        a.lower.y > b.upper.y + reach || b.lower.y > a.upper.y + reach ||
        a.lower.z > b.upper.z + reach || b.lower.z > a.upper.z + reach) return false;

    // A single near point may be an isolated scan artifact or a corner.
    std::size_t contacts = 0;
    const auto stride = std::max<std::size_t>(1, b.points.size() / 128);
    for (std::size_t i = 0; i < b.points.size(); i += stride) {
        bool found = false;
        grid.near(b.points[i]->position, [&](std::size_t j) {
            found |= a.points[j]->id != b.points[i]->id;
        });
        if (found && ++contacts >= 3) return true;
    }
    return false;
}

bool spansFragments(const std::unordered_set<PointId>& support, const std::vector<PlaneFragment>& fragments,
                    const std::vector<std::size_t>& region, std::size_t minimumPoints) {
    std::size_t count = 0;
    for (auto index : region) {
        const auto& patch = fragments[index].points;
        const auto shared = std::count_if(patch.begin(), patch.end(), [&](const PointSample* p) {
            return support.count(p->id) != 0;
        });
        // One detected plane may extend well outside this local window.
        if (shared >= std::min(.5 * patch.size(), double(minimumPoints)) && ++count >= 2) return true;
    }
    return false;
}

bool sameSphere(const SphereEvidence& a, const SphereEvidence& b, double tolerance) {
    return length(subtract(a.sphere.center, b.sphere.center)) <= tolerance &&
        std::abs(a.sphere.radius - b.sphere.radius) <= tolerance;
}

bool extendSphere(PrimitiveEvidence& target, const SphereEvidence& addition,
                  const std::unordered_map<PointId, const PointSample*>& observations,
                  const SphereDetectionOptions& options) {
    auto& sphere = std::get<SphereEvidence>(target);
    auto& support = sphere.supportPointIds;
    const auto before = support.size();
    std::unordered_set<PointId> seen(support.begin(), support.end());
    for (auto id : addition.supportPointIds) {
        const auto& point = *observations.at(id);
        if (surfaceResidual(target, point.position) <= options.distanceThreshold &&
            (!point.normal || surfaceNormalAlignment(target, point.position, *point.normal) >=
                options.minimumNormalAlignment) && seen.insert(id).second)
            support.push_back(id);
    }
    if (support.size() == before) return false;
    std::sort(support.begin(), support.end());
    return true;
}

double regionCost(const PrimitiveEvidence& candidate, const PointStore& region,
                  const SurfaceAnalysisOptions& options) {
    const bool sphere = primitiveKind(candidate) == PrimitiveKind::Sphere;
    const auto& detection = options.detection;
    const double tolerance = detection.detectSpheres && detection.detectCones ?
        std::min(detection.sphere.distanceThreshold, detection.cone.distanceThreshold) :
        (sphere ? detection.sphere.distanceThreshold : detection.cone.distanceThreshold);
    const double cutoff = 2 * tolerance;
    double sum = 0;
    for (const auto& point : region.points()) {
        const double residual = std::min(surfaceResidual(candidate, point.position), cutoff) / tolerance;
        sum += residual * residual;
    }
    const double penalty = options.supportValidation.modelComplexityWeight * std::log(region.size());
    return (sum + (sphere ? 4 : 6) * penalty) / region.size();
}

bool fitRegion(const PointStore& local, const std::vector<PlaneFragment>& fragments,
               const std::vector<std::size_t>& region, const SurfaceAnalysisOptions& options,
               const SurfaceSamplingReference& reference, double reach,
               std::vector<PrimitiveEvidence>& recovered, std::string& error,
               const std::function<bool()>& cancelled) {
    auto detection = options.detection;
    detection.detectPlanes = detection.detectCylinders = detection.detectTori = false;
    std::vector<PrimitiveEvidence> candidates;
    const auto fragmentSupport = options.supportMeshing.minimumSurfacePoints * 2;
    for (const auto kind : {PrimitiveKind::Sphere, PrimitiveKind::Cone}) {
        const bool sphere = kind == PrimitiveKind::Sphere;
        if (!(sphere ? options.detection.detectSpheres : options.detection.detectCones)) continue;
        if (cancelled && cancelled()) { error = "Cancelled"; return false; }
        detection.detectSpheres = sphere;
        detection.detectCones = !sphere;
        detection.sphere.maximumSpheres = std::min<std::size_t>(3, options.detection.sphere.maximumSpheres);
        detection.cone.maximumCones = std::min<std::size_t>(3, options.detection.cone.maximumCones);
        if (detection.sphere.clusterEpsilon == 0) detection.sphere.clusterEpsilon = reach;
        if (detection.cone.clusterEpsilon == 0) detection.cone.clusterEpsilon = reach;
        PrimitiveDetectionResult result;
        std::string ignored;
        if (!detectPrimitiveEvidence(local, detection, result, ignored)) continue;
        for (auto& candidate : result.evidence) {
            const std::unordered_set<PointId> support(ids(candidate).begin(), ids(candidate).end());
            if (spansFragments(support, fragments, region, fragmentSupport))
                candidates.push_back(std::move(candidate));
        }
    }
    if (candidates.empty()) return true;
    std::vector<PrimitiveEvidence> accepted;
    SurfaceSupportValidationReport report;
    if (!validateSurfaceSupport(local, candidates, options, accepted, report, error, &reference, cancelled))
        return false;

    double bestCost = INFINITY;
    PrimitiveEvidence* best = nullptr;
    for (auto& candidate : accepted) {
        const std::unordered_set<PointId> support(ids(candidate).begin(), ids(candidate).end());
        if (!spansFragments(support, fragments, region, fragmentSupport)) continue;
        const double cost = regionCost(candidate, local, options);
        if (cost < bestCost) { bestCost = cost; best = &candidate; }
    }
    if (best) recovered.push_back(std::move(*best));
    return true;
}

bool recoverCurvedFragments(const PointStore& points, const std::vector<PrimitiveEvidence>& proposals,
    const std::vector<PrimitiveEvidence>& accepted, const SurfaceSupportValidationReport& report,
    const SurfaceAnalysisOptions& options, const std::vector<PointId>& protectedIds,
    std::vector<PrimitiveEvidence>& recovered, std::string& error, const std::function<bool()>& cancelled) {
    const auto& detection = options.detection;
    if (report.mode != SupportValidationMode::Enforce || !detection.detectPlanes ||
        !(detection.detectSpheres || detection.detectCones)) return true;

    double broadArea = INFINITY;
    if (report.reference.available) {
        std::vector<double> areas;
        for (const auto& decision : report.decisions)
            if (std::find(report.reference.patchIds.begin(), report.reference.patchIds.end(), decision.patchId) !=
                report.reference.patchIds.end()) areas.push_back(decision.area);
        if (!areas.empty()) {
            std::sort(areas.begin(), areas.end());
            broadArea = areas[areas.size() / 2] * options.supportValidation.planePreferenceMinReferenceAreaFraction;
        }
    }

    const auto observations = pointMap(points);
    std::unordered_set<PointId> excluded(protectedIds.begin(), protectedIds.end());
    for (const auto& decision : report.decisions) {
        if (decision.kind != PrimitiveKind::Plane || decision.verdict != SupportVerdict::Accepted ||
            !decision.footprint || decision.finalArea < broadArea ||
            decision.effectivePoints < options.supportValidation.referenceMinPoints) continue;
        const auto& covered = decision.footprint->coveredPointIds;
        excluded.insert(covered.begin(), covered.end());
    }

    std::vector<PlaneFragment> fragments;
    for (const auto& decision : report.decisions) {
        if (decision.kind != PrimitiveKind::Plane || decision.verdict != SupportVerdict::Accepted ||
            !decision.footprint || decision.protectedManual) continue;
        const auto& covered = decision.footprint->coveredPointIds;
        if (covered.size() > maximumRegionPoints / 2) continue;
        PlaneFragment fragment;
        fragment.normal = std::get<PlaneEvidence>(proposals.at(decision.proposalId - 1)).plane.normal;
        fragment.lower = {INFINITY, INFINITY, INFINITY};
        fragment.upper = {-INFINITY, -INFINITY, -INFINITY};
        for (auto id : covered) {
            if (excluded.count(id)) continue;
            const auto* point = observations.at(id);
            fragment.points.push_back(point);
            const auto p = point->position;
            fragment.lower = {std::min(fragment.lower.x, p.x), std::min(fragment.lower.y, p.y), std::min(fragment.lower.z, p.z)};
            fragment.upper = {std::max(fragment.upper.x, p.x), std::max(fragment.upper.y, p.y), std::max(fragment.upper.z, p.z)};
        }
        if (fragment.points.size() >= options.supportMeshing.minimumSurfacePoints)
            fragments.push_back(std::move(fragment));
    }
    if (fragments.size() < 2) return true;

    const double reach = 3 * std::max(detection.plane.distanceThreshold, report.reference.spacing);
    const double parallelCosine = std::cos(detection.plane.duplicatePlaneAngularToleranceRadians);
    std::vector<std::vector<std::size_t>> neighbors(fragments.size());
    for (std::size_t i = 0; i < fragments.size(); ++i) {
        if (cancelled && cancelled()) { error = "Cancelled"; return false; }
        const Grid grid(fragments[i].points, reach);
        for (std::size_t j = i + 1; j < fragments.size(); ++j) {
            if (!adjacent(fragments[i], fragments[j], grid, reach, parallelCosine)) continue;
            neighbors[i].push_back(j);
            neighbors[j].push_back(i);
        }
    }

    const spatial::KdTree tree(points.points());
    std::vector<PrimitiveEvidence> candidates;
    const auto recover = [&](const std::vector<std::size_t>& region, const PlaneFragment* anchor) {
        if (cancelled && cancelled()) { error = "Cancelled"; return false; }
        std::vector<PointSample> samples;
        if (anchor) {
            // Search around each small patch. Unassigned observations in this
            // window can carry curvature that tangent-plane seeds left out.
            const double margin = std::max(reach, .5 * length(subtract(anchor->upper, anchor->lower)));
            const Vec3 halo{margin, margin, margin};
            tree.visitBox(subtract(anchor->lower, halo), add(anchor->upper, halo), [&](const PointSample& point) {
                if (!excluded.count(point.id) && samples.size() <= maximumRegionPoints)
                    samples.push_back(point);
            });
        } else {
            std::unordered_set<PointId> seen;
            for (auto index : region)
                for (const auto* point : fragments[index].points)
                    if (seen.insert(point->id).second) samples.push_back(*point);
        }
        if (samples.size() > maximumRegionPoints || samples.size() < options.minimumRecoveryRegionPoints)
            return true;
        std::sort(samples.begin(), samples.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
        return fitRegion(PointStore(std::move(samples), points.unit()), fragments, region, options,
                         report.reference, reach, candidates, error, cancelled);
    };

    std::vector<bool> visited(fragments.size());
    for (std::size_t i = 0; i < fragments.size(); ++i) {
        if (visited[i] || neighbors[i].empty()) continue;
        std::vector<std::size_t> component{i};
        visited[i] = true;
        for (std::size_t head = 0; head < component.size(); ++head)
            for (auto j : neighbors[component[head]])
                if (!visited[j]) { visited[j] = true; component.push_back(j); }
        if (!recover(component, nullptr)) return false;
        for (auto j : component)
            if (!recover(component, &fragments[j])) return false;
    }

    // Search windows overlap. Publish the strongest validated explanation for
    // each observed patch instead of competing with several copies of it.
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return ids(a).size() > ids(b).size();
    });
    std::vector<std::pair<PrimitiveKind, std::unordered_set<PointId>>> existingSupports;
    for (const auto& surface : accepted) {
        const auto kind = primitiveKind(surface);
        if (kind == PrimitiveKind::Sphere || kind == PrimitiveKind::Cone)
            existingSupports.emplace_back(kind, std::unordered_set<PointId>(ids(surface).begin(), ids(surface).end()));
    }
    std::vector<std::unordered_set<PointId>> localSupports;
    for (auto& candidate : candidates) {
        const auto& support = ids(candidate);
        const auto overlaps = [&](const auto& other, double fraction) {
            return std::count_if(support.begin(), support.end(), [&](auto id) { return other.count(id); }) >=
                fraction * support.size();
        };
        bool represented = false;
        for (std::size_t i = 0; i < localSupports.size(); ++i) {
            if (!overlaps(localSupports[i], .5)) continue;
            represented = true;
            const auto* selected = std::get_if<SphereEvidence>(&recovered[i]);
            const auto* overlap = std::get_if<SphereEvidence>(&candidate);
            if (selected && overlap && sameSphere(*selected, *overlap, detection.sphere.distanceThreshold) &&
                extendSphere(recovered[i], *overlap, observations, detection.sphere)) {
                const auto& extended = ids(recovered[i]);
                localSupports[i].insert(extended.begin(), extended.end());
            }
            break;
        }
        if (represented) continue;
        const bool duplicate = std::any_of(existingSupports.begin(), existingSupports.end(), [&](const auto& existing) {
            return existing.first == primitiveKind(candidate) &&
                existing.second.size() >= .9 * support.size() && overlaps(existing.second, .9);
        });
        if (duplicate) continue;
        localSupports.emplace_back(support.begin(), support.end());
        recovered.push_back(std::move(candidate));
    }
    return true;
}

bool mergeRecoveredSpheres(const PointStore& points, const SphereDetectionOptions& options,
    const std::vector<PrimitiveEvidence>& accepted, std::vector<PrimitiveEvidence>& proposals,
    std::vector<PrimitiveEvidence>& recovered) {
    if (std::none_of(recovered.begin(), recovered.end(), [](const auto& surface) {
        return primitiveKind(surface) == PrimitiveKind::Sphere;
    })) return false;
    bool extended = false;
    const auto observations = pointMap(points);
    recovered.erase(std::remove_if(recovered.begin(), recovered.end(), [&](const auto& proposal) {
        const auto* addition = std::get_if<SphereEvidence>(&proposal);
        if (!addition) return false;
        for (const auto& surface : accepted) {
            const auto* existing = std::get_if<SphereEvidence>(&surface);
            if (!existing || !sameSphere(*existing, *addition, options.distanceThreshold)) continue;
            const auto& support = existing->supportPointIds;
            const std::unordered_set<PointId> members(support.begin(), support.end());
            const auto shared = std::count_if(addition->supportPointIds.begin(), addition->supportPointIds.end(),
                [&](auto id) { return members.count(id) != 0; });
            if (shared < .25 * std::min(support.size(), addition->supportPointIds.size())) continue;
            auto& target = proposals.at(existing->id - 1);
            extended |= extendSphere(target, *addition, observations, options);
            return true;
        }
        return false;
    }), recovered.end());
    return extended;
}

} // namespace

bool validateDetectedSurfaceSupport(const PointStore& points, std::vector<PrimitiveEvidence>& proposals,
    const SurfaceAnalysisOptions& options, std::vector<PrimitiveEvidence>& accepted,
    SurfaceSupportValidationReport& report, std::string& error,
    const std::vector<PointId>& protectedIds, const std::function<bool()>& cancelled,
    const std::vector<ProtectedSurfaceSeed>& protectedSeeds, const SurfaceAnalysisProgress& progress) {
    if (!validateSurfaceSupport(points, proposals, options, accepted, report, error, nullptr,
                                cancelled, protectedIds, protectedSeeds, progress)) return false;
    std::vector<PrimitiveEvidence> recovered;
    if (!diagnostics::profileStage("surface.recover_curved_fragments", [&] {
        return recoverCurvedFragments(points, proposals, accepted, report, options, protectedIds,
                                      recovered, error, cancelled);
    })) return false;
    if (recovered.empty()) return true;

    // A second pass on source resolution can extend a sphere established
    // earlier on the working cloud. Retain its proposal identity and support.
    if (mergeRecoveredSpheres(points, options.detection.sphere, accepted, proposals, recovered)) {
        proposals.insert(proposals.end(), std::make_move_iterator(recovered.begin()),
                         std::make_move_iterator(recovered.end()));
        const auto reference = report.reference;
        return validateSurfaceSupport(points, proposals, options, accepted, report, error,
                                      &reference, cancelled, protectedIds, protectedSeeds, progress);
    }
    if (recovered.empty()) return true;

    // The reference and the existing decisions are already valid. Validate
    // the additional proposals against that same reference and append IDs.
    std::vector<PrimitiveEvidence> extraAccepted;
    SurfaceSupportValidationReport extraReport;
    if (!validateSurfaceSupport(points, recovered, options, extraAccepted, extraReport, error,
                                &report.reference, cancelled, protectedIds, {}, {{}, progress.patch})) return false;
    const auto proposalOffset = proposals.size();
    PatchId patchOffset = 0;
    for (const auto& decision : report.decisions) patchOffset = std::max(patchOffset, decision.patchId);
    for (auto& decision : extraReport.decisions) {
        decision.proposalId += proposalOffset;
        decision.patchId += patchOffset;
        report.decisions.push_back(std::move(decision));
    }
    for (auto& surface : extraAccepted) {
        std::visit([&](auto& primitive) { primitive.id += proposalOffset; }, surface);
        accepted.push_back(std::move(surface));
    }
    proposals.insert(proposals.end(), std::make_move_iterator(recovered.begin()),
                     std::make_move_iterator(recovered.end()));
    return true;
}

} // namespace reconstruction::detail::analysis
