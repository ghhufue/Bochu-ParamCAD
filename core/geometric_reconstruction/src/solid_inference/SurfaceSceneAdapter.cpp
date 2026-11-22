#include "reconstruction/solid_inference/SurfaceScene.h"

#include "reconstruction/surface/SurfaceFootprint.h"
#include "reconstruction/surface/SurfaceSupportValidation.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <optional>
#include <set>

namespace reconstruction {
namespace {
std::uint64_t hashBytes(std::uint64_t hash, const void* data, std::size_t size) noexcept {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

std::optional<std::vector<BoundaryLoop3d>> footprintLoops(const SurfaceFootprint& footprint) {
    const auto finiteVector = [](modeling::Vec3 value) {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    };
    if (!finiteVector(footprint.origin) || !finiteVector(footprint.axisU) ||
        !finiteVector(footprint.axisV) || footprint.triangles.empty()) return std::nullopt;
    for (const auto& position : footprint.positions)
        if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2]))
            return std::nullopt;
    using Edge = std::pair<std::uint64_t, std::uint64_t>;
    std::map<Edge, std::size_t> uses;
    for (const auto& triangle : footprint.triangles) {
        if (triangle[0] >= footprint.positions.size() || triangle[1] >= footprint.positions.size() ||
            triangle[2] >= footprint.positions.size() || triangle[0] == triangle[1] ||
            triangle[1] == triangle[2] || triangle[2] == triangle[0]) return std::nullopt;
        for (int i = 0; i < 3; ++i) {
            auto a = triangle[std::size_t(i)], b = triangle[std::size_t((i + 1) % 3)];
            if (a > b) std::swap(a, b);
            ++uses[{a, b}];
        }
    }
    std::map<std::uint64_t, std::vector<std::uint64_t>> adjacency;
    for (const auto& entry : uses) {
        if (entry.second > 2) return std::nullopt;
        if (entry.second == 1) {
            adjacency[entry.first.first].push_back(entry.first.second);
            adjacency[entry.first.second].push_back(entry.first.first);
        }
    }
    std::set<Edge> visited;
    std::vector<BoundaryLoop3d> loops;
    for (const auto& entry : adjacency) for (auto next : entry.second) {
        auto first = entry.first, previous = entry.first, current = next;
        auto key = std::minmax(previous, current);
        if (visited.count(key)) continue;
        BoundaryLoop3d loop;
        loop.role = SceneLoopRole::Unknown;
        loop.closed = true;
        loop.confidence = .5;
        std::size_t guard = 0;
        bool returnedToStart = false;
        while (++guard <= adjacency.size() + 1) {
            if (current == first && !loop.vertices.empty()) {
                const auto& p = footprint.positions.at(std::size_t(current));
                loop.vertices.push_back({footprint.origin.x + footprint.axisU.x * p[0] + footprint.axisV.x * p[1],
                                         footprint.origin.y + footprint.axisU.y * p[0] + footprint.axisV.y * p[1],
                                         footprint.origin.z + footprint.axisU.z * p[0] + footprint.axisV.z * p[1]});
                visited.insert(std::minmax(previous, current));
                returnedToStart = true;
                break;
            }
            const auto& p = footprint.positions.at(std::size_t(current));
            loop.vertices.push_back({footprint.origin.x + footprint.axisU.x * p[0] + footprint.axisV.x * p[1],
                                     footprint.origin.y + footprint.axisU.y * p[0] + footprint.axisV.y * p[1],
                                     footprint.origin.z + footprint.axisU.z * p[0] + footprint.axisV.z * p[1]});
            visited.insert(std::minmax(previous, current));
            const auto adjacent = adjacency.find(current);
            if (adjacent == adjacency.end() || adjacent->second.size() != 2) { loop.closed = false; break; }
            const auto candidate = adjacent->second[0] == previous ? adjacent->second[1] : adjacent->second[0];
            previous = current;
            current = candidate;
        }
        loop.closed = returnedToStart;
        if (loop.vertices.size() >= 3) loops.push_back(std::move(loop));
    }
    if (loops.empty() || std::any_of(loops.begin(), loops.end(), [](const auto& loop) { return !loop.closed; }))
        return std::nullopt;
    return loops;
}

double loopArea(const BoundaryLoop3d& loop, modeling::Vec3 origin, modeling::Vec3 u, modeling::Vec3 v) {
    double area = 0;
    for (std::size_t i=0;i<loop.vertices.size();++i) {
        const auto project=[&](modeling::Vec3 p) {
            const auto d=modeling::Vec3{p.x-origin.x,p.y-origin.y,p.z-origin.z};
            return std::array<double,2>{d.x*u.x+d.y*u.y+d.z*u.z,d.x*v.x+d.y*v.y+d.z*v.z};
        };
        const auto a=project(loop.vertices[i]),b=project(loop.vertices[(i+1)%loop.vertices.size()]);
        area+=a[0]*b[1]-b[0]*a[1];
    }
    return area*.5;
}

}

std::uint64_t surfaceSceneSettingsFingerprint(const SurfaceAnalysisOptions& options) noexcept {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto add = [&](const auto& value) { hash = hashBytes(hash, &value, sizeof(value)); };
    const auto& d = options.detection;
    for (double value : {double(d.randomSeed), double(d.detectPlanes), double(d.detectCylinders),
        double(d.detectSpheres), double(d.detectCones), double(d.detectTori), options.connectivityDistance,
        double(options.independentFamilies), double(options.recoverResidualSurfaces),
        double(options.expandPlaneSupport), options.relations.distanceTolerance,
        options.relations.angularToleranceRadians}) add(value);
    const auto addDetection = [&](const auto& parameters, double maximum) {
        for (double value : {parameters.distanceThreshold, double(parameters.minimumSupportPoints), maximum,
            parameters.probability, parameters.minimumNormalAlignment, parameters.clusterEpsilon}) add(value);
    };
    addDetection(d.plane, double(d.plane.maximumPlanes));
    addDetection(d.cylinder, double(d.cylinder.maximumCylinders));
    addDetection(d.sphere, double(d.sphere.maximumSpheres));
    addDetection(d.cone, double(d.cone.maximumCones));
    addDetection(d.torus, double(d.torus.maximumTori));
    for (double value : {d.plane.duplicatePlaneAngularToleranceRadians, d.plane.duplicatePlaneDistanceMultiplier,
        d.plane.mergeAngularToleranceRadians, d.plane.mergeResidualToleranceMultiplier,
        d.cylinder.minimumRadius, d.cylinder.maximumRadius, d.sphere.minimumRadius, d.sphere.maximumRadius,
        d.cone.minimumOpeningAngleRadians, d.cone.maximumOpeningAngleRadians,
        d.torus.minimumMajorRadius, d.torus.maximumMajorRadius, d.torus.minimumMinorRadius,
        d.torus.maximumMinorRadius}) add(value);
    for (double value : surfaceSupportSettingsSignature(options)) add(value);
    return hash;
}

bool exportSurfaceScene(const PointStore& points, const SurfaceAnalysisResult& analysis,
                        SurfaceScene& output, std::string& error,
                        const PointStore* sourceObservations) {
    output = {};
    if (!analysisMatches(points, analysis)) { error = "SnapshotMismatch: surface analysis does not match observations"; return false; }
    if (points.unit() != LengthUnit::Millimeter) { error = "WrongUnit: solid inference requires millimetres"; return false; }
    if (sourceObservations && sourceObservations->unit() != points.unit()) {
        error = "WrongUnit: source and working observations use different units";
        return false;
    }
    SurfaceScene scene;
    scene.header.unit = points.unit();
    scene.header.analysisFingerprint = analysis.pointFingerprint;
    scene.header.sourceFingerprint = sourceObservations
        ? pointStoreFingerprint(*sourceObservations)
        : analysis.sourceSupportValidation.pointFingerprint;
    scene.header.settingsFingerprint = surfaceSceneSettingsFingerprint(analysis.options);
    scene.header.distanceToleranceMm = analysis.options.relations.distanceTolerance;
    scene.header.angularToleranceRad = analysis.options.relations.angularToleranceRadians;
    scene.surfaces = analysis.surfaces;
    scene.unassignedPointIds = analysis.unassignedPointIds;

    std::set<PointId> observationIds;
    for (const auto& point : points.points()) observationIds.insert(point.id);
    for (const auto& patch : analysis.patches)
        for (const auto id : patch.supportPointIds)
            if (!observationIds.count(id)) {
                error = "MappingAmbiguous: patch support references a point outside the analysis snapshot";
                return false;
            }

    const SurfaceSupportValidationReport* support = nullptr;
    if (analysis.supportValidation.pointFingerprint == analysis.pointFingerprint &&
        analysis.supportValidation.unit == points.unit() &&
        analysis.supportValidation.settingsSignature == surfaceSupportSettingsSignature(analysis.options))
        support = &analysis.supportValidation;
    const auto& sourceSupport = analysis.sourceSupportValidation;
    const bool sourceReportCurrent = sourceSupport.pointFingerprint != 0 &&
        sourceSupport.unit == points.unit() &&
        sourceSupport.settingsSignature == surfaceSupportSettingsSignature(analysis.options) &&
        (!analysis.recoverySourceFingerprint || analysis.recoverySourceFingerprint == sourceSupport.pointFingerprint) &&
        (!sourceObservations || pointStoreFingerprint(*sourceObservations) == sourceSupport.pointFingerprint);
    std::set<PointId> sourceObservationIds;
    if (sourceObservations)
        for (const auto& point : sourceObservations->points()) sourceObservationIds.insert(point.id);

    const auto sourceDecisionForPatch = [&](const SurfacePatch& patch) -> const SurfaceSupportDecision* {
        std::vector<const SurfaceSupportDecision*> matches;
        for (const auto& decision : sourceSupport.decisions) {
            if (decision.verdict != SupportVerdict::Accepted || !decision.footprint ||
                decision.supportPointIds.empty()) continue;
            const std::set<PointId> decisionIds(decision.supportPointIds.begin(), decision.supportPointIds.end());
            if (!std::all_of(patch.supportPointIds.begin(), patch.supportPointIds.end(),
                [&](PointId id) { return decisionIds.count(id) != 0; })) continue;
            if (sourceObservations && !std::all_of(decisionIds.begin(), decisionIds.end(),
                [&](PointId id) { return sourceObservationIds.count(id) != 0; })) continue;
            const auto containsOtherPatch = std::any_of(analysis.patches.begin(), analysis.patches.end(),
                [&](const SurfacePatch& other) {
                    return other.id != patch.id && !other.supportPointIds.empty() &&
                        std::all_of(other.supportPointIds.begin(), other.supportPointIds.end(),
                            [&](PointId id) { return decisionIds.count(id) != 0; });
                });
            if (!containsOtherPatch) matches.push_back(&decision);
        }
        return matches.size() == 1 ? matches.front() : nullptr;
    };
    for (const auto& patch : analysis.patches) {
        if (!patch.surfaceId || patch.surfaceId > analysis.surfaces.size()) {
            error = "InvalidSurfaceReference: patch references missing fitted surface"; return false;
        }
        ScenePatch converted;
        converted.id = patch.id;
        converted.surfaceId = patch.surfaceId;
        converted.supportPointIds = patch.supportPointIds;
        converted.centroid = patch.centroid;
        converted.materialNormal = patch.materialNormal;
        converted.orientationKnown = patch.orientationKnown;
        const auto& evidence = analysis.surfaces[std::size_t(patch.surfaceId - 1)];
        converted.supportConfidence = std::visit([](const auto& item) { return item.confidence; }, evidence);
        converted.residualMm = std::visit([](const auto& item) { return item.meanAbsoluteResidual; }, evidence);
        if (support && std::isfinite(support->reference.spacing))
            converted.spacingMm = support->reference.spacing;
        const SurfaceSupportDecision* decision = nullptr;
        if (sourceReportCurrent) {
            decision = sourceDecisionForPatch(patch);
        } else if (support) {
            const auto found = std::find_if(support->decisions.begin(), support->decisions.end(),
                [&](const auto& item) {
                    if (item.patchId != patch.id || item.verdict != SupportVerdict::Accepted ||
                        !item.footprint || item.supportPointIds.empty()) return false;
                    const std::set<PointId> supportIds(item.supportPointIds.begin(), item.supportPointIds.end());
                    return std::all_of(patch.supportPointIds.begin(), patch.supportPointIds.end(),
                        [&](PointId id) { return supportIds.count(id) != 0; });
                });
            if (found != support->decisions.end()) decision = &*found;
        }
        if (decision && std::holds_alternative<PlaneEvidence>(evidence)) {
                const auto extractedLoops = footprintLoops(*decision->footprint);
                if (extractedLoops) {
                    converted.loops = *extractedLoops;
                }
                const auto normal=modeling::Vec3{std::get<PlaneEvidence>(evidence).plane.normal.x,
                    std::get<PlaneEvidence>(evidence).plane.normal.y,std::get<PlaneEvidence>(evidence).plane.normal.z};
                const double normalLength=std::hypot(normal.x,normal.y,normal.z);
                if(normalLength>1e-12 && extractedLoops) {
                    const auto n=modeling::Vec3{normal.x/normalLength,normal.y/normalLength,normal.z/normalLength};
                    const auto u=decision->footprint->axisU,v=decision->footprint->axisV;
                    const auto projected=[&](modeling::Vec3 p){const auto r=modeling::Vec3{
                        p.x-decision->footprint->origin.x,p.y-decision->footprint->origin.y,p.z-decision->footprint->origin.z};
                        return std::array<double,2>{r.x*u.x+r.y*u.y+r.z*u.z,r.x*v.x+r.y*v.y+r.z*v.z};};
                    std::vector<double> areas;
                    for(const auto& loop:converted.loops)areas.push_back(loopArea(loop,decision->footprint->origin,u,v));
                    bool loopsAreClosed = !converted.loops.empty();
                    std::size_t outerLoopCount = 0;
                    for(std::size_t i=0;i<converted.loops.size();++i) {
                        if (!converted.loops[i].closed) {
                            loopsAreClosed = false;
                            continue;
                        }
                        if(!converted.loops[i].closed||std::abs(areas[i])<1e-12)continue;
                        std::size_t depth=0;
                        const auto q=projected(converted.loops[i].vertices.front());
                        for(std::size_t j=0;j<converted.loops.size();++j)if(i!=j) {
                            const auto& other=converted.loops[j];bool inside=false;
                            for(std::size_t a=0,b=other.vertices.size()-1;a<other.vertices.size();b=a++) {
                                const auto pa=projected(other.vertices[a]),pb=projected(other.vertices[b]);
                                if(((pa[1]>q[1])!=(pb[1]>q[1])) && q[0]<(pb[0]-pa[0])*(q[1]-pa[1])/(pb[1]-pa[1])+pa[0])inside=!inside;
                            }
                            if(inside&&std::abs(areas[j])>std::abs(areas[i])+1e-12)++depth;
                        }
                        if (depth == 0) {
                            converted.loops[i].role = SceneLoopRole::Outer;
                            ++outerLoopCount;
                        } else if (depth % 2 == 1) {
                            converted.loops[i].role = SceneLoopRole::Hole;
                        } else {
                            converted.loops[i].role = SceneLoopRole::Island;
                        }
                    }
                    const double footprintOrientation =
                        decision->footprint->axisU.x * (decision->footprint->axisV.y * n.z - decision->footprint->axisV.z * n.y) +
                        decision->footprint->axisU.y * (decision->footprint->axisV.z * n.x - decision->footprint->axisV.x * n.z) +
                        decision->footprint->axisU.z * (decision->footprint->axisV.x * n.y - decision->footprint->axisV.y * n.x);
                    if (std::abs(footprintOrientation) > 1e-9) {
                        for (std::size_t i = 0; i < converted.loops.size(); ++i) {
                            const auto role = converted.loops[i].role;
                            if (role != SceneLoopRole::Outer && role != SceneLoopRole::Hole) continue;
                            const bool shouldBePositive = (role == SceneLoopRole::Outer) ==
                                (footprintOrientation > 0);
                            if ((areas[i] > 0) != shouldBePositive)
                                std::reverse(converted.loops[i].vertices.begin(), converted.loops[i].vertices.end());
                        }
                    } else {
                        loopsAreClosed = false;
                    }
                    const bool supportedSimpleDomain = loopsAreClosed && outerLoopCount == 1 &&
                        std::all_of(converted.loops.begin(), converted.loops.end(), [](const auto& loop) {
                            return loop.role == SceneLoopRole::Outer || loop.role == SceneLoopRole::Hole;
                        });
                    if (supportedSimpleDomain)
                        converted.boundaryStatus = SceneBoundaryStatus::ObservedClosed;
                }
        }
        scene.patches.push_back(std::move(converted));
    }
    for (const auto& relation : analysis.relations) {
        scene.relations.push_back({relation.first, relation.second, relation.kind,
                                   relation.error, relation.confidence, !relation.automatic});
    }
    const auto checked = validateSurfaceScene(scene, analysis.pointFingerprint);
    if (!checked) { error = checked.message; return false; }
    output = std::move(scene);
    error.clear();
    return true;
}

SurfaceSceneValidation validateSurfaceScene(const SurfaceScene& scene, std::uint64_t expectedFingerprint) {
    const auto fail = [](SurfaceSceneError code, PatchId patch, const char* message) {
        return SurfaceSceneValidation{code, patch, message};
    };
    if (scene.header.schemaVersion != 1) return fail(SurfaceSceneError::SnapshotMismatch, 0, "Unsupported SurfaceScene schema");
    if (scene.header.unit != LengthUnit::Millimeter) return fail(SurfaceSceneError::WrongUnit, 0, "SurfaceScene coordinates must be millimetres");
    if (!scene.header.analysisFingerprint || (expectedFingerprint && scene.header.analysisFingerprint != expectedFingerprint))
        return fail(SurfaceSceneError::SnapshotMismatch, 0, "SurfaceScene analysis fingerprint mismatch");
    if (!std::isfinite(scene.header.distanceToleranceMm) || scene.header.distanceToleranceMm <= 0 ||
        !std::isfinite(scene.header.angularToleranceRad) || scene.header.angularToleranceRad <= 0)
        return fail(SurfaceSceneError::SnapshotMismatch, 0, "Invalid SurfaceScene tolerances");
    std::set<PatchId> ids;
    for (const auto& patch : scene.patches) {
        if (!patch.id || !ids.insert(patch.id).second || !patch.surfaceId || patch.surfaceId > scene.surfaces.size())
            return fail(SurfaceSceneError::InvalidSurfaceReference, patch.id, "Patch has a duplicate ID or missing fitted surface");
        if (!std::isfinite(patch.supportConfidence) || patch.supportConfidence < 0 || patch.supportConfidence > 1 ||
            !std::isfinite(patch.residualMm) || patch.residualMm < 0 ||
            !std::isfinite(patch.spacingMm) || patch.spacingMm < 0 ||
            !std::isfinite(patch.centroid.x) || !std::isfinite(patch.centroid.y) ||
            !std::isfinite(patch.centroid.z) || !std::isfinite(patch.materialNormal.x) ||
            !std::isfinite(patch.materialNormal.y) || !std::isfinite(patch.materialNormal.z))
            return fail(SurfaceSceneError::InvalidSurfaceReference, patch.id, "Patch support metrics are invalid");
        if (patch.boundaryStatus == SceneBoundaryStatus::ObservedClosed && patch.loops.empty())
            return fail(SurfaceSceneError::MissingBoundary, patch.id, "Closed boundary status requires boundary loops");
        std::set<PointId> supportIds;
        for (const auto id : patch.supportPointIds)
            if (!id || !supportIds.insert(id).second)
                return fail(SurfaceSceneError::InvalidSurfaceReference, patch.id,
                            "Patch support point IDs must be nonzero and unique");
        for (const auto& loop : patch.loops) {
            const std::size_t minimumVertices = loop.closed ? 3 : 2;
            if ((patch.boundaryStatus == SceneBoundaryStatus::ObservedClosed && !loop.closed) ||
                loop.vertices.size() < minimumVertices || !std::isfinite(loop.confidence) ||
                loop.confidence < 0 || loop.confidence > 1)
                return fail(SurfaceSceneError::InvalidLoop, patch.id, "Boundary loop is open or malformed");
            for (const auto& p : loop.vertices)
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                    return fail(SurfaceSceneError::InvalidLoop, patch.id, "Boundary loop contains nonfinite coordinates");
        }
    }
    for (const auto id : scene.unassignedPointIds)
        if (!id) return fail(SurfaceSceneError::InvalidSurfaceReference, 0,
                             "Unassigned point IDs must be nonzero");
    for (const auto& relation : scene.relations)
        if (!ids.count(relation.first) || !ids.count(relation.second) || relation.first == relation.second ||
            !std::isfinite(relation.confidence) || relation.confidence < 0 || relation.confidence > 1 ||
            !std::isfinite(relation.error) || relation.error < 0)
            return fail(SurfaceSceneError::InvalidSurfaceReference, relation.first, "Relation references an invalid patch or metric");
    return {};
}
} // namespace reconstruction
