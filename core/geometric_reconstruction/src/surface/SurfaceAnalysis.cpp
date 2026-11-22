#include "detail/AnalysisInternal.h"
#include "reconstruction/segmentation/MultiScaleDetector.h"
#include "reconstruction/surface/PlaneSupportRecovery.h"
#include "reconstruction/data/StageProfile.h"
#include "common/Parallel.h"
#include <CGAL/config.h>
#include <iterator>
#include <mutex>

namespace reconstruction {
using namespace detail::analysis;

bool collectSurfaceEvidence(const PointStore& points, const SurfaceAnalysisOptions& options,
                     SurfaceEvidenceCollection& output, std::string& error,
                     const std::vector<PrimitiveEvidence>& seeds, const std::function<bool()>& cancelled) {
    output = {}; error.clear();
    if (!valid(options)) { error = "Invalid surface analysis options"; return false; }
    std::vector<PrimitiveEvidence> evidence;
    std::size_t passes = 0;
    const auto& d = options.detection;
    const std::array<bool,5> enabled{d.detectPlanes,d.detectCylinders,d.detectSpheres,d.detectCones,d.detectTori};
    if (std::none_of(enabled.begin(),enabled.end(),[](bool x){return x;})) { error = "No surface family enabled"; return false; }
    std::string failure;
    static constexpr const char* familyStages[] = {"detect.plane", "detect.cylinder", "detect.sphere", "detect.cone", "detect.torus"};
    if(options.recoverResidualSurfaces) {
        PrimitiveDetectionResult result;MultiScaleDetectionReport report;
        if (!diagnostics::profileStage("detect.multiscale", [&] {
            return detectMultiScalePrimitiveEvidence(points, d, result, report, failure, cancelled);
        })) {
            if(failure=="Cancelled"){error=failure;return false;}
        }
        evidence=std::move(result.evidence);passes=report.detectionPasses;
    } else if (!options.independentFamilies) {
        if (cancelled && cancelled()) { error = "Cancelled"; return false; }
        PrimitiveDetectionResult result;
        passes = 1;
        if (diagnostics::profileStage("detect.combined", [&] {
            return detectPrimitiveEvidence(points, d, result, failure);
        })) evidence = std::move(result.evidence);
    } else {
        std::vector<std::size_t> families;
        for (std::size_t family = 0; family < enabled.size(); ++family)
            if (enabled[family]) families.push_back(family);
        std::size_t workers = 1;
#ifdef CGAL_HAS_THREADS
        if (options.familyDetectionWorkerCount || points.size() >= 4096)
            workers = options.familyDetectionWorkerCount;
#endif
        struct FamilyResult {
            PrimitiveDetectionResult detection;
            std::string failure;
            bool found = false;
        };
        std::vector<FamilyResult> results(families.size());
        std::mutex cancellationMutex;
        bool stopped = false;
        diagnostics::profileStage("detect.independent_families", [&] {
            detail::parallelTasks(families.size(), workers, [&](std::size_t index) {
                if (cancelled) {
                    const std::lock_guard<std::mutex> lock(cancellationMutex);
                    if (!stopped) stopped = cancelled();
                    if (stopped) return;
                }
                const auto family = families[index];
                auto detection = d;
                detection.detectPlanes = family == 0;
                detection.detectCylinders = family == 1;
                detection.detectSpheres = family == 2;
                detection.detectCones = family == 3;
                detection.detectTori = family == 4;
                auto& result = results[index];
                result.found = diagnostics::profileStage(familyStages[family], [&] {
                    return detectPrimitiveEvidence(points, detection, result.detection, result.failure);
                });
            });
        });
        if (stopped || (cancelled && cancelled())) { error = "Cancelled"; return false; }
        passes = families.size();
        for (auto& result : results) {
            failure = std::move(result.failure);
            if (result.found) {
                auto& proposals = result.detection.evidence;
                evidence.insert(evidence.end(), std::make_move_iterator(proposals.begin()),
                    std::make_move_iterator(proposals.end()));
            }
        }
    }
    if(options.expandPlaneSupport) {
        std::vector<PointId> protectedIds;
        for(const auto& seed:seeds) protectedIds.insert(protectedIds.end(),ids(seed).begin(),ids(seed).end());
        if (!diagnostics::profileStage("surface.expand_planes", [&] {
            return recoverPlaneSupport(points, d.plane, evidence, error, protectedIds, cancelled);
        })) return false;
    }
    // Manual evidence replaces only same-family observations with substantial
    // source overlap, never unrelated nearby surfaces sharing an equation.
    for (const auto& seed : seeds) {
        std::unordered_set<PointId> fixed(ids(seed).begin(),ids(seed).end());
        evidence.erase(std::remove_if(evidence.begin(),evidence.end(),[&](const auto& e) {
            if (primitiveKind(e) != primitiveKind(seed)) return false;
            std::size_t shared = 0; for (auto id : ids(e)) shared += fixed.count(id);
            return shared > 0 && ids(e).size() <= fixed.size()*1.25 &&
                shared >= 0.7*std::min(ids(e).size(),fixed.size());
        }),evidence.end());
        evidence.push_back(seed);
    }
    if (cancelled && cancelled()) { error = "Cancelled"; return false; }
    if (evidence.empty()) { error = failure.empty() ? "No supported surface evidence" : failure; return false; }
    std::unordered_set<PointId> covered;
    for (const auto& entry : evidence)
        covered.insert(ids(entry).begin(), ids(entry).end());
    for (const auto& point : points.points())
        if (!covered.count(point.id)) output.unassignedPointIds.push_back(point.id);
    output.evidence = std::move(evidence);
    output.detectionPasses = passes;
    return true;
}

void reportSurfaceAnalysis(const SurfaceAnalysisResult& analysis, const SurfaceAnalysisProgress& progress) {
    if (!progress.patch) return;
    if (progress.reset) progress.reset();
    for (const auto& patch : analysis.patches)
        progress.patch(analysis.surfaces.at(patch.surfaceId - 1), patch.supportPointIds, nullptr);
}

bool analyzeSurfaces(const PointStore& points, const SurfaceAnalysisOptions& options,
    SurfaceAnalysisResult& output, std::string& error,
    const std::vector<PrimitiveEvidence>& seeds, const std::function<bool()>& cancelled,
    const SurfaceAnalysisProgress& progress) {
    output = {};
    SurfaceEvidenceCollection collected;
    if (!diagnostics::profileStage("surface.collect_evidence", [&] {
        return collectSurfaceEvidence(points, options, collected, error, seeds, cancelled);
    })) return false;
    std::vector<PointId> protectedIds;
    std::vector<ProtectedSurfaceSeed> protectedSeeds;
    for(const auto& seed:seeds) {
        const auto& support=ids(seed);
        protectedIds.insert(protectedIds.end(),support.begin(),support.end());
        protectedSeeds.push_back({primitiveKind(seed),support});
    }
    if (!diagnostics::profileStage("surface.validate_and_index", [&] {
        return indexDetectedSurfaceEvidence(points, collected.evidence, options, output, error,
            protectedIds, cancelled, protectedSeeds, progress);
    })) return false;
    output.detectionPasses = collected.detectionPasses;
    return true;
}

} // namespace reconstruction
