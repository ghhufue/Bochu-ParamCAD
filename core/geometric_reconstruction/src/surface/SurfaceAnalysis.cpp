#include "detail/AnalysisInternal.h"
#include "reconstruction/segmentation/MultiScaleDetector.h"
#include "reconstruction/surface/PlaneSupportRecovery.h"

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
    if(options.recoverResidualSurfaces) {
        PrimitiveDetectionResult result;MultiScaleDetectionReport report;
        if(!detectMultiScalePrimitiveEvidence(points,d,result,report,failure,cancelled)) {
            if(failure=="Cancelled"){error=failure;return false;}
        }
        evidence=std::move(result.evidence);passes=report.detectionPasses;
    } else for (int family = 0; family < (options.independentFamilies ? 5 : 1); ++family) {
        if (cancelled && cancelled()) { error = "Cancelled"; return false; }
        if (options.independentFamilies && !enabled[family]) continue;
        auto detection = d;
        if (options.independentFamilies) {
            detection.detectPlanes = family == 0; detection.detectCylinders = family == 1;
            detection.detectSpheres = family == 2; detection.detectCones = family == 3; detection.detectTori = family == 4;
        }
        PrimitiveDetectionResult result; ++passes;
        if (detectPrimitiveEvidence(points,detection,result,failure))
            evidence.insert(evidence.end(),result.evidence.begin(),result.evidence.end());
    }
    if(options.expandPlaneSupport) {
        std::vector<PointId> protectedIds;
        for(const auto& seed:seeds) protectedIds.insert(protectedIds.end(),ids(seed).begin(),ids(seed).end());
        if(!recoverPlaneSupport(points,d.plane,evidence,error,protectedIds,cancelled)) return false;
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

bool analyzeSurfaces(const PointStore& points, const SurfaceAnalysisOptions& options,
    SurfaceAnalysisResult& output, std::string& error,
    const std::vector<PrimitiveEvidence>& seeds, const std::function<bool()>& cancelled) {
    output = {};
    SurfaceEvidenceCollection collected;
    if (!collectSurfaceEvidence(points, options, collected, error, seeds, cancelled)) return false;
    std::vector<PointId> protectedIds;
    for(const auto& seed:seeds)protectedIds.insert(protectedIds.end(),ids(seed).begin(),ids(seed).end());
    if (!indexSurfaceEvidence(points, collected.evidence, options, output, error,protectedIds,cancelled)) return false;
    output.detectionPasses = collected.detectionPasses;
    return true;
}

} // namespace reconstruction
