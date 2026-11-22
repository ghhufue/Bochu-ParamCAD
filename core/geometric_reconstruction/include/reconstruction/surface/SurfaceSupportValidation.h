#pragma once
#include "reconstruction/surface/SurfaceAnalysis.h"
#include "reconstruction/surface/SurfaceFootprint.h"

namespace reconstruction {
// Transactional: neither output is published when cancelled or malformed.
bool validateSurfaceSupport(const PointStore& points,
    const std::vector<PrimitiveEvidence>& proposals, const SurfaceAnalysisOptions& options,
    std::vector<PrimitiveEvidence>& accepted, SurfaceSupportValidationReport& report,
    std::string& error, const SurfaceSamplingReference* frozenReference=nullptr,
    const std::function<bool()>& cancelled={}, const std::vector<PointId>& protectedIds={},
    const std::vector<ProtectedSurfaceSeed>& protectedSeeds={});
std::vector<double> surfaceSupportSettingsSignature(const SurfaceAnalysisOptions& options);
// Rebuild work evidence from source-validated proposals without mixing sample densities.
bool revalidateSurfaceSupportOnSource(const PointStore& working,const PointStore& source,
    SurfaceAnalysisResult& analysis,std::string& error,const std::function<bool()>& cancelled={},
    const SurfaceAnalysisResult* validatedSource=nullptr);
// Index the accepted full-resolution evidence for point ownership and display.
bool indexValidatedSourceSurfaces(const PointStore& source, const SurfaceAnalysisResult& workingAnalysis,
    SurfaceAnalysisResult& sourceIndex, std::string& error);
} // namespace reconstruction
