#pragma once
#include "reconstruction/surface/SurfaceAnalysis.h"

namespace reconstruction {
// Shared defaults for production recognition. Multi-family proposals are gathered
// independently, so one rejected family cannot consume another family's points.
// Quotas previously embedded in recognitionAnalysisOptions. Defaults reproduce
// the historical production behaviour; the client overrides them from
// recognition.conf.
struct RecognitionAnalysisSettings {
    std::size_t maximumPlanes = 128;
    std::size_t maximumCylinders = 32;
    std::size_t planeSupportCap = 30;
    std::size_t cylinderSupportCap = 30;
    std::size_t curvedSupportFloor = 30;
    std::size_t curvedSupportCeiling = 128;
    std::size_t curvedMaximumCandidates = 32;
    double curvedProbability = 0.001;
    double relationDistanceToleranceMultiplier = 4.0;
};
SurfaceAnalysisOptions recognitionAnalysisOptions(const PointStore& points,
                                                 const PlaneDetectionOptions& planes,
                                                 const CylinderDetectionOptions& cylinders,
                                                 const RecognitionAnalysisSettings& settings = {});
} // namespace reconstruction
