#pragma once
#include "reconstruction/recognition/CompoundRecognition.h"

namespace reconstruction {
struct RepairTransactionReport {
    bool committed = false, reused = false;
    double beforeCoverage = 0, afterCoverage = 0;
    std::string reason;
    std::vector<LocalAnalysisFailure> failures;
};
struct RecognitionSettings;
// Publishes an augmented analysis only after complete recognition and stability checks.
// Failure leaves the supplied analysis unchanged and clears the output candidate.
bool recognizeAndRepairParametricSolid(const PointStore& points, SurfaceAnalysisResult& analysis,
    CompoundCandidate& output, std::string& error, const PointStore* source = nullptr,
    RepairTransactionReport* report = nullptr,
    const std::vector<SurfaceRelationHint>& hints = {},
    const RecognitionSettings* settings = nullptr);
// Client and validator share this entry point. Curved-feature fallback is
// accepted only if the complete surface agrees with its predicted boundary.
bool reconstructParametricSolid(const PointStore &points, const PlaneDetectionOptions &planes,
                                const CylinderDetectionOptions &cylinders,
                                CompoundCandidate &output, std::string &error,
                                const std::vector<PrimitiveEvidence>& seeds = {});
} // namespace reconstruction
