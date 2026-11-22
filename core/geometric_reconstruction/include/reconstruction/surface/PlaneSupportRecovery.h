#pragma once

#include "reconstruction/segmentation/PrimitiveDetector.h"
#include <functional>

namespace reconstruction {
struct PlaneMergeDiagnostic {
    std::vector<SurfaceId> firstCandidates;
    std::vector<SurfaceId> secondCandidates;
    std::vector<SurfaceId> firstResultProposals;
    std::vector<SurfaceId> secondResultProposals;
    std::string phase;
    std::string outcome;
    std::string reason;
    double angleDegrees = 0;
    double maximumAngleDegrees = 0;
    bool spatiallyConnected = false;
    bool spatialCheckEvaluated = false;
    double maximumAllowedResidual = 0;
    std::vector<double> componentP95Residuals;
    bool residualCheckEvaluated = false;
};

// Expand automatically detected planes over connected observations. Geometry
// and point IDs are preserved; this never establishes material orientation.
// Protected IDs cannot be claimed by automatic planes. Publication is atomic
// on cancellation or invalid input.
bool recoverPlaneSupport(const PointStore& points, const PlaneDetectionOptions& options,
    std::vector<PrimitiveEvidence>& evidence, std::string& error,
    const std::vector<PointId>& protectedIds = {}, const std::function<bool()>& cancelled = {},
    std::vector<PlaneMergeDiagnostic>* mergeDiagnostics = nullptr,
    bool* diagnosticsTruncated = nullptr);
}
