#pragma once

#include "reconstruction/surface/SurfaceAnalysis.h"

namespace reconstruction {

// Distances use the source snapshot's unit. No implicit rescaling or setting I/O.
struct SurfaceOwnershipOptions {
    double distanceTolerance = 0.1;
    double supportRadius = 0.4;
    bool useSourceNormals = true;
};

struct SurfaceOwnershipResult {
    // Evidence support IDs are disjoint source PointIds, never display row indices.
    std::vector<PrimitiveEvidence> surfaces;
    SurfaceSupportValidationReport finalSupportValidation;
    std::size_t assignedPoints = 0;
    std::size_t preferredPlaneCount = 0;
    bool requiresFreshAnalysis = false;
};

// Resolve finite patches into unique observation labels, then validate and perform
// at most one reassignment. Input proposals stay available in the supplied analysis.
// On failure/cancellation output remains unchanged; unassigned source points survive.
bool resolveSurfaceOwnership(const PointStore& source,
    const SurfaceAnalysisResult& sourceAnalysis,
    const SurfaceOwnershipOptions& options, SurfaceOwnershipResult& output,
    std::string& error, const std::function<bool()>& cancelled = {});

} // namespace reconstruction
