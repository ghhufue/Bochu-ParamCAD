#pragma once

#include "reconstruction/segmentation/PrimitiveDetector.h"
#include <functional>

namespace reconstruction {
// Expand automatically detected planes over connected observations. Geometry
// and point IDs are preserved; this never establishes material orientation.
// Protected IDs cannot be claimed by automatic planes. Publication is atomic
// on cancellation or invalid input.
bool recoverPlaneSupport(const PointStore& points, const PlaneDetectionOptions& options,
    std::vector<PrimitiveEvidence>& evidence, std::string& error,
    const std::vector<PointId>& protectedIds = {}, const std::function<bool()>& cancelled = {});
}
