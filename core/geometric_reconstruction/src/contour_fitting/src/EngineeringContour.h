#pragma once

#include "contour_fitting/ContourParameterization.h"

namespace contour_fitting {

std::optional<FittedRing> fitEngineeringRing(const Ring& source,
    const ParameterizationOptions::Engineering& options, const std::function<bool()>& cancelled,
    const std::vector<bool>& protectedSamples = {}, const Ring& holeSamples = {},
    bool bridgeProtectedGaps = false, const std::vector<const Ring*>& exteriorRings = {});
std::optional<Circle> fitEngineeringCircle(const Ring& source, double tolerance, double spacing);
double circularInlierTolerance(double radius, double tolerance, double spacing);
Ring sampleEngineeringRing(const FittedRing& ring, double spacing);

} // namespace contour_fitting
