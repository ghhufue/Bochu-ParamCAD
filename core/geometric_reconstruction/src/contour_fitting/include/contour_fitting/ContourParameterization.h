#pragma once

#include "contour_fitting/ContourFitting.h"

#include <functional>

namespace contour_fitting {

struct Circle {
    Point2D center;
    double radius = 0;
};

struct FittedRing {
    Ring polyline;
    std::optional<Circle> circle;
};

struct FittedContour {
    FittedRing outer;
    std::vector<FittedRing> holes;
};

struct ParameterizationOptions {
    double tolerance = 0;
    std::size_t maximumEdgesPerContour = 256;
    bool recognizeCircles = true;
};

enum class FitFailure { None, InvalidBoundary, EdgeBudget, Cancelled, TouchingHole };

struct ParameterizationResult {
    std::vector<FittedContour> contours;
    std::size_t edgeCount = 0;
    double maximumDeviation = 0; // Upper bound in both directions.
    double p95Deviation = 0;     // Sampled distances in both directions.
    std::size_t separatedHoles = 0;
    FitFailure failure = FitFailure::None;
};

// Refit ordered boundaries without extracting a new topology. Ring winding
// is normalized and each connected region keeps its original holes.
ParameterizationResult parameterizeContours(const std::vector<Contour>& contours,
    const ParameterizationOptions& options, const std::function<bool()>& cancelled = {});

} // namespace contour_fitting
