#pragma once

#include "contour_fitting/ContourFitting.h"

#include <functional>

namespace contour_fitting {

struct Circle {
    Point2D center;
    double radius = 0;
};

struct FittedArc {
    Circle circle;
    double startAngle = 0;
    double sweep = 0;
    Point2D pointAt(double fraction) const;
};

struct FittedRing {
    Ring polyline;
    std::optional<Circle> circle;
    // An entry describes the edge from vertex i to vertex (i+1) mod n.
    std::vector<std::optional<FittedArc>> arcs;
};

struct FittedContour {
    FittedRing outer;
    std::vector<FittedRing> holes;
};

struct ParameterizationOptions {
    double tolerance = 0;
    std::size_t maximumEdgesPerContour = 256;
    bool recognizeCircles = true;
    struct Engineering {
        double spacing = 0;
        double maximumLineTolerance = 0;
        double circleTolerance = 0;
        bool recognizeFillets = true;
    };
    std::optional<Engineering> engineering;
};

enum class FitFailure { None, InvalidBoundary, EdgeBudget, Cancelled, TouchingHole };

struct ParameterizationResult {
    std::vector<FittedContour> contours;
    std::size_t edgeCount = 0;
    double maximumDeviation = 0; // Upper bound in both directions.
    double p95Deviation = 0;     // Sampled distances in both directions.
    std::size_t separatedHoles = 0;
    std::size_t circleCount = 0;
    std::size_t arcCount = 0;
    std::vector<Point2D> downweightedPoints;
    FitFailure failure = FitFailure::None;
};

// Refit ordered boundaries without extracting a new topology. Ring winding
// is normalized and each connected region keeps its original holes.
ParameterizationResult parameterizeContours(const std::vector<Contour>& contours,
    const ParameterizationOptions& options, const std::function<bool()>& cancelled = {});

} // namespace contour_fitting
