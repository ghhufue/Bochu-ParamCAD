#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace contour_fitting {

struct Point2D {
    double x = 0;
    double y = 0;
};

// Closed implicitly, with no repeated terminal point.
using Ring = std::vector<Point2D>;

struct Contour {
    Ring outer;               // Counterclockwise.
    std::vector<Ring> holes;  // Clockwise.
};

using IndexRing = std::vector<std::size_t>;
struct TriangleContour {
    IndexRing outer;
    std::vector<IndexRing> holes;
};

// Extract implicitly closed rings from an indexed planar triangle
// mesh. Outer rings are counterclockwise and holes clockwise in the supplied
// coordinates. Indices refer to the original vertices, without moving them.
// Invalid triangle indices or non-manifold edges yield no contours.
std::vector<TriangleContour> extractTriangleContours(
    const std::vector<Point2D>& vertices, const std::vector<std::array<std::size_t, 3>>& triangles);

struct Options {
    double alphaRadius = 0; // Zero estimates a scale from the samples.
    std::optional<double> simplificationTolerance; // Zero keeps the original boundary vertices.
    std::size_t minimumRegionPoints = 0;
    double minimumRegionFraction = 0; // Fraction of all distinct projected samples, in [0, 1].
    double automaticAlphaScale = 1;
};

struct Result {
    std::size_t uniquePointCount = 0;
    double alphaRadius = 0;
    double simplificationTolerance = 0;
    std::vector<Contour> contours;
    std::size_t discardedRegionCount = 0;
};

// Fits regions supported by unordered samples in a common 2D coordinate system.
// Duplicate points are merged. Degenerate input yields no contours.
// Non-finite coordinates or invalid options throw std::invalid_argument.
Result fitContours(const std::vector<Point2D>& points, const Options& options = {});

// Simplifies already validated, ordered contours as a single constraint set.
// Returns the same contour and hole order.
std::vector<Contour> simplifyContours(const std::vector<Contour>& contours, double tolerance);

} // namespace contour_fitting
