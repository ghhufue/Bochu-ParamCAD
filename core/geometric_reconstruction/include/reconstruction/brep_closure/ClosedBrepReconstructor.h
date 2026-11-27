#pragma once

#include "reconstruction/surface/SurfaceAnalysis.h"

#include <TopoDS_Solid.hxx>

namespace reconstruction {

struct BrepClosureOptions {
    double planeMarginFraction = 0.15;
    double freeformExtendFraction = 0.12;
    double baseFuzzyToleranceFraction = 0.1;
    bool enableParallelBop = false;
    bool separateRegularHoles = true;
    double maxAllowedP95Multiplier = 4.0;
};

struct BrepClosureQualityReport {
    bool isValidTopology = false;
    int freeEdgeCount = -1;
    double volumeMm3 = 0.0;
    double rmsErrorMm = 0.0;
    double p95ErrorMm = 0.0;
    std::size_t subSolidCount = 0;
    std::size_t insideSolidCount = 0;
    std::size_t regularHoleToolCount = 0;
    std::size_t freeformFaceCount = 0;
    std::size_t scoredPointCount = 0;
};

struct ClosedBrepResult {
    TopoDS_Solid solid;
    BrepClosureQualityReport quality;
    std::vector<std::string> diagnostics;
};

// Points and analysis are in the same millimetre frame. The optional original
// observations are scored after construction, including points removed by downsampling.
bool reconstructClosedBrep(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const BrepClosureOptions& options, ClosedBrepResult& output, std::string& error,
    const PointStore* originalObservations = nullptr, const std::function<bool()>& cancelled = {});

} // namespace reconstruction
