#pragma once

#include "reconstruction/surface/SurfaceAnalysis.h"

#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>

#include <array>
#include <memory>

namespace reconstruction {

struct BrepClosureOptions {
    double planeMarginFraction = 0.15;
    double freeformExtendFraction = 0.12;
    double baseFuzzyToleranceFraction = 0.1;
    bool enableParallelBop = false;
    bool separateRegularHoles = true;
    double maxAllowedP95Multiplier = 4.0;
    bool collectInspection = false;
};

struct BrepInspectionCell {
    enum class Label { Unclassified, Inside, Outside, ExteriorAnchor };
    TopoDS_Solid solid;
    Label label = Label::Unclassified;
    double volumeMm3 = 0;
};

struct BrepInspectionAttempt {
    enum class Stage { CuttingFaces, Partitioned, Classified };
    Stage stage = Stage::CuttingFaces;
    std::size_t number = 0;
    double fuzzyToleranceMm = 0;
    std::string error;
    std::vector<TopoDS_Face> cuttingFaces;
    std::vector<BrepInspectionCell> cells;
    TopoDS_Solid solidBeforeHoleCuts;
    std::vector<TopoDS_Shape> holeTools;
};

struct BrepInspection {
    // Layout matches VTK bounds: minimum and maximum for X, Y, and Z.
    std::array<double, 6> boundsMm{};
    std::vector<modeling::Vec3> observationsMm;
    BrepInspectionAttempt attempt;
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
    // Captured from the successful attempt, or the furthest failed attempt.
    std::shared_ptr<const BrepInspection> inspection;
};

// Points and analysis are in the same millimetre frame. The optional original
// observations are scored after construction, including points removed by downsampling.
bool reconstructClosedBrep(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const BrepClosureOptions& options, ClosedBrepResult& output, std::string& error,
    const PointStore* originalObservations = nullptr, const std::function<bool()>& cancelled = {});

} // namespace reconstruction
