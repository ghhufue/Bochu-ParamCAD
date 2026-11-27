#pragma once

#include "reconstruction/brep_closure/ClosedBrepReconstructor.h"
#include "common/Math3.h"
#include "common/spatial/KdTree.h"

#include <Bnd_Box.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <unordered_map>

namespace reconstruction::detail::closure {

using namespace math3;
using modeling::Vec3;
using spatial::KdTree;
using PointIndex = std::unordered_map<PointId, const PointSample*>;

inline gp_Pnt point(Vec3 p) { return {p.x, p.y, p.z}; }
inline gp_Dir direction(Vec3 p) { return {p.x, p.y, p.z}; }
inline Vec3 vector(const gp_Pnt& p) { return {p.X(), p.Y(), p.Z()}; }
inline Vec3 vector(const gp_Dir& p) { return {p.X(), p.Y(), p.Z()}; }

inline std::optional<Vec3> sampleNormal(const PointSample& sample) {
    return sample.normal && length(*sample.normal) > 1e-12
        ? std::optional<Vec3>(normalized(*sample.normal)) : std::nullopt;
}

inline std::pair<Vec3, Vec3> bounds(const Bnd_Box& box) {
    Vec3 lo, hi;
    box.Get(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z);
    return {lo, hi};
}

struct CuttingGeometry {
    std::vector<TopoDS_Face> faces;
    std::vector<TopoDS_Shape> holeTools;
};

bool supportedAnalyticPatch(const PrimitiveEvidence& evidence, const std::vector<PointId>& ids,
                            const PointIndex& samples);

CuttingGeometry makeCuttingGeometry(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const PointIndex& samples, const KdTree& tree, double spacing, double tolerance,
    const BrepClosureOptions& options, ClosedBrepResult& result,
    const std::function<bool()>& cancelled);

std::vector<TopoDS_Face> fitFreeformFaces(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const PointIndex& samples, const KdTree& tree, double spacing, double tolerance, double extendFraction,
    std::vector<std::string>& diagnostics, const std::function<bool()>& cancelled);

TopoDS_Solid partitionAndSelect(const PointStore& points, const KdTree& tree, const Bnd_Box& envelope,
    const CuttingGeometry& geometry, double spacing, double tolerance, double fuzzy,
    const BrepClosureOptions& options, ClosedBrepResult& result, std::string& error,
    const std::function<bool()>& cancelled, BrepInspectionAttempt* inspection);

} // namespace reconstruction::detail::closure
