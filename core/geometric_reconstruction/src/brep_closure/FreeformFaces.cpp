#include "ClosureInternal.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLib.hxx>
#include <GeomPlate_BuildPlateSurface.hxx>
#include <GeomPlate_MakeApprox.hxx>
#include <GeomPlate_PointConstraint.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Plane.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <gp_Ax3.hxx>
#include <gp_Vec.hxx>

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace reconstruction::detail::closure {
namespace {
struct ProjectionFrame {
    Vec3 origin, x, y, normal;
    Vec3 project(Vec3 p) const {
        const auto d = subtract(p, origin);
        return {dot(d, x), dot(d, y), dot(d, normal)};
    }
    Vec3 world(Vec3 p) const {
        return add(origin, add(scale(x, p.x), add(scale(y, p.y), scale(normal, p.z))));
    }
};

ProjectionFrame projectFrame(const std::vector<const PointSample*>& cluster) {
    Vec3 mean;
    for (const auto* p : cluster) mean = add(mean, p->position);
    mean = scale(mean, 1.0 / cluster.size());
    std::array<std::array<double, 3>, 3> covariance{};
    for (const auto* p : cluster) {
        const auto a = subtract(p->position, mean);
        const std::array<double, 3> components{a.x, a.y, a.z};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) covariance[i][j] += components[i] * components[j];
    }
    const auto eigen = symmetricEigenDecomposition(covariance);
    return {mean, eigen.vectors[2], cross(eigen.vectors[0], eigen.vectors[2]), eigen.vectors[0]};
}

struct Domain {
    double u0, u1, v0, v1;
};

Domain domain(const occ::handle<Geom_BoundedSurface>& s) {
    Domain bounds{};
    s->Bounds(bounds.u0, bounds.u1, bounds.v0, bounds.v1);
    return bounds;
}

bool regularProjection(const occ::handle<Geom_BoundedSurface>& s, const ProjectionFrame& frame,
                       const Domain& core) {
    const auto b = domain(s);
    constexpr int samples = 12;
    const auto normal = [&](double u, double v) {
        gp_Pnt p;
        gp_Vec du, dv;
        s->D1(u, v, p, du, dv);
        const auto n = du.Crossed(dv);
        if (n.SquareMagnitude() <= 1e-20) return Vec3{};
        return vector(gp_Dir(n));
    };
    const double sign = dot(normal((core.u0 + core.u1) / 2, (core.v0 + core.v1) / 2), frame.normal);
    for (int i = 0; i <= samples; ++i)
        for (int j = 0; j <= samples; ++j) {
            const double u = b.u0 + (b.u1 - b.u0) * i / samples;
            const double v = b.v0 + (b.v1 - b.v0) * j / samples;
            const auto n = normal(u, v);
            if (dot(n, frame.normal) * std::copysign(1., sign) < .08) return false;
            const auto seam = normal(std::clamp(u, core.u0, core.u1), std::clamp(v, core.v0, core.v1));
            if (dot(n, seam) < .5) return false;
        }
    return true;
}

TopoDS_Face extend(const occ::handle<Geom_BSplineSurface>& fit, const ProjectionFrame& frame,
                   double uLength, double vLength, double fraction) {
    const auto core = domain(fit);
    for (int attempt = 0; attempt < 5; ++attempt, fraction *= .5) {
        try {
            auto surface = occ::handle<Geom_BoundedSurface>::DownCast(fit->Copy());
            for (const bool alongU : {true, false})
                for (const bool after : {true, false})
                    GeomLib::ExtendSurfByLength(surface, fraction * (alongU ? uLength : vLength), 1, alongU, after);
            if (!regularProjection(surface, frame, core)) continue;
            BRepBuilderAPI_MakeFace face(surface, Precision::Confusion());
            if (face.IsDone() && BRepCheck_Analyzer(face.Face()).IsValid()) return face.Face();
        } catch (const Standard_Failure&) {
            // Retry against the original fit at half the extension length.
        }
    }
    return {};
}

void anchorNearPlane(Vec3& position, const PointSample& sample, const SurfaceAnalysisResult& analysis,
                     double tolerance, double spacing) {
    const auto n = sampleNormal(sample);
    if (!n) return;
    for (const auto& patch : analysis.patches) {
        const auto* plane = std::get_if<PlaneEvidence>(&analysis.surfaces[patch.surfaceId - 1]);
        if (!plane || std::abs(dot(plane->plane.normal, *n)) < .94) continue;
        const auto& lo = patch.lower;
        const auto& hi = patch.upper;
        const double gap = 3 * spacing;
        if (position.x < lo.x-gap || position.x > hi.x+gap || position.y < lo.y-gap ||
            position.y > hi.y+gap || position.z < lo.z-gap || position.z > hi.z+gap) continue;
        const double d = dot(plane->plane.normal, position) + plane->plane.offset;
        if (std::abs(d) <= 1.5 * tolerance) {
            position = subtract(position, scale(plane->plane.normal, .65 * d));
            return;
        }
    }
}

occ::handle<Geom_BSplineSurface> plateFit(const ProjectionFrame& frame,
    const std::vector<double>& heights, const std::vector<PointSample>& projected, double tolerance) {
    const occ::handle<Geom_Plane> plane = new Geom_Plane(
        gp_Ax3(point(frame.origin), direction(frame.normal), direction(frame.x)));
    GeomPlate_BuildPlateSurface plate(plane, 3, 10, 3, 1e-5, tolerance);
    const auto size = std::min<std::size_t>(80, projected.size());
    for (std::size_t i = 0; i < size; ++i) {
        const auto k = i * projected.size() / size;
        auto p = projected[k].position;
        p.z = heights[k];
        plate.Add(new GeomPlate_PointConstraint(point(frame.world(p)), 0, tolerance));
    }
    plate.Perform();
    if (!plate.IsDone()) return {};
    GeomPlate_MakeApprox approx(plate.Surface(), tolerance, 12, 6, 2 * tolerance, 0, GeomAbs_C2, 1.0);
    return approx.Surface();
}

TopoDS_Face fitCluster(const std::vector<const PointSample*>& cluster, const SurfaceAnalysisResult& analysis,
                       double spacing, double tolerance, double fraction, std::string& reason) {
    const auto frame = projectFrame(cluster);
    std::vector<PointSample> projected;
    std::vector<double> heights;
    double u0 = INFINITY, v0 = INFINITY, u1 = -INFINITY, v1 = -INFINITY;
    std::size_t normals = 0, stableNormals = 0;
    for (const auto* sample : cluster) {
        if (const auto n = sampleNormal(*sample)) {
            ++normals;
            stableNormals += std::abs(dot(*n, frame.normal)) > .3;
        }
        auto position = sample->position;
        anchorNearPlane(position, *sample, analysis, tolerance, spacing);
        auto p = frame.project(position);
        u0 = std::min(u0, p.x); u1 = std::max(u1, p.x);
        v0 = std::min(v0, p.y); v1 = std::max(v1, p.y);
        heights.push_back(p.z);
        p.z = 0;
        projected.push_back({sample->id, p, std::nullopt, sample->confidence, std::nullopt, false});
    }
    const double width = u1 - u0, height = v1 - v0;
    if (std::min(width, height) < 3 * spacing || stableNormals < .8 * normals) {
        reason = "insufficient extent or unstable projected normals";
        return {};
    }
    const KdTree tree(projected);
    const int nu = static_cast<int>(std::clamp(width / (2 * spacing), 5., 22.));
    const int nv = static_cast<int>(std::clamp(height / (2 * spacing), 5., 22.));
    NCollection_Array2<gp_Pnt> grid(1, nu, 1, nv);
    std::size_t gaps = 0;
    for (int i = 1; i <= nu; ++i)
        for (int j = 1; j <= nv; ++j) {
            Vec3 p{u0 + width * (i - 1) / (nu - 1), v0 + height * (j - 1) / (nv - 1), 0};
            const auto nearest = tree.nearest(p, 16);
            gaps += nearest.front().first > 4 * spacing * spacing;
            Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
            Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
            for (const auto& [distance, k] : nearest) {
                const auto d = subtract(projected[k].position, p);
                const Eigen::Vector3d row(1, d.x / spacing, d.y / spacing);
                const double weight = 1 / (1 + distance / (spacing * spacing));
                normal += weight * row * row.transpose();
                rhs += weight * row * heights[k];
            }
            normal.diagonal().array() += 1e-6;
            p.z = normal.ldlt().solve(rhs).x();
            grid.SetValue(i, j, point(frame.world(p)));
        }
    occ::handle<Geom_BSplineSurface> fit;
    const auto coverage = 1. - static_cast<double>(gaps) / (nu * nv);
    if (coverage >= .65) {
        try {
            GeomAPI_PointsToBSplineSurface approximation(grid, 1.0, .01, .001, 8, GeomAbs_C2, tolerance);
            if (approximation.IsDone()) fit = approximation.Surface();
        } catch (const Standard_Failure&) {
            // The grid is already smoothed by the local least-squares fit.
            GeomAPI_PointsToBSplineSurface approximation(grid, 3, 6, GeomAbs_C2, tolerance);
            if (approximation.IsDone()) fit = approximation.Surface();
        }
    }
    if (fit.IsNull() && projected.size() <= 150 && coverage >= .3)
        fit = plateFit(frame, heights, projected, tolerance);
    if (fit.IsNull()) {
        reason = "surface fit failed at projected grid coverage " + std::to_string(coverage);
        return {};
    }
    const auto count = std::min<std::size_t>(96, cluster.size());
    std::size_t inliers = 0;
    for (std::size_t i = 0; i < count; ++i) {
        GeomAPI_ProjectPointOnSurf projection(point(cluster[i * cluster.size() / count]->position), fit);
        inliers += projection.IsDone() && projection.NbPoints() &&
            projection.LowerDistance() <= std::max(2 * tolerance, .35 * spacing);
    }
    if (inliers < .88 * count) {
        reason = "fit supported " + std::to_string(inliers) + " of " + std::to_string(count) + " observations";
        return {};
    }
    auto face = extend(fit, frame, width, height, fraction);
    if (face.IsNull()) reason = "G1 extension failed face and projected-normal validation";
    return face;
}
} // namespace

std::vector<TopoDS_Face> fitFreeformFaces(const PointStore& points, const SurfaceAnalysisResult& analysis,
    const PointIndex& samples, const KdTree& tree, double spacing, double tolerance, double extendFraction,
    std::vector<std::string>& diagnostics, const std::function<bool()>& cancelled) {
    std::vector<TopoDS_Face> faces;
    const auto clusters = connectedPointComponents(points, analysis.unassignedPointIds,
        std::max(2.4 * spacing, 3 * tolerance));
    std::size_t rejected = 0;
    for (const auto& ids : clusters) {
        if (cancelled && cancelled()) return {};
        if (ids.size() < std::max<std::size_t>(24, analysis.options.minimumRecoveryRegionPoints)) continue;
        std::vector<const PointSample*> cluster;
        for (const auto id : ids) {
            const auto item = samples.find(id);
            if (item == samples.end()) continue;
            const auto near = tree.nearest(item->second->position, 6);
            if (near.size() == 6 && near.back().first <= 16 * spacing * spacing) cluster.push_back(item->second);
        }
        if (cluster.size() < 24) continue;
        std::string reason;
        try {
            auto face = fitCluster(cluster, analysis, spacing, tolerance, extendFraction, reason);
            if (!face.IsNull()) { faces.push_back(face); continue; }
        } catch (const Standard_Failure& failure) {
            reason = failure.what();
        }
        if (rejected < 8) diagnostics.push_back("Residual cluster rejected: " + reason);
        ++rejected;
    }
    diagnostics.push_back("Residual freeform patches: " + std::to_string(faces.size()) + " accepted, " +
        std::to_string(rejected) + " rejected");
    return faces;
}
} // namespace reconstruction::detail::closure
