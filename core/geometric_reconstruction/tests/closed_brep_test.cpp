#include "reconstruction/brep_closure/ClosedBrepReconstructor.h"

#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace {
using namespace reconstruction;
using modeling::Vec3;
constexpr double pi = 3.141592653589793;

struct Fixture {
    std::vector<PointSample> samples;
    SurfaceAnalysisResult analysis;
    PointId nextId = 1;
    void add(Vec3 p, Vec3 n, std::vector<PointId>& ids, bool oriented) {
        const auto id = nextId++;
        samples.push_back({id, p, n, 1, {}, oriented});
        ids.push_back(id);
    }
    void patch(PrimitiveEvidence evidence) {
        auto& p = analysis.patches.emplace_back();
        p.id = p.surfaceId = analysis.surfaces.size() + 1;
        std::visit([&](auto& s) { s.id = p.id; p.supportPointIds = s.supportPointIds; }, evidence);
        analysis.surfaces.push_back(std::move(evidence));
    }
    template<class Sample>
    void plane(Vec3 n, double offset, Sample sample, bool oriented = true) {
        PlaneEvidence p;
        p.plane = {n, offset};
        for (int i = 0; i <= 28; ++i)
            for (int j = 0; j <= 28; ++j)
                if (const auto position = sample(i / 28., j / 28.)) add(*position, n, p.supportPointIds, oriented);
        patch(std::move(p));
    }
};

void require(bool value, const std::string& error) {
    if (!value) throw std::runtime_error(error);
}

ClosedBrepResult run(Fixture fixture, const BrepClosureOptions& options = {}) {
    const PointStore points(std::move(fixture.samples));
    fixture.analysis.pointFingerprint = pointStoreFingerprint(points);
    fixture.analysis.unit = points.unit();
    fixture.analysis.options.detection.plane.distanceThreshold = .12;
    ClosedBrepResult result;
    std::string error;
    if (!reconstructClosedBrep(points, fixture.analysis, options, result, error)) {
        for (const auto& diagnostic : result.diagnostics) std::cerr << diagnostic << '\n';
        throw std::runtime_error(error);
    }
    require(result.quality.isValidTopology && result.quality.freeEdgeCount == 0, "Closed topology required");
    require(result.quality.subSolidCount > 1 && result.quality.insideSolidCount > 0, "Expected partition and graph cut");
    return result;
}

Fixture block(double innerFloor, bool oriented = true, bool freeform = false, bool stepped = false) {
    Fixture f;
    const auto top = [&](double x, double y) {
        return freeform ? 8. + .018 * (x - 10) * (x - 10) + .009 * (y - 8) * (y - 8) : 8.;
    };
    for (const double x : {0., 20.})
        f.plane({x == 0 ? -1. : 1., 0, 0}, -x, [&](double u, double v) {
            return std::optional<Vec3>{{x, 16 * u, top(x, 16 * u) * v}};
        }, oriented);
    for (const double y : {0., 16.})
        f.plane({0, y == 0 ? -1. : 1., 0}, -y, [&](double u, double v) {
            return std::optional<Vec3>{{20 * u, y, top(20 * u, y) * v}};
        }, oriented);
    for (const double z : {0., 8.}) {
        if (z == 8 && freeform) continue;
        f.plane({0, 0, z == 0 ? -1. : 1.}, -z, [&](double u, double v) -> std::optional<Vec3> {
            const Vec3 p{20 * u, 16 * v, z};
            const double radius = stepped && z == 8 ? 4.5 : 3.;
            if (innerFloor >= 0 && (z == 8 || innerFloor == 0) && std::hypot(p.x - 10, p.y - 8) < radius)
                return {};
            return p;
        }, oriented);
    }
    if (freeform) {
        for (int i = 0; i <= 30; ++i)
            for (int j = 0; j <= 27; ++j) {
                const double x = 20. * i / 30, y = 16. * j / 27;
                const auto dx = -.036 * (x - 10), dy = -.018 * (y - 8);
                const auto length = std::sqrt(1 + dx * dx + dy * dy);
                f.add({x, y, top(x, y)}, {dx / length, dy / length, 1 / length}, f.analysis.unassignedPointIds, oriented);
            }
    }
    if (innerFloor < 0) return f;
    const auto wall = [&](double bottom, double topZ, double radius) {
        CylinderEvidence wall;
        wall.cylinder = {{10, 8, 0}, {0, 0, 1}, radius};
        for (int i = 0; i < 64; ++i)
            for (int j = 0; j <= 20; ++j) {
                const double u = 2 * pi * i / 64, z = bottom + (topZ - bottom) * j / 20;
                f.add({10 + radius * std::cos(u), 8 + radius * std::sin(u), z},
                      {-std::cos(u), -std::sin(u), 0}, wall.supportPointIds, oriented);
            }
        f.patch(std::move(wall));
    };
    wall(innerFloor, stepped ? 5. : 8., 3.);
    if (stepped) {
        wall(5., 8., 4.5);
        f.plane({0, 0, 1}, -5., [&](double u, double v) {
            const double r = 3 + u * 1.5;
            return std::optional<Vec3>{{10 + r * std::cos(2 * pi * v), 8 + r * std::sin(2 * pi * v), 5}};
        }, oriented);
        f.analysis.relations.push_back({7, 8, SurfaceRelationKind::Coaxial, 0, 1, true});
    }
    if (innerFloor > 0)
        f.plane({0, 0, 1}, -innerFloor, [&](double u, double v) {
            return std::optional<Vec3>{{10 + 3 * u * std::cos(2 * pi * v),
                                       8 + 3 * u * std::sin(2 * pi * v), innerFloor}};
        }, oriented);
    return f;
}

Fixture obliqueHole(double angle) {
    auto f = block(-1);
    const double sine = std::sin(angle), cosine = std::cos(angle);
    const Vec3 axis{sine, 0, cosine};
    constexpr double radius = 2.8;
    const auto offset = [&](Vec3 p) { return (p.x - 9) * cosine - p.z * sine; };
    std::unordered_set<PointId> excluded;
    for (const auto& p : f.samples)
        if ((p.position.z == 0 || p.position.z == 8) &&
            std::hypot(offset(p.position), p.position.y - 8) < radius) excluded.insert(p.id);
    f.samples.erase(std::remove_if(f.samples.begin(), f.samples.end(),
        [&](const auto& p) { return excluded.count(p.id) != 0; }), f.samples.end());
    const auto remove = [&](auto& ids) {
        ids.erase(std::remove_if(ids.begin(), ids.end(), [&](auto id) { return excluded.count(id) != 0; }), ids.end());
    };
    for (auto& patch : f.analysis.patches) remove(patch.supportPointIds);
    for (auto& surface : f.analysis.surfaces) remove(std::get<PlaneEvidence>(surface).supportPointIds);
    CylinderEvidence wall;
    wall.cylinder = {{9, 8, 0}, axis, radius};
    for (int i = 0; i < 64; ++i)
        for (int j = 0; j <= 24; ++j) {
            const double theta = 2 * pi * i / 64, z = 8. * j / 24;
            const double u = std::cos(theta), v = std::sin(theta);
            const double height = (z + radius * sine * u) / cosine;
            f.add({9 + height * sine + radius * cosine * u, 8 + radius * v, z},
                  {-cosine * u, -v, sine * u}, wall.supportPointIds, true);
        }
    f.patch(std::move(wall));
    return f;
}

void compareVolume(const ClosedBrepResult& result, double expected, double relativeTolerance) {
    const auto volume = result.quality.volumeMm3;
    require(std::abs(expected - volume) <= expected * relativeTolerance,
        "Volume expected " + std::to_string(expected) + " but measured " + std::to_string(volume));
}

void regularizeMoreThanTwelveSurfaces() {
    Fixture fixture;
    for (int i = 0; i < 18; ++i) {
        const double inclination = i % 2 ? .003 : -.002;
        fixture.plane({0, std::sin(inclination), std::cos(inclination)}, -i, [&](double u, double v) {
            return std::optional<Vec3>{{3 * u, 3 * v, (i - 3 * v * std::sin(inclination)) / std::cos(inclination)}};
        });
        if (i) fixture.analysis.relations.push_back({PatchId(i), PatchId(i + 1), SurfaceRelationKind::Parallel, 0, .9, true});
    }
    const PointStore points(std::move(fixture.samples));
    fixture.analysis.pointFingerprint = pointStoreFingerprint(points);
    fixture.analysis.unit = points.unit();
    RelationOptimizationReport report;
    std::string error;
    const bool ok = regularizeBrepRelations(points, fixture.analysis, report, error);
    require(ok, error);
    require(report.accepted.size() == 17 && report.rejected.empty(), "Expected all 18 surfaces to regularize");
}
} // namespace

int main() {
    try {
        regularizeMoreThanTwelveSurfaces();
        const auto rectangular = run(block(-1, false));
        compareVolume(rectangular, 2560, 1e-6);
        std::stringstream native;
        BRepTools::Write(rectangular.solid, native);
        BRep_Builder builder;
        TopoDS_Shape restored;
        BRepTools::Read(restored, native, builder);
        require(restored.ShapeType() == TopAbs_SOLID, "Native B-Rep round trip lost the solid");
        for (const double floor : {0., 3.}) {
            const auto drilled = run(block(floor));
            require(drilled.quality.regularHoleToolCount == 1, "Expected a deferred regular hole");
            compareVolume(drilled, 2560 - 9 * pi * (8 - floor), 1e-6);
            BRepClass3d_SolidClassifier classifier(drilled.solid, gp_Pnt(10, 8, 6), 1e-6);
            require(classifier.State() == TopAbs_OUT, "Hole axis should cross empty space");
        }
        BrepClosureOptions partitionHoles;
        partitionHoles.separateRegularHoles = false;
        compareVolume(run(block(0), partitionHoles), 2560 - 72 * pi, 1e-6);
        const auto stepped = run(block(2, true, false, true));
        require(stepped.quality.regularHoleToolCount == 2, "Expected two coaxial tools for stepped blind hole");
        compareVolume(stepped, 2560 - (9 + 4.5 * 4.5) * 3 * pi, 1e-6);
        const auto oblique = run(obliqueHole(.23));
        require(oblique.quality.regularHoleToolCount == 0, "Oblique holes participate in partitioning");
        compareVolume(oblique, 2560 - 2.8 * 2.8 * 8 * pi / std::cos(.23), 1e-5);
        BrepClosureOptions parallel;
        parallel.enableParallelBop = true;
        for (int i = 0; i < 3; ++i) {
            const auto repeat = run(block(0), parallel);
            compareVolume(repeat, 2560 - 72 * pi, 1e-6);
            require(repeat.quality.subSolidCount == rectangular.quality.subSolidCount,
                    "Parallel run changed cell partition");
        }
        const auto hybrid = run(block(-1, true, true));
        require(hybrid.quality.freeformFaceCount == 1, "Expected one residual B-spline face");
        compareVolume(hybrid, 20 * 16 * (8 + .018 * 100 / 3 + .009 * 64 / 3), .015);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
