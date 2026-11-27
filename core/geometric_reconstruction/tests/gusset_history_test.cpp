#include "modeling/SemanticFaces.h"
#include "reconstruction/modeling/ModelingAdapter.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace modeling;
using namespace reconstruction;

void require(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}

struct Fixture {
    SurfaceScene scene;
    std::vector<PointSample> samples;

    Fixture() {
        scene.header.unit = LengthUnit::Millimeter;
        scene.header.distanceToleranceMm = .01;
        scene.header.angularToleranceRad = .02;
        for (const double x : {0.0, 80.0})
            face({x == 0 ? -1.0 : 1.0, 0, 0},
                {{x,0,0},{x,60,0},{x,60,8},{x,8,8},{x,8,68},{x,0,68}});
        face({0,0,-1}, {{0,0,0},{80,0,0},{80,60,0},{0,60,0}});
        face({0,-1,0}, {{0,0,0},{80,0,0},{80,0,68},{0,0,68}});
        face({0,1,0}, {{0,60,0},{80,60,0},{80,60,8},{0,60,8}});
        face({0,0,1}, {{0,0,68},{80,0,68},{80,8,68},{0,8,68}});
        face({0,0,1}, {{0,8,8},{36,8,8},{36,48,8},{44,48,8},
            {44,8,8},{80,8,8},{80,60,8},{0,60,8}});
        face({0,1,0}, {{0,8,8},{36,8,8},{36,8,58},{44,8,58},
            {44,8,8},{80,8,8},{80,8,68},{0,8,68}});
        face({-1,0,0}, {{36,8,8},{36,48,8},{36,8,58}});
        face({1,0,0}, {{44,8,58},{44,8,8},{44,48,8}});
        const double slopeLength = std::hypot(50, 40);
        face({0,50/slopeLength,40/slopeLength}, {{36,48,8},{44,48,8},{44,8,58},{36,8,58}});
        scene.header.analysisFingerprint = pointStoreFingerprint(PointStore(samples));
    }

    void face(Vec3 normal, std::vector<Vec3> vertices) {
        const auto toPoint = [](Vec3 p) { return gp_Pnt(p.x, p.y, p.z); };
        const auto toVector = [](Vec3 p) { return gp_Vec(p.x, p.y, p.z); };
        const auto origin = toPoint(vertices.front());
        const gp_Vec u = gp_Vec(origin, toPoint(vertices[1])).Normalized();
        const gp_Vec v = toVector(normal).Crossed(u);
        PlaneEvidence evidence;
        evidence.plane = {normal, -gp_Vec(gp_Pnt(0,0,0), origin).Dot(toVector(normal))};
        evidence.confidence = 1;
        scene.surfaces.push_back(evidence);
        ScenePatch patch;
        patch.id = patch.surfaceId = scene.surfaces.size();
        patch.materialNormal = normal;
        patch.orientationKnown = true;
        patch.supportConfidence = 1;
        patch.boundaryStatus = SceneBoundaryStatus::ObservedClosed;
        std::vector<Point2D> polygon;
        for (const auto vertex : vertices) {
            const gp_Vec local(origin, toPoint(vertex));
            polygon.push_back({local.Dot(u), local.Dot(v)});
        }
        const auto x = [](Point2D a, Point2D b) { return a.x < b.x; };
        const auto y = [](Point2D a, Point2D b) { return a.y < b.y; };
        const auto xs = std::minmax_element(polygon.begin(), polygon.end(), x);
        const auto ys = std::minmax_element(polygon.begin(), polygon.end(), y);
        for (double a = xs.first->x + 1; a < xs.second->x; a += 3) {
            for (double b = ys.first->y + 1; b < ys.second->y; b += 3) {
                bool inside = false;
                for (std::size_t i = 0; i < polygon.size(); ++i) {
                    const auto p = polygon[i], q = polygon[(i + 1) % polygon.size()];
                    if ((p.y > b) != (q.y > b) && a < p.x + (q.x - p.x) * (b - p.y) / (q.y - p.y))
                        inside = !inside;
                }
                if (!inside) continue;
                const auto world = origin.Translated(u * a + v * b);
                PointSample point;
                point.id = samples.size() + 1;
                point.position = {world.X(), world.Y(), world.Z()};
                point.normal = normal;
                point.normalOrientationKnown = true;
                samples.push_back(point);
                patch.supportPointIds.push_back(point.id);
            }
        }
        patch.loops.push_back({SceneLoopRole::Outer, std::move(vertices), true, 1});
        scene.patches.push_back(std::move(patch));
    }
};

TopoDS_Shape referenceShape(double xmin, double xmax, double ribStart = 0, double ribEnd = 0) {
    const auto bottom = BRepPrimAPI_MakeBox(gp_Pnt(xmin,0,0), xmax-xmin, 60, 8).Shape();
    const auto upright = BRepPrimAPI_MakeBox(gp_Pnt(xmin,0,8), xmax-xmin, 8, 60).Shape();
    BRepAlgoAPI_Fuse base(bottom, upright);
    require(base.IsDone(), "reference L-stock must fuse");
    if (ribEnd <= ribStart) return base.Shape();
    BRepBuilderAPI_MakePolygon triangle;
    triangle.Add(gp_Pnt(ribStart,8,8));
    triangle.Add(gp_Pnt(ribStart,48,8));
    triangle.Add(gp_Pnt(ribStart,8,58));
    triangle.Close();
    const auto rib = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(triangle.Wire()).Face(),
        gp_Vec(ribEnd-ribStart,0,0)).Shape();
    BRepAlgoAPI_Fuse full(base.Shape(), rib);
    require(full.IsDone(), "reference gusset must fuse");
    return full.Shape();
}

double volume(const TopoDS_Shape& shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    return props.Mass();
}

bool sameMaterial(const TopoDS_Shape& shape, const TopoDS_Shape& expected) {
    if (shape.IsNull() || !BRepCheck_Analyzer(shape).IsValid()) return false;
    TopExp_Explorer solids(shape, TopAbs_SOLID);
    if (!solids.More()) return false;
    solids.Next();
    if (solids.More()) return false;
    BRepAlgoAPI_Common overlap(shape, expected);
    if (!overlap.IsDone() || overlap.Shape().IsNull()) return false;
    return std::abs(volume(shape) + volume(expected) - 2 * volume(overlap.Shape())) < 1e-5;
}

bool attachedGusset(const SolidCandidate& candidate) {
    return candidate.featureProgram.size() == 2 &&
        candidate.featureProgram[1].operation == ExtrudeOperation::Join &&
        candidate.featureProgram[1].sketchFace.has_value();
}

void checkHistory(const Fixture& fixture) {
    std::string error;
    SolidInferenceResult inferred;
    const PointStore observations(fixture.samples);
    if (!inferSolids(fixture.scene, inferred, error, {}, &observations))
        throw std::runtime_error("infer gusset scene: " + error);
    std::vector<SolidCandidate> scored;
    for (const auto& proposal : inferred.candidates) {
        if (proposal.status == SolidCandidateStatus::Rejected) continue;
        SolidCandidate result;
        if (scoreSolidCandidate(proposal, fixture.scene, observations, .01, result, error))
            scored.push_back(std::move(result));
    }
    const auto* selected = selectSolidCandidate(scored, error);
    if (!selected || !attachedGusset(*selected)) throw std::runtime_error("select attached gusset: " + error);
    require(selected->featureProgram[1].sketch.entities.size() == 3 &&
        std::count_if(selected->inferredSurfaces.begin(), selected->inferredSurfaces.end(), [](const auto& face) {
            return face.reason == "Gusset contact inferred along the reentrant stock corner";
        }) == 2, "recover triangular section and both hidden contacts");

    CompoundCandidate history;
    history.projectionPrograms.push_back(selected->featureProgram);
    ModelingCore model;
    require(commitCompoundCandidate(history, model).success && model.features().size() == 4,
        "commit separate stock and gusset features");
    require(sameMaterial(model.bodyShape(), referenceShape(0,80,36,44)), "initial material matches independent fixture");

    const auto features = model.features();
    auto stock = std::get<ExtrudeFeatureParams>(features[1].params);
    auto rib = std::get<ExtrudeFeatureParams>(features[3].params);
    const auto& sketch = std::get<SketchFeatureParams>(features[2].params);
    const auto& host = std::get<FaceReference>(sketch.plane);
    require(host.ownerFeature == features[1].id && host.role == FaceRole::EndFace &&
        std::abs(host.normalOffset + 44) < 1e-6 && rib.targetBody == features[1].id,
        "gusset depends on both the stock end plane and target body");
    Plane3d initialPlane, movedPlane;
    require(resolveSketchPlane(features, sketch.plane, initialPlane, error), "resolve original host-relative sketch");
    const bool alongPositiveX = initialPlane.normal.x > 0;
    rib.depth = 12;
    require(model.execute(EditFeatureCommand{features[3].id, rib}).success && model.rebuild() &&
        sameMaterial(model.bodyShape(), alongPositiveX ? referenceShape(0,80,36,48) : referenceShape(0,80,32,44)),
        "rib thickness edit changes only the rib volume");

    rib.depth = 8;
    stock.depth += 12;
    const double xmin = alongPositiveX ? 0 : -12;
    const double xmax = alongPositiveX ? 92 : 80;
    const double shift = alongPositiveX ? 12 : -12;
    require(model.execute(EditFeatureCommand{features[3].id, rib}).success &&
        model.execute(EditFeatureCommand{features[1].id, stock}).success && model.rebuild() &&
        resolveSketchPlane(model.features(), sketch.plane, movedPlane, error) &&
        std::abs(movedPlane.origin.x - initialPlane.origin.x - shift) < 1e-6 &&
        sameMaterial(model.bodyShape(), referenceShape(xmin,xmax,36+shift,44+shift)),
        "moving the host end plane keeps the rib attached and translates it with the host");
    require(model.execute(SetFeatureSuppressedCommand{features[3].id, true}).success && model.rebuild() &&
        sameMaterial(model.bodyShape(), referenceShape(xmin,xmax)),
        "suppressing the gusset restores the complete L-shaped stock");
}
} // namespace

int main() {
    try {
        Fixture fixture;
        checkHistory(fixture);
        std::reverse(fixture.scene.patches.begin(), fixture.scene.patches.end());
        checkHistory(fixture);

        // A shortened slope still lies on the same equation, but no longer spans
        // both observed triangular sides.
        auto& slope = fixture.scene.patches.front().loops.front();
        slope.vertices[1].x = 40;
        slope.vertices[2].x = 40;
        SolidInferenceResult inferred;
        std::string error;
        require(inferSolids(fixture.scene, inferred, error) &&
            std::none_of(inferred.candidates.begin(), inferred.candidates.end(), attachedGusset),
            "a plane equation without spanning finite slope support cannot anchor a gusset");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
