#include "modeling/SemanticFaces.h"
#include "reconstruction/modeling/ModelingAdapter.h"

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace modeling;
using namespace reconstruction;
constexpr double pi = 3.14159265358979323846;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

gp_Pnt point(Vec3 p) { return {p.x,p.y,p.z}; }
gp_Vec vector(Vec3 p) { return {p.x,p.y,p.z}; }
Vec3 coordinates(const gp_Pnt& p) { return {p.X(),p.Y(),p.Z()}; }
Vec3 coordinates(const gp_Vec& v) { return {v.X(),v.Y(),v.Z()}; }

struct Hole {
    Vec3 entry;
    Vec3 axis;
    double span;
    double radius;
};

struct Fixture {
    SurfaceScene scene;
    std::vector<PointSample> samples;
    double topWidth;
    std::vector<Hole> holes;
    gp_Trsf pose;

    Fixture(double top, bool twoHoles) : topWidth(top), holes{{{4,8,3}, {0,-1,0}, 8, 1}} {
        if (twoHoles) holes.push_back({{12,4,7}, {-1,0,0}, 12, .8});
        scene.header.unit = LengthUnit::Millimeter;
        scene.header.distanceToleranceMm = .01;
        scene.header.angularToleranceRad = .02;
        const std::array<Vec3,4> outline{{{0,0,0},{12,0,0},{top,8,0},{0,8,0}}};
        const gp_Vec sweep(0,0,10);
        for (const double z : {0.0,10.0}) {
            std::vector<Vec3> vertices;
            for (auto p : outline) { p.z = z; vertices.push_back(p); }
            const Vec3 normal{0,0,z == 0 ? -1.0 : 1.0};
            auto patch = plane(normal, vertices);
            for (double y = .5; y < 8; y += 1)
                for (double x = .5; x < 12 + (top-12)*y/8; x += 1) sample(patch, {x,y,z}, normal);
            scene.patches.push_back(std::move(patch));
        }
        for (std::size_t i = 0; i < outline.size(); ++i) {
            const auto from = point(outline[i]), to = point(outline[(i+1) % outline.size()]);
            const gp_Vec edge(from, to);
            const auto normal = coordinates(edge.Crossed(sweep).Normalized());
            auto patch = plane(normal, {coordinates(from), coordinates(to),
                coordinates(to.Translated(sweep)), coordinates(from.Translated(sweep))});
            std::vector<std::pair<gp_Pnt,double>> openings;
            for (const auto& hole : holes) {
                for (const double axial : {0.0, hole.span}) {
                    const auto center = point(hole.entry).Translated(vector(hole.axis) * axial);
                    if (std::abs(gp_Vec(from,center).Dot(vector(normal))) > 1e-9) continue;
                    openings.emplace_back(center,hole.radius);
                    BoundaryLoop3d rim{SceneLoopRole::Hole, {}, true, 1};
                    for (int a = 0; a < 24; ++a)
                        rim.vertices.push_back(coordinates(center.Translated(radial(hole.axis, hole.radius, a))));
                    patch.loops.push_back(std::move(rim));
                }
            }
            for (double t = .5; t < edge.Magnitude(); t += 1)
                for (double z = .5; z < 10; z += 1) {
                    const auto p = from.Translated(edge.Normalized()*t + gp_Vec(0,0,z));
                    if (std::none_of(openings.begin(), openings.end(), [&](const auto& opening) {
                            return p.Distance(opening.first) < opening.second;
                        })) sample(patch, coordinates(p), normal);
                }
            scene.patches.push_back(std::move(patch));
        }
        for (const auto& hole : holes) {
            CylinderEvidence wall;
            wall.cylinder = {hole.entry, hole.axis, hole.radius};
            wall.axialMaximum = hole.span;
            wall.angularCoverage = 1;
            wall.confidence = 1;
            ScenePatch patch;
            patch.id = patch.surfaceId = scene.surfaces.size()+1;
            patch.supportConfidence = 1;
            patch.boundaryStatus = SceneBoundaryStatus::ObservedOpen;
            for (double t = .5; t < hole.span; t += 1) {
                const auto center = point(hole.entry).Translated(vector(hole.axis)*t);
                for (int angle = 0; angle < 24; ++angle) {
                    const auto r = radial(hole.axis, hole.radius, angle);
                    sample(patch, coordinates(center.Translated(r)), coordinates(-r.Normalized()));
                }
            }
            wall.supportPointIds = patch.supportPointIds;
            scene.surfaces.push_back(wall);
            scene.patches.push_back(std::move(patch));
        }
        scene.header.analysisFingerprint = pointStoreFingerprint(PointStore(samples));
    }

    void rotate() {
        pose.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(1,2,3)), .61);
        pose.SetTranslationPart(gp_Vec(17,-9,22));
        const auto transformPoint = [&](Vec3 p) { return coordinates(point(p).Transformed(pose)); };
        const auto transformVector = [&](Vec3 p) { return coordinates(vector(p).Transformed(pose)); };
        for (auto& patch : scene.patches) {
            patch.materialNormal = transformVector(patch.materialNormal);
            for (auto& loop : patch.loops)
                for (auto& vertex : loop.vertices) vertex = transformPoint(vertex);
            auto& evidence = scene.surfaces[patch.surfaceId-1];
            if (auto* plane = std::get_if<PlaneEvidence>(&evidence)) {
                plane->plane.normal = transformVector(plane->plane.normal);
                plane->plane.offset = -vector(plane->plane.normal).Dot(vector(patch.loops.front().vertices.front()));
            } else if (auto* cylinder = std::get_if<CylinderEvidence>(&evidence)) {
                cylinder->cylinder.axisPoint = transformPoint(cylinder->cylinder.axisPoint);
                cylinder->cylinder.axisDirection = transformVector(cylinder->cylinder.axisDirection);
            }
        }
        for (auto& sample : samples) {
            sample.position = transformPoint(sample.position);
            sample.normal = transformVector(*sample.normal);
        }
        scene.header.analysisFingerprint = pointStoreFingerprint(PointStore(samples));
    }

    static gp_Vec radial(Vec3 axis, double radius, int index) {
        const double angle = 2*pi*index/24;
        return gp_Vec(0,0,1) * (radius*std::cos(angle)) +
            vector(axis).Crossed(gp_Vec(0,0,1)) * (radius*std::sin(angle));
    }

    ScenePatch plane(Vec3 normal, std::vector<Vec3> boundary) {
        PlaneEvidence evidence;
        evidence.plane = {normal, -vector(normal).Dot(vector(boundary.front()))};
        evidence.confidence = 1;
        scene.surfaces.push_back(evidence);
        ScenePatch patch;
        patch.id = patch.surfaceId = scene.surfaces.size();
        patch.orientationKnown = true;
        patch.materialNormal = normal;
        patch.supportConfidence = 1;
        patch.boundaryStatus = SceneBoundaryStatus::ObservedClosed;
        patch.loops.push_back({SceneLoopRole::Outer, std::move(boundary), true, 1});
        return patch;
    }

    void sample(ScenePatch& patch, Vec3 position, Vec3 normal) {
        PointSample sample;
        sample.id = samples.size()+1;
        sample.position = position;
        sample.normal = normal;
        sample.normalOrientationKnown = true;
        samples.push_back(sample);
        patch.supportPointIds.push_back(sample.id);
    }
};

TopoDS_Shape reference(double topWidth, double width, const std::vector<Hole>& holes) {
    BRepBuilderAPI_MakePolygon outline;
    for (const auto& p : {gp_Pnt(0,0,0),gp_Pnt(12,0,0),gp_Pnt(topWidth,width,0),gp_Pnt(0,width,0)})
        outline.Add(p);
    outline.Close();
    auto result = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(outline.Wire()).Face(), gp_Vec(0,0,10)).Shape();
    for (const auto& hole : holes) {
        const auto tool = BRepPrimAPI_MakeCylinder(gp_Ax2(point(hole.entry), gp_Dir(vector(hole.axis))),
            hole.radius, hole.span).Shape();
        BRepAlgoAPI_Cut cut(result, tool);
        require(cut.IsDone(), "reference side bore must cut");
        result = cut.Shape();
    }
    return result;
}

double volume(const TopoDS_Shape& shape) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    return props.Mass();
}
bool sameMaterial(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    if (a.IsNull() || b.IsNull()) return false;
    BRepAlgoAPI_Common overlap(a,b);
    return overlap.IsDone() && !overlap.Shape().IsNull() &&
        std::abs(volume(a)+volume(b)-2*volume(overlap.Shape())) < 1e-5;
}

std::size_t sideCuts(const SolidCandidate& candidate) {
    return static_cast<std::size_t>(std::count_if(candidate.featureProgram.begin(), candidate.featureProgram.end(),
        [](const auto& operation) {
            return operation.throughAll && operation.sketchFace &&
                operation.sketchFace->role == FaceRole::ExtrusionSide && operation.sketchFace->profileEdge;
        }));
}

void check(const Fixture& fixture, bool checkEditing) {
    const PointStore observations(fixture.samples);
    SolidInferenceResult inferred;
    std::string error;
    if (!inferSolids(fixture.scene, inferred, error, {}, &observations)) throw std::runtime_error(error);
    std::vector<SolidCandidate> scored;
    for (const auto& item : inferred.candidates) {
        if (item.status == SolidCandidateStatus::Rejected) continue;
        SolidCandidate result;
        if (scoreSolidCandidate(item, fixture.scene, observations, .01, result, error))
            scored.push_back(std::move(result));
    }
    const auto* selected = selectSolidCandidate(scored, error);
    if (!selected || !sideCuts(*selected) || selected->featureProgram.size() != fixture.holes.size()+1)
        throw std::runtime_error("select cross-direction hole history: " + error);
    // Equivalent box histories can start from different stock directions.
    require(std::any_of(scored.begin(), scored.end(), [&](const SolidCandidate& result) {
        return result.status == SolidCandidateStatus::Accepted && sideCuts(result) == fixture.holes.size();
    }), "the same host sweep can carry all observed side bores");
    CompoundCandidate history;
    history.projectionPrograms.push_back(selected->featureProgram);
    ModelingCore model;
    const auto expected = BRepBuilderAPI_Transform(reference(fixture.topWidth,8,fixture.holes), fixture.pose).Shape();
    require(commitCompoundCandidate(history, model).success &&
        sameMaterial(model.bodyShape(), expected), "replayed holes match material");
    const auto features = model.features();
    const auto& section = std::get<SketchFeatureParams>(features[0].params);
    for (std::size_t i = 2; i < features.size(); i += 2) {
        const auto& holeSketch = std::get<SketchFeatureParams>(features[i].params);
        const auto& cut = std::get<ExtrudeFeatureParams>(features[i+1].params);
        const auto& face = std::get<FaceReference>(holeSketch.plane);
        Plane3d stockPlane, holePlane;
        require(face.ownerFeature == features[1].id && cut.throughAll &&
            (face.role == FaceRole::ExtrusionSide) == (face.profileEdge != kInvalidSketchEntityId) &&
            cut.targetBody == features[i-1].id &&
            resolveSketchPlane(features, section.plane, stockPlane, error) &&
            resolveSketchPlane(features, holeSketch.plane, holePlane, error),
            "cuts retain their original stock reference");
        const double alignment = std::abs(vector(stockPlane.normal).Dot(vector(holePlane.normal)));
        if (face.role == FaceRole::ExtrusionSide)
            require(alignment < 1e-9, "side cut is perpendicular to the host sweep");
        else
            require(std::abs(alignment - 1) < 1e-9, "end-cap cut retains the host sweep axis");
    }
    if (!checkEditing) return;

    auto edited = section;
    const auto& sketch = std::get<SketchFeatureParams>(features[2].params);
    Plane3d stockPlane, moved;
    require(resolveSketchPlane(features, section.plane, stockPlane, error), "resolve stock sketch");
    const auto x = vector(stockPlane.xDirection);
    const auto y = vector(stockPlane.normal).Crossed(x);
    const auto move = [&](double& u, double& v) {
        const auto p = point(stockPlane.origin).Translated(x*u + y*v);
        if (std::abs(p.Y()-8) < 1e-9) {
            u += 3*x.Y();
            v += 3*y.Y();
        }
    };
    for (auto& entity : edited.entities) {
        auto& line = std::get<Line2D>(entity.geometry);
        move(line.x1,line.y1);
        move(line.x2,line.y2);
    }
    auto holes = fixture.holes;
    holes.front().entry.y = holes.front().span = 11;
    require(model.execute(EditFeatureCommand{features[0].id, edited}).success && model.rebuild() &&
        resolveSketchPlane(model.features(), sketch.plane, moved, error) &&
        std::abs(moved.origin.y-11) < 1e-9 &&
        sameMaterial(model.bodyShape(), reference(fixture.topWidth,11,holes)),
        "moving the profile side keeps the bore attached and through the wider host");
    require(model.execute(SetFeatureSuppressedCommand{features[3].id, true}).success && model.rebuild() &&
        sameMaterial(model.bodyShape(), reference(fixture.topWidth,11,{})), "suppressing the bore restores the stock");
}
} // namespace

int main() {
    try {
        Fixture tapered(10,false);
        check(tapered,true);
        Fixture rotated(12,true);
        rotated.rotate();
        check(rotated,false);
        // Retain the finite wall and entry, but erase the observed exit rim.
        tapered.scene.patches[2].loops.resize(1);
        SolidInferenceResult inferred;
        std::string error;
        require(inferSolids(tapered.scene,inferred,error) &&
            std::none_of(inferred.candidates.begin(),inferred.candidates.end(), [](const auto& candidate) {
                return sideCuts(candidate) != 0;
            }), "missing exit rim cannot generate a side through-all cut");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
