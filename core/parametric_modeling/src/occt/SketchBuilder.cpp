#include "occt/Builders.h"
#include "occt/OcctError.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GC_MakeCircle.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_State.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <type_traits>
#include <vector>

namespace modeling::occt {
namespace {

bool finite(double value) {
    return std::isfinite(value);
}

bool samePoint(double ax, double ay, double bx, double by) {
    return std::abs(ax - bx) <= Precision::Confusion() &&
        std::abs(ay - by) <= Precision::Confusion();
}

gp_Pnt pointOnPlane(const PlaneFrame& frame, double x, double y) {
    gp_Vec displacement(frame.xDirection);
    displacement.Multiply(x);
    gp_Vec yDisplacement(frame.yDirection);
    yDisplacement.Multiply(y);
    displacement.Add(yDisplacement);
    return frame.origin.Translated(displacement);
}

bool buildRectangle(
    const Rectangle2D& rectangle,
    const PlaneFrame& frame,
    TopoDS_Wire& wire,
    std::string& error) {
    if (!finite(rectangle.x) || !finite(rectangle.y) ||
        !finite(rectangle.width) || !finite(rectangle.height) ||
        rectangle.width <= Precision::Confusion() ||
        rectangle.height <= Precision::Confusion()) {
        error = "Rectangle dimensions and coordinates must be finite and positive";
        return false;
    }

    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(pointOnPlane(frame, rectangle.x, rectangle.y));
    polygon.Add(pointOnPlane(frame, rectangle.x + rectangle.width, rectangle.y));
    polygon.Add(pointOnPlane(
        frame, rectangle.x + rectangle.width, rectangle.y + rectangle.height));
    polygon.Add(pointOnPlane(frame, rectangle.x, rectangle.y + rectangle.height));
    polygon.Close();
    if (!polygon.IsDone()) {
        error = "OpenCASCADE could not build the rectangle wire";
        return false;
    }
    wire = polygon.Wire();
    return true;
}

bool buildCircle(
    const Circle2D& circle,
    const PlaneFrame& frame,
    TopoDS_Wire& wire,
    std::string& error) {
    if (!finite(circle.x) || !finite(circle.y) || !finite(circle.radius) ||
        circle.radius <= Precision::Confusion()) {
        error = "Circle center and radius must be finite, with a positive radius";
        return false;
    }

    const gp_Pnt center = pointOnPlane(frame, circle.x, circle.y);
    const gp_Ax2 axes(center, frame.normal, frame.xDirection);
    const Handle(Geom_Circle) geometry = new Geom_Circle(axes, circle.radius);
    BRepBuilderAPI_MakeEdge edgeBuilder(geometry);
    if (!edgeBuilder.IsDone()) {
        error = "OpenCASCADE could not build the circle edge";
        return false;
    }
    BRepBuilderAPI_MakeWire wireBuilder(edgeBuilder.Edge());
    if (!wireBuilder.IsDone()) {
        error = "OpenCASCADE could not build the circle wire";
        return false;
    }
    wire = wireBuilder.Wire();
    return true;
}

bool buildEdgeWires(
    const std::vector<SketchEntity>& entities,
    const PlaneFrame& frame,
    std::vector<TopoDS_Wire>& wires,
    std::string& error) {
    struct ProfileEdge {
        Point2D start, end;
        TopoDS_Edge shape;
        bool curved;
    };
    const auto sameEndpoint = [](Point2D a, Point2D b) { return samePoint(a.x, a.y, b.x, b.y); };
    const auto onPlane = [&](Point2D p) { return pointOnPlane(frame, p.x, p.y); };
    std::vector<ProfileEdge> edges;
    edges.reserve(entities.size());
    for (const auto& entity : entities) {
        if (const auto* line = std::get_if<Line2D>(&entity.geometry)) {
            const Point2D a{line->x1, line->y1}, b{line->x2, line->y2};
            if (!finite(a.x) || !finite(a.y) || !finite(b.x) || !finite(b.y) || sameEndpoint(a, b)) {
                error = "Line coordinates must be finite and endpoints distinct";
                return false;
            }
            BRepBuilderAPI_MakeEdge edge(onPlane(a), onPlane(b));
            if (!edge.IsDone()) {
                error = "OpenCASCADE could not build a line edge";
                return false;
            }
            edges.push_back({a, b, edge.Edge(), false});
        } else {
            const auto* arc = std::get_if<Arc2D>(&entity.geometry);
            if (!arc || !arcParameters(*arc)) {
                error = "A circular arc needs three distinct, non-collinear points";
                return false;
            }
            GC_MakeArcOfCircle curve(onPlane(arc->start), onPlane(arc->middle), onPlane(arc->end));
            if (!curve.IsDone()) {
                error = "OpenCASCADE could not fit a circular arc";
                return false;
            }
            BRepBuilderAPI_MakeEdge edge(curve.Value());
            if (!edge.IsDone()) {
                error = "OpenCASCADE could not build an arc edge";
                return false;
            }
            edges.push_back({arc->start, arc->end, edge.Edge(), true});
        }
    }

    std::vector<bool> used(edges.size(), false);
    for (std::size_t first = 0; first < edges.size(); ++first) {
        if (used[first]) continue;
        BRepBuilderAPI_MakeWire wireBuilder;
        wireBuilder.Add(edges[first].shape);
        used[first] = true;
        std::size_t count = 1;
        bool curved = edges[first].curved;
        const auto start = edges[first].start;
        auto end = edges[first].end;
        while (!sameEndpoint(end, start)) {
            std::size_t index = 0;
            for (; index < edges.size(); ++index) {
                if (used[index]) continue;
                const auto& candidate = edges[index];
                if (sameEndpoint(candidate.start, end)) {
                    wireBuilder.Add(candidate.shape);
                    end = candidate.end;
                } else if (sameEndpoint(candidate.end, end)) {
                    wireBuilder.Add(TopoDS::Edge(candidate.shape.Reversed()));
                    end = candidate.start;
                } else {
                    continue;
                }
                used[index] = true;
                ++count;
                curved = curved || candidate.curved;
                break;
            }
            if (index == edges.size() || !wireBuilder.IsDone()) {
                error = "Sketch edges contain an open or disconnected chain";
                return false;
            }
        }
        if (count < (curved ? 2U : 3U) || !wireBuilder.IsDone()) {
            error = "Sketch edges do not form a closed profile";
            return false;
        }
        wires.push_back(wireBuilder.Wire());
    }
    return true;
}

bool buildFaceFromWires(
    const std::vector<TopoDS_Wire>& wires,
    const PlaneFrame& frame,
    TopoDS_Face& output,
    std::string& error) {
    if (wires.empty()) {
        error = "Sketch must contain one closed profile";
        return false;
    }

    std::vector<TopoDS_Face> candidateFaces;
    std::vector<double> areas;
    candidateFaces.reserve(wires.size());
    areas.reserve(wires.size());
    // Orient every wire against the same sketch plane. Independently inferred
    // planes can have opposite normals for clockwise and counterclockwise loops.
    const gp_Pln plane(gp_Ax3(frame.origin, frame.normal, frame.xDirection));
    for (const TopoDS_Wire& wire : wires) {
        BRepBuilderAPI_MakeFace candidateBuilder(plane, wire, true);
        if (!candidateBuilder.IsDone()) {
            error = "Sketch wire is not a valid closed planar profile";
            return false;
        }
        const TopoDS_Face candidate = candidateBuilder.Face();
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(candidate, properties);
        candidateFaces.push_back(candidate);
        areas.push_back(std::abs(properties.Mass()));
    }

    const auto outerIterator = std::max_element(areas.begin(), areas.end());
    const std::size_t outerIndex = static_cast<std::size_t>(
        std::distance(areas.begin(), outerIterator));
    const TopoDS_Wire outerWire = BRepTools::OuterWire(candidateFaces[outerIndex]);
    BRepBuilderAPI_MakeFace faceBuilder(plane, outerWire, true);

    for (std::size_t index = 0; index < candidateFaces.size(); ++index) {
        if (index == outerIndex) {
            continue;
        }
        const TopoDS_Wire innerWire = BRepTools::OuterWire(candidateFaces[index]);
        TopExp_Explorer vertexExplorer(innerWire, TopAbs_VERTEX);
        if (!vertexExplorer.More()) {
            error = "Sketch contains an empty profile wire";
            return false;
        }
        const gp_Pnt sample = BRep_Tool::Pnt(TopoDS::Vertex(vertexExplorer.Current()));
        BRepClass_FaceClassifier classifier(
            candidateFaces[outerIndex], sample, Precision::Confusion());
        if (classifier.State() != TopAbs_IN) {
            error = "Sketch profiles must form one outer loop with inner holes";
            return false;
        }
        TopoDS_Wire hole = innerWire;
        hole.Reverse();
        faceBuilder.Add(hole);
    }

    if (!faceBuilder.IsDone()) {
        error = "OpenCASCADE could not build the multi-loop sketch face";
        return false;
    }
    output = faceBuilder.Face();
    if (!BRepCheck_Analyzer(output).IsValid()) {
        error = "Sketch produced an invalid OpenCASCADE face";
        return false;
    }
    return true;
}

}  // namespace

PlaneFrame datumPlaneFrame(DatumPlane plane) {
    switch (plane) {
    case DatumPlane::XY:
        return {};
    case DatumPlane::YZ:
        return {gp_Pnt(0.0, 0.0, 0.0),
                gp_Dir(0.0, 1.0, 0.0),
                gp_Dir(0.0, 0.0, 1.0),
                gp_Dir(1.0, 0.0, 0.0)};
    case DatumPlane::XZ:
        return {gp_Pnt(0.0, 0.0, 0.0),
                gp_Dir(1.0, 0.0, 0.0),
                gp_Dir(0.0, 0.0, 1.0),
                gp_Dir(0.0, -1.0, 0.0)};
    }
    return {};
}

bool buildSketchFace(
    const SketchFeatureParams& params,
    const PlaneFrame& frame,
    TopoDS_Face& output,
    std::string& error) {
    try {
        std::vector<SketchEntity> edgeEntities;
        std::vector<TopoDS_Wire> wires;
        edgeEntities.reserve(params.entities.size());
        for (const SketchEntity& entity : params.entities) {
            if (std::holds_alternative<Point2D>(entity.geometry)) {
                continue;
            }
            if (const auto* rectangle = std::get_if<Rectangle2D>(&entity.geometry)) {
                TopoDS_Wire wire;
                if (!buildRectangle(*rectangle, frame, wire, error)) {
                    return false;
                }
                wires.push_back(wire);
            } else if (const auto* circle = std::get_if<Circle2D>(&entity.geometry)) {
                TopoDS_Wire wire;
                if (!buildCircle(*circle, frame, wire, error)) {
                    return false;
                }
                wires.push_back(wire);
            } else {
                edgeEntities.push_back(entity);
            }
        }

        if (!buildEdgeWires(edgeEntities, frame, wires, error)) {
            return false;
        }
        return buildFaceFromWires(wires, frame, output, error);
    } catch (const Standard_Failure& failure) {
        error = std::string("OpenCASCADE sketch failure: ") +
            failureMessage(failure);
        return false;
    }
}

}  // namespace modeling::occt
