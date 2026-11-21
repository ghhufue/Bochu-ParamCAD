#include "reconstruction/parts/PlanarSurfaceJunctions.h"

#include "BoundaryGeometry.h"
#include "common/Math3.h"

#include <contour_fitting/ContourFitting.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>

namespace reconstruction {
namespace {
using namespace math3;
using namespace detail::assembly;
using P = modeling::Point2D;
using Interval = std::pair<double, double>;

struct Face {
    FittedPlanarSurface mesh;
    Vec3 origin, normal, x, y;
    double spacing = 0, reach = 0;
    std::vector<BoundaryLoop> rings;
    std::vector<bool> holes;
    std::vector<BoundaryLine> constraints;

    P project(Vec3 p) const {
        const auto delta = subtract(p, origin);
        return {dot(delta, x), dot(delta, y)};
    }
    Vec3 evaluate(P p) const { return add(origin, add(scale(x, p.x), scale(y, p.y))); }
};

void checkCancellation(const std::function<bool()>& cancelled) {
    if (cancelled && cancelled()) throw std::runtime_error("Cancelled");
}

Face prepare(PlanarSurfaceInput input) {
    if (input.id <= 0 || input.vertices.size() < 3 || input.triangles.empty() ||
        !std::isfinite(input.plane.offset) || !std::isfinite(input.residual) || input.residual < 0)
        throw std::invalid_argument("Invalid planar surface or missing finite mesh");

    Face face;
    const auto normalLength = length(input.plane.normal);
    face.normal = normalized(input.plane.normal);
    const auto shift = (dot(input.plane.normal, input.vertices.front()) + input.plane.offset) / normalLength;
    face.origin = subtract(input.vertices.front(), scale(face.normal, shift));
    const Vec3 reference = std::abs(face.normal.x) < .8 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    face.x = normalized(subtract(reference, scale(face.normal, dot(face.normal, reference))));
    face.y = cross(face.normal, face.x);

    std::vector<contour_fitting::Point2D> points;
    points.reserve(input.vertices.size());
    for (auto& vertex : input.vertices) {
        const auto p = face.project(vertex);
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            throw std::invalid_argument("Non-finite vertex in planar surface");
        points.push_back({p.x, p.y});
        vertex = face.evaluate(p);
    }
    for (const auto& triangle : input.triangles)
        for (const auto index : triangle)
            if (index >= points.size()) throw std::invalid_argument("Invalid planar mesh triangle");

    const auto contours = contour_fitting::extractTriangleContours(points, input.triangles);
    std::vector<double> spacings;
    const auto append = [&](const contour_fitting::IndexRing& ring, bool hole) {
        BoundaryLoop projected;
        projected.reserve(ring.size());
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const auto a = points[ring[i]], b = points[ring[(i + 1) % ring.size()]];
            projected.push_back({a.x, a.y});
            const auto distance = std::hypot(a.x - b.x, a.y - b.y);
            if (distance > 0) spacings.push_back(distance);
        }
        face.rings.push_back(std::move(projected));
        face.holes.push_back(hole);
    };
    for (const auto& contour : contours) {
        append(contour.outer, false);
        for (const auto& hole : contour.holes) append(hole, true);
    }
    if (spacings.empty()) throw std::invalid_argument("The planar mesh has no usable boundary");

    const auto median = spacings.begin() + (spacings.size() - 1) / 2;
    std::nth_element(spacings.begin(), median, spacings.end());
    face.spacing = *median;
    // Let nearby, independently sampled borders meet at their theoretical
    // intersection while keeping the search local to each surface's density.
    double area = 0;
    for (const auto& ring : face.rings) area += boundaryArea(ring);
    face.reach = std::min(.08 * std::sqrt(std::abs(area)),
        std::max(1.5 * face.spacing, std::min(3 * input.residual, 3 * face.spacing)));
    if (!(face.reach > 0) || !std::isfinite(face.reach))
        throw std::invalid_argument("The planar mesh has no usable area");
    face.mesh.id = input.id;
    face.mesh.vertices = std::move(input.vertices);
    face.mesh.triangles = std::move(input.triangles);
    return face;
}

std::vector<Interval> supportedIntervals(const Face& face, Vec3 origin, Vec3 direction,
                                         const std::function<bool()>& cancelled) {
    const auto transverse = cross(face.normal, direction);
    std::vector<P> coordinates;
    coordinates.reserve(face.mesh.vertices.size());
    for (const auto vertex : face.mesh.vertices) {
        const auto d = subtract(vertex, origin);
        coordinates.push_back({dot(d, direction), dot(d, transverse)});
    }

    std::vector<Interval> intervals;
    std::size_t processed = 0;
    for (const auto& triangle : face.mesh.triangles) {
        if (processed++ % 256 == 0) checkCancellation(cancelled);
        Interval extent{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
        const auto include = [&](double station) {
            extent.first = std::min(extent.first, station);
            extent.second = std::max(extent.second, station);
        };
        // Project only the portion of this triangle inside the strip. Its
        // extrema lie at vertices or where an edge crosses either strip side.
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const auto a = coordinates[triangle[edge]], b = coordinates[triangle[(edge + 1) % 3]];
            if (std::abs(a.y) <= face.reach) include(a.x);
            for (const double limit : {-face.reach, face.reach}) {
                if ((a.y < limit && b.y > limit) || (b.y < limit && a.y > limit))
                    include(a.x + (b.x - a.x) * (limit - a.y) / (b.y - a.y));
            }
        }
        if (extent.first < extent.second) intervals.push_back(extent);
    }
    std::sort(intervals.begin(), intervals.end());
    std::vector<Interval> merged;
    for (const auto& interval : intervals) {
        if (merged.empty() || interval.first > merged.back().second + .05 * face.spacing)
            merged.push_back(interval);
        else
            merged.back().second = std::max(merged.back().second, interval.second);
    }
    return merged;
}

void constrain(Face& face, const PlanarIntersectionSegment& segment, Vec3 direction) {
    const auto start = face.project(segment[0]), end = face.project(segment[1]);
    const auto transverse = cross(face.normal, direction);
    const double a = dot(transverse, face.x), b = dot(transverse, face.y);
    face.constraints.push_back({a, b, -a * start.x - b * start.y,
        static_cast<int>(face.constraints.size() + 1), start, std::array<P, 2>{start, end}});
}

PlanarSurfaceJunction intersect(Face& first, Face& second, const std::function<bool()>& cancelled) {
    PlanarSurfaceJunction result{first.mesh.id, second.mesh.id, PlanarJunctionKind::Disjoint, {}};
    const auto axis = cross(first.normal, second.normal);
    const auto axisLength = length(axis);
    constexpr double minimumSine = .01745240643728351; // One degree.
    if (axisLength < minimumSine) {
        result.kind = PlanarJunctionKind::NearlyParallel;
        return result;
    }

    const auto center = scale(add(first.origin, second.origin), .5);
    const auto firstShift = dot(first.normal, subtract(first.origin, center));
    const auto secondShift = dot(second.normal, subtract(second.origin, center));
    const auto shift = cross(subtract(scale(second.normal, firstShift), scale(first.normal, secondShift)), axis);
    const auto origin = add(center, scale(shift, 1 / (axisLength * axisLength)));
    const auto direction = scale(axis, 1 / axisLength);
    const auto a = supportedIntervals(first, origin, direction, cancelled);
    const auto b = supportedIntervals(second, origin, direction, cancelled);
    const auto minimumLength = std::min(first.reach, second.reach);
    std::size_t i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        const auto low = std::max(a[i].first, b[j].first);
        const auto high = std::min(a[i].second, b[j].second);
        if (high - low >= minimumLength) {
            auto& segment = result.segments.emplace_back(PlanarIntersectionSegment{
                add(origin, scale(direction, low)), add(origin, scale(direction, high))});
            constrain(first, segment, direction);
            constrain(second, segment, direction);
        }
        if (a[i].second <= b[j].second) ++i;
        else ++j;
    }
    if (!result.segments.empty()) result.kind = PlanarJunctionKind::Intersecting;
    return result;
}

bool replaceMesh(Face& face, const std::vector<BoundaryLoop>& rings,
                 const std::function<bool()>& cancelled) {
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    struct Domain { int parity = -1; };
    using Base = CGAL::Constrained_triangulation_face_base_2<Kernel>;
    using Info = CGAL::Triangulation_face_base_with_info_2<Domain, Kernel, Base>;
    using Data = CGAL::Triangulation_data_structure_2<CGAL::Triangulation_vertex_base_2<Kernel>, Info>;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Data, CGAL::Exact_predicates_tag>;
    CDT triangulation;
    for (const auto& ring : rings) {
        checkCancellation(cancelled);
        std::vector<Kernel::Point_2> points;
        points.reserve(ring.size());
        for (const auto p : ring) points.emplace_back(p.x, p.y);
        triangulation.insert_constraint(points.begin(), points.end(), true);
    }
    if (triangulation.dimension() != 2) return false;

    // Crossing a constrained edge changes the domain parity. This preserves
    // holes and disconnected islands without merging their separate loops.
    std::queue<CDT::Face_handle> pending;
    const auto outside = triangulation.infinite_face();
    outside->info().parity = 0;
    pending.push(outside);
    while (!pending.empty()) {
        const auto current = pending.front();
        pending.pop();
        for (int index = 0; index < 3; ++index) {
            const auto neighbor = current->neighbor(index);
            const int parity = current->info().parity ^ int(triangulation.is_constrained({current, index}));
            if (neighbor->info().parity == parity) continue;
            if (neighbor->info().parity != -1) return false;
            neighbor->info().parity = parity;
            pending.push(neighbor);
        }
    }

    std::vector<Vec3> vertices;
    std::vector<PlanarSurfaceTriangle> triangles;
    std::map<CDT::Vertex_handle, std::size_t> indices;
    for (auto triangle = triangulation.finite_faces_begin(); triangle != triangulation.finite_faces_end(); ++triangle) {
        if (triangle->info().parity != 1) continue;
        PlanarSurfaceTriangle output{};
        for (int corner = 0; corner < 3; ++corner) {
            const auto vertex = triangle->vertex(corner);
            const auto [it, added] = indices.emplace(vertex, vertices.size());
            if (added) vertices.push_back(face.evaluate({vertex->point().x(), vertex->point().y()}));
            output[corner] = it->second;
        }
        triangles.push_back(output);
    }
    if (triangles.empty()) return false;
    face.mesh.vertices = std::move(vertices);
    face.mesh.triangles = std::move(triangles);
    return true;
}

FittedPlanarSurface fitBoundaries(Face& face, const std::function<bool()>& cancelled) {
    if (!face.constraints.empty()) {
        BoundaryRecoveryDiagnostics diagnostics;
        auto simplified = face.rings;
        for (auto& ring : simplified)
            ring = simplifyBoundary(ring, std::min(.45 * face.spacing, .5 * face.reach), diagnostics);
        if (!sameBoundaryTopology(face.rings, simplified)) simplified = face.rings;

        auto fitted = simplified;
        std::size_t snapped = 0;
        for (auto& ring : fitted) {
            checkCancellation(cancelled);
            ring = snapBoundary(ring, face.constraints, face.spacing, face.reach, diagnostics, snapped);
        }
        if (snapped && sameBoundaryTopology(face.rings, fitted) && replaceMesh(face, fitted, cancelled)) {
            face.mesh.boundaryAdjusted = true;
            face.rings = std::move(fitted);
        }
    }

    for (std::size_t i = 0; i < face.rings.size(); ++i) {
        auto& boundary = face.mesh.boundaries.emplace_back();
        boundary.hole = face.holes[i];
        boundary.vertices.reserve(face.rings[i].size());
        for (const auto p : face.rings[i]) boundary.vertices.push_back(face.evaluate(p));
    }
    return std::move(face.mesh);
}
} // namespace

PlanarSurfaceJunctionResult fitPlanarSurfaceJunctions(std::vector<PlanarSurfaceInput> inputs,
    const std::function<bool()>& cancelled) {
    PlanarSurfaceJunctionResult result;
    std::vector<Face> faces;
    faces.reserve(inputs.size());
    for (auto& input : inputs) {
        checkCancellation(cancelled);
        faces.push_back(prepare(std::move(input)));
    }
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (std::size_t j = i + 1; j < faces.size(); ++j) {
            checkCancellation(cancelled);
            result.junctions.push_back(intersect(faces[i], faces[j], cancelled));
        }
    }
    for (auto& face : faces) {
        checkCancellation(cancelled);
        result.surfaces.push_back(fitBoundaries(face, cancelled));
    }
    return result;
}

} // namespace reconstruction
