#include "contour_fitting/ContourFitting.h"
#include "contour_fitting/detail/BoundaryWalk.h"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyline_simplification_2/Stop_above_cost_threshold.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>
#include <CGAL/Polyline_simplification_2/Vertex_base_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/squared_distance_2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>

namespace contour_fitting {
namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_2;
using Polygon = CGAL::Polygon_2<Kernel>;

struct FaceState {
    bool kept = false;
    std::array<bool, 3> boundaryVisited{};
    bool componentVisited = false;
};

using VertexBase = CGAL::Triangulation_vertex_base_with_info_2<std::size_t, Kernel>;
using FaceBase = CGAL::Triangulation_face_base_with_info_2<FaceState, Kernel>;
using Triangulation = CGAL::Delaunay_triangulation_2<Kernel,
    CGAL::Triangulation_data_structure_2<VertexBase, FaceBase>>;

double estimateRadius(const Triangulation& triangulation)
{
    std::vector<double> nearest(triangulation.number_of_vertices(), std::numeric_limits<double>::infinity());
    for (auto edge = triangulation.finite_edges_begin(); edge != triangulation.finite_edges_end(); ++edge) {
        const auto [face, side] = *edge;
        const auto first = face->vertex(triangulation.ccw(side));
        const auto last = face->vertex(triangulation.cw(side));
        const double distance = CGAL::squared_distance(first->point(), last->point());
        nearest[first->info()] = std::min(nearest[first->info()], distance);
        nearest[last->info()] = std::min(nearest[last->info()], distance);
    }
    const auto percentile = nearest.begin() + (nearest.size() - 1) * 9 / 10;
    std::nth_element(nearest.begin(), percentile, nearest.end());
    const double radius = 6 * std::sqrt(*percentile);

    // Uniform interior samples need a smaller radius than projected data with
    // dense rows. Bound the local estimate using the mean rectangular density.
    const auto bounds = CGAL::bbox_2(triangulation.points_begin(), triangulation.points_end());
    const double meanSpacing = std::sqrt(bounds.xmax() - bounds.xmin())
        * std::sqrt(bounds.ymax() - bounds.ymin()) / std::sqrt(double(nearest.size()));
    return std::min(radius, 2 * meanSpacing);
}

bool kept(const Triangulation& triangulation, Triangulation::Face_handle face)
{
    return !triangulation.is_infinite(face) && face->info().kept;
}

std::size_t discardWeakRegions(Triangulation& triangulation, std::size_t minimumPoints)
{
    // Connected retained triangles define material. Their distinct vertices
    // support the outer region, with samples inside holes counted separately.
    std::vector<std::size_t> vertexStamp(triangulation.number_of_vertices(), 0);
    std::vector<Triangulation::Face_handle> component;
    std::size_t stamp = 0, discarded = 0;
    for (auto face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face) {
        if (!face->info().kept || face->info().componentVisited) continue;
        ++stamp;
        component.clear();
        component.push_back(face);
        face->info().componentVisited = true;
        std::size_t points = 0;
        for (std::size_t i = 0; i < component.size(); ++i) {
            const auto current = component[i];
            for (int side = 0; side < 3; ++side) {
                if (points < minimumPoints) {
                    auto& seen = vertexStamp[current->vertex(side)->info()];
                    if (seen != stamp) {
                        seen = stamp;
                        ++points;
                    }
                }
                const auto neighbor = current->neighbor(side);
                if (!kept(triangulation, neighbor) || neighbor->info().componentVisited) continue;
                neighbor->info().componentVisited = true;
                component.push_back(neighbor);
            }
        }
        if (points >= minimumPoints) continue;
        for (const auto triangle : component) triangle->info().kept = false;
        ++discarded;
    }
    return discarded;
}

std::vector<Polygon> traceRings(Triangulation& triangulation, double radius,
                               std::size_t minimumPoints, std::size_t& discarded)
{
    for (auto face = triangulation.finite_faces_begin(); face != triangulation.finite_faces_end(); ++face) {
        face->info().kept = CGAL::squared_radius(face->vertex(0)->point(), face->vertex(1)->point(),
            face->vertex(2)->point()) <= radius * radius;
    }
    if (minimumPoints > 3) discarded = discardWeakRegions(triangulation, minimumPoints);
    std::vector<Point> vertices(triangulation.number_of_vertices());
    for (auto vertex = triangulation.finite_vertices_begin(); vertex != triangulation.finite_vertices_end(); ++vertex)
        vertices[vertex->info()] = vertex->point();
    std::vector<Polygon> rings;
    for (const auto& cycle : detail::traceBoundaryRings(triangulation)) {
        Polygon ring;
        for (const auto index : cycle) ring.push_back(vertices[index]);
        if (ring.is_simple() && ring.orientation() != CGAL::COLLINEAR) rings.push_back(std::move(ring));
    }
    return rings;
}

void simplify(std::vector<Polygon>& rings, double tolerance)
{
    if (rings.empty() || tolerance == 0) return;
    namespace ps = CGAL::Polyline_simplification_2;
    using Data = CGAL::Triangulation_data_structure_2<ps::Vertex_base_2<Kernel>,
        CGAL::Constrained_triangulation_face_base_2<Kernel>>;
    using Cdt = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Data, CGAL::Exact_predicates_tag>;
    CGAL::Constrained_triangulation_plus_2<Cdt> constraints;
    std::vector<decltype(constraints)::Constraint_id> ids;
    for (const auto& ring : rings)
        ids.push_back(constraints.insert_constraint(ring.vertices_begin(), ring.vertices_end(), true));

    ps::simplify(constraints, ps::Squared_distance_cost{}, ps::Stop_above_cost_threshold{tolerance * tolerance});
    std::vector<Polygon> simplified;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        Polygon ring;
        for (const auto vertex : constraints.vertices_in_constraint(ids[i])) ring.push_back(vertex->point());
        if (ring.size() > 1 && ring[0] == ring[ring.size() - 1]) ring.erase(ring.vertices_end() - 1);
        if (ring.size() < 3 || !ring.is_simple() || ring.orientation() != rings[i].orientation()) return;
        // Closed constraints can retain an anchor in the middle of a straight edge.
        Polygon corners;
        for (std::size_t j = 0; j < ring.size(); ++j) {
            const auto& previous = ring[(j + ring.size() - 1) % ring.size()];
            const auto& next = ring[(j + 1) % ring.size()];
            if (!CGAL::collinear(previous, ring[j], next)) corners.push_back(ring[j]);
        }
        simplified.push_back(std::move(corners));
    }
    rings = std::move(simplified);
}

Ring outputRing(const Polygon& polygon)
{
    Ring result;
    auto first = std::min_element(polygon.vertices_begin(), polygon.vertices_end());
    for (auto point = first; point != polygon.vertices_end(); ++point) result.push_back({point->x(), point->y()});
    for (auto point = polygon.vertices_begin(); point != first; ++point) result.push_back({point->x(), point->y()});
    return result;
}

struct RingGroup {
    std::size_t outer;
    std::vector<std::size_t> holes;
};

std::vector<RingGroup> groupRings(const std::vector<Polygon>& rings)
{
    std::vector<std::size_t> order(rings.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
        return std::abs(rings[a].area()) > std::abs(rings[b].area());
    });
    std::vector<RingGroup> groups;
    for (const auto index : order) {
        const auto& ring = rings[index];
        if (ring.is_counterclockwise_oriented()) {
            groups.push_back({index, {}});
            continue;
        }
        // Larger rings were processed first. The last containing outer is the
        // smallest one, so a nested island receives its own holes.
        const auto containing = std::find_if(groups.rbegin(), groups.rend(), [&](const RingGroup& group) {
            for (const auto& point : ring) {
                const auto side = rings[group.outer].bounded_side(point);
                if (side != CGAL::ON_BOUNDARY) return side == CGAL::ON_BOUNDED_SIDE;
            }
            return false;
        });
        if (containing != groups.rend()) containing->holes.push_back(index);
    }
    return groups;
}

} // namespace

Result fitContours(const std::vector<Point2D>& points, const Options& options)
{
    const auto valid = [](double value) { return std::isfinite(value) && value >= 0; };
    if (!valid(options.alphaRadius) || (options.simplificationTolerance && !valid(*options.simplificationTolerance)))
        throw std::invalid_argument("Contour fitting options must be finite and nonnegative");
    if (!valid(options.minimumRegionFraction) || options.minimumRegionFraction > 1)
        throw std::invalid_argument("Minimum region fraction must be in [0, 1]");
    if (!std::isfinite(options.automaticAlphaScale) || options.automaticAlphaScale <= 0)
        throw std::invalid_argument("Automatic alpha scale must be finite and positive");
    std::vector<Point> input;
    input.reserve(points.size());
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            throw std::invalid_argument("Contour fitting requires finite 2D coordinates");
        input.emplace_back(point.x, point.y);
    }
    std::sort(input.begin(), input.end());
    input.erase(std::unique(input.begin(), input.end()), input.end());
    Result result;
    result.uniquePointCount = input.size();
    result.alphaRadius = options.alphaRadius;
    result.simplificationTolerance = options.simplificationTolerance.value_or(options.alphaRadius / 10);
    if (input.size() < 3) return result;
    Triangulation triangulation(input.begin(), input.end());
    if (triangulation.dimension() != 2) return result;

    std::size_t index = 0;
    for (auto vertex = triangulation.finite_vertices_begin(); vertex != triangulation.finite_vertices_end(); ++vertex)
        vertex->info() = index++;
    if (result.alphaRadius == 0) result.alphaRadius = estimateRadius(triangulation)*options.automaticAlphaScale;
    result.simplificationTolerance = options.simplificationTolerance.value_or(result.alphaRadius / 10);
    auto fractionMinimum = std::size_t(options.minimumRegionFraction * result.uniquePointCount);
    if (double(fractionMinimum) / double(result.uniquePointCount) < options.minimumRegionFraction)
        ++fractionMinimum;
    const auto minimumPoints = std::max(options.minimumRegionPoints, fractionMinimum);
    auto rings = traceRings(triangulation, result.alphaRadius, minimumPoints, result.discardedRegionCount);
    simplify(rings, result.simplificationTolerance);
    for (const auto& group : groupRings(rings)) {
        Contour contour{outputRing(rings[group.outer]), {}};
        for (const auto hole : group.holes) contour.holes.push_back(outputRing(rings[hole]));
        result.contours.push_back(std::move(contour));
    }
    return result;
}

std::vector<TriangleContour> extractTriangleContours(
    const std::vector<Point2D>& vertices, const std::vector<std::array<std::size_t, 3>>& triangles)
{
    using Edge = std::pair<std::size_t, int>;
    constexpr Edge noNeighbor{0, -1};
    struct Face {
        std::array<std::size_t, 3> vertices;
        std::array<Edge, 3> neighbors;
        std::array<bool, 3> visited{};
    };
    std::vector<Point> points;
    points.reserve(vertices.size());
    for (const auto& point : vertices) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return {};
        points.emplace_back(point.x, point.y);
    }
    const auto nextSide = [](int side) { return (side + 1) % 3; };
    const auto previousSide = [](int side) { return (side + 2) % 3; };
    std::vector<Face> faces;
    faces.reserve(triangles.size());
    std::map<std::pair<std::size_t, std::size_t>, Edge> edges;
    for (auto triangle : triangles) {
        if (*std::max_element(triangle.begin(), triangle.end()) >= points.size()) return {};
        const auto orientation = CGAL::orientation(points[triangle[0]], points[triangle[1]], points[triangle[2]]);
        if (orientation == CGAL::COLLINEAR) continue;
        if (orientation == CGAL::CLOCKWISE) std::swap(triangle[1], triangle[2]);
        const auto face = faces.size();
        faces.push_back({triangle, {noNeighbor, noNeighbor, noNeighbor}});
        for (int side = 0; side < 3; ++side) {
            const auto from = triangle[nextSide(side)], to = triangle[previousSide(side)];
            const auto [entry, inserted] = edges.emplace(std::minmax(from, to), Edge{face, side});
            if (inserted) continue;
            const auto [other, otherSide] = entry->second;
            if (faces[other].neighbors[otherSide] != noNeighbor ||
                faces[other].vertices[nextSide(otherSide)] != to) return {};
            faces[other].neighbors[otherSide] = {face, side};
            faces[face].neighbors[side] = {other, otherSide};
        }
    }
    const auto next = [&](Edge edge) {
        auto [face, side] = edge;
        side = nextSide(side);
        while (faces[face].neighbors[side] != noNeighbor) {
            const auto neighbor = faces[face].neighbors[side];
            face = neighbor.first;
            side = nextSide(neighbor.second);
        }
        return Edge{face, side};
    };
    const auto visit = [&](Edge edge) {
        auto [face, side] = edge;
        faces[face].visited[side] = true;
        return faces[face].vertices[nextSide(side)];
    };
    std::vector<IndexRing> rings;
    for (std::size_t face = 0; face < faces.size(); ++face) {
        for (int side = 0; side < 3; ++side) {
            if (!faces[face].visited[side] && faces[face].neighbors[side] == noNeighbor)
                detail::appendBoundaryRings(Edge{face, side}, next, visit, rings);
        }
    }
    std::vector<Polygon> polygons;
    polygons.reserve(rings.size());
    for (auto& ring : rings) {
        Polygon polygon;
        for (const auto vertex : ring) polygon.push_back(points[vertex]);
        if (!polygon.is_simple() || polygon.orientation() == CGAL::COLLINEAR) return {};
        const auto first = std::min_element(ring.begin(), ring.end(), [&](auto a, auto b) {
            return points[a] < points[b];
        });
        std::rotate(ring.begin(), first, ring.end());
        polygons.push_back(std::move(polygon));
    }
    std::vector<TriangleContour> contours;
    for (const auto& group : groupRings(polygons)) {
        TriangleContour contour{std::move(rings[group.outer]), {}};
        for (const auto hole : group.holes) contour.holes.push_back(std::move(rings[hole]));
        contours.push_back(std::move(contour));
    }
    return contours;
}

} // namespace contour_fitting
