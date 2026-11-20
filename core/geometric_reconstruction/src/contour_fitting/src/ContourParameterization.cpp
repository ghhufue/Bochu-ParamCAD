#include "contour_fitting/ContourParameterization.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/intersections.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace contour_fitting {
namespace {
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_2;
using Segment = Kernel::Segment_2;
using Polygon = CGAL::Polygon_2<Kernel>;
constexpr double pi = 3.14159265358979323846;

Point point(Point2D p) { return {p.x, p.y}; }
bool same(Point2D a, Point2D b) { return a.x == b.x && a.y == b.y; }
double distance(Point2D a, Point2D b) { return std::hypot(a.x - b.x, a.y - b.y); }

double segmentDistance(Point2D p, Point2D a, Point2D b)
{
    if (same(p, a) || same(p, b)) return 0;
    const double x = b.x - a.x, y = b.y - a.y;
    const double lengthSquared = x*x + y*y;
    const double t = lengthSquared > 0 ? std::clamp(((p.x-a.x)*x + (p.y-a.y)*y)/lengthSquared, 0., 1.) : 0.;
    return distance(p, {a.x+t*x, a.y+t*y});
}

bool normalize(Ring& ring, bool outer)
{
    if (!std::all_of(ring.begin(), ring.end(), [](auto p) { return std::isfinite(p.x) && std::isfinite(p.y); }))
        return false;
    ring.erase(std::unique(ring.begin(), ring.end(), same), ring.end());
    if (ring.size() > 1 && same(ring.front(), ring.back())) ring.pop_back();
    if (ring.size() < 3) return false;
    Polygon polygon;
    for (const auto p : ring) polygon.push_back(point(p));
    if (!polygon.is_simple() || polygon.orientation() == CGAL::COLLINEAR) return false;
    if (polygon.is_counterclockwise_oriented() != outer) std::reverse(ring.begin(), ring.end());
    return true;
}

struct Boundary {
    Polygon polygon;
    std::vector<Segment> edges;
    std::vector<CGAL::Bbox_2> edgeBounds;
    std::optional<Circle> circle;
    CGAL::Bbox_2 bounds;
    Point2D sample;

    explicit Boundary(const Ring& ring) : sample(ring.front()) {
        for (const auto p : ring) polygon.push_back(point(p));
        bounds = polygon.bbox();
        for (auto edge = polygon.edges_begin(); edge != polygon.edges_end(); ++edge) {
            edges.push_back(*edge);
            edgeBounds.push_back(edge->bbox());
        }
    }
    explicit Boundary(Circle c) : circle(c),
        bounds(c.center.x-c.radius, c.center.y-c.radius, c.center.x+c.radius, c.center.y+c.radius),
        sample{c.center.x+c.radius, c.center.y} {}

    bool contains(Point2D p) const {
        return circle ? distance(p, circle->center) < circle->radius
                      : polygon.bounded_side(point(p)) == CGAL::ON_BOUNDED_SIDE;
    }
};

bool overlaps(const CGAL::Bbox_2& a, const CGAL::Bbox_2& b)
{
    return a.xmin() <= b.xmax() && b.xmin() <= a.xmax()
        && a.ymin() <= b.ymax() && b.ymin() <= a.ymax();
}

bool touchesCircle(const Boundary& round, const Boundary& other)
{
    const auto& circle = *round.circle;
    if (other.circle) {
        const double centers = distance(circle.center, other.circle->center);
        return centers <= circle.radius + other.circle->radius
            && centers >= std::abs(circle.radius - other.circle->radius);
    }
    for (std::size_t i = 0; i < other.edges.size(); ++i) {
        if (!overlaps(round.bounds, other.edgeBounds[i])) continue;
        const auto& edge = other.edges[i];
        const Point2D a{edge.source().x(), edge.source().y()}, b{edge.target().x(), edge.target().y()};
        if (std::max(distance(circle.center,a), distance(circle.center,b)) >= circle.radius
            && segmentDistance(circle.center,a,b) <= circle.radius) return true;
    }
    return false;
}

enum class Relation { Disjoint, Contains, Inside, Touching };

Relation relation(const Boundary& first, const Boundary& second)
{
    if (!overlaps(first.bounds, second.bounds)) return Relation::Disjoint;
    if (first.circle || second.circle) {
        if (first.circle ? touchesCircle(first, second) : touchesCircle(second, first)) return Relation::Touching;
    } else {
        for (std::size_t i = 0; i < first.edges.size(); ++i) {
            if (!overlaps(first.edgeBounds[i], second.bounds)) continue;
            for (std::size_t j = 0; j < second.edges.size(); ++j)
                if (overlaps(first.edgeBounds[i], second.edgeBounds[j])
                    && CGAL::do_intersect(first.edges[i], second.edges[j])) return Relation::Touching;
        }
    }
    if (first.contains(second.sample)) return Relation::Contains;
    if (second.contains(first.sample)) return Relation::Inside;
    return Relation::Disjoint;
}

std::vector<const Ring*> flatten(const std::vector<Contour>& contours)
{
    std::vector<const Ring*> rings;
    for (const auto& contour : contours) {
        rings.push_back(&contour.outer);
        for (const auto& hole : contour.holes) rings.push_back(&hole);
    }
    return rings;
}

std::vector<Boundary> boundaries(const std::vector<const Ring*>& rings)
{
    std::vector<Boundary> result;
    result.reserve(rings.size());
    for (const auto* ring : rings) result.emplace_back(*ring);
    return result;
}

using Topology = std::vector<std::vector<Relation>>;
std::optional<Topology> topologyOf(const std::vector<Contour>& contours, const std::vector<Boundary>& rings,
                                   const std::function<bool()>& cancelled)
{
    Topology topology(rings.size(), std::vector<Relation>(rings.size(), Relation::Disjoint));
    for (std::size_t i = 0; i < rings.size(); ++i) {
        if (cancelled && cancelled()) return {};
        for (std::size_t j = i + 1; j < rings.size(); ++j) {
            const auto type = relation(rings[i], rings[j]);
            if (type == Relation::Touching) return {};
            topology[i][j] = type;
            topology[j][i] = type == Relation::Contains ? Relation::Inside
                : type == Relation::Inside ? Relation::Contains : type;
        }
    }
    std::size_t outer = 0;
    for (const auto& contour : contours) {
        const auto end = outer + 1 + contour.holes.size();
        for (auto hole = outer + 1; hole < end; ++hole) {
            if (topology[outer][hole] != Relation::Contains) return {};
            for (auto neighbor = hole + 1; neighbor < end; ++neighbor)
                if (topology[hole][neighbor] != Relation::Disjoint) return {};
        }
        std::size_t other = 0;
        for (const auto& component : contours) {
            if (other != outer && topology[outer][other] == Relation::Contains
                && std::none_of(topology[other].begin() + outer + 1, topology[other].begin() + end,
                                [](auto type) { return type == Relation::Inside; })) return {};
            other += 1 + component.holes.size();
        }
        outer = end;
    }
    return topology;
}

bool matches(const Boundary& candidate, std::size_t index,
             const std::vector<Boundary>& others, const Topology& topology)
{
    for (std::size_t j = 0; j < others.size(); ++j)
        if (j != index && relation(candidate, others[j]) != topology[index][j]) return false;
    return true;
}

struct Deviation {
    double bound = 0;
    std::vector<double> samples;
};

std::optional<Deviation> measurePolyline(const Ring& source, const Ring& fitted)
{
    // Simplification removes vertices, preserving their cyclic order. Each
    // source span starts and ends at its chord endpoints. Projection onto the
    // chord covers it continuously, so the forward bound also bounds reverse error.
    const auto first = std::find_if(source.begin(), source.end(), [&](auto p) { return same(p, fitted.front()); });
    if (first == source.end()) return {};
    std::size_t index = std::size_t(first - source.begin()), walked = 0;
    Deviation result;
    result.samples.reserve(source.size() + fitted.size());
    for (std::size_t edge = 0; edge < fitted.size(); ++edge) {
        const auto a = fitted[edge], b = fitted[(edge+1)%fitted.size()];
        const Point2D middle{(a.x+b.x)*.5, (a.y+b.y)*.5};
        double reverse = std::numeric_limits<double>::infinity();
        do {
            const auto previous = index;
            index = (index+1)%source.size();
            if (++walked > source.size()) return {};
            const double forward = segmentDistance(source[index], a, b);
            result.bound = std::max(result.bound, forward);
            result.samples.push_back(forward);
            reverse = std::min(reverse, segmentDistance(middle, source[previous], source[index]));
        } while (!same(source[index], b));
        result.samples.push_back(reverse);
    }
    if (walked != source.size()) return {};
    for (auto& sample : result.samples) sample = std::min(sample, result.bound);
    return result;
}

std::optional<Circle> fitCircle(const Ring& ring)
{
    if (ring.size() < 8) return {};
    // Centered least squares for x² + y² = 2*cx*x + 2*cy*y + constant.
    Point2D mean;
    for (const auto p : ring) { mean.x += p.x/ring.size(); mean.y += p.y/ring.size(); }
    double xx = 0, xy = 0, yy = 0, xz = 0, yz = 0;
    for (const auto p : ring) {
        const double x = p.x - mean.x, y = p.y - mean.y, z = x*x + y*y;
        xx += x*x; xy += x*y; yy += y*y; xz += x*z; yz += y*z;
    }
    const double determinant = xx*yy - xy*xy;
    if (determinant <= 1e-10*(xx+yy)*(xx+yy)) return {};
    const double x = .5*(yy*xz - xy*yz)/determinant, y = .5*(xx*yz - xy*xz)/determinant;
    Circle circle{{mean.x+x, mean.y+y}, std::sqrt((xx+yy)/ring.size() + x*x+y*y)};
    if (!std::isfinite(circle.radius) || circle.radius <= 0) return {};
    std::vector<double> angles;
    angles.reserve(ring.size());
    for (const auto p : ring) angles.push_back(std::atan2(p.y-circle.center.y,p.x-circle.center.x));
    std::sort(angles.begin(), angles.end());
    double previous = angles.back() - 2*pi;
    for (const auto angle : angles) {
        if (angle - previous > pi/3) return {};
        previous = angle;
    }
    return circle;
}

Deviation measureCircle(const Ring& source, const Circle& circle)
{
    // A closed source ring enclosing the center meets every ray from it.
    // Radial extrema occur at endpoints and at the segment's closest point.
    Deviation result;
    for (std::size_t i = 0; i < source.size(); ++i) {
        const auto a = source[i], b = source[(i+1)%source.size()];
        const auto forward = std::abs(distance(a, circle.center) - circle.radius);
        result.bound = std::max({result.bound, forward,
                                 std::abs(segmentDistance(circle.center, a, b) - circle.radius)});
        result.samples.push_back(forward);
    }
    for (int i = 0; i < 96; ++i) {
        const double angle = 2*pi*i/96;
        const Point2D sample{circle.center.x + circle.radius*std::cos(angle),
                             circle.center.y + circle.radius*std::sin(angle)};
        double reverse = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < source.size(); ++j)
            reverse = std::min(reverse, segmentDistance(sample, source[j], source[(j+1)%source.size()]));
        result.samples.push_back(reverse);
    }
    return result;
}

ParameterizationResult failure(FitFailure reason) {
    ParameterizationResult result;
    result.failure = reason;
    return result;
}
} // namespace

ParameterizationResult parameterizeContours(const std::vector<Contour>& contours,
    const ParameterizationOptions& options, const std::function<bool()>& cancelled)
{
    const auto stopped = [&] { return cancelled && cancelled(); };
    if (stopped()) return failure(FitFailure::Cancelled);
    if (contours.empty() || !std::isfinite(options.tolerance) || options.tolerance < 0
        || !options.maximumEdgesPerContour) return failure(FitFailure::InvalidBoundary);
    auto source = contours;
    for (auto& contour : source) {
        if (!normalize(contour.outer, true)) return failure(FitFailure::InvalidBoundary);
        for (auto& hole : contour.holes)
            if (!normalize(hole, false)) return failure(FitFailure::InvalidBoundary);
    }
    const auto rings = flatten(source);
    const auto original = boundaries(rings);
    const auto topology = topologyOf(source, original, cancelled);
    if (!topology) return failure(stopped() ? FitFailure::Cancelled : FitFailure::InvalidBoundary);

    // The simplifier's local removal cost is not a global error bound.
    // Retry with a stricter cost before falling back to the observed rings.
    std::vector<FittedRing> fitted;
    std::vector<Deviation> deviations;
    std::vector<Boundary> geometry;
    for (const double factor : {1., .5, .25, 0.}) {
        if (stopped()) return failure(FitFailure::Cancelled);
        const auto candidate = simplifyContours(source, factor*options.tolerance);
        const auto reduced = flatten(candidate);
        geometry = boundaries(reduced);
        fitted.clear();
        deviations.clear();
        for (std::size_t i = 0; i < rings.size(); ++i) {
            if (stopped()) return failure(FitFailure::Cancelled);
            auto deviation = measurePolyline(*rings[i], *reduced[i]);
            if (!deviation || deviation->bound > options.tolerance || !matches(geometry[i], i, geometry, *topology))
                break;
            fitted.push_back({*reduced[i], {}});
            deviations.push_back(std::move(*deviation));
        }
        if (fitted.size() == rings.size()) break;
    }
    if (fitted.size() != rings.size()) return failure(FitFailure::InvalidBoundary);

    if (options.recognizeCircles) for (std::size_t i = 0; i < rings.size(); ++i) {
        if (stopped()) return failure(FitFailure::Cancelled);
        const auto circle = fitCircle(*rings[i]);
        if (!circle || !original[i].contains(circle->center)) continue;
        Boundary candidate(*circle);
        if (!matches(candidate, i, geometry, *topology)) continue;
        auto deviation = measureCircle(*rings[i], *circle);
        if (deviation.bound > std::min(options.tolerance, .15*circle->radius)) continue;
        geometry[i] = std::move(candidate);
        fitted[i] = {{}, circle};
        deviations[i] = std::move(deviation);
    }

    ParameterizationResult result;
    std::size_t index = 0;
    for (const auto& contour : source) {
        FittedContour next{std::move(fitted[index++]), {}};
        const auto edges = [](const FittedRing& ring) { return ring.circle ? 1u : ring.polyline.size(); };
        auto count = edges(next.outer);
        for (std::size_t i = 0; i < contour.holes.size(); ++i) {
            count += edges(fitted[index]);
            next.holes.push_back(std::move(fitted[index++]));
        }
        if (count > options.maximumEdgesPerContour) return failure(FitFailure::EdgeBudget);
        result.edgeCount += count;
        result.contours.push_back(std::move(next));
    }
    std::vector<double> samples;
    for (const auto& deviation : deviations) {
        result.maximumDeviation = std::max(result.maximumDeviation, deviation.bound);
        samples.insert(samples.end(), deviation.samples.begin(), deviation.samples.end());
    }
    const auto p95 = samples.end() - 1 - samples.size()/20;
    std::nth_element(samples.begin(), p95, samples.end());
    result.p95Deviation = *p95;
    return result;
}
} // namespace contour_fitting
