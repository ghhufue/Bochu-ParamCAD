#include "EngineeringContour.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace contour_fitting {
namespace {
constexpr double pi = 3.14159265358979323846;
using EngineeringOptions = ParameterizationOptions::Engineering;

Point2D operator+(Point2D a, Point2D b) { return {a.x+b.x, a.y+b.y}; }
Point2D operator-(Point2D a, Point2D b) { return {a.x-b.x, a.y-b.y}; }
Point2D operator*(Point2D a, double scale) { return {a.x*scale, a.y*scale}; }
double dot(Point2D a, Point2D b) { return a.x*b.x + a.y*b.y; }
double cross(Point2D a, Point2D b) { return a.x*b.y - a.y*b.x; }
double length(Point2D p) { return std::hypot(p.x, p.y); }
Point2D left(Point2D p) { return {-p.y, p.x}; }

double percentile(std::vector<double> values, double fraction)
{
    const auto found = values.begin() + static_cast<std::size_t>(fraction*(values.size()-1));
    std::nth_element(values.begin(), found, values.end());
    return *found;
}

double distanceToSegment(Point2D p, Point2D a, Point2D b)
{
    const auto axis = b-a;
    const auto squared = dot(axis, axis);
    return length(p-a-axis*(squared > 0 ? std::clamp(dot(p-a, axis)/squared, 0., 1.) : 0.));
}

struct Line {
    Point2D origin, direction;
    double residual(Point2D p) const { return cross(direction, p-origin); }
    Point2D project(Point2D p) const { return origin + direction*dot(direction, p-origin); }
};

struct Span {
    std::size_t first, last;
    Line line;
    bool bridged = false;
};

Line fitLine(const Ring& ring, std::size_t first, std::size_t last, double scale)
{
    const auto chord = ring[last]-ring[first];
    const double chordLength = length(chord);
    Line fit{ring[first], chordLength > 0 ? chord*(1./chordLength) : Point2D{1., 0.}};
    std::vector<double> weights(last-first+1, 1.);
    for (int iteration = 0; iteration < 5; ++iteration) {
        double weightSum = 0.;
        Point2D center{};
        for (std::size_t i = first; i <= last; ++i) {
            const auto weight = weights[i-first];
            center = center + ring[i]*weight;
            weightSum += weight;
        }
        center = center*(1./weightSum);
        double xx = 0., xy = 0., yy = 0.;
        for (std::size_t i = first; i <= last; ++i) {
            const auto p = ring[i]-center;
            const auto weight = weights[i-first];
            xx += weight*p.x*p.x;
            xy += weight*p.x*p.y;
            yy += weight*p.y*p.y;
        }
        const double angle = .5*std::atan2(2*xy, xx-yy);
        auto direction = Point2D{std::cos(angle), std::sin(angle)};
        if (dot(direction, chord) < 0) direction = direction*(-1.);
        fit = {center, direction};
        for (std::size_t i = first; i <= last; ++i)
            weights[i-first] = scale/std::max(scale, std::abs(fit.residual(ring[i])));
    }
    return fit;
}

std::size_t advance(const std::vector<double>& lengths, std::size_t first, double distance)
{
    const auto found = std::lower_bound(lengths.begin()+first+1, lengths.end(), lengths[first]+distance);
    return found == lengths.end() ? lengths.size()-1 : std::size_t(found-lengths.begin());
}

std::optional<Line> supportedLine(const Ring& ring, const std::vector<double>& lengths,
                                   std::size_t first, std::size_t last, const EngineeringOptions& options)
{
    if (last-first < 2) return {};
    const auto d = options.spacing;
    const auto line = fitLine(ring, first, last, .65*d);
    const double span = dot(ring[last]-ring[first], line.direction);
    if (span < 3*d || span < .65*(lengths[last]-lengths[first])) return {};
    const auto tolerance = std::min(options.maximumLineTolerance, d*std::sqrt(1+span/(32*d)));
    std::vector<double> errors;
    errors.reserve(last-first+1);
    for (std::size_t i = first; i <= last; ++i) errors.push_back(std::abs(line.residual(ring[i])));
    if (percentile(errors, .9) > tolerance ||
        *std::max_element(errors.begin(), errors.end()) > std::max(4*d, 2*tolerance)) return {};

    // A long edge cannot hide a sustained step or bend in the local samples.
    const double localLimit = std::min(.85*d, tolerance);
    int previousSide = 0;
    for (std::size_t begin = first; begin < last;) {
        const auto end = std::min(last, advance(lengths, begin, 4*d));
        std::vector<double> signedErrors;
        for (auto i = begin; i <= end; ++i) signedErrors.push_back(line.residual(ring[i]));
        const auto median = percentile(signedErrors, .5);
        const int side = median > localLimit ? 1 : median < -localLimit ? -1 : 0;
        if (side != 0 && (previousSide == side || std::abs(median) > 1.75*localLimit)) return {};
        if (end == last && end-begin >= 3 && std::abs(median) > 1.25*localLimit) return {};
        previousSide = side;
        begin = end;
    }
    return line;
}

std::optional<Line> bridgeStraightSpans(const Ring& ring, const Span& first, const Span& last,
                                       const EngineeringOptions& options)
{
    const double d = options.spacing;
    const auto axis = first.line.direction;
    const auto firstEnd = first.line.project(ring[first.last]);
    const auto lastStart = last.line.project(ring[last.first]);
    const auto gap = dot(lastStart-firstEnd, axis);
    if (gap < -2*d || gap > 24*d || dot(axis, last.line.direction) < std::cos(4*pi/180) ||
        std::abs(first.line.residual(lastStart)) > 1.3*d ||
        std::abs(last.line.residual(firstEnd)) > 1.3*d) return {};

    const auto line = fitLine(ring, first.first, last.last, .65*d);
    const auto start = ring[first.first];
    const double span = dot(ring[last.last]-start, line.direction);
    const auto tolerance = std::min(options.maximumLineTolerance, d*std::sqrt(1+span/(32*d)));
    const double windows = std::ceil(span/(4*d));
    if (windows < 3 || windows > 2.*(last.last-first.first+1)) return {};
    std::vector<std::vector<double>> residuals(static_cast<std::size_t>(windows));
    std::vector<double> errors;
    double furthest = 0.;
    for (auto i = first.first; i <= last.last; ++i) {
        const auto progress = dot(ring[i]-start, line.direction);
        if (progress < furthest-4*d || progress > span+4*d) return {};
        furthest = std::max(furthest, progress);
        const auto error = line.residual(ring[i]);
        if (std::abs(error) > std::max(4*d, 2*tolerance)) return {};
        const auto window = static_cast<std::size_t>(std::clamp(std::floor(progress/(4*d)), 0., windows-1));
        residuals[window].push_back(error);
        errors.push_back(std::abs(error));
    }
    if (percentile(errors, .9) > tolerance) return {};

    // Use projected distance so a spike cannot count as several consecutive
    // windows just because the raw scan takes a long path around its tip.
    const auto localLimit = std::min(.85*d, tolerance);
    int previousSide = 0;
    bool previousEmpty = false;
    for (std::size_t i = 0; i < residuals.size(); ++i) {
        auto& window = residuals[i];
        if (window.empty()) {
            if (previousEmpty) return {};
            previousEmpty = true;
            continue;
        }
        previousEmpty = false;
        const auto median = percentile(std::move(window), .5);
        const int side = median > localLimit ? 1 : median < -localLimit ? -1 : 0;
        // The terminal windows can overlap the taper of an adjacent corner.
        // The later junction fit controls where the straight edge actually ends.
        const bool terminal = i == 0 || i+1 == residuals.size();
        if (side && (side == previousSide || (!terminal && std::abs(median) > 2*localLimit))) return {};
        previousSide = side;
    }
    return line;
}

void clearExteriorRings(Span& span, const Ring& ring, const std::vector<const Ring*>& exteriorRings,
                        const EngineeringOptions& options)
{
    if (!span.bridged) return;
    const auto d = options.spacing;
    const auto start = ring[span.first], axis = span.line.direction;
    const auto length = dot(ring[span.last]-start, axis);
    const auto tolerance = std::min(options.maximumLineTolerance, d*std::sqrt(1+length/(32*d)));
    double shift = 0.;
    Ring obstacles;
    for (const auto* exterior : exteriorRings) {
        double lowest = std::numeric_limits<double>::infinity(), highest = -lowest;
        const bool alongside = std::all_of(exterior->begin(), exterior->end(), [&](auto point) {
            const auto position = dot(point-start, axis);
            if (position < 0 || position > length) return false;
            const auto offset = span.line.residual(point);
            lowest = std::min(lowest, offset);
            highest = std::max(highest, offset);
            return true;
        });
        // Keep a detached component outside this nearly straight edge.
        if (alongside && lowest <= d && highest >= 0. && highest < tolerance) {
            shift = std::max(shift, highest+.2*d);
            obstacles.insert(obstacles.end(), exterior->begin(), exterior->end());
        }
    }
    if (shift <= 0 || shift > tolerance) return;
    const auto supported = [&](const Line& line) {
        std::vector<double> errors;
        for (auto i = span.first; i <= span.last; ++i)
            errors.push_back(std::abs(line.residual(ring[i])));
        return percentile(errors, .9) <= tolerance &&
               *std::max_element(errors.begin(), errors.end()) <= std::max(4*d, 2*tolerance);
    };

    // Anchor the end farther from the obstacle first. A slight change in
    // direction can preserve its junction while creating the needed clearance.
    const auto meanPosition = std::accumulate(obstacles.begin(), obstacles.end(), 0.,
        [&](double total, Point2D p) { return total + dot(p-start, axis); }) / obstacles.size();
    const bool anchorStart = meanPosition >= length*.5;
    const auto pivot = span.line.project(anchorStart ? ring[span.first] : ring[span.last]);
    const double sign = anchorStart ? 1. : -1.;
    double angle = 0.;
    for (auto p : obstacles) {
        const double along = sign*dot(p-pivot, axis);
        if (along <= d) { angle = pi; break; }
        const double across = span.line.residual(p);
        angle = std::max(angle, std::atan2(across, along)+std::asin(.2*d/std::hypot(along, across)));
    }
    if (angle <= 2*pi/180 && length*std::sin(angle) <= tolerance) {
        const Line anchored{pivot, axis*std::cos(angle) + left(axis)*(sign*std::sin(angle))};
        if (supported(anchored)) {
            span.line = anchored;
            return;
        }
    }
    const Line shifted{span.line.origin + left(axis)*shift, axis};
    if (supported(shifted)) span.line = shifted;
}

std::size_t startAtCorner(const Ring& source, double d)
{
    const auto size = source.size();
    std::size_t best = 0;
    double turning = -1.;
    for (std::size_t i = 0; i < size; ++i) {
        auto before = (i+size-1)%size, after = (i+1)%size;
        for (unsigned j = 0; j < 16 && length(source[i]-source[before]) < 4*d; ++j)
            before = (before+size-1)%size;
        for (unsigned j = 0; j < 16 && length(source[after]-source[i]) < 4*d; ++j)
            after = (after+1)%size;
        const auto a = source[i]-source[before], b = source[after]-source[i];
        if (length(a) < 3*d || length(b) < 3*d) continue;
        const auto score = std::abs(std::atan2(cross(a,b), dot(a,b)));
        if (score > turning) { turning = score; best = i; }
    }
    return best;
}

std::optional<Point2D> intersection(const Line& a, const Line& b)
{
    const auto sine = cross(a.direction, b.direction);
    if (std::abs(sine) < .16) return {};
    return a.origin + a.direction*(cross(b.origin-a.origin, b.direction)/sine);
}

struct Junction {
    Point2D incoming, outgoing;
    Ring between;
    std::optional<FittedArc> arc;
};

std::optional<Junction> fillet(const Ring& gap, const Span& a, const Span& b, Point2D corner,
                              double availableA, double availableB, double d)
{
    if (gap.size() < 5) return {};
    const auto u = a.line.direction, v = b.line.direction;
    const auto sweep = std::atan2(cross(u,v),dot(u,v));
    if (std::abs(sweep) < pi/9 || std::abs(sweep) > 4*pi/5) return {};
    const auto available = std::min(availableA, availableB)*.72;
    const auto observed = std::max(length(gap.front()-corner), length(gap.back()-corner));
    const double minimum = std::max(2.5*d, observed*.45);
    const double maximum = std::min(available, observed+8*d);
    if (maximum <= minimum) return {};

    double best = std::numeric_limits<double>::infinity();
    std::optional<Junction> result;
    for (int i = 0; i <= 32; ++i) {
        const auto trim = minimum + (maximum-minimum)*i/32.;
        const auto start = corner-u*trim, end = corner+v*trim;
        const double radius = trim/std::tan(std::abs(sweep)*.5);
        if (radius < 2*d) continue;
        const auto center = start+left(u)*std::copysign(radius, sweep);
        const auto initial = std::atan2(start.y-center.y, start.x-center.x);
        const FittedArc arc{{center, radius}, initial, sweep};
        std::vector<double> errors, cornerErrors;
        unsigned coverage = 0;
        for (const auto p : gap) {
            const auto radial = p-center;
            auto angle = std::remainder(std::atan2(radial.y,radial.x)-initial, 2*pi);
            const auto fraction = angle/sweep;
            if (fraction < 0 || fraction > 1) continue;
            const auto sector = std::min(3u, static_cast<unsigned>(4*fraction));
            coverage |= 1u << sector;
            errors.push_back(std::abs(length(radial)-radius));
            cornerErrors.push_back(std::min(distanceToSegment(p, start, corner), distanceToSegment(p, corner, end)));
        }
        if (coverage != 15 || errors.size() < 5 || errors.size()*2 < gap.size()) continue;
        const auto deviation = percentile(errors, .8);
        if (deviation > .85*d || deviation >= .55*percentile(cornerErrors, .8) ||
            percentile(errors, .95) > 1.5*d) continue;
        if (deviation < best) {
            best = deviation;
            result = Junction{start, end, {}, arc};
        }
    }
    return result;
}

Ring simplifyOpen(Ring points, double tolerance)
{
    std::vector<bool> keep(points.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::pair<std::size_t, std::size_t>> pending{{0, points.size()-1}};
    while (!pending.empty()) {
        const auto [first, last] = pending.back();
        pending.pop_back();
        std::size_t split = first;
        double worst = tolerance;
        for (auto i = first+1; i < last; ++i) {
            const double error = distanceToSegment(points[i], points[first], points[last]);
            if (error > worst) { worst = error; split = i; }
        }
        if (split != first) {
            keep[split] = true;
            pending.emplace_back(first, split);
            pending.emplace_back(split, last);
        }
    }
    Ring result;
    for (std::size_t i = 0; i < points.size(); ++i)
        if (keep[i]) result.push_back(points[i]);
    return result;
}
} // namespace

Point2D FittedArc::pointAt(double fraction) const
{
    const auto angle = startAngle + sweep*fraction;
    return {circle.center.x + circle.radius*std::cos(angle),
            circle.center.y + circle.radius*std::sin(angle)};
}

Ring sampleEngineeringRing(const FittedRing& ring, double spacing)
{
    if (ring.arcs.empty()) return ring.polyline;
    Ring sampled;
    for (std::size_t i = 0; i < ring.polyline.size(); ++i) {
        sampled.push_back(ring.polyline[i]);
        if (const auto& arc = ring.arcs[i]) {
            const auto size = static_cast<int>(std::clamp(std::ceil(arc->circle.radius*std::abs(arc->sweep)/spacing), 8., 256.));
            for (int j = 1; j < size; ++j) sampled.push_back(arc->pointAt(double(j)/size));
        }
    }
    return sampled;
}

std::optional<FittedRing> fitEngineeringRing(const Ring& source, const EngineeringOptions& options,
                                             const std::function<bool()>& cancelled,
                                             const std::vector<bool>& protectedSamples, const Ring& holeSamples,
                                             bool bridgeProtectedGaps, const std::vector<const Ring*>& exteriorRings)
{
    const auto d = options.spacing;
    const auto start = startAtCorner(source, d);
    const auto count = source.size();
    Ring ring;
    ring.reserve(count+1);
    std::vector<bool> protection;
    protection.reserve(count+1);
    for (std::size_t i = 0; i <= count; ++i) {
        const auto index = (start+i)%count;
        ring.push_back(source[index]);
        protection.push_back(!protectedSamples.empty() && protectedSamples[index]);
    }
    std::vector<double> lengths(ring.size());
    for (std::size_t i = 1; i < ring.size(); ++i) lengths[i] = lengths[i-1]+length(ring[i]-ring[i-1]);
    const auto tryLine = [&](std::size_t first, std::size_t last) -> std::optional<Line> {
        if (std::any_of(protection.begin()+first, protection.begin()+last+1, [](bool value){ return value; }))
            return {};
        return supportedLine(ring, lengths, first, last, options);
    };
    std::vector<Span> spans;
    for (std::size_t first = 0; first+3 < ring.size();) {
        if (cancelled && cancelled()) return {};
        auto last = advance(lengths, first, 8*d);
        auto line = tryLine(first, last);
        if (!line) { ++first; continue; }
        while (last < count) {
            auto next = advance(lengths, last, 4*d);
            auto extended = tryLine(first, next);
            if (!extended) {
                next = last+1;
                extended = tryLine(first, next);
            }
            if (!extended) break;
            last = next;
            line = extended;
            if (cancelled && cancelled()) return {};
        }
        if (dot(ring[last]-ring[first], line->direction) >= 5*d) spans.push_back({first, last, *line});
        first = last;
    }
    if (spans.size() < 2) return {};
    // Extend into the gaps from the opposite side, then join nearly collinear
    // spans by testing the intervening observations against one common line.
    for (std::size_t i = 1; i < spans.size(); ++i) {
        auto& span = spans[i];
        const auto limit = spans[i-1].last;
        while (span.first > limit) {
            const auto previous = span.first-1;
            const auto extended = tryLine(previous, span.last);
            if (!extended) break;
            span = {previous, span.last, *extended};
        }
    }
    std::vector<Span> joined;
    for (const auto& span : spans) {
        if (!joined.empty()) {
            auto& a = joined.back();
            if (lengths[span.first]-lengths[a.last] <= 8*d &&
                dot(a.line.direction,span.line.direction) >= std::cos(3*pi/180) &&
                std::abs(a.line.residual(ring[span.first])) < d) {
                if (const auto merged = tryLine(a.first, span.last)) {
                    a = {a.first, span.last, *merged};
                    continue;
                }
            }
        }
        joined.push_back(span);
    }
    if (joined.size() < 2) return {};
    for (std::size_t i = 0; i+1 < joined.size();) {
        auto& first = joined[i];
        if (length(ring[first.last]-ring[first.first]) < 12*d) { ++i; continue; }
        bool bridged = false;
        for (auto j = std::min(i+8, joined.size()-1); j > i; --j) {
            const auto& last = joined[j];
            if (length(ring[last.last]-ring[last.first]) < 12*d ||
                (!bridgeProtectedGaps &&
                 std::any_of(protection.begin()+first.last, protection.begin()+last.first+1,
                             [](bool value) { return value; }))) continue;
            if (const auto line = bridgeStraightSpans(ring, first, last, options)) {
                first = {first.first, last.last, *line, true};
                joined.erase(joined.begin()+i+1, joined.begin()+j+1);
                bridged = true;
                break;
            }
        }
        if (!bridged) ++i;
        if (cancelled && cancelled()) return {};
    }
    if (joined.size() < 2) return {};
    for (auto& span : joined) {
        // Move a well-supported exterior line slightly outward if its initial
        // position would cut into a nearby hole, while staying within the
        // local measurement scale.
        const auto axis = span.line.direction;
        const auto end = dot(ring[span.last]-ring[span.first], axis);
        double shift = 0.;
        for (const auto point : holeSamples) {
            const auto at = dot(point-ring[span.first], axis);
            if (at < 0 || at > end) continue;
            const auto offset = span.line.residual(point);
            const double wanted = .28*d-offset;
            if (wanted > 0 && wanted <= std::min(.85*d, options.maximumLineTolerance))
                shift = std::max(shift, wanted);
        }
        span.line.origin = span.line.origin-left(axis)*shift;
        clearExteriorRings(span, ring, exteriorRings, options);
    }

    const auto tryFillet = [&](const Span& a, const Span& b, std::size_t wrap) -> std::optional<Junction> {
        if (!options.recognizeFillets) return {};
        const auto corner = intersection(a.line, b.line);
        if (!corner) return {};
        const auto aEnd = a.line.project(ring[a.last]), bStart = b.line.project(ring[b.first]);
        const double availableA = .5*length(aEnd-a.line.project(ring[a.first]))+length(aEnd-*corner);
        const double availableB = .5*length(bStart-b.line.project(ring[b.last]))+length(bStart-*corner);
        const auto fitWindow = [&](std::size_t begin, std::size_t stop) -> std::optional<Junction> {
            Ring gap;
            for (auto i = begin; i <= stop+wrap; ++i) {
                if (protection[i%count]) return {};
                gap.push_back(ring[i%count]);
            }
            return fillet(gap, a, b, *corner, availableA, availableB, d);
        };
        const auto begin = std::max(a.first, a.last > 4 ? a.last-4 : std::size_t{0});
        const auto end = std::min(b.first+4, b.last);
        if (auto fitted = fitWindow(begin, end)) return fitted;
        if (!a.bridged && !b.bridged) return {};
        // Long spans can extend into the transition by a few samples. Recover
        // evidence on both sides before replacing the fillet with a connector.
        const auto previous = std::lower_bound(lengths.begin()+a.first, lengths.begin()+a.last,
                                                lengths[a.last]-8*d);
        const auto next = std::min(b.last, advance(lengths,b.first,8*d));
        return fitWindow(std::min(begin, std::size_t(previous-lengths.begin())), std::max(end,next));
    };
    // Small straight guesses on a curved transition can divide its evidence.
    // Look across a few neighboring spans before settling on the junction.
    if (options.recognizeFillets) for (std::size_t i = 0; i+2 < joined.size(); ++i) {
        const auto last = std::min(i+5, joined.size()-1);
        for (std::size_t j = last; j > i+1; --j) {
            if (lengths[joined[j].first]-lengths[joined[i].last] > 96*d) continue;
            if (!tryFillet(joined[i], joined[j], 0)) continue;
            joined.erase(joined.begin()+i+1, joined.begin()+j);
            break;
        }
    }
    std::vector<Junction> junctions;
    junctions.reserve(joined.size());
    for (std::size_t i = 0; i < joined.size(); ++i) {
        const auto& a = joined[i];
        const auto& b = joined[(i+1)%joined.size()];
        const auto aEnd = a.line.project(ring[a.last]), bStart = b.line.project(ring[b.first]);
        const auto wrap = i+1 == joined.size() ? count : 0;
        const auto stop = b.first+wrap;
        Ring gap{aEnd}, remaining;
        bool hasProtected = false;
        for (auto index = a.last; index <= stop; ++index) {
            const auto point = ring[index%count];
            gap.push_back(point);
            if (!protection[index%count]) continue;
            hasProtected = true;
            const auto section = simplifyOpen(std::move(gap), std::min(.8*d, options.maximumLineTolerance));
            remaining.insert(remaining.end(), section.begin()+1, section.end());
            gap = {point};
        }
        if (const auto arc = tryFillet(a, b, wrap)) {
            junctions.push_back(*arc);
            continue;
        }
        if (const auto corner = intersection(a.line, b.line)) {
            const double reach = std::min(2.5*d, options.maximumLineTolerance);
            if (!hasProtected && length(aEnd-*corner) <= reach && length(bStart-*corner) <= reach &&
                std::all_of(gap.begin(), gap.end(), [&](auto p) {
                    return std::min(distanceToSegment(p, aEnd, *corner),
                                    distanceToSegment(p, *corner, bStart)) <= reach;
                })) {
                junctions.push_back({*corner, *corner, {}, {}});
                continue;
            }
        }
        gap.push_back(bStart);
        const auto section = simplifyOpen(std::move(gap), std::min(.8*d, options.maximumLineTolerance));
        remaining.insert(remaining.end(), section.begin()+1, section.end()-1);
        junctions.push_back({aEnd, bStart, std::move(remaining), {}});
    }

    FittedRing result;
    auto current = junctions.back().outgoing;
    const auto edge = [&](Point2D end, const std::optional<FittedArc>& arc = {}) {
        if (length(end-current) > 1e-8*d) {
            result.polyline.push_back(current);
            result.arcs.push_back(arc);
        }
        current = end;
    };
    for (std::size_t i = 0; i < junctions.size(); ++i) {
        const auto& join = junctions[i];
        if (dot(join.incoming-current, joined[i].line.direction) <= d) return {};
        edge(join.incoming);
        for (auto point : join.between) edge(point);
        edge(join.outgoing, join.arc);
    }
    if (result.polyline.size() < 3) return {};
    if (std::none_of(result.arcs.begin(), result.arcs.end(), [](const auto& arc) { return arc.has_value(); }))
        result.arcs.clear();
    return result;
}

double circularInlierTolerance(double radius, double tolerance, double spacing)
{
    const auto relative = std::clamp(.12*radius+.55*spacing, .15*radius, .27*radius);
    return std::min(tolerance, relative);
}

std::optional<Circle> fitEngineeringCircle(const Ring& source, double tolerance, double spacing)
{
    if (source.size() < 8 || !(tolerance > 0)) return {};
    Point2D origin{};
    for (const auto p : source) origin = origin+p*(1./source.size());
    std::vector<double> weights(source.size(), 1.);
    Circle fit{};
    for (int iteration = 0; iteration < 8; ++iteration) {
        double sum = 0.;
        Point2D center{};
        for (std::size_t i = 0; i < source.size(); ++i) {
            sum += weights[i];
            center = center+(source[i]-origin)*weights[i];
        }
        center = center*(1./sum);
        double xx = 0., xy = 0., yy = 0., xz = 0., yz = 0.;
        for (std::size_t i = 0; i < source.size(); ++i) {
            const auto p = source[i]-origin-center;
            const auto w = weights[i], z = dot(p,p);
            xx += w*p.x*p.x; xy += w*p.x*p.y; yy += w*p.y*p.y;
            xz += w*p.x*z; yz += w*p.y*z;
        }
        const double determinant = xx*yy-xy*xy;
        if (determinant <= 1e-10*(xx+yy)*(xx+yy)) return {};
        const Point2D offset{.5*(yy*xz-xy*yz)/determinant, .5*(xx*yz-xy*xz)/determinant};
        fit.center = origin+center+offset;
        std::vector<double> radii;
        radii.reserve(source.size());
        for (const auto p : source) radii.push_back(length(p-fit.center));
        fit.radius = percentile(radii, .5);
        const double scale = std::min(tolerance*.5, .06*fit.radius);
        if (!std::isfinite(scale) || !(scale > 0)) return {};
        for (std::size_t i = 0; i < radii.size(); ++i) {
            const double normalized = std::abs(radii[i]-fit.radius)/scale;
            weights[i] = 1/(1+normalized*normalized);
        }
    }
    const auto limit = circularInlierTolerance(fit.radius, tolerance, spacing);
    std::vector<double> angles;
    for (const auto p : source) {
        const auto radial = p-fit.center;
        const auto error = std::abs(length(radial)-fit.radius);
        if (error > std::min(4*tolerance, .45*fit.radius)) return {};
        if (error <= limit) angles.push_back(std::atan2(radial.y, radial.x));
    }
    if (angles.size() < 8 || angles.size()*20 < source.size()*17) return {};
    std::sort(angles.begin(), angles.end());
    auto previous = angles.back()-2*pi;
    for (auto angle : angles) {
        const double maximumGap = fit.radius < 4*spacing ? 5*pi/12 : pi/3;
        if (angle-previous > maximumGap) return {};
        previous = angle;
    }
    return fit;
}
} // namespace contour_fitting
