#include <contour_fitting/ContourParameterization.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace contour_fitting;

namespace {
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}

Contour slottedPlate()
{
    Contour plate;
    for (int x = 0; x < 60; ++x) plate.outer.push_back({double(x), x%2 ? .15 : 0.});
    for (const auto corner : Ring{{60,0},{60,40},{32,40},{32,25},{28,25},{28,40},{0,40}})
        plate.outer.push_back(corner);
    Ring hole;
    for (int i = 0; i < 64; ++i) {
        const double angle = 2*std::acos(-1.)*i/64;
        hole.push_back({16+4*std::cos(angle), 15+4*std::sin(angle)});
    }
    plate.holes.push_back(std::move(hole));
    return plate;
}

void testFit()
{
    const auto plate = slottedPlate();
    const Contour separate{{{80,0},{90,0},{90,10},{80,10}}, {}};
    const std::vector<Contour> input{plate, separate};
    const auto coarse = parameterizeContours(input, {.8, 256, true});
    const auto fine = parameterizeContours(input, {.04, 256, true});
    require(coarse.failure == FitFailure::None && fine.failure == FitFailure::None, "both tolerances produce a fit");
    require(coarse.contours.size() == 2 && coarse.contours.front().holes.size() == 1,
            "disconnected regions and the hole retain their ownership");
    require(coarse.edgeCount < fine.edgeCount, "coarse fitting reduces the number of edges");
    const auto& fitted = coarse.contours.front();
    require(fitted.holes.front().circle && std::abs(fitted.holes.front().circle->radius-4) < 1e-9,
            "a supported round hole becomes an analytic circle");
    for (const double x : {28., 32.})
        require(std::any_of(fitted.outer.polyline.begin(), fitted.outer.polyline.end(),
            [x](auto p) { return p.x == x && p.y == 25.; }), "coarse fitting preserves both bottom corners of the narrow slot");
    require(coarse.maximumDeviation <= .8 && coarse.p95Deviation <= coarse.maximumDeviation,
            "coarse diagnostics respect the certified bidirectional bound");
    require(fine.maximumDeviation <= .04, "fine diagnostics respect the requested tolerance");
    const auto polygons = parameterizeContours({plate}, {.8, 256, false});
    require(polygons.failure == FitFailure::None && !polygons.contours.front().holes.front().circle,
            "the circle switch keeps polygonal holes");
    const auto exact = parameterizeContours({plate}, {0, 256, true});
    require(exact.failure == FitFailure::None && exact.maximumDeviation == 0 &&
            exact.contours.front().outer.polyline.size() == plate.outer.size(), "zero tolerance preserves original edges");
    require(parameterizeContours({plate}, {.8, 3, true}).failure == FitFailure::EdgeBudget,
            "the edge budget reports failure without relaxing the tolerance");
}

void testTopology()
{
    const Contour shell{{{0,0},{20,0},{20,20},{0,20}}, {{{5,5},{5,15},{15,15},{15,5}}}};
    const Contour island{{{8,8},{12,8},{12,12},{8,12}}, {}};
    require(parameterizeContours({shell,island}, {2, 256, true}).contours.size() == 2,
            "an island in a void has its own sketch");
    auto touching = shell;
    touching.holes.front().front().x = 0;
    require(parameterizeContours({touching}, {2, 256, true}).failure == FitFailure::InvalidBoundary,
            "touching hole is rejected");
    const Contour crossing{{{0,0},{5,5},{5,0},{0,5}}, {}};
    require(parameterizeContours({crossing}, {2, 256, true}).failure == FitFailure::InvalidBoundary,
            "self-intersection is rejected");
    const Contour withVertexContact{{{0,0},{20,0},{20,20},{0,20},{0,10}},
                                    {{{0,10},{5,13},{5,7}}}};
    const Contour cornerIsland{{{20,20},{26,20},{26,26},{20,26}}, {}};
    const auto separated = parameterizeContours({withVertexContact,cornerIsland}, {1, 256, false});
    require(separated.failure == FitFailure::None && separated.separatedHoles == 1
        && separated.contours.size() == 2 && separated.contours.front().holes.size() == 1,
        "a single vertex touching the outer ring is separated without losing the hole or the island");
    const auto& beveledHole = separated.contours.front().holes.front().polyline;
    require(separated.maximumDeviation <= 1 && separated.p95Deviation <= separated.maximumDeviation
        && std::all_of(beveledHole.begin(), beveledHole.end(), [](auto p) { return p.x > 0; }),
        "the bevel creates a gap and counts toward the bidirectional fit error");
    require(parameterizeContours({withVertexContact}, {0, 256, false}).failure == FitFailure::TouchingHole,
        "an exact fit reports that the touching hole cannot be separated");
    require(parameterizeContours({Contour{withVertexContact.outer, {}},cornerIsland}, {0, 256, false})
        .contours.size() == 2, "independent outer rings sharing only a corner each form a sketch at zero tolerance");
    const Contour sharedEdgeIsland{{{20,0},{26,0},{26,20},{20,20}}, {}};
    require(parameterizeContours({withVertexContact,sharedEdgeIsland}, {1, 256, false}).failure == FitFailure::InvalidBoundary,
        "shared edges are not treated as vertex contacts");
    const Contour bridgingHole{{{0,0},{20,0},{20,10},{20,20},{0,20},{0,10}},
                               {{{0,10},{10,12},{20,10},{10,8}}}};
    require(parameterizeContours({bridgingHole}, {1, 256, false}).failure == FitFailure::InvalidBoundary,
        "separating multiple contacts could reconnect material and is rejected");
    require(parameterizeContours({shell}, {2, 256, true}, [] { return true; }).failure == FitFailure::Cancelled,
            "the worker can cancel a superseded fit");
}

void testEngineeringContour()
{
    Contour plate;
    for (int x = 0; x <= 170; ++x) {
        const auto noise = x == 80 ? 2.4 : x%2 ? .36 : -.36;
        plate.outer.push_back({double(x), x == 0 || x == 170 ? 0. : noise});
    }
    const auto quarter = std::acos(-1.)*.5;
    for (int i = 1; i <= 32; ++i) {
        const auto angle = quarter*(i/32.-1.);
        plate.outer.push_back({170+20*std::cos(angle), 20+20*std::sin(angle)});
    }
    for (int y = 21; y <= 80; ++y) plate.outer.push_back({190., double(y)});
    for (int x = 189; x >= 95; --x) plate.outer.push_back({double(x), 80.});
    plate.outer.push_back({95., 86.});
    for (int x = 94; x >= 0; --x) plate.outer.push_back({double(x), 86.});
    for (int y = 85; y > 0; --y) plate.outer.push_back({0., double(y)});
    Ring hole;
    for (int i = 0; i < 80; ++i) {
        const double angle = 4*quarter*i/80.;
        const double radius = i == 12 || i == 43 ? 15. : 12.;
        hole.push_back({50+radius*std::cos(angle), 36+radius*std::sin(angle)});
    }
    plate.holes = {hole, {{106, 27}, {124, 27}, {124, 33}, {106, 33}}};
    ParameterizationOptions options{.25, 1024, true};
    const auto classic = parameterizeContours({plate}, options);
    options.engineering = ParameterizationOptions::Engineering{1, 3, 1.25};
    const auto fitted = parameterizeContours({plate}, options);
    require(classic.failure == FitFailure::None && fitted.failure == FitFailure::None,
            "engineering contour fits the noisy rounded plate");
    require(fitted.edgeCount < classic.edgeCount / 4 && !fitted.downweightedPoints.empty(),
            "a long straight edge absorbs isolated spikes and reports residuals");
    require(fitted.circleCount == 1 && fitted.contours.front().holes.front().circle &&
        std::abs(fitted.contours.front().holes.front().circle->radius-12.) < .15,
        "the noisy circular hole becomes an analytic circle and the narrow slot stays polygonal");
    const auto& outer = fitted.contours.front().outer;
    bool step = false;
    for (std::size_t i = 0; i < outer.polyline.size(); ++i) {
        const auto a = outer.polyline[i], b = outer.polyline[(i+1)%outer.polyline.size()];
        step |= std::abs(a.x-95.) < 1.1 && std::abs(b.x-95.) < 1.1 && std::abs(a.y-b.y) > 4.;
    }
    require(step, "a sustained step interrupts the adjoining horizontal lines");
    require(fitted.arcCount == 1, "the supported rounded corner becomes a tangent arc");
    for (std::size_t i = 0; i < outer.arcs.size(); ++i) if (const auto arc = outer.arcs[i]) {
        require(std::abs(arc->sweep-quarter) < .12 && std::abs(arc->circle.radius-20.) < 3.,
                "the fillet has the measured radius and turn");
        const auto a = outer.polyline[i], before = outer.polyline[(i+outer.polyline.size()-1)%outer.polyline.size()];
        const auto b = outer.polyline[(i+1)%outer.polyline.size()], after = outer.polyline[(i+2)%outer.polyline.size()];
        const auto orthogonal = [](Point2D tangent, Point2D p, Point2D center) {
            return std::abs(tangent.x*(p.x-center.x)+tangent.y*(p.y-center.y)) < 1e-6;
        };
        require(orthogonal({a.x-before.x, a.y-before.y}, a, arc->circle.center) &&
            orthogonal({after.x-b.x, after.y-b.y}, b, arc->circle.center),
            "the reconstructed fillet is tangent at both shared endpoints");
    }
}

void testAdaptiveStraightTolerance()
{
    Contour plate;
    constexpr double noise[]{0., 1.45, 0., -1.45};
    for (int x = 0; x <= 260; x += 2)
        plate.outer.push_back({double(x), x > 78 && x < 236 ? noise[(x/2)%4] : 0.});
    for (int y = 2; y <= 60; y += 2) plate.outer.push_back({260., double(y)});
    for (int x = 258; x >= 0; x -= 2) plate.outer.push_back({double(x), 60.});
    for (int y = 58; y > 0; y -= 2) plate.outer.push_back({0., double(y)});
    ParameterizationOptions options{.25, 1024, false};
    options.engineering = ParameterizationOptions::Engineering{1, 1, 1.25, false};
    const auto capped = parameterizeContours({plate}, options);
    options.engineering->maximumLineTolerance = 3;
    const auto adaptive = parameterizeContours({plate}, options);
    require(capped.failure == FitFailure::None && adaptive.failure == FitFailure::None,
            "both straightness settings keep a valid closed contour");
    const auto longestBottomEdge = [](const FittedRing& ring) {
        double longest = 0;
        for (std::size_t i = 0; i < ring.polyline.size(); ++i) {
            const auto a = ring.polyline[i], b = ring.polyline[(i+1)%ring.polyline.size()];
            if (std::abs(a.y) < 3 && std::abs(b.y) < 3) longest = std::max(longest, std::abs(a.x-b.x));
        }
        return longest;
    };
    require(longestBottomEdge(adaptive.contours.front().outer) > 245. &&
            longestBottomEdge(capped.contours.front().outer) < 220.,
            "length-dependent tolerance joins the rough span into one long edge");
}

void testStraightBridgeBesideDetachedIsland()
{
    Contour plate;
    for (int x = 0; x <= 240; ++x) plate.outer.push_back({double(x), 0.});
    for (int y = 1; y <= 60; ++y) plate.outer.push_back({240., double(y)});
    for (int x = 239; x >= 0; --x) {
        double y = x%2 ? 59.7 : 60.3;
        if (x >= 58 && x <= 62) y = 57.9 + .5*std::abs(x-60);
        plate.outer.push_back({double(x), x == 0 ? 60. : y});
    }
    for (int y = 59; y > 0; --y) plate.outer.push_back({0., double(y)});
    // The secondary contour touches one vertex of the small dip in the scan.
    const Contour island{{{60.,57.9}, {61.,58.7}, {61.,59.6}, {59.,59.6}, {59.,58.7}}, {}};
    ParameterizationOptions options{.2, 1024, false};
    options.engineering = ParameterizationOptions::Engineering{1., 3., 1.25, false};
    const auto fit = parameterizeContours({plate, island}, options);
    require(fit.failure == FitFailure::None && fit.contours.size() == 2,
            "straightening near a vertex-contact island retains both closed profiles");
    const auto& fitted = fit.contours.front().outer.polyline;
    bool straight = false;
    for (std::size_t i = 0; i < fitted.size(); ++i) {
        const auto a = fitted[i], b = fitted[(i+1)%fitted.size()];
        if (a.y < 54. || b.y < 54. || std::abs(a.x-b.x) < 230.) continue;
        const auto heightAtIsland = a.y+(b.y-a.y)*(60.-a.x)/(b.x-a.x);
        straight |= heightAtIsland < 57.9 && heightAtIsland > 56.5;
    }
    require(straight, "one supported long edge clears the detached island without following the local dip");
}
}

int main()
{
    try {
        testFit();
        testTopology();
        testEngineeringContour();
        testAdaptiveStraightTolerance();
        testStraightBridgeBesideDetachedIsland();
        std::cout << "Contour parameterization tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
