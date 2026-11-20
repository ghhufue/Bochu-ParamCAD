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
}

int main()
{
    try {
        testFit();
        testTopology();
        std::cout << "Contour parameterization tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
