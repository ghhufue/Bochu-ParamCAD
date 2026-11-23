#pragma once
#include <cstddef>

namespace reconstruction {
// Dimensionless thresholds follow the observed point spacing, in source units.
// The final minimum also applies after ownership and to individual mesh islands.
struct SurfaceMeshingSettings {
    std::size_t minimumSurfacePoints = 12;
    std::size_t meshPointBudget = 16000;
    bool boundaryContourCorrectionEnabled = true;
    double boundarySimplificationSpacingMultiplier = .45;
    std::size_t densityNeighborCount = 24;
    double localSupportRadiusMultiplier = 3;
    double connectivityRadiusMultiplier = 3;
    double minimumTransverseVarianceRatio = .001;
    double regularSpacingTolerance = .15;
    double regularSpacingLimitMultiplier = 1.2;
    double densityGapMultiplier = 3;
    double coverageRadiusMultiplier = .8;
    double planarAlphaRadiusMultiplier = 2;
    double curvedAlphaRadiusMultiplier = 2.5;
    double expectedHolePoints = 20;
    double smallDensityScale = .8;
    double holeDensityRatio = .2;
    double boundaryBandSpacingMultiplier = .5;
    double boundaryDominanceRatio = .8;
    double deepInteriorDistanceMultiplier = 1.25;
    double minimumDeepInteriorAreaSpacings = 2;
    double minimumExpectedInteriorPoints = 8;
    double minimumInteriorSupportRatio = .2;
    double slotMinimumLengthSpacings = 6;
    double slotMaximumWidthSpacings = 3;
    double slotMinimumSideSpacings = 3;
    double slotEdgeAlignment = .95;
    double slotPerimeterSupportRatio = .75;
};
}
