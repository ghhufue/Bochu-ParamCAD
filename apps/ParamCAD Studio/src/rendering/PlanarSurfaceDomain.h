#pragma once
#include "rendering/PointCloudSurfaceMesh.h"

struct PlanarSurfaceSample
{
    std::array<double, 2> position;
    vtkIdType sourcePointId = 0;
};

// Returns coordinates in the fitted plane's XY frame. The caller maps them
// back onto the original fitted equation, without refitting that equation.
// Local observed area density sets the resolution of internal voids. Stable
// boundaries, open notches and disconnected support retain their own trims.
PointCloudSurfaceMesh meshPlanarSurfaceDomain(const std::vector<PlanarSurfaceSample>& samples,
    const reconstruction::SurfaceMeshingSettings& settings);
