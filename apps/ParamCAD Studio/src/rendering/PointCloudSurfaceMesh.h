#pragma once
#include "core/PointCloudSelection.h"
#include <reconstruction/settings/SurfaceMeshingSettings.h>
#include <array>
#include <map>

// Display geometry only: neither source observations nor fitted equations change.
struct PlanarDomainSupportCheck
{
    size_t observedPoints = 0;
    size_t outerBoundaryPoints = 0;
    size_t holeBoundaryPoints = 0;
    size_t interiorPoints = 0;
    size_t occupiedInteriorCells = 0;
    double boundaryBandWidth = 0;
    double area = 0;
    double interiorArea = 0;
    double expectedInteriorPoints = 0;
    bool interiorAreaMeasured = false;
    bool rejected = false;
};

struct PointCloudSurfaceMesh
{
    std::vector<std::array<double, 3>> positions;
    std::vector<std::array<vtkIdType, 3>> triangles;
    std::vector<vtkIdType> sourcePointIds;
    // Only these observations have supported surface coverage. Unmeshed line
    // tails / isolated points stay visible when the collection displays a face.
    std::vector<vtkIdType> coveredPointIds;
    // Shared-plane domain diagnostics, before partitioning triangles by owner.
    std::vector<PlanarDomainSupportCheck> domainSupportChecks;
};

struct PointCloudSurfaceDisplay
{
    std::map<int, PointCloudSurfaceMesh> meshes;
    std::map<int, int> colorRegions;
    // Includes rejected domains even when a whole family stays in point mode.
    std::map<int, std::vector<PlanarDomainSupportCheck>> planeSupportChecks;
};

PointCloudSurfaceMesh meshPointCloudSurface(const PointCloudRegion& region,
    const PlyCloud& cloud, const std::vector<vtkIdType>& members);
PointCloudSurfaceMesh meshPointCloudSurface(const PointCloudRegion& region,
    const PlyCloud& cloud, const std::vector<vtkIdType>& members,
    const reconstruction::SurfaceMeshingSettings& settings);

// Collections sharing one fitted plane are meshed together, then partitioned
// back into collection-owned triangles for independent display switches.
PointCloudSurfaceDisplay meshPointCloudSurfaces(const PointCloudSelection& selection);
PointCloudSurfaceDisplay meshPointCloudSurfaces(const PointCloudSelection& selection,
    const reconstruction::SurfaceMeshingSettings& settings);
