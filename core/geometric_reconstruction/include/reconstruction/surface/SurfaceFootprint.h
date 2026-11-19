#pragma once
#include "reconstruction/data/PointStore.h"
#include "reconstruction/settings/SurfaceMeshingSettings.h"
#include <array>
#include <functional>
#include <memory>
#include <utility>

namespace reconstruction {
struct PlanarDomainSupportCheck {
    std::size_t observedPoints=0, outerBoundaryPoints=0, holeBoundaryPoints=0;
    std::size_t interiorPoints=0, occupiedInteriorCells=0;
    double boundaryBandWidth=0, area=0, interiorArea=0, expectedInteriorPoints=0;
    bool interiorAreaMeasured=false, rejected=false;
};
struct PlanarSurfaceSample {
    std::array<double,2> position{};
    PointId sourcePointId=0;
};
struct PlanarFootprintHole {
    std::vector<std::array<double,2>> boundary;
    double area=0, spacing=0;
};
struct SurfaceFootprint {
    modeling::Vec3 origin{},axisU{1,0,0},axisV{0,1,0};
    std::vector<std::array<double,3>> positions;
    std::vector<std::array<std::uint64_t,3>> triangles;
    std::vector<PointId> sourcePointIds, coveredPointIds;
    // Covered repeat observations paired with their representative source ID.
    std::vector<std::pair<PointId, PointId>> sampleAnchors;
    std::size_t effectiveCoveredPoints=0;
    std::vector<PlanarDomainSupportCheck> domainSupportChecks;
    std::vector<PlanarFootprintHole> holes;
};
class SurfaceFootprintQuery {
public:
    explicit SurfaceFootprintQuery(const SurfaceFootprint& domain,double tolerance=1e-10);
    ~SurfaceFootprintQuery();
    SurfaceFootprintQuery(const SurfaceFootprintQuery&)=delete;
    SurfaceFootprintQuery& operator=(const SurfaceFootprintQuery&)=delete;
    // The caller must separately enforce the distance to the analytic plane.
    bool containsProjection(modeling::Vec3 point) const;
    std::optional<PointId> anchorAtProjection(modeling::Vec3 point) const;
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
// Pure numerical finite domain. IDs belong to the caller's observation snapshot.
enum class SurfaceSampleSpace { ObservedPlane, UnwrappedSurface };
SurfaceFootprint buildPlanarSurfaceFootprint(const std::vector<PlanarSurfaceSample>& samples,
    const SurfaceMeshingSettings& settings, double maximumSupportSpacing=0,
    SurfaceSampleSpace sampleSpace=SurfaceSampleSpace::ObservedPlane);
} // namespace reconstruction
