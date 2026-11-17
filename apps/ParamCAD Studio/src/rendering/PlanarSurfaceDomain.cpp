#include "rendering/PlanarSurfaceDomain.h"
#include <reconstruction/analysis/SurfaceFootprint.h>

PointCloudSurfaceMesh meshPlanarSurfaceDomain(const std::vector<PlanarSurfaceSample>& input,
    const reconstruction::SurfaceMeshingSettings& settings,double maximumSupportSpacing)
{
    std::vector<reconstruction::PlanarSurfaceSample> samples;
    for(const auto& s:input)samples.push_back({s.position,static_cast<reconstruction::PointId>(s.sourcePointId)});
    const auto domain=reconstruction::buildPlanarSurfaceFootprint(samples,settings,maximumSupportSpacing);
    PointCloudSurfaceMesh mesh;mesh.positions=domain.positions;
    for(const auto& t:domain.triangles)mesh.triangles.push_back({vtkIdType(t[0]),vtkIdType(t[1]),vtkIdType(t[2])});
    for(auto id:domain.sourcePointIds)mesh.sourcePointIds.push_back(vtkIdType(id));
    for(auto id:domain.coveredPointIds)mesh.coveredPointIds.push_back(vtkIdType(id));
    mesh.domainSupportChecks=domain.domainSupportChecks;
    return mesh;
}
