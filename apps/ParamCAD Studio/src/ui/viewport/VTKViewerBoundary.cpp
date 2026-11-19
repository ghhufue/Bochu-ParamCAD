#include "ui/viewport/VTKViewer.h"

#include <vtkActor.h>
#include <vtkBoxWidget2.h>
#include <vtkCellArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

#include <algorithm>

namespace {
enum Layer { Points, Surface, OuterRings, InnerRings };

vtkSmartPointer<vtkActor> inspectionActor(vtkPolyData* data, double red, double green, double blue)
{
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(data);
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->PickableOff();
    actor->GetProperty()->SetColor(red, green, blue);
    actor->GetProperty()->LightingOff();
    return actor;
}

void addClosedRing(vtkCellArray* cells, const std::vector<std::size_t>& ring)
{
    std::vector<vtkIdType> indices(ring.begin(), ring.end());
    indices.push_back(indices.front());
    cells->InsertNextCell(static_cast<vtkIdType>(indices.size()), indices.data());
}
}

std::optional<VTKViewer::BoundaryInspectionCounts> VTKViewer::inspectPointCloudBoundary(int regionId)
{
    if (m_sketchInteraction || m_referenceSelection) return {};
    const auto& selection = pointCloudSelection();
    const auto* region = selection.region(regionId);
    const auto* mesh = m_pointCloudActor.surfaceMesh(regionId);
    if (!region || !mesh) return {};
    const auto boundary = planarBoundaryOfMesh(*region, *mesh);
    if (!boundary) return {};

    const auto members = selection.regionPoints(regionId);
    const auto& cloud = selection.sourceCloud();
    vtkNew<vtkPoints> sourcePoints;
    sourcePoints->SetNumberOfPoints(static_cast<vtkIdType>(members.size()));
    vtkNew<vtkCellArray> vertices;
    for (std::size_t i = 0; i < members.size(); ++i) {
        const auto index = static_cast<vtkIdType>(i);
        sourcePoints->SetPoint(index, cloud.positions[static_cast<std::size_t>(members[i])].data());
        vertices->InsertNextCell(1, &index);
    }
    vtkNew<vtkPolyData> pointData;
    pointData->SetPoints(sourcePoints);
    pointData->SetVerts(vertices);

    vtkNew<vtkPoints> fittedPoints;
    fittedPoints->SetDataTypeToDouble();
    fittedPoints->SetNumberOfPoints(static_cast<vtkIdType>(mesh->positions.size()));
    for (std::size_t i = 0; i < mesh->positions.size(); ++i)
        fittedPoints->SetPoint(static_cast<vtkIdType>(i), mesh->positions[i].data());
    vtkNew<vtkCellArray> polygons;
    for (const auto& triangle : mesh->triangles) polygons->InsertNextCell(3, triangle.data());
    vtkNew<vtkPolyData> surfaceData;
    surfaceData->SetPoints(fittedPoints);
    surfaceData->SetPolys(polygons);

    vtkNew<vtkCellArray> outerLines;
    vtkNew<vtkCellArray> innerLines;
    for (const auto& component : boundary->regions) {
        addClosedRing(outerLines, component.outer);
        for (const auto& hole : component.holes) addClosedRing(innerLines, hole);
    }
    vtkNew<vtkPolyData> outerData;
    outerData->SetPoints(fittedPoints);
    outerData->SetLines(outerLines);
    vtkNew<vtkPolyData> innerData;
    innerData->SetPoints(fittedPoints);
    innerData->SetLines(innerLines);

    closeBoundaryInspection();
    m_boundaryInspectionRevision = selection.surfaceRevision();
    m_boundaryInspectionRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_boundaryInspectionRenderer->SetBackground(m_renderer->GetBackground());
    m_boundaryInspectionRenderer->SetBackground2(m_renderer->GetBackground2());
    m_boundaryInspectionRenderer->GradientBackgroundOn();
    m_boundaryInspectionActors[Points] = inspectionActor(pointData, .80, .89, .98);
    m_boundaryInspectionActors[Points]->GetProperty()->SetPointSize(2.5);
    m_boundaryInspectionActors[Surface] = inspectionActor(surfaceData, .23, .54, .71);
    m_boundaryInspectionActors[OuterRings] = inspectionActor(outerData, 1., .76, .29);
    m_boundaryInspectionActors[InnerRings] = inspectionActor(innerData, 1., .44, .67);
    for (const auto layer : {OuterRings, InnerRings}) {
        auto* actor = m_boundaryInspectionActors[layer].GetPointer();
        actor->GetProperty()->SetLineWidth(2.5);
        auto* mapper = vtkPolyDataMapper::SafeDownCast(actor->GetMapper());
        mapper->SetResolveCoincidentTopologyToPolygonOffset();
        mapper->SetRelativeCoincidentTopologyLineOffsetParameters(0, -4);
    }
    for (const auto& actor : m_boundaryInspectionActors) m_boundaryInspectionRenderer->AddActor(actor);
    m_pointBox->SetEnabled(0);
    m_orientationWidget->SetEnabled(0);
    m_interactorStyle->setOrbitEnabled(false);
    m_renderer->InteractiveOff();
    m_renderer->DrawOff();
    m_referenceRenderer->DrawOff();
    m_renderWindow->AddRenderer(m_boundaryInspectionRenderer);

    const auto& [cx, cy, cz] = boundary->center;
    const auto& [nx, ny, nz] = boundary->normal;
    const auto& [ux, uy, uz] = boundary->up;
    const double center[]{cx, cy, cz}, normal[]{nx, ny, nz}, up[]{ux, uy, uz};
    const double aspect = double(std::max(1, width())) / std::max(1, height());
    const double halfHeight = .56 * std::max({boundary->height, boundary->width / aspect, 1e-6});
    const double distance = 2 * std::max({boundary->width, boundary->height, 1e-6});
    alignCameraToPlane(m_boundaryInspectionRenderer, center, normal, up, distance, halfHeight);
    requestRender();
    return BoundaryInspectionCounts{members.size(), mesh->triangles.size(), boundary->regions.size(),
                                    static_cast<std::size_t>(innerLines->GetNumberOfCells())};
}

void VTKViewer::setBoundaryInspectionLayers(bool points, bool surface, bool outerRings, bool innerRings)
{
    if (!m_boundaryInspectionRenderer) return;
    m_boundaryInspectionActors[Points]->SetVisibility(points);
    m_boundaryInspectionActors[Surface]->SetVisibility(surface);
    m_boundaryInspectionActors[Surface]->GetProperty()->SetOpacity(points ? .58 : 1.);
    m_boundaryInspectionActors[OuterRings]->SetVisibility(outerRings);
    m_boundaryInspectionActors[InnerRings]->SetVisibility(innerRings);
    m_boundaryInspectionRenderer->ResetCameraClippingRange();
    requestRender();
}

void VTKViewer::closeBoundaryInspection()
{
    if (!m_boundaryInspectionRenderer) return;
    m_renderWindow->RemoveRenderer(m_boundaryInspectionRenderer);
    m_boundaryInspectionRenderer = nullptr;
    m_boundaryInspectionActors = {};
    m_renderer->InteractiveOn();
    m_renderer->DrawOn();
    m_referenceRenderer->DrawOn();
    m_orientationWidget->SetEnabled(1);
    m_orientationWidget->InteractiveOff();
    m_interactorStyle->setOrbitEnabled(true);
    m_pointBox->SetEnabled(m_pointSelectionMode == 2 && m_pointCloudDocument->hasCloud());
    m_renderer->ResetCameraClippingRange();
    requestRender();
    emit boundaryInspectionClosed();
}
