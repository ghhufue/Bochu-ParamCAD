#include "ui/viewport/VTKViewer.h"
#include "core/SketchFrame.h"
#include "rendering/PointCloudContour.h"

#include <contour_fitting/ContourParameterization.h>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

#include <algorithm>
#include <cmath>

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

PointCloudContourInput contourInput(const PointCloudPlanarBoundary& boundary, const PointCloudSurfaceMesh& mesh)
{
    PointCloudContourInput result;
    const auto& [nx, ny, nz] = boundary.normal;
    const auto& [ux, uy, uz] = boundary.up;
    const double normal[]{nx, ny, nz}, up[]{ux, uy, uz};
    double right[3];
    vtkMath::Cross(up, normal, right);
    result.plane = {boundary.center, boundary.normal, {right[0], right[1], right[2]}};
    std::vector<double> spacings;
    const auto project = [&](const auto& indices) {
        contour_fitting::Ring ring;
        ring.reserve(indices.size());
        const auto& center = boundary.center;
        for (const auto index : indices) {
            const auto& point = mesh.positions[index];
            const double relative[]{point[0]-center.x, point[1]-center.y, point[2]-center.z};
            ring.push_back({vtkMath::Dot(relative, right), vtkMath::Dot(relative, up)});
        }
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const auto a = ring[i], b = ring[(i+1)%ring.size()];
            const auto distance = std::hypot(a.x-b.x, a.y-b.y);
            if (distance > 0) spacings.push_back(distance);
        }
        return ring;
    };
    for (const auto& component : boundary.regions) {
        contour_fitting::Contour contour{project(component.outer), {}};
        for (const auto& hole : component.holes) contour.holes.push_back(project(hole));
        result.contours.push_back(std::move(contour));
    }
    if (!spacings.empty()) {
        const auto median = spacings.begin() + spacings.size()/2;
        std::nth_element(spacings.begin(), median, spacings.end());
        result.spacing = *median;
    }
    return result;
}
}

std::optional<VTKViewer::BoundaryInspectionCounts> VTKViewer::inspectPointCloudBoundary(
    int regionId, PointCloudContourInput* input)
{
    if (m_sketchInteraction || m_referenceSelection) return {};
    const auto& selection = pointCloudSelection();
    const auto* region = selection.region(regionId);
    const auto* mesh = m_pointCloudActor.surfaceMesh(regionId);
    if (!region || !mesh) return {};
    const auto boundary = planarBoundaryOfMesh(*region, *mesh);
    if (!boundary) return {};
    if (input) *input = contourInput(*boundary, *mesh);

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

void VTKViewer::setBoundaryContourPreview(const PointCloudContourInput& input,
    const contour_fitting::ParameterizationResult& result, double sourceScale)
{
    if (!m_boundaryInspectionRenderer) return;
    clearBoundaryContourPreview();
    const auto frame = sketchapp::planeFrameOf(input.plane);
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    vtkNew<vtkCellArray> lines;
    const auto append = [&](const contour_fitting::FittedRing& ring) {
        std::vector<std::size_t> indices;
        const auto insert = [&](double x, double y) {
            double world[3];
            for (int axis = 0; axis < 3; ++axis)
                world[axis] = frame.origin[axis] + sourceScale*(x*frame.xDirection[axis] + y*frame.yDirection[axis]);
            indices.push_back(static_cast<std::size_t>(points->InsertNextPoint(world)));
        };
        if (ring.circle) {
            const auto& circle = *ring.circle;
            constexpr int segments = 160;
            for (int i = 0; i < segments; ++i) {
                const double angle = 2*vtkMath::Pi()*i/segments;
                insert(circle.center.x + circle.radius*std::cos(angle), circle.center.y + circle.radius*std::sin(angle));
            }
        } else {
            for (const auto point : ring.polyline) insert(point.x, point.y);
        }
        if (!indices.empty()) addClosedRing(lines, indices);
    };
    for (const auto& contour : result.contours) {
        append(contour.outer);
        for (const auto& hole : contour.holes) append(hole);
    }
    vtkNew<vtkPolyData> previewData;
    previewData->SetPoints(points);
    previewData->SetLines(lines);
    m_boundaryContourActor = inspectionActor(previewData, .36, 1., .92);
    m_boundaryContourActor->GetProperty()->SetLineWidth(1.5);
    auto* mapper = vtkPolyDataMapper::SafeDownCast(m_boundaryContourActor->GetMapper());
    mapper->SetResolveCoincidentTopologyToPolygonOffset();
    mapper->SetRelativeCoincidentTopologyLineOffsetParameters(0, -10);
    m_boundaryInspectionRenderer->AddActor(m_boundaryContourActor);
    requestRender();
}

void VTKViewer::clearBoundaryContourPreview()
{
    if (!m_boundaryContourActor) return;
    m_boundaryInspectionRenderer->RemoveActor(m_boundaryContourActor);
    m_boundaryContourActor = nullptr;
    requestRender();
}

void VTKViewer::closeBoundaryInspection()
{
    if (!m_boundaryInspectionRenderer) return;
    m_renderWindow->RemoveRenderer(m_boundaryInspectionRenderer);
    m_boundaryInspectionRenderer = nullptr;
    m_boundaryInspectionActors = {};
    m_boundaryContourActor = nullptr;
    m_renderer->InteractiveOn();
    m_renderer->DrawOn();
    m_referenceRenderer->DrawOn();
    m_orientationWidget->SetEnabled(1);
    m_orientationWidget->InteractiveOff();
    m_interactorStyle->setOrbitEnabled(true);
    m_renderer->ResetCameraClippingRange();
    requestRender();
    emit boundaryInspectionClosed();
}
