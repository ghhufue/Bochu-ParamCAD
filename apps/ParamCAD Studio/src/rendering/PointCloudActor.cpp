#include "rendering/PointCloudActor.h"
#include "core/PointCloudSelection.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>

#include <algorithm>
#include <array>

PointCloudActor::PointCloudActor()
{
    m_points = vtkSmartPointer<vtkPoints>::New();
    m_vertices = vtkSmartPointer<vtkCellArray>::New();

    m_polyData = vtkSmartPointer<vtkPolyData>::New();
    m_polyData->SetPoints(m_points);
    m_polyData->SetVerts(m_vertices);

    m_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_mapper->SetInputData(m_polyData);
    m_mapper->ScalarVisibilityOff();

    m_actor = vtkSmartPointer<vtkActor>::New();
    m_actor->SetMapper(m_mapper);
    m_actor->GetProperty()->SetPointSize(2.0);
    m_actor->GetProperty()->SetColor(0.45, 0.72, 0.95);
    // An unoriented cloud has no normals, so lighting would only darken it.
    m_actor->GetProperty()->LightingOff();
    m_actor->SetVisibility(0);
    m_selectionPoints = vtkSmartPointer<vtkPoints>::New();
    m_selectionVertices = vtkSmartPointer<vtkCellArray>::New();
    m_selectionData = vtkSmartPointer<vtkPolyData>::New();
    m_selectionData->SetPoints(m_selectionPoints);
    m_selectionData->SetVerts(m_selectionVertices);
    auto selectedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    selectedMapper->SetInputData(m_selectionData);
    selectedMapper->ScalarVisibilityOff();
    m_selectionActor = vtkSmartPointer<vtkActor>::New();
    m_selectionActor->SetMapper(selectedMapper);
    m_selectionActor->GetProperty()->SetColor(1., 145./255., 35./255.);
    m_selectionActor->GetProperty()->SetPointSize(3.);
    m_selectionActor->GetProperty()->LightingOff();
    m_selectionActor->PickableOff();
    m_selectionActor->SetVisibility(0);
    m_surfaceData = vtkSmartPointer<vtkPolyData>::New();
    auto surfaceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    surfaceMapper->SetInputData(m_surfaceData);
    surfaceMapper->SetScalarModeToUseCellData();
    surfaceMapper->SetColorModeToDirectScalars();
    m_surfaceActor = vtkSmartPointer<vtkActor>::New();
    m_surfaceActor->SetMapper(surfaceMapper);
    m_surfaceActor->GetProperty()->LightingOff();
    m_surfaceActor->SetVisibility(0);
    m_surfaceHighlightData = vtkSmartPointer<vtkPolyData>::New();
    auto highlightMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    highlightMapper->SetInputData(m_surfaceHighlightData);
    highlightMapper->ScalarVisibilityOff();
    highlightMapper->SetResolveCoincidentTopologyToPolygonOffset();
    highlightMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(-1, -1);
    m_surfaceHighlightActor = vtkSmartPointer<vtkActor>::New();
    m_surfaceHighlightActor->SetMapper(highlightMapper);
    m_surfaceHighlightActor->GetProperty()->SetColor(1., 145./255., 35./255.);
    m_surfaceHighlightActor->GetProperty()->SetOpacity(.65);
    m_surfaceHighlightActor->GetProperty()->LightingOff();
    m_surfaceHighlightActor->PickableOff();
    m_surfaceHighlightActor->SetVisibility(0);
}

bool PointCloudActor::setCloud(const PlyCloud& cloud)
{
    if (cloud.positions.empty()) {
        clear();
        return false;
    }

    const auto count = static_cast<vtkIdType>(cloud.positions.size());
    m_surfaceMeshes.clear();
    m_surfaceSupportChecks.clear();
    m_surfaceColorRegions.clear();
    m_surfaceOverrides.clear();
    m_surfaceSelection = nullptr;
    m_surfacePointMask.assign(cloud.positions.size(), 0);
    m_showAnnotations = false;
    m_surfaceData->Initialize();
    m_surfaceCellRegions.clear();
    m_surfaceHighlightData->Initialize();
    m_surfaceHighlightActor->SetVisibility(0);
    m_surfaceActor->SetVisibility(0);
    m_baseColorsReady = false;
    m_selectionPoints->Initialize();
    m_selectionVertices->Initialize();
    m_selectionActor->SetVisibility(0);
    m_deleted.assign(cloud.positions.size(), 0);
    m_sourceColors = cloud.colors;

    m_points->Initialize();
    m_points->SetNumberOfPoints(count);
    for (vtkIdType index = 0; index < count; ++index) {
        const std::array<float, 3>& position = cloud.positions[static_cast<std::size_t>(index)];
        m_points->SetPoint(index, position[0], position[1], position[2]);
    }
    m_points->Modified();

    // The vendored VTK ships without FiltersGeneral, so vtkVertexGlyphFilter is
    // unavailable and the one-point cells are built by hand.
    m_vertices->Initialize();
    m_vertices->AllocateEstimate(count, 1);
    for (vtkIdType index = 0; index < count; ++index) {
        m_vertices->InsertNextCell(1);
        m_vertices->InsertCellPoint(index);
    }
    m_vertices->Modified();

    if (cloud.colors.size() == cloud.positions.size()) {
        if (!m_colors) {
            m_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            m_colors->SetName("Colors");
            m_colors->SetNumberOfComponents(3);
        }
        m_colors->Initialize();
        m_colors->SetNumberOfTuples(count);
        for (vtkIdType index = 0; index < count; ++index) {
            const std::array<unsigned char, 3>& color = cloud.colors[static_cast<std::size_t>(index)];
            m_colors->SetTuple3(index, color[0], color[1], color[2]);
        }
        m_colors->Modified();
        m_polyData->GetPointData()->SetScalars(m_colors);
        m_mapper->ScalarVisibilityOn();
        m_mapper->SetScalarModeToUsePointData();
        // The array already holds 0-255 bytes, so it is used as-is.
        m_mapper->SetColorModeToDirectScalars();
    } else {
        m_polyData->GetPointData()->SetScalars(nullptr);
        m_mapper->ScalarVisibilityOff();
    }

    m_polyData->Modified();
    m_hasCloud = true;
    m_actor->SetVisibility(m_visible ? 1 : 0);
    return true;
}

void PointCloudActor::clear()
{
    m_surfaceSupportChecks.clear();
    m_surfaceCellRegions.clear();
    m_surfaceHighlightData->Initialize();
    m_surfaceHighlightActor->SetVisibility(0);
    m_surfaceMeshes.clear();
    m_surfaceColorRegions.clear();
    m_surfaceOverrides.clear();
    m_surfacePointMask.clear();
    m_surfaceSelection = nullptr;
    m_showAnnotations = false;
    m_surfaceData->Initialize();
    m_surfaceActor->SetVisibility(0);
    m_baseColorsReady = false;
    m_annotationSelection = nullptr;
    m_selectionPoints->Initialize();
    m_selectionVertices->Initialize();
    m_selectionActor->SetVisibility(0);
    m_deleted.clear();
    m_sourceColors.clear();
    m_points->Initialize();
    m_points->Modified();
    m_vertices->Initialize();
    m_vertices->Modified();
    if (m_colors) {
        m_colors->Initialize();
    }
    m_polyData->GetPointData()->SetScalars(nullptr);
    m_polyData->Modified();
    m_mapper->ScalarVisibilityOff();
    m_hasCloud = false;
    m_actor->SetVisibility(0);
}

bool PointCloudActor::hasCloud() const noexcept
{
    return m_hasCloud;
}

int PointCloudActor::pointCount() const noexcept
{
    return m_hasCloud ? static_cast<int>(std::count(m_deleted.begin(), m_deleted.end(), 0)) : 0;
}

void PointCloudActor::setVisible(bool visible)
{
    m_visible = visible;
    m_actor->SetVisibility(m_visible && m_hasCloud && m_vertices->GetNumberOfCells() > 0);
    m_selectionActor->SetVisibility(m_visible && m_hasCloud && m_selectionPoints->GetNumberOfPoints() > 0);
    m_surfaceActor->SetVisibility(m_visible && m_hasCloud && m_showAnnotations && m_surfaceData->GetNumberOfPolys() > 0);
    m_surfaceHighlightActor->SetVisibility(m_surfaceActor->GetVisibility() && m_surfaceHighlightData->GetNumberOfPolys() > 0);
}

void PointCloudActor::setPointSize(double size)
{
    m_actor->GetProperty()->SetPointSize(std::clamp(size, 0.5, 20.0));
    m_selectionActor->GetProperty()->SetPointSize(std::clamp(size, 0.5, 20.0) + 1.0);
}

void PointCloudActor::bounds(double out[6]) const
{
    if (!m_hasCloud) {
        for (int index = 0; index < 6; ++index) {
            out[index] = 0.0;
        }
        return;
    }
    bool first = true;
    double p[3];
    for (vtkIdType i = 0; i < m_points->GetNumberOfPoints(); ++i) {
        if (m_deleted[static_cast<size_t>(i)]) continue;
        m_points->GetPoint(i, p);
        for (int axis = 0; axis < 3; ++axis) {
            out[2*axis] = first ? p[axis] : std::min(out[2*axis], p[axis]);
            out[2*axis+1] = first ? p[axis] : std::max(out[2*axis+1], p[axis]);
        }
        first = false;
    }
    if (first) std::fill(out, out + 6, 0.0);
}

vtkActor* PointCloudActor::actor() const noexcept
{
    return m_actor;
}

vtkActor* PointCloudActor::selectionActor() const noexcept { return m_selectionActor; }
vtkActor* PointCloudActor::surfaceActor() const noexcept { return m_surfaceActor; }
vtkActor* PointCloudActor::surfaceHighlightActor() const noexcept { return m_surfaceHighlightActor; }

int PointCloudActor::surfaceRegionOfCell(vtkIdType cell) const noexcept
{
    return cell >= 0 && size_t(cell) < m_surfaceCellRegions.size() ? m_surfaceCellRegions[size_t(cell)] : 0;
}

void PointCloudActor::updateSurfaceHighlight(const PointCloudSelection* selection)
{
    std::set<int> regions;
    if (selection) for (auto id : selection->selected()) {
        if (selection->active(id)) regions.insert(selection->memberships()[size_t(id)]);
    }
    auto triangles = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType count = 0; const vtkIdType* ids = nullptr;
    vtkIdType cell = 0;
    m_surfaceData->GetPolys()->InitTraversal();
    while (m_surfaceData->GetPolys()->GetNextCell(count, ids)) {
        if (regions.count(surfaceRegionOfCell(cell))) triangles->InsertNextCell(count, ids);
        ++cell;
    }
    m_surfaceHighlightData->SetPoints(m_surfaceData->GetPoints());
    m_surfaceHighlightData->SetPolys(triangles);
    m_surfaceHighlightData->Modified();
    setVisible(m_visible);
}

void PointCloudActor::setPointMask(const std::vector<unsigned char>& deleted)
{
    if (deleted.size() != static_cast<size_t>(m_points->GetNumberOfPoints())) return;
    if (deleted == m_deleted) return;
    m_deleted = deleted;
    applySurfaceDisplay();
}

void PointCloudActor::setAnnotations(const PointCloudSelection* selection)
{
    if (!m_hasCloud) return;
    const bool showAnnotations = selection != nullptr;
    const bool meshesChanged = selection && refreshSurfaceMeshes(*selection);
    if (m_showAnnotations != showAnnotations || meshesChanged) {
        m_showAnnotations = showAnnotations;
        applySurfaceDisplay();
    }
    const vtkIdType count = m_points->GetNumberOfPoints();
    const auto revision = selection ? selection->revision() : 0;
    const bool refreshColors = !m_baseColorsReady || m_annotationSelection != selection || m_annotationRevision != revision;
    m_selectionPoints->Initialize();
    m_selectionVertices->Initialize();
    if (selection) for (const auto id : selection->selected()) {
        if (id < 0 || id >= count || m_deleted[static_cast<size_t>(id)] || m_surfacePointMask[static_cast<size_t>(id)]) continue;
        double point[3]; m_points->GetPoint(id, point);
        const auto vertex = m_selectionPoints->InsertNextPoint(point);
        m_selectionVertices->InsertNextCell(1, &vertex);
    }
    m_selectionPoints->Modified();
    m_selectionVertices->Modified();
    m_selectionData->Modified();
    m_selectionActor->SetVisibility(m_visible && m_selectionPoints->GetNumberOfPoints() > 0);
    updateSurfaceHighlight(selection);
    if (!refreshColors) return;
    m_annotationSelection = selection;
    m_annotationRevision = revision;
    m_baseColorsReady = true;
    if (!m_colors) {
        m_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        m_colors->SetName("Colors");
        m_colors->SetNumberOfComponents(3);
    }
    m_colors->SetNumberOfTuples(count);
    for (vtkIdType i = 0; i < count; ++i) {
        auto color = m_sourceColors.size() == static_cast<size_t>(count)
            ? m_sourceColors[static_cast<size_t>(i)] : std::array<unsigned char, 3>{115, 184, 242};
        if (selection && selection->memberships().size() == static_cast<size_t>(count)) {
            const int region = selection->memberships()[static_cast<size_t>(i)];
            if (region > 0) color = surfaceColor(region);
        }
        m_colors->SetTypedTuple(i, color.data());
    }
    m_colors->Modified();
    m_polyData->GetPointData()->SetScalars(m_colors);
    m_mapper->ScalarVisibilityOn();
    m_mapper->SetScalarModeToUsePointData();
    m_mapper->SetColorModeToDirectScalars();
}

std::array<unsigned char, 3> PointCloudActor::regionColor(int region)
{
    constexpr std::array<std::array<unsigned char, 3>, 8> palette{{
        {85, 205, 235}, {150, 220, 100}, {195, 130, 240}, {245, 120, 165},
        {80, 215, 170}, {230, 210, 90}, {125, 155, 245}, {215, 180, 145}}};
    return palette[static_cast<size_t>(std::max(1, region) - 1) % palette.size()];
}

bool PointCloudActor::refreshSurfaceMeshes(const PointCloudSelection& selection)
{
    if (m_surfaceSelection == &selection && m_surfaceRevision == selection.revision()) return false;
    m_surfaceSelection = &selection;
    m_surfaceRevision = selection.revision();
    auto display = meshPointCloudSurfaces(selection);
    m_surfaceMeshes = std::move(display.meshes);
    m_surfaceSupportChecks = std::move(display.planeSupportChecks);
    m_surfaceColorRegions = std::move(display.colorRegions);
    for (auto it = m_surfaceOverrides.begin(); it != m_surfaceOverrides.end();)
        if (!selection.region(it->first)) it = m_surfaceOverrides.erase(it); else ++it;
    return true;
}

std::array<unsigned char, 3> PointCloudActor::surfaceColor(int region) const
{
    const auto found = m_surfaceColorRegions.find(region);
    return regionColor(found == m_surfaceColorRegions.end() ? region : found->second);
}

std::set<int> PointCloudActor::availableSurfaceRegions() const
{
    std::set<int> result;
    for (const auto& entry : m_surfaceMeshes) result.insert(entry.first);
    return result;
}

PointCloudSurfaceDisplay PointCloudActor::surfaceDisplaySnapshot() const
{
    return {m_surfaceMeshes,m_surfaceColorRegions,m_surfaceSupportChecks};
}

std::set<int> PointCloudActor::displayedSurfaceRegions() const
{
    std::set<int> result;
    if (!m_showAnnotations) return result;
    for (const auto& entry : m_surfaceMeshes) {
        const auto override = m_surfaceOverrides.find(entry.first);
        if (override == m_surfaceOverrides.end() ? m_automaticSurfaces : override->second) result.insert(entry.first);
    }
    return result;
}

void PointCloudActor::setAutomaticSurfaceDisplay(bool enabled)
{
    m_automaticSurfaces = enabled;
    applySurfaceDisplay();
}

void PointCloudActor::setRegionSurfaceDisplay(int region, bool enabled)
{
    if (!m_surfaceMeshes.count(region)) return;
    m_surfaceOverrides[region] = enabled;
    applySurfaceDisplay();
}

void PointCloudActor::applySurfaceDisplay()
{
    m_surfaceCellRegions.clear();
    m_surfaceHighlightData->Initialize();
    const auto displayed = displayedSurfaceRegions();
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();
    auto triangles = vtkSmartPointer<vtkCellArray>::New();
    auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    for (int region : displayed) {
        const auto& mesh = m_surfaceMeshes.at(region);
        const auto offset = points->GetNumberOfPoints();
        for (const auto& p : mesh.positions) points->InsertNextPoint(p.data());
        const auto color = surfaceColor(region);
        for (const auto& t : mesh.triangles) {
            const vtkIdType ids[]{offset+t[0],offset+t[1],offset+t[2]};
            triangles->InsertNextCell(3, ids);
            m_surfaceCellRegions.push_back(region);
            colors->InsertNextTypedTuple(color.data());
        }
    }
    m_surfaceData->SetPoints(points);
    m_surfaceData->SetPolys(triangles);
    m_surfaceData->GetCellData()->SetScalars(colors);
    m_surfaceData->Modified();
    m_surfacePointMask.assign(m_deleted.size(), 0);
    for(int region:displayed)for(auto id:m_surfaceMeshes.at(region).coveredPointIds)
        if(id>=0 && size_t(id)<m_surfacePointMask.size())m_surfacePointMask[size_t(id)]=1;
    m_vertices->Initialize();
    m_vertices->AllocateEstimate(vtkIdType(m_deleted.size()), 1);
    for (size_t i = 0; i < m_deleted.size(); ++i) {
        if (m_deleted[i] || m_surfacePointMask[i]) continue;
        const auto id = vtkIdType(i);
        m_vertices->InsertNextCell(1, &id);
    }
    m_vertices->Modified();
    m_polyData->Modified();
    setVisible(m_visible);
}
