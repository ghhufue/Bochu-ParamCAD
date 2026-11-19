#pragma once

#include "io/PlyReader.h"
#include "rendering/PointCloudSurfaceMesh.h"

#include <vtkSmartPointer.h>
#include <map>
#include <set>

class vtkActor;
class vtkCellArray;
class vtkPoints;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkUnsignedCharArray;
class PointCloudSelection;

// Owns the VTK pipeline that draws one reconstructed point cloud. Like
// BodyActor it is a display cache; PointCloudDocument owns the working state.
class PointCloudActor
{
public:
    PointCloudActor();

    bool setCloud(const PlyCloud& cloud); // false for an empty cloud, which also clears
    void clear();
    bool hasCloud() const noexcept;
    int pointCount() const noexcept;

    void setVisible(bool visible);
    void setPointSize(double size);
    void setAnnotations(const PointCloudSelection* selection);
    void setPointMask(const std::vector<unsigned char>& deleted);
    void setAutomaticSurfaceDisplay(bool enabled);
    void setRegionSurfaceDisplay(int region, bool enabled);
    std::set<int> availableSurfaceRegions() const;
    std::set<int> displayedSurfaceRegions() const;
    const PointCloudSurfaceMesh* surfaceMesh(int region) const;
    PointCloudSurfaceDisplay surfaceDisplaySnapshot() const;
    void invalidateSurfaceMeshes() {m_surfaceSelection=nullptr;m_baseColorsReady=false;}
    static std::array<unsigned char, 3> regionColor(int region);

    // Bounds of the loaded cloud, or six zeroes when there is none.
    void bounds(double out[6]) const;

    vtkActor* actor() const noexcept;
    vtkActor* selectionActor() const noexcept;
    vtkActor* surfaceActor() const noexcept;
    vtkActor* surfaceHighlightActor() const noexcept;
    int surfaceRegionOfCell(vtkIdType cell) const noexcept;

private:
    bool refreshSurfaceMeshes(const PointCloudSelection& selection);
    void applySurfaceDisplay();
    void updateSurfaceHighlight(const PointCloudSelection* selection);
    std::array<unsigned char, 3> surfaceColor(int region) const;
    vtkSmartPointer<vtkPoints> m_points;
    vtkSmartPointer<vtkCellArray> m_vertices;
    vtkSmartPointer<vtkUnsignedCharArray> m_colors;
    vtkSmartPointer<vtkPolyData> m_polyData;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPoints> m_selectionPoints;
    vtkSmartPointer<vtkCellArray> m_selectionVertices;
    vtkSmartPointer<vtkPolyData> m_selectionData;
    vtkSmartPointer<vtkActor> m_selectionActor;
    vtkSmartPointer<vtkPolyData> m_surfaceData;
    vtkSmartPointer<vtkActor> m_surfaceActor;
    vtkSmartPointer<vtkPolyData> m_surfaceHighlightData;
    vtkSmartPointer<vtkActor> m_surfaceHighlightActor;
    std::vector<int> m_surfaceCellRegions;
    std::map<int, PointCloudSurfaceMesh> m_surfaceMeshes;
    std::map<int, std::vector<PlanarDomainSupportCheck>> m_surfaceSupportChecks;
    std::map<int, int> m_surfaceColorRegions;
    std::map<int, bool> m_surfaceOverrides;
    std::vector<unsigned char> m_surfacePointMask;
    const PointCloudSelection* m_surfaceSelection = nullptr;
    quint64 m_surfaceRevision = 0;
    bool m_automaticSurfaces = true;
    bool m_showAnnotations = false;
    const PointCloudSelection* m_annotationSelection = nullptr;
    quint64 m_annotationRevision = 0;
    bool m_baseColorsReady = false;
    bool m_hasCloud = false;
    bool m_visible = true;
    std::vector<std::array<unsigned char, 3>> m_sourceColors;
    std::vector<unsigned char> m_deleted;
};
