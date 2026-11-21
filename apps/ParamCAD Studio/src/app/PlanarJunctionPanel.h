#pragma once

#include "io/AnalysisBundleReader.h"

#include <QPointer>
#include <QWidget>
#include <vtkSmartPointer.h>

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QThread;
class PointCloudDocument;
class VTKViewer;
class vtkActor;

class PlanarJunctionPanel final : public QWidget
{
    Q_OBJECT
public:
    PlanarJunctionPanel(PointCloudDocument* document, VTKViewer* documentViewer, QWidget* parent);
    ~PlanarJunctionPanel() override;

    bool openBundle(const QString& directory, QString& error);
    VTKViewer* viewport() const;

signals:
    void viewportRequested(VTKViewer* viewport);
    void boundaryInspectionRequested(int region);

private:
    struct SurfaceActors {
        int id;
        vtkSmartPointer<vtkActor> surface, boundary;
    };
    struct IntersectionActor {
        int first, second;
        vtkSmartPointer<vtkActor> actor;
    };

    void useDocument();
    void listPlanes(const std::vector<AnalysisBundlePlaneEntry>& entries);
    void updateFitAction();
    void fitSelected();
    void showPreview();
    void showFit(const reconstruction::PlanarSurfaceJunctionResult& result);
    void clearFit();
    void updateVisibility();
    QString planeName(int id) const;

    PointCloudDocument* const m_document;
    VTKViewer* const m_documentViewer;
    QPointer<VTKViewer> m_viewer;
    QPushButton* m_openBundle;
    QPushButton* m_useDocument;
    QPushButton* m_openPreview;
    QPushButton* m_fitCamera;
    QLineEdit* m_filter;
    QListWidget* m_planes;
    QComboBox* m_visibleFace;
    QPushButton* m_fit;
    QPushButton* m_inspectBoundary;
    QCheckBox* m_boundaries;
    QCheckBox* m_intersections;
    QLabel* m_status;
    QPlainTextEdit* m_report;
    QThread* m_worker = nullptr;
    QString m_bundle;
    quint64 m_surfaceRevision = 0;
    std::vector<SurfaceActors> m_surfaces;
    std::vector<IntersectionActor> m_seams;
};
