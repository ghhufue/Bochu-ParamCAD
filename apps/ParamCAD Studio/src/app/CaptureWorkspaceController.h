#pragma once

#include "app/WorkspaceController.h"

class QAction;
class CaptureController;
class PropertyPanel;
class VTKViewer;
namespace capture {class CaptureSessionController;}

struct CaptureWorkspaceUi
{
    VTKViewer* viewer;
    PropertyPanel* propertyPanel;
    QAction* captureToolAction;
};

// Coordinates capture UI, output sizes and camera feedback during orbit batches.
class CaptureWorkspaceController final : public WorkspaceController
{
    Q_OBJECT
public:
    CaptureWorkspaceController(CaptureController* captureController,
                               const CaptureWorkspaceUi& ui,
                               QWidget* dialogParent, QObject* parent = nullptr);
    void connectUi();
    void captureImage();
    void captureViewport();
    void syncCaptureDistance();
    bool startStructuredScan(bool multipleStations,const QString& outputDirectory={});
    bool structuredScanRunning() const;
signals:
    void captureBusyChanged(bool busy);
    void structuredCaptureCompleted(const QString& directory);
    void structuredCaptureFailed(const QString& error);
    void structuredCaptureCancelled(const QString& directory);

private:
    void capturePhoto();
    void captureOrbit(double distance);

    CaptureController* const m_captureController;
    const CaptureWorkspaceUi m_ui;
    capture::CaptureSessionController* m_structured = nullptr;
};
