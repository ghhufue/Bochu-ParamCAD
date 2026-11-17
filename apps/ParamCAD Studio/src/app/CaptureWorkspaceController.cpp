#include "app/CaptureWorkspaceController.h"
#include "app/CapturePlan.h"
#include "app/CaptureSettings.h"
#include "app/ProjectPaths.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "capture/CaptureSessionController.h"
#include "capture/VtkProjectionBackend.h"
#include "capture/MitsubaBackend.h"
#include "capture/ScanPlanner.h"
#include "ui/viewport/VTKViewer.h"

#include <QAction>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <exception>
#include <cmath>

CaptureWorkspaceController::CaptureWorkspaceController(
    CaptureController* captureController, const CaptureWorkspaceUi& ui,
    QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent, parent), m_captureController(captureController), m_ui(ui)
{
    auto* panel = m_ui.propertyPanel->capturePanel();
    panel->setBaseDirectory(m_captureController->baseDirectory());
    panel->setDefaultDistance(m_captureController->currentDistance());
    m_structured=new capture::CaptureSessionController(new capture::VtkProjectionBackend,this);
    connect(m_structured,&capture::CaptureSessionController::completed,this,&CaptureWorkspaceController::structuredCaptureCompleted);
    connect(m_structured,&capture::CaptureSessionController::failed,this,&CaptureWorkspaceController::structuredCaptureFailed);
    connect(m_structured,&capture::CaptureSessionController::cancelled,this,&CaptureWorkspaceController::structuredCaptureCancelled);
}

void CaptureWorkspaceController::connectUi()
{
    CapturePanel* capturePanel = m_ui.propertyPanel->capturePanel();
    connect(capturePanel,&CapturePanel::structuredScanRequested,this,[this](bool multiple){startStructuredScan(multiple);});
    connect(capturePanel,&CapturePanel::structuredCancelRequested,m_structured,&capture::CaptureSessionController::cancel);
    connect(m_structured,&capture::CaptureSessionController::progress,this,[this,capturePanel](int done,int total) {
        capturePanel->setStatusText(workspaceTr("结构光拍摄：%1/%2 帧").arg(done).arg(total));
    });
    const auto release=[this,capturePanel] {capturePanel->setStructuredBusy(false);emit captureBusyChanged(false);};
    connect(m_structured,&capture::CaptureSessionController::completed,this,[this,capturePanel,release](const QString& directory) {
        release();capturePanel->setStatusText(workspaceTr("结构光拍摄完成：%1").arg(directory));
        m_ui.propertyPanel->reconstructPanel()->setImageDirectory(directory);
        emit logInfo(workspaceTr("Structured-light capture completed: %1").arg(directory));
        emit statusMessage(workspaceTr("拍摄完成，可在图像重建页生成点云"),5000);
    });
    connect(m_structured,&capture::CaptureSessionController::failed,this,[this,capturePanel,release](const QString& error) {
        release();capturePanel->setStatusText(error);emit logError(error);
    });
    connect(m_structured,&capture::CaptureSessionController::cancelled,this,[capturePanel,release](const QString&) {
        release();capturePanel->setStatusText(workspaceTr("结构光拍摄已取消"));
    });
    connect(m_ui.captureToolAction, &QAction::toggled, this, [this](bool visible) {
        m_ui.propertyPanel->showCapturePanel(visible);
    });
    connect(capturePanel, &CapturePanel::baseDirectoryChanged,
            m_captureController, &CaptureController::setBaseDirectory);
    connect(capturePanel, &CapturePanel::capturePhotoRequested,
            this, &CaptureWorkspaceController::capturePhoto);
    connect(capturePanel, &CapturePanel::captureOrbitRequested,
            this, &CaptureWorkspaceController::captureOrbit);

    connect(m_captureController, &CaptureController::shotsChanged, this, [this, capturePanel]() {
        capturePanel->setShots(m_captureController->shots());
    });
    connect(m_captureController, &CaptureController::progressChanged,
            this, [this, capturePanel](const QString& message) {
                capturePanel->setStatusText(message);
                emit statusMessage(message, 4000);
            });
    connect(m_captureController, &CaptureController::batchStateChanged,
            this, [this, capturePanel](bool running) {
                capturePanel->setBusy(running);
                // The orbit sweep drives the camera directly, so the panel must
                // stop echoing it back while the batch owns the pose.
                m_ui.propertyPanel->cameraPanel()->setFeedbackSuppressed(running);
                if (running) {
                    capturePanel->setStatusText(workspaceTr("正在环绕拍摄…"));
                } else {
                    // The pose was restored while feedback was still suppressed,
                    // so re-read the live camera instead of trusting the stale
                    // readout to happen to match.
                    m_ui.propertyPanel->cameraPanel()->setParameters(m_ui.viewer->cameraParameters());
                }
            });
    connect(m_captureController, &CaptureController::captureFailed,
            this, [this](const QString& message) {
                emit logError(message);
                QMessageBox::warning(m_dialogParent, workspaceTr("拍摄"), message);
            });
}

bool CaptureWorkspaceController::structuredScanRunning() const {return m_structured->isRunning();}
void CaptureWorkspaceController::cancelStructuredScan() {m_structured->cancel();}

bool CaptureWorkspaceController::startStructuredScan(bool multipleStations,const QString& outputDirectory)
{
    auto* panel=m_ui.propertyPanel->capturePanel();
    if(m_structured->isRunning())return false;
    try {
        const auto camera=m_ui.viewer->cameraParameters();
        if(camera.parallelProjection)throw std::invalid_argument("Structured-light capture requires a perspective camera");
        auto options=panel->structuredOptions();options.fieldOfViewDegrees=camera.fieldOfView;
        // The material is a property of the model, not of this capture, so the
        // scan uses exactly the values the viewport preview is showing.
        const auto material=m_ui.propertyPanel->presentation();
        options.material.metallic=material.metallic;
        options.material.roughness=material.roughness;
        if(options.renderBackend=="mitsuba") {
            QStringList prefix;
            const auto python=ProjectPaths::pythonInterpreter(prefix);
            const auto worker=QDir(ProjectPaths::projectRoot()).filePath("core/reconstruction/render_structured_light.py");
            m_structured->setBackend(new capture::MitsubaBackend(python,prefix,worker,
                options.samplesPerPixel,options.maxPathDepth,options.randomSeed));
        } else if(options.renderBackend=="vtk")m_structured->setBackend(new capture::VtkProjectionBackend);
        else throw std::invalid_argument("Unknown capture rendering backend");
        QString directory=outputDirectory;
        if(directory.isEmpty()) {
            const QString base=m_captureController->baseDirectory();
            const QString stamp=QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
            directory=QDir(base).filePath(stamp);
            for(int i=2;QFileInfo::exists(directory);++i)directory=QDir(base).filePath(stamp+"-"+QString::number(i));
        }
        const auto pose=capture::lookAt({camera.position[0],camera.position[1],camera.position[2]},
            {camera.target[0],camera.target[1],camera.target[2]},{camera.up[0],camera.up[1],camera.up[2]});
        if(!multipleStations)options.distance=std::hypot(camera.position[0]-camera.target[0],
            camera.position[1]-camera.target[1],camera.position[2]-camera.target[2]);
        auto job=capture::planScan(m_ui.viewer->captureGeometry(),options,directory,multipleStations?nullptr:&pose);
        panel->setStructuredBusy(true);emit captureBusyChanged(true);
        if(!m_structured->start(std::move(job))) {
            panel->setStructuredBusy(false);emit captureBusyChanged(false);
            throw std::runtime_error(m_structured->lastError().toStdString());
        }
        return true;
    } catch(const std::exception& e) {
        const QString error=QString::fromUtf8(e.what());panel->setStatusText(error);emit logError(error);return false;
    }
}

void CaptureWorkspaceController::captureImage()
{
    // A single-frame capture is named after the moment it was taken and offered
    // inside the project's outputs folder; a relative default would resolve
    // against the process working directory and could land outside the project.
    // The folder is created up front because the save dialog cannot start in a
    // directory that does not exist yet.
    const QString screenshotsRoot = ProjectPaths::screenshotsRoot();
    QDir().mkpath(screenshotsRoot);
    const QString defaultPath = QDir(screenshotsRoot).filePath(
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))
        + QStringLiteral(".png"));

    QString filePath = QFileDialog::getSaveFileName(m_dialogParent, workspaceTr("拍摄图像"), defaultPath, workspaceTr("PNG 图像 (*.png)"));
    if (filePath.isEmpty()) {
        return;
    }
    if (!filePath.endsWith(QStringLiteral(".png"), Qt::CaseInsensitive)) {
        filePath += QStringLiteral(".png");
    }

    RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    if (m_ui.viewer->captureImage(filePath, renderPanel->outputWidth(), renderPanel->outputHeight())) {
        emit logInfo(workspaceTr("Image captured: %1").arg(QFileInfo(filePath).fileName()));
        emit statusMessage(workspaceTr("图像已保存到 %1").arg(filePath), 5000);
    } else {
        emit logError(workspaceTr("Failed to capture image: %1").arg(filePath));
        QMessageBox::warning(m_dialogParent, workspaceTr("拍摄图像"), workspaceTr("图像无法保存。"));
    }
}

void CaptureWorkspaceController::captureViewport()
{
    const QString screenshotsRoot = ProjectPaths::screenshotsRoot();
    if (!QDir().mkpath(screenshotsRoot)) {
        const QString message = workspaceTr("无法创建截图目录：%1").arg(screenshotsRoot);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("视图截图"), message);
        return;
    }

    // A timestamped name keeps every press; the suffix avoids overwriting two
    // captures taken within the same second.
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    QString filePath = QDir(screenshotsRoot).filePath(stamp + QStringLiteral(".png"));
    for (int suffix = 1; QFileInfo::exists(filePath); ++suffix) {
        filePath = QDir(screenshotsRoot).filePath(
            QStringLiteral("%1_%2.png").arg(stamp).arg(suffix));
    }

    if (m_ui.viewer->captureViewport(filePath)) {
        emit logInfo(workspaceTr("Viewport screenshot saved: %1").arg(filePath));
        emit statusMessage(workspaceTr("截图已保存到 %1").arg(filePath), 5000);
    } else {
        const QString message = workspaceTr("视图截图保存失败：%1").arg(filePath);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("视图截图"), message);
    }
}

void CaptureWorkspaceController::capturePhoto()
{
    const RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    if (m_captureController->captureSingle(renderPanel->outputWidth(),
                                           renderPanel->outputHeight())) {
        const CaptureShot& shot = m_captureController->shots().back();
        emit logInfo(workspaceTr("Photo captured: %1/%2").arg(shot.groupName, shot.imageName));
    }
}

void CaptureWorkspaceController::captureOrbit(double distance)
{
    // Five latitude rings at 30-degree longitude steps cover the body, while
    // one canonical pose at each pole avoids twelve duplicate images there.
    // The plan is externalised in config/capture.conf.
    std::vector<OrbitRing> rings;
    try {
        rings = loadCaptureSettings().orbitRings();
    } catch (const std::exception&) {
        rings = sphericalCaptureRings();
    }

    const RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    const std::size_t before = m_captureController->shots().size();
    if (m_captureController->captureOrbit(rings, distance,
                                          renderPanel->outputWidth(),
                                          renderPanel->outputHeight())) {
        const std::size_t captured = m_captureController->shots().size() - before;
        emit logInfo(workspaceTr("Spherical capture finished: %1 photos on a 30-degree grid")
                    .arg(captured));
    }
}

void CaptureWorkspaceController::syncCaptureDistance()
{
    m_ui.propertyPanel->capturePanel()->setDefaultDistance(
        m_captureController->currentDistance());
}

