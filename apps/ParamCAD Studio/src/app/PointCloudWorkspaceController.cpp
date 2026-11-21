#include "app/PointCloudWorkspaceController.h"
#include "cli/CommandArguments.h"
#include "cli/CommandProtocol.h"
#include "core/PointCloudDocument.h"
#include "app/CadReconstructionTask.h"
#include "app/ModelingController.h"
#include "app/ProjectPaths.h"
#include "app/ProjectionDebugReport.h"
#include "app/ReconstructionController.h"
#include "io/PointCloudAdapter.h"
#include "io/PointCloudAnalysis.h"
#include "io/AnalysisBundleExporter.h"
#include "io/PointCloudDownsampleStore.h"
#include "io/PointCloudDownsampler.h"
#include "io/PointCloudMetadata.h"
#include "io/PlyReader.h"
#include "io/PlyWriter.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/panels/ProjectionSketchPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/viewport/VTKViewer.h"
#include <reconstruction/parts/ProjectionSettings.h>

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QScopeGuard>
#include <QThread>
#include <QToolButton>
#include <QVBoxLayout>
#include <map>
#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>
#include <utility>

namespace {
reconstruction::LengthUnit cloudUnit(const PointCloudDocument& document)
{
    if (QFileInfo::exists(document.sourcePath()))
        return PointCloudAdapter::lengthUnitFor(document.sourcePath());
    const auto& unit = document.sourceUnit();
    return unit == "mm" ? reconstruction::LengthUnit::Millimeter
        : unit == "m" ? reconstruction::LengthUnit::Meter : reconstruction::LengthUnit::Arbitrary;
}

QByteArray surfaceSupportFingerprint(const PointCloudDocument& document)
{
    const auto& selection=document.selection();
    const auto& cloud=selection.sourceCloud();
    QCryptographicHash fingerprint(QCryptographicHash::Sha256);
    const auto hashVector=[&](const auto& values) {
        const auto size=values.size();
        fingerprint.addData(QByteArrayView(reinterpret_cast<const char*>(&size),sizeof(size)));
        if (size) fingerprint.addData(QByteArrayView(reinterpret_cast<const char*>(values.data()),
            qsizetype(size*sizeof(values.front()))));
    };
    fingerprint.addData(selection.positionsFingerprint().toUtf8());
    fingerprint.addData(document.sourceUnit().toUtf8());
    hashVector(cloud.normals);
    hashVector(cloud.sourceRows);
    hashVector(selection.deleted());
    hashVector(selection.memberships());
    QJsonArray regions;
    for (const auto& region:selection.regions()) {
        regions.append(QJsonObject{{"id",region.id},{"geometry",region.geometry},
            {"parameters",region.parameters},{"stale",region.fitStale},{"locked",region.templateLocked},
            {"residual",region.residual},{"confidence",region.confidence},{"support",region.supportValidation}});
    }
    fingerprint.addData(QJsonDocument(regions).toJson(QJsonDocument::Compact));
    return fingerprint.result();
}

QString projectionOperationLabel(modeling::ExtrudeOperation operation)
{
    using Operation = modeling::ExtrudeOperation;
    switch (operation) {
    case Operation::NewBody: return workspaceTr("新建拉伸体");
    case Operation::Join: return workspaceTr("加材");
    case Operation::Cut: return workspaceTr("切除");
    case Operation::Intersect: return workspaceTr("取交集");
    }
    return {};
}
}

PointCloudWorkspaceController::PointCloudWorkspaceController(
    PointCloudDocument* document, ModelingController* modelingController,
    ReconstructionController* reconstructionController,
    const PointCloudWorkspaceUi& ui, QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent, parent), m_document(document),
      m_modelingController(modelingController),
      m_reconstructionController(reconstructionController), m_ui(ui),
      m_cadTask(new CadReconstructionTask(this))
{
    connect(m_cadTask, &CadReconstructionTask::runningChanged, this, [this](bool running) {
        updateCadActions();
        if (!running && std::exchange(m_analyseAfterCad, false)) startPointCloudAnalysis(0, false);
        if (!running && std::exchange(m_inferAfterCad, false)) startRelationInference(false);
    });
    connect(m_cadTask, &CadReconstructionTask::stageChanged, this,
        [this](CadReconstructionTask::Stage stage) {
            QString message;
            switch (stage) {
            case CadReconstructionTask::Stage::Preprocessing:
                message = workspaceTr("正在后台预处理点云…"); break;
            case CadReconstructionTask::Stage::FittingTemplates:
                message = workspaceTr("正在拟合手动模板，并据此继续整体 CAD 重建…"); break;
            case CadReconstructionTask::Stage::Recognizing:
                message = workspaceTr("正在后台识别 CAD 实体…"); break;
            case CadReconstructionTask::Stage::SeparatingParts:
                message = workspaceTr("正在分离部件、恢复有限边界并检查曲面连接与闭合…");break;
            case CadReconstructionTask::Stage::Projecting:
                message = workspaceTr("正在后台执行单件投影拟合…"); break;
            case CadReconstructionTask::Stage::Cancelling:
                message = workspaceTr("已请求取消 CAD 重建，等待当前计算步骤结束…"); break;
            }
            m_ui.relationsPanel->setStatus(message);
            m_ui.modelingPanel->setStatusText(message);
            emit statusMessage(message, 0);
            updateCadActions();
        });
    connect(m_cadTask, &CadReconstructionTask::finished,
            this, &PointCloudWorkspaceController::onCadReconstructionFinished);
}

PointCloudWorkspaceController::~PointCloudWorkspaceController()
{
    if (m_contourFitThread) {
        disconnect(m_contourFitThread, nullptr, this, nullptr);
        m_contourFitThread->requestInterruption();
        m_contourFitThread->wait();
    }
    if (m_importThread) {
        disconnect(m_importThread, nullptr, this, nullptr);
        m_importThread->wait();
    }
    if(m_exportThread) {
        disconnect(m_exportThread,nullptr,this,nullptr);
        m_exportThread->requestInterruption();m_exportThread->wait();delete m_exportThread;
    }
    // Join both workers before the window destroys borrowed views.
    disconnect(m_cadTask, nullptr, this, nullptr);
    delete m_cadTask;
    if (m_downsampleThread) {
        disconnect(m_downsampleThread, nullptr, this, nullptr);
        m_downsampleThread->requestInterruption();
        m_downsampleThread->wait();
    }
    if (m_analysisThread) {
        disconnect(m_analysisThread, nullptr, this, nullptr);
        m_analysisThread->requestInterruption();
        m_analysisThread->wait();
    }
}

void PointCloudWorkspaceController::connectUi()
{
    connect(m_modelingController, &ModelingController::modelRebuilt, this, [this] {
        if (m_modelingController->hasProtectedReconstruction() && m_cadTask->isRunning())
            m_cadTask->cancel();
    });
    connect(m_ui.scenePanel, &ScenePanel::nodeVisibilityRequested, this,
            [this](SceneNodeType type, bool visible) {
        if (type == SceneNodeType::PointCloud) m_ui.viewer->setPointCloudVisible(visible);
    });
    connect(m_ui.scenePanel, &ScenePanel::nodeDeleteRequested, this, [this](SceneNodeType type) {
        if (type != SceneNodeType::PointCloud) return;
        prepareProjectReplacement();
        m_document->clear();
        emit logInfo(workspaceTr("Point cloud deleted"));
    });
    connect(m_ui.viewer, &VTKViewer::pointCloudVisibilityChanged, this, [this](bool visible) {
        m_ui.scenePanel->setNodeState(SceneNodeType::PointCloud, m_document->hasCloud(), visible);
    });
    connect(m_document, &PointCloudDocument::cloudChanged, this, [this] {
        ++m_importGeneration;
        m_pendingImport.clear();
        m_analyseAfterCad = false;
        m_inferAfterCad = false;
        m_surfaceAnalysis.reset();
        m_surfaceSupportReport = {};
        m_surfaceSupportReportFingerprint.clear();
        clearProjectionDebugResult();
        updateCadActions();
        refreshDownsampleControls();
    });
    connect(m_ui.viewer, &VTKViewer::pointCloudLoaded, this, [this] {
        if (m_analysisThread) m_analysisThread->requestInterruption();
        m_ui.relationsPanel->setDefaultTolerances(m_ui.viewer->pointCloudDiagonal());
        m_ui.relationsPanel->setStatus(workspaceTr("识别未归属点以建立曲面集合。"));
        m_ui.propertyPanel->showPointCloudProperties();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::assignRequested,
        m_document, &PointCloudDocument::assignRegion);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::reconstructRequested,
        this, &PointCloudWorkspaceController::reconstructCad);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::unassignRequested,
        m_document, &PointCloudDocument::unassignSelection);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::saveRequested, this, [this] {
        QString path = QFileDialog::getSaveFileName(m_dialogParent, workspaceTr("保存点云标注"),
            m_document->sourcePath() + QStringLiteral(".regions.json"), workspaceTr("点云标注 (*.json)"));
        if (path.isEmpty()) return;
        if (!path.endsWith(".json", Qt::CaseInsensitive)) path += ".json";
        QString error;
        if (!m_document->saveAnnotations(path, error)) {
            QMessageBox::warning(m_dialogParent, workspaceTr("保存点云标注"), error);
            return;
        }
        emit logInfo(workspaceTr("点云标注已保存：%1").arg(path));
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::analysisExportRequested,
            this, &PointCloudWorkspaceController::startAnalysisExport);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::loadRequested, this, [this] {
        const QString path = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("载入点云标注"),
            QFileInfo(m_document->sourcePath()).absolutePath(), workspaceTr("点云标注 (*.json)"));
        if (path.isEmpty()) return;
        QString error;
        if (!m_document->loadAnnotations(path, error)) {
            QMessageBox::warning(m_dialogParent, workspaceTr("载入点云标注"), error);
            return;
        }
        emit logInfo(workspaceTr("点云标注已载入：%1").arg(path));
    });
    const auto refreshRelationships = [this] {
        m_ui.relationsPanel->setSourceInfo(m_document->sourcePath(), m_document->sourceUnit());
        m_ui.relationsPanel->setDocument(m_document->selection(), m_document->hasCloud());
        updateCadActions();
        if (m_document->hasCloud()) m_ui.scenePanel->updateNodeLabel(SceneNodeType::PointCloud,
            workspaceTr("%1 (%2 有效点)").arg(QFileInfo(m_document->sourcePath()).fileName()).arg(m_document->pointCount()));
        else m_ui.scenePanel->updateNodeLabel(SceneNodeType::PointCloud, workspaceTr("点云"));
        m_ui.scenePanel->setNodeState(SceneNodeType::PointCloud, m_document->hasCloud(),
                                     m_ui.viewer->pointCloudVisible());
    };
    connect(m_document, &PointCloudDocument::documentChanged, this, refreshRelationships);
    connect(m_document, &PointCloudDocument::selectionChanged, m_ui.relationsPanel, &PointCloudRelationsPanel::updateSelection);
    connect(m_ui.viewer, &VTKViewer::pointCloudFacePicked,
            m_ui.relationsPanel, &PointCloudRelationsPanel::setPickedFace);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::regionSelected, this, [this](int id) {
        if (id == 0) m_document->selectUnassigned();
        else m_document->selectRegion(id);
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::relationSelected, m_document, &PointCloudDocument::selectRelation);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::relationGroupSelected,
            m_document, &PointCloudDocument::selectRegions);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::showAllBoundariesChanged,
            m_ui.viewer, &VTKViewer::setShowAllAssemblyBoundaries);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::automaticSurfaceDisplayChanged,
            m_ui.viewer, &VTKViewer::setAutomaticPointCloudSurfaces);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::regionSurfaceDisplayRequested,
            m_ui.viewer, &VTKViewer::setPointCloudRegionSurface);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::boundaryInspectionRequested,
            this, &PointCloudWorkspaceController::inspectCloudBoundary);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::boundaryInspectionLayersChanged,
            m_ui.viewer, &VTKViewer::setBoundaryInspectionLayers);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::boundaryInspectionCloseRequested,
            m_ui.viewer, &VTKViewer::closeBoundaryInspection);
    connect(m_ui.viewer, &VTKViewer::boundaryInspectionClosed,
            this, &PointCloudWorkspaceController::closeContourInspection);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::contourFitSettingsChanged,
            this, &PointCloudWorkspaceController::invalidateContourPreview);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::contourFitRequested,
            this, &PointCloudWorkspaceController::startContourFit);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::contourSketchRequested,
            this, &PointCloudWorkspaceController::createContourSketch);
    connect(m_ui.viewer, &VTKViewer::pointCloudSurfaceDisplayChanged, this, [this] {
        m_ui.relationsPanel->setSurfaceDisplayState(m_ui.viewer->availablePointCloudSurfaces(),
                                                   m_ui.viewer->displayedPointCloudSurfaces());
    });
    m_ui.viewer->setAutomaticPointCloudSurfaces(m_ui.relationsPanel->automaticSurfaceDisplay());
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::editRegionRequested,
        m_document, &PointCloudDocument::editRegion);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::appendSelectionRequested, this, [this](int id) {
        const auto* region = m_document->selection().region(id);
        if (region) m_document->assignRegion(region->geometry, region->name, id);
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::replaceSelectionRequested,this,[this](int id) {
        m_document->replaceRegionPoints(id);
        m_document->lockTemplate(id);
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::removeRegionRequested,
        m_document, &PointCloudDocument::removeRegion);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::deleteRegionPointsRequested,
        m_document, &PointCloudDocument::deleteRegionPoints);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::deleteSelectedPointsRequested,
        m_document, &PointCloudDocument::deleteSelectedPoints);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::setRelationRequested, this,
        [this](int id, int first, int second, const QString& type) {
            if (!m_document->setRelation(id, first, second, type))
                m_ui.relationsPanel->setStatus(workspaceTr("关系未保存：端点须为两个非空集合，且不能与现有关系重复。"));
        });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::removeRelationRequested, m_document, &PointCloudDocument::removeRelation);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::undoRequested,
        m_document, &PointCloudDocument::undo);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::detectRequested, this, [this] { startPointCloudAnalysis(); });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::cancelCadRequested,
            m_ui.cancelCadAction, &QAction::trigger);
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::refitRequested,this,[this](int id) {
        if(!m_document->selection().region(id)) return;
        m_document->lockTemplate(id);
        reconstructCad();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::inferRequested, this, [this] {
        startRelationInference();
    });
    connect(m_ui.relationsPanel, &PointCloudRelationsPanel::exportRequested, this, [this] {
        const QFileInfo source(m_document->sourcePath());
        QString path = QFileDialog::getSaveFileName(m_dialogParent, workspaceTr("导出当前工作点云"),
            source.dir().filePath(source.completeBaseName() + ".edited.ply"), workspaceTr("点云 (*.ply)"));
        if (path.isEmpty()) return;
        if (!path.endsWith(".ply", Qt::CaseInsensitive)) path += ".ply";
        if (QFileInfo(path).absoluteFilePath().compare(source.absoluteFilePath(), Qt::CaseInsensitive) == 0) {
            m_ui.relationsPanel->setStatus(workspaceTr("请选择新文件名，保留原始点云。")); return;
        }
        QString error;
        if (!m_document->exportCloud(path, error)) { m_ui.relationsPanel->setStatus(error); return; }
        const auto unit = PointCloudAdapter::lengthUnitFor(source.absoluteFilePath());
        QSaveFile metadata(QFileInfo(path).dir().filePath(QFileInfo(path).completeBaseName() + ".json"));
        const auto bytes = QJsonDocument(QJsonObject{{"unit", unit == reconstruction::LengthUnit::Meter ? "m"
            : unit == reconstruction::LengthUnit::Millimeter ? "mm" : "arbitrary"}}).toJson();
        if (!metadata.open(QIODevice::WriteOnly) || metadata.write(bytes) != bytes.size() || !metadata.commit()) {
            m_ui.relationsPanel->setStatus(workspaceTr("点云已导出，但单位信息保存失败：%1").arg(metadata.errorString())); return;
        }
        m_ui.relationsPanel->setStatus(workspaceTr("已导出 %1 个有效点：%2").arg(m_document->pointCount()).arg(path));
    });
    refreshRelationships();

    ReconstructPanel* reconstructPanel = m_ui.propertyPanel->reconstructPanel();
    connect(m_ui.reconstructToolAction, &QAction::toggled, this, [this](bool visible) {
        m_ui.propertyPanel->showReconstructPanel(visible);
    });
    connect(reconstructPanel, &ReconstructPanel::reconstructRequested,
            this, [this, reconstructPanel](ReconstructRequest request) {
                // The fusion spacing is a point-cloud property now, so both the
                // in-memory reconstruction fusion and later downsampling share it.
                const double spacing = reconstructPanel->downsampleSpacing();
                if (spacing > 0.0) m_lastDownsampleSpacing = spacing;
                request.voxelSizeMm = m_lastDownsampleSpacing;
                startReconstruction(request);
            });
    connect(reconstructPanel, &ReconstructPanel::downsampleRequested,
            this, &PointCloudWorkspaceController::applyDownsample);
    connect(reconstructPanel, &ReconstructPanel::cancelRequested,
            m_reconstructionController, &ReconstructionController::cancel);

    connect(m_reconstructionController, &ReconstructionController::started,
            this, [this, reconstructPanel](const QString& runDirectory, const QString& cameraModeHint) {
                reconstructPanel->setStatusText(cameraModeHint);
                emit logInfo(workspaceTr("Reconstruction started in %1").arg(runDirectory));
            });
    connect(m_reconstructionController, &ReconstructionController::progressChanged,
            this, [this, reconstructPanel](int stage, int totalStages, const QString& label) {
                reconstructPanel->setProgress(stage, totalStages);
                reconstructPanel->setStatusText(
                    workspaceTr("阶段 %1/%2：%3").arg(stage).arg(totalStages).arg(label));
                emit statusMessage(workspaceTr("重建 %1/%2：%3")
                                             .arg(stage).arg(totalStages).arg(label), 4000);
            });
    connect(m_reconstructionController, &ReconstructionController::logMessage,
            this, &PointCloudWorkspaceController::logInfo);
    connect(m_reconstructionController, &ReconstructionController::failed,
            this, [this, reconstructPanel](const QString& message) {
                reconstructPanel->setBusy(false);
                reconstructPanel->setStatusText(message);
                if(m_cliOperation=="reconstruct cloud")finishCliOperation(false,message);
                if (m_reconstructionController->wasCancelled()) {
                    // The user asked for this, so it is a state change rather
                    // than a fault worth interrupting them with a dialog.
                    emit logWarning(message);
                    return;
                }
                emit logError(message);
                if(!m_dialogParent->property("paramcadBatchCapture").toBool())
                    QMessageBox::warning(m_dialogParent, workspaceTr("点云重建"), message);
            });
    connect(m_reconstructionController, &ReconstructionController::finished,
            this, &PointCloudWorkspaceController::onReconstructionFinished);
}

void PointCloudWorkspaceController::startReconstruction(const ReconstructRequest& request)
{
    ReconstructPanel* panel = m_ui.propertyPanel->reconstructPanel();
    panel->setBusy(true);
    if (!m_reconstructionController->start(request)) {
        panel->setBusy(false);
        const QString error = m_reconstructionController->lastError();
        emit logError(error);
        if(!m_dialogParent->property("paramcadBatchCapture").toBool())
            QMessageBox::warning(m_dialogParent, workspaceTr("点云重建"), error);
    }
}

void PointCloudWorkspaceController::onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                          const QString& cameraMode, bool dense)
{
    bool loaded=false;
    const auto finishCommand=qScopeGuard([&] {
        if(m_cliOperation=="reconstruct cloud")finishCliOperation(loaded,
            loaded?QString():QStringLiteral("Reconstructed cloud could not be loaded into Studio"),
            {{"point_cloud",pointCloudPath},{"point_count",pointCount},{"camera_mode",cameraMode},{"dense",dense}});
    });
    ReconstructPanel* panel = m_ui.propertyPanel->reconstructPanel();
    panel->setBusy(false);

    QString error;
    if (!m_document->load(pointCloudPath, error)) {
        emit logError(error);
        panel->setStatusText(error);
        if(!m_dialogParent->property("paramcadBatchCapture").toBool())
            QMessageBox::warning(m_dialogParent, workspaceTr("点云重建"), error);
        return;
    }

    // The freshly built cloud is now what the recognized-model entry works on.
    updateCadActions();
    m_ui.propertyPanel->cameraPanel()->releaseOriginLock();
    m_ui.viewer->resetCameraToPointCloud();
    emit sceneFramed();
    m_ui.scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(QFileInfo(pointCloudPath).fileName()).arg(pointCount));

    panel->setStatusText(workspaceTr("点云已载入：%1 个点（%2）")
                             .arg(pointCount)
                             .arg(dense ? workspaceTr("稠密") : workspaceTr("稀疏")));
    // Fixed wording: the UI automation script asserts on this exact line.
    emit logInfo(workspaceTr("Point Cloud loaded: %1 points").arg(pointCount));

    if (cameraMode == QStringLiteral("estimated")) {
        emit logWarning(workspaceTr("Camera poses were estimated by COLMAP; the point cloud "
                      "scale is arbitrary until a reconstruction with capture "
                      "metadata is available."));
    }
    m_ui.relationsPanel->setStatus(workspaceTr("点云已载入；需要表面识别时点击“识别表面”。"));
    loaded=true;
}

void PointCloudWorkspaceController::openPointCloud()
{
    // A cloud on disk carries no capture metadata, so no scale warning applies;
    // the file is read straight into the same actor the pipeline feeds.
    const QString filePath = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("打开点云"), ProjectPaths::outputsRoot(),
        workspaceTr("点云文件 (*.ply)"));
    if (filePath.isEmpty()) {
        return;
    }

    importPointCloudFromFile(filePath);
}

void PointCloudWorkspaceController::importPointCloudFromFile(const QString& filePath)
{
    const auto generation = ++m_importGeneration;
    emit statusMessage(workspaceTr("正在载入点云：%1").arg(QFileInfo(filePath).fileName()), 0);
    if (m_importThread) {
        m_pendingImport = filePath;
        return;
    }

    struct ImportResult {
        PointCloudDocument::PreparedCloud cloud;
        QString error;
        bool loaded = false;
    };
    const auto result = std::make_shared<ImportResult>();
    auto* thread = QThread::create([filePath, result] {
        try {
            result->loaded = PointCloudDocument::prepareLoad(filePath, result->cloud, result->error);
        } catch (const std::exception& error) {
            result->error = QString::fromUtf8(error.what());
        }
    });
    m_importThread = thread;
    thread->setParent(this);
    updateCadActions();
    connect(thread, &QThread::finished, this, [this, filePath, generation, result] {
        m_importThread = nullptr;
        const auto pending = std::exchange(m_pendingImport, {});
        if (!pending.isEmpty()) {
            importPointCloudFromFile(pending);
            return;
        }
        updateCadActions();
        if (generation != m_importGeneration) return;
        if (!result->loaded) {
            reportCloudLoadError(result->error);
            return;
        }
        prepareProjectReplacement();
        m_document->adoptCloud(std::move(result->cloud));
        frameImportedCloud(filePath);
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    thread->start();
}

void PointCloudWorkspaceController::loadPointCloudFromFile(const QString& filePath)
{
    QString error;
    if (!m_document->load(filePath, error)) {
        reportCloudLoadError(error);
        return;
    }
    frameImportedCloud(filePath);
}

void PointCloudWorkspaceController::reportCloudLoadError(const QString& error)
{
    emit logError(error);
    emit statusMessage(error, 5000);
    QMessageBox::warning(m_dialogParent, workspaceTr("打开点云"), error);
}

void PointCloudWorkspaceController::frameImportedCloud(const QString& filePath)
{
    const QFileInfo info(filePath);
    const int pointCount = m_document->pointCount();
    // A cloud is now in the viewport, so the recognized-model entry becomes
    // usable and works on this file.
    updateCadActions();
    m_ui.propertyPanel->cameraPanel()->releaseOriginLock();
    m_ui.viewer->resetCameraToPointCloud();
    emit sceneFramed();
    m_ui.scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(info.fileName()).arg(pointCount));
    emit statusTextChanged(workspaceTr("就绪 | 点云：%1 | 相机：%2")
                               .arg(info.fileName(),
                                    m_ui.viewer->cameraParameters().parallelProjection
                                        ? workspaceTr("正交投影") : workspaceTr("透视投影")));
    emit logInfo(workspaceTr("Point cloud loaded from %1: %2 points").arg(info.fileName()).arg(pointCount));
    emit statusMessage({}, 0);
    m_ui.viewer->setFocus(Qt::OtherFocusReason);
}

void PointCloudWorkspaceController::refreshDownsampleControls()
{
    ReconstructPanel* panel = m_ui.propertyPanel->reconstructPanel();
    const QString active = m_document->sourcePath();
    if (active.isEmpty() || !m_document->hasCloud()) {
        panel->setDownsampleAvailable(false);
        panel->setDownsampleStatus(workspaceTr("载入点云后可按间距生成降采样点云。"));
        return;
    }
    const QString candidate = pointcloud::rootCloudPath(active);
    const QString root = QFileInfo::exists(candidate) ? candidate : active;
    double current = 0.0;
    pointcloud::isDownsampledCloud(active, &current);
    panel->setDownsampleScales(pointcloud::existingScales(root), current);
    panel->setDownsampleAvailable(true);
}

void PointCloudWorkspaceController::applyDownsample(double voxelSizeMm)
{
    ReconstructPanel* panel = m_ui.propertyPanel->reconstructPanel();
    if (m_importThread || m_downsampleThread || m_analysisThread || m_cadTask->isRunning() || m_startingCad) return;
    const QString active = m_document->sourcePath();
    if (active.isEmpty() || !m_document->hasCloud()) {
        panel->setDownsampleStatus(workspaceTr("当前没有可降采样的点云。"));
        return;
    }
    const QString candidate = pointcloud::rootCloudPath(active);
    const QString root = QFileInfo::exists(candidate) ? candidate : active;
    if (voxelSizeMm <= 0.0) {
        if (QFileInfo(active).canonicalFilePath() == QFileInfo(root).canonicalFilePath()) {
            panel->setDownsampleStatus(workspaceTr("当前已经是原始点云。"));
            return;
        }
        QString error;
        if (!switchActiveCloud(root, error)) {
            emit logError(error);
            panel->setDownsampleStatus(error);
            return;
        }
        panel->setDownsampleStatus(workspaceTr("已恢复原始点云：%1 个点")
                                        .arg(m_document->pointCount()));
        emit logInfo(workspaceTr("恢复原始点云：%1").arg(QFileInfo(root).fileName()));
        return;
    }
    m_lastDownsampleSpacing = voxelSizeMm;
    double current = 0.0;
    if (pointcloud::isDownsampledCloud(active, &current) && qFuzzyCompare(current, voxelSizeMm)) {
        panel->setDownsampleStatus(workspaceTr("当前已是 %1 mm 的降采样点云。").arg(voxelSizeMm));
        return;
    }
    const QString target = pointcloud::downsamplePath(root, voxelSizeMm);
    if (!QFileInfo::exists(target)) {
        panel->setDownsampleBusy(true);
        panel->setDownsampleStatus(workspaceTr("正在按 %1 mm 融合…").arg(voxelSizeMm));
        struct Result { QString error, warning; bool cancelled = false; };
        auto result = std::make_shared<Result>();
        const auto revision = m_document->revision();
        m_downsampleThread = QThread::create([root, target, voxelSizeMm, result] {
            const auto cancelled = [&] {
                result->cancelled = QThread::currentThread()->isInterruptionRequested();
                return result->cancelled;
            };
            try {
                PlyCloud source, reduced;
                if (cancelled() || !PlyReader::read(root, source, result->error) || cancelled()) return;
                if (!pointcloud::voxelDownsample(source, voxelSizeMm, reduced, result->error) || cancelled()) return;
                if (!QDir().mkpath(pointcloud::downsampleDirectory(root))) {
                    result->error = workspaceTr("无法创建降采样目录：%1").arg(pointcloud::downsampleDirectory(root));
                    return;
                }
                if (!PlyWriter::write(reduced, target, result->error)) return;
                pointcloud::writeUnitSidecar(target, pointCloudUnitName(root), voxelSizeMm, result->warning);
                cancelled();
            } catch (const std::exception& error) { result->error = QString::fromUtf8(error.what()); }
        });
        auto* thread = m_downsampleThread;
        thread->setParent(this);
        connect(thread, &QThread::finished, this, [this, panel, thread, root, target, voxelSizeMm, revision, result] {
            m_downsampleThread = nullptr;
            panel->setDownsampleBusy(false);
            updateCadActions();
            if (result->cancelled || thread->isInterruptionRequested() || m_document->revision() != revision) {
                panel->setDownsampleStatus(workspaceTr("点云已修改或任务已取消，本轮降采样未应用。"));
                return;
            }
            if (!result->error.isEmpty()) {
                emit logError(result->error);
                panel->setDownsampleStatus(result->error);
                if (!m_dialogParent->property("paramcadBatchCapture").toBool())
                    QMessageBox::warning(m_dialogParent, workspaceTr("降采样"), result->error);
                return;
            }
            if (!result->warning.isEmpty()) emit logWarning(result->warning);
            finishDownsample(root, target, voxelSizeMm);
        });
        connect(thread, &QThread::finished, thread, &QObject::deleteLater);
        updateCadActions();
        thread->start();
        return;
    }
    finishDownsample(root, target, voxelSizeMm);
}

void PointCloudWorkspaceController::finishDownsample(const QString& root, const QString& target, double voxelSizeMm)
{
    auto* panel = m_ui.propertyPanel->reconstructPanel();
    QString error;
    if (!switchActiveCloud(target, error)) {
        emit logError(error);
        panel->setDownsampleStatus(error);
        if(!m_dialogParent->property("paramcadBatchCapture").toBool())
            QMessageBox::warning(m_dialogParent, workspaceTr("降采样"), error);
        return;
    }
    const int count = m_document->pointCount();
    panel->setDownsampleStatus(
        workspaceTr("已切换为 %1 mm 降采样点云：%2 个点").arg(voxelSizeMm).arg(count));
    emit logInfo(workspaceTr("降采样点云 %1 mm：%2 -> %3（%4 个点）")
                     .arg(voxelSizeMm)
                     .arg(QFileInfo(root).fileName())
                     .arg(QFileInfo(target).fileName())
                     .arg(count));
    refreshDownsampleControls();
}

bool PointCloudWorkspaceController::switchActiveCloud(const QString& path, QString& error)
{
    if (!m_document->load(path, error)) return false;
    updateCadActions();
    const QFileInfo info(path);
    m_ui.scenePanel->updateNodeLabel(
        SceneNodeType::PointCloud,
        QStringLiteral("%1 (%2 points)").arg(info.fileName()).arg(m_document->pointCount()));
    emit statusTextChanged(workspaceTr("就绪 | 点云：%1").arg(info.fileName()));
    return true;
}

void PointCloudWorkspaceController::reconstructCad()
{
    startCadReconstruction(true);
}

void PointCloudWorkspaceController::reconstructCadWithoutProjection()
{
    startCadReconstruction(false);
}

std::optional<reconstruction::LengthUnit> PointCloudWorkspaceController::confirmCloudUnit()
{
    const auto sourceUnit = cloudUnit(*m_document);
    if (sourceUnit != reconstruction::LengthUnit::Arbitrary) return sourceUnit;
    const auto choice = QMessageBox::warning(m_dialogParent, workspaceTr("点云尺度未知"),
        workspaceTr("该点云没有公制尺度元数据。继续将把 1 个源单位视为 CAD 模型中的 1 毫米。是否按此明确假设继续？"),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (choice != QMessageBox::Yes) return std::nullopt;
    emit logWarning(workspaceTr("Point-cloud scale is unknown; using the user-approved "
                                "assumption 1 source unit = 1 mm"));
    return reconstruction::LengthUnit::Millimeter;
}

void PointCloudWorkspaceController::runProjectionDebug()
{
    if (!m_ui.projectionDebugAction->isEnabled()) return;
    if (!m_projectionPage) {
        auto* page = new QWidget(m_ui.propertyPanel);
        page->setObjectName(QStringLiteral("projectionDebugPage"));
        m_projectionPage = page;
        auto* layout = new QVBoxLayout(page);
        auto* mode = new QComboBox(page);
        mode->setObjectName(QStringLiteral("projectionDebugMode"));
        using Path = reconstruction::ProjectionPath;
        mode->addItem(workspaceTr("单轴拉伸"), int(Path::SingleAxis));
        mode->addItem(workspaceTr("两轴拉伸求交"), int(Path::TwoAxisIntersection));
        mode->addItem(workspaceTr("分段拉伸与加材 / 切除"), int(Path::Segmented));
        m_projectionMode = mode;
        layout->addWidget(new QLabel(workspaceTr("投影重建方式"), page));
        layout->addWidget(mode);

        auto* buttons = new QDialogButtonBox(page);
        auto* run = buttons->addButton(workspaceTr("运行投影"), QDialogButtonBox::ActionRole);
        run->setObjectName(QStringLiteral("runProjectionDebugButton"));
        m_runProjection = run;
        auto* inspect = buttons->addButton(workspaceTr("查看二维点"), QDialogButtonBox::ActionRole);
        inspect->setObjectName(QStringLiteral("inspectProjectionSketchButton"));
        m_inspectProjection = inspect;
        auto* cancel = new QToolButton(page);
        cancel->setDefaultAction(m_ui.cancelCadAction);
        cancel->setText(workspaceTr("取消"));
        cancel->setToolButtonStyle(Qt::ToolButtonTextOnly);
        buttons->addButton(workspaceTr("关闭"), QDialogButtonBox::RejectRole)
            ->setObjectName(QStringLiteral("closeProjectionDebugButton"));
        auto* controls = new QHBoxLayout;
        controls->addWidget(cancel);
        controls->addWidget(buttons);
        layout->addLayout(controls);

        m_projectionStatus = new QLabel(page);
        m_projectionStatus->setWordWrap(true);
        m_projectionStatus->setTextFormat(Qt::PlainText);
        layout->addWidget(m_projectionStatus);
        layout->addStretch();
        connect(run, &QPushButton::clicked, this, [this, mode] {
            startProjectionDebug(static_cast<Path>(mode->currentData().toInt()), false);
        });
        connect(inspect, &QPushButton::clicked, this, [this, mode] {
            startProjectionDebug(static_cast<Path>(mode->currentData().toInt()), true);
        });
        connect(mode, &QComboBox::currentIndexChanged, this, [this] {
            clearProjectionDebugResult();
            updateCadActions();
        });
        connect(buttons, &QDialogButtonBox::rejected, this, [this, page] {
            clearProjectionDebugResult();
            m_ui.propertyPanel->removeProjectionDebug(page);
            m_projectionPage.clear();
            m_projectionStatus.clear();
            m_runProjection.clear();
            m_inspectProjection.clear();
            m_projectionMode.clear();
            page->deleteLater();
        });
        clearProjectionDebugResult();
    }
    m_ui.propertyPanel->showProjectionDebug(m_projectionPage);
    updateCadActions();
}

void PointCloudWorkspaceController::clearProjectionDebugResult()
{
    if (m_projectionReport) {
        m_projectionPage->layout()->removeWidget(m_projectionReport);
        m_projectionReport->hide();
        m_projectionReport->deleteLater();
        m_projectionReport.clear();
    }
    m_ui.viewer->clearProjectionPreview();
    if (m_projectionStatus)
        m_projectionStatus->setText(workspaceTr("选择一种方式，然后运行投影。"));
}

void PointCloudWorkspaceController::startProjectionDebug(reconstruction::ProjectionPath path, bool inspectOnly)
{
    if (!m_projectionPage || !m_ui.projectionDebugAction->isEnabled()) return;
    clearProjectionDebugResult();
    m_startingCad = true;
    const auto resetStarting = qScopeGuard([this] {
        m_startingCad = false;
        updateCadActions();
    });
    updateCadActions();
    m_cadCloudRevision = m_document->revision();
    m_cadModelRevision = m_modelingController->revision();
    m_cadBodyRevision = m_ui.viewer->bodyRevision();
    m_cadSourcePath = m_document->sourcePath();
    const auto unit = confirmCloudUnit();
    if (!unit || !m_projectionPage || m_document->revision() != m_cadCloudRevision
        || m_document->sourcePath() != m_cadSourcePath)
        return;
    m_projectionStatus->setText(inspectOnly ? workspaceTr("正在准备二维投影点…")
                                           : workspaceTr("正在后台拟合所选方式…"));
    m_cadTask->startProjection(m_document->sourceCloud(), *unit, path, inspectOnly);
}

void PointCloudWorkspaceController::startCadReconstruction(bool projectionFallback)
{
    if (m_importThread || m_cadTask->isRunning() || m_startingCad || m_analysisThread || m_downsampleThread) return;
    if (m_document->sourcePath().isEmpty()) {
        emit logWarning(workspaceTr("Load a point cloud before reconstructing a model."));
        emit statusMessage(workspaceTr("尚未载入点云"), 3500);
        return;
    }

    m_startingCad = true;
    const auto resetStarting = qScopeGuard([this] {
        m_startingCad = false;
        updateCadActions();
        if (!m_cadTask->isRunning() && std::exchange(m_analyseAfterCad, false))
            startPointCloudAnalysis();
    });
    updateCadActions();
    m_cadCloudRevision = m_document->revision();
    m_cadModelRevision = m_modelingController->revision();
    m_cadBodyRevision = m_ui.viewer->bodyRevision();
    m_cadSourcePath = m_document->sourcePath();
    m_cadOverwriteManual = false;
    if (m_modelingController->hasProtectedReconstruction()) {
        const auto answer = QMessageBox::question(m_dialogParent, workspaceTr("替换手动修改"),
            workspaceTr("当前重建特征包含手动修改。重新拟合会替换这些特征及其尺寸调整。是否继续？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) return;
        m_cadOverwriteManual = true;
    }
    std::vector<vtkIdType> ids;
    PlyCloud cloud = m_document->workingCloud(&ids);
    std::map<vtkIdType, reconstruction::PointId> pointIds;
    for (const auto id : ids) pointIds[id] = static_cast<reconstruction::PointId>(id) + 1;
    std::vector<reconstruction::TemplateHint> hints;
    for (const auto& region : m_document->selection().regions()) {
        if (!region.templateLocked || m_document->selection().regionPointCount(region.id) == 0)
            continue;
        reconstruction::TemplateHint hint;
        hint.regionId = region.id;
        const auto& type = region.geometry;
        using Kind = reconstruction::TemplateKind;
        hint.kind = type == "cuboid" ? Kind::Box : type == "sphere" ? Kind::Sphere
            : type == "cylinder" ? Kind::Cylinder : type == "cone" ? Kind::Cone
            : type == "ring" ? Kind::Torus : Kind::Plane;
        for (auto id : m_document->selection().regionPoints(region.id))
            hint.points.push_back(pointIds.at(id));
        hints.push_back(std::move(hint));
    }
    if (cloud.positions.empty()) {
        m_ui.relationsPanel->setStatus(workspaceTr("当前工作点云为空，无法重建 CAD。"));
        return;
    }

    const auto sourceUnit = confirmCloudUnit();
    if (!sourceUnit) return;

    // A scale confirmation runs a nested event loop; verify the snapshot again
    // before launching in case an import/edit occurred while it was open.
    if (m_document->revision() != m_cadCloudRevision
        || m_modelingController->revision() != m_cadModelRevision
        || m_ui.viewer->bodyRevision() != m_cadBodyRevision) {
        m_ui.relationsPanel->setStatus(workspaceTr("点云或模型已修改，请重新启动 CAD 重建。"));
        return;
    }
    m_analyseAfterCad = false;
    m_inferAfterCad = false;
    std::vector<reconstruction::SurfaceRelationHint> relations;
    for(const auto& relation:m_document->selection().relations()) {
        if(relation.automatic) continue;
        reconstruction::SurfaceRelationHint hint;
        bool known=false;
        for(int i=0;i<=static_cast<int>(reconstruction::SurfaceRelationKind::AxisParallelToPlane);++i) {
            const auto kind=static_cast<reconstruction::SurfaceRelationKind>(i);
            if(relation.type==QString::fromLatin1(reconstruction::surfaceRelationName(kind))) {hint.kind=kind;known=true;break;}
        }
        if(!known) continue;
        for(auto id:m_document->selection().regionPoints(relation.first)) if(pointIds.count(id)) hint.firstPoints.push_back(pointIds.at(id));
        for(auto id:m_document->selection().regionPoints(relation.second)) if(pointIds.count(id)) hint.secondPoints.push_back(pointIds.at(id));
        relations.push_back(std::move(hint));
    }
    m_cadTask->start(std::move(cloud), *sourceUnit, std::move(hints),
                     m_ui.relationsPanel->surfaceTolerance(), m_surfaceAnalysis,std::move(relations),projectionFallback);
    emit logInfo(workspaceTr("仅点云重建：使用观测坐标、法向、单位和用户模板，不读取源模型或面对应表。"));
}

void PointCloudWorkspaceController::cancelCadReconstruction()
{
    if (m_downsampleThread) m_downsampleThread->requestInterruption();
    if (m_analysisThread) m_analysisThread->requestInterruption();
    m_cadTask->cancel();
}

void PointCloudWorkspaceController::prepareProjectReplacement()
{
    m_ui.viewer->closeBoundaryInspection();
    if (m_importThread) {
        ++m_importGeneration;
        m_pendingImport.clear();
        emit statusMessage({}, 0);
    }
    if (m_downsampleThread) m_downsampleThread->requestInterruption();
    m_analyseAfterCad = false;
    m_inferAfterCad = false;
    cancelCadReconstruction();
    if (m_analysisThread) m_analysisThread->requestInterruption();
}

void PointCloudWorkspaceController::startAnalysisExport()
{
    if(m_exportThread || !m_document->hasCloud())return;
    const auto revision=m_document->revision();const auto source=m_document->sourcePath();
    auto path=QFileDialog::getSaveFileName(m_dialogParent,workspaceTr("导出识别结果（AI 分析）"),
        source+QStringLiteral(".pcad-analysis"),workspaceTr("识别结果目录 (*.pcad-analysis)"));
    if(path.isEmpty())return;
    if(!path.endsWith(".pcad-analysis",Qt::CaseInsensitive))path+=".pcad-analysis";
    if(m_document->revision()!=revision || m_document->sourcePath()!=source || !m_document->hasCloud()) {
        emit logWarning(workspaceTr("选择位置期间点云已更改，请重新导出识别结果。"));return;
    }
    startAnalysisExportTo(path);
}

void PointCloudWorkspaceController::startAnalysisExportTo(const QString& path)
{
    const auto revision=m_document->revision();const auto source=m_document->sourcePath();
    AnalysisBundleSnapshot snapshot;
    const auto& selection=m_document->selection();
    snapshot.cloud=m_document->sourceCloud();snapshot.deleted=selection.deleted();
    snapshot.memberships=selection.memberships();snapshot.regions=selection.regions();
    snapshot.relations=selection.relations();snapshot.selected=selection.selected();
    snapshot.positionsFingerprint=selection.positionsFingerprint();
    snapshot.sourceName=QFileInfo(source).fileName();snapshot.sourceUnit=m_document->sourceUnit();
    snapshot.documentRevision=revision;
    if(!m_surfaceSupportReport.isEmpty() && m_surfaceSupportReportFingerprint==surfaceSupportFingerprint(*m_document))
        snapshot.supportValidation=m_surfaceSupportReport;
    snapshot.surfaces=m_ui.viewer->pointCloudSurfaceDisplaySnapshot();
    snapshot.displayedSurfaces=m_ui.viewer->displayedPointCloudSurfaces();
    const auto camera=m_ui.viewer->cameraParameters();
    const auto vector=[](const double* p){return QJsonArray{p[0],p[1],p[2]};};
    snapshot.view={{"camera_position",vector(camera.position)},{"camera_target",vector(camera.target)},
        {"camera_up",vector(camera.up)},{"field_of_view",camera.fieldOfView},
        {"parallel_projection",camera.parallelProjection},{"parallel_scale",m_ui.viewer->parallelScale()},
        {"cloud_visible",m_ui.viewer->pointCloudVisible()},{"point_size",m_ui.viewer->pointCloudPointSize()}};
    struct Result {bool ok=false;QString error;};
    auto result=std::make_shared<Result>();
    m_ui.relationsPanel->setAnalysisExportBusy(true);
    emit statusMessage(workspaceTr("正在后台导出识别结果…"),0);
    m_exportThread=QThread::create([snapshot=std::move(snapshot),path,result] {
        result->ok=exportAnalysisBundle(snapshot,path,result->error,[] {return QThread::currentThread()->isInterruptionRequested();});
    });
    auto* thread=m_exportThread;thread->setParent(this);
    connect(thread,&QThread::finished,this,[this,thread,result,path] {
        m_exportThread=nullptr;thread->deleteLater();m_ui.relationsPanel->setAnalysisExportBusy(false);
        finishCliOperation(result->ok,result->error,{{"path",path}});
        if(result->ok) {
            emit logInfo(workspaceTr("识别结果已导出：%1").arg(path));
            emit statusMessage(workspaceTr("识别结果已导出，可从 README.md 开始查看"),6000);
        } else {
            emit logError(workspaceTr("识别结果导出失败：%1").arg(result->error));
            QMessageBox::warning(m_dialogParent,workspaceTr("导出识别结果"),result->error);
        }
    });
    thread->start();
}

void PointCloudWorkspaceController::updateCadActions()
{
    const bool running = m_cadTask->isRunning();
    const bool busy = running || m_startingCad || m_analysisThread || m_downsampleThread || m_importThread;
    m_ui.reconstructCadAction->setText(workspaceTr("重建 CAD"));
    m_ui.reconstructCadAction->setToolTip(workspaceTr("仅使用点云的位置、法向和用户模板拟合 CAD，不读取源模型或拓扑对应表。"));
    m_ui.reconstructCadAction->setEnabled(!busy && m_document->hasCloud()
                                         && m_document->pointCount() > 0);
    m_ui.projectionDebugAction->setEnabled(!busy && m_document->hasCloud()
                                          && !m_document->sourcePath().isEmpty());
    if (m_runProjection) m_runProjection->setEnabled(m_ui.projectionDebugAction->isEnabled());
    if (m_inspectProjection) {
        const auto path = static_cast<reconstruction::ProjectionPath>(m_projectionMode->currentData().toInt());
        m_inspectProjection->setVisible(path == reconstruction::ProjectionPath::SingleAxis ||
                                        path == reconstruction::ProjectionPath::TwoAxisIntersection);
        m_inspectProjection->setEnabled(m_ui.projectionDebugAction->isEnabled());
    }
    if (m_projectionMode) m_projectionMode->setEnabled(!busy);
    m_ui.cancelCadAction->setEnabled((running && !m_cadTask->isCancellationRequested()) ||
        (m_downsampleThread && !m_downsampleThread->isInterruptionRequested()) ||
        (m_analysisThread && !m_analysisThread->isInterruptionRequested()));
    m_ui.relationsPanel->setBusy(busy);
    m_ui.relationsPanel->setCadReconstructionBusy(running, m_ui.cancelCadAction->isEnabled());
}

void PointCloudWorkspaceController::showProjectionDebugResult(const CadReconstructionResult& result)
{
    if (!m_projectionPage || m_document->revision() != m_cadCloudRevision
        || m_document->sourcePath() != m_cadSourcePath) {
        clearProjectionDebugResult();
        return;
    }
    using Outcome = CadReconstructionResult::Outcome;
    if (result.outcome == Outcome::Cancelled) {
        const auto message = workspaceTr("单件投影已取消。");
        m_projectionStatus->setText(message);
        m_ui.relationsPanel->setStatus(message);
        m_ui.modelingPanel->setStatusText(message);
        emit statusMessage(message, 5000);
        return;
    }
    const auto& debug = result.projectionDebug.value();
    const bool accepted = result.outcome == Outcome::Success;
    const auto message = debug.inspectOnly
        ? accepted ? workspaceTr("二维投影点已就绪 · %1 组").arg(qulonglong(debug.fit.sketchInputs.size()))
                   : workspaceTr("无法生成二维投影点 · %1").arg(result.error)
        : accepted
        ? workspaceTr("投影拟合通过 · %1 条候选轴 · 内点率 %2%")
            .arg(debug.fit.axes.size()).arg(100*debug.fit.quality.surfaceInlierFraction, 0, 'f', 1)
        : workspaceTr("投影拟合未通过 · %1").arg(result.error);
    m_ui.relationsPanel->setStatus(message);
    m_ui.modelingPanel->setStatusText(message);
    m_projectionStatus->setText(message);
    emit statusMessage(message, 5000);
    emit logInfo(message);

    auto* report = new QWidget(m_projectionPage);
    m_projectionReport = report;
    auto* pageLayout = static_cast<QVBoxLayout*>(m_projectionPage->layout());
    pageLayout->insertWidget(pageLayout->count()-1, report, 1);
    auto* layout = new QVBoxLayout(report);
    layout->setContentsMargins(0, 0, 0, 0);

    const auto& program = accepted ? debug.fit.operations : debug.fit.rejectedOperations;
    if (!debug.fit.sketchInputs.empty()) {
        layout->addWidget(new ProjectionSketchPanel(debug.fit.sketchInputs,
            program.empty() ? nullptr : &program.front(), report),1);
    }
    if (debug.inspectOnly) return;
    if (!program.empty() && m_ui.viewer->bodyRevision() == m_cadBodyRevision) {
        const double scale = debug.sourceUnit == reconstruction::LengthUnit::Meter ? .001 : 1.;
        if (program.size() > 1) {
            auto* step = new QComboBox(report);
            step->setObjectName(QStringLiteral("projectionDebugStep"));
            step->addItem(workspaceTr("显示全部步骤"));
            for (const auto& operation : program) {
                step->addItem(workspaceTr("%1. %2 · %3 mm")
                    .arg(step->count()).arg(projectionOperationLabel(operation.operation))
                    .arg(operation.depth, 0, 'g', 6));
            }
            layout->addWidget(step);
            connect(step, &QComboBox::currentIndexChanged, this,
                [this, program, scale, cloudRevision = m_cadCloudRevision, bodyRevision = m_cadBodyRevision](int index) {
                    if (index < 0 || m_document->revision() != cloudRevision
                        || m_ui.viewer->bodyRevision() != bodyRevision) return;
                    m_ui.viewer->setProjectionPreviewOperations(program, scale, index-1);
                });
        }
        auto* solid = new QCheckBox(accepted ? workspaceTr("显示结果实体") : workspaceTr("显示未通过的最佳候选"), report);
        solid->setChecked(!debug.previewShape.IsNull());
        solid->setEnabled(solid->isChecked());
        const auto& first = program.front();
        const auto description = program.size() == 1
            ? workspaceTr("橙色为起始草图与拉伸方向。浅色轮廓标出拉伸终点。深度 %1 mm。")
                .arg(first.depth, 0, 'g', 6)
            : program.back().operation == modeling::ExtrudeOperation::Intersect
                ? workspaceTr("橙色和紫色分别标出两轴的草图与拉伸方向。半透明实体是交集。")
                : workspaceTr("橙色为基体，绿色为加材，红色为切除。可选中某一步查看其草图和方向。");
        auto* legend = new QLabel(description, report);
        legend->setWordWrap(true);
        layout->addWidget(solid);
        layout->addWidget(legend);
        if (!debug.previewShape.IsNull()) m_ui.viewer->setProjectionPreviewBody(debug.previewShape, scale);
        m_ui.viewer->setProjectionPreviewOperations(program, scale);
        connect(solid, &QCheckBox::toggled, m_ui.viewer, &VTKViewer::setProjectionPreviewBodyVisible);
    }

    const auto bytes = QJsonDocument(projectionDebugReport(m_cadSourcePath, result)).toJson();
    auto* details = new QCheckBox(workspaceTr("显示诊断报告"), report);
    auto* text = new QPlainTextEdit(QString::fromUtf8(bytes), report);
    text->setObjectName(QStringLiteral("projectionDebugReportText"));
    text->setReadOnly(true);
    text->setLineWrapMode(QPlainTextEdit::NoWrap);
    text->hide();
    connect(details, &QCheckBox::toggled, text, &QWidget::setVisible);
    layout->addWidget(details);
    layout->addWidget(text, 1);
    auto* buttons = new QDialogButtonBox(report);
    layout->addWidget(buttons);
    auto* save = buttons->addButton(workspaceTr("保存报告…"), QDialogButtonBox::ActionRole);
    const auto filename = QFileInfo(m_cadSourcePath).completeBaseName() + QStringLiteral(".projection.json");
    connect(save, &QPushButton::clicked, report, [report, bytes, filename] {
        const auto path = QFileDialog::getSaveFileName(report, workspaceTr("保存投影调试报告"),
            filename, workspaceTr("JSON 文件 (*.json)"));
        if (path.isEmpty()) return;
        QSaveFile file(path);
        if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
            QMessageBox::warning(report, workspaceTr("保存投影调试报告"), file.errorString());
    });

    if (accepted) {
        auto* apply = buttons->addButton(workspaceTr("应用到 CAD"), QDialogButtonBox::ActionRole);
        apply->setObjectName(QStringLiteral("applyProjectionDebugCadButton"));
        apply->setToolTip(workspaceTr("用本次投影结果替换之前的重建 CAD"));
        reconstruction::CompoundCandidate candidate;
        candidate.projectionPrograms.push_back(debug.fit.operations);
        connect(apply, &QPushButton::clicked, this,
            [this, report, apply, candidate = std::move(candidate), cloudRevision = m_cadCloudRevision,
             modelRevision = m_cadModelRevision, bodyRevision = m_cadBodyRevision, sourcePath = m_cadSourcePath] {
                if (m_cadTask->isRunning() || m_startingCad || m_analysisThread
                    || m_document->revision() != cloudRevision || m_document->sourcePath() != sourcePath
                    || m_modelingController->revision() != modelRevision || m_ui.viewer->bodyRevision() != bodyRevision) {
                    QMessageBox::warning(report, workspaceTr("应用到 CAD"),
                        workspaceTr("点云或 CAD 工作区已变化或正忙，请重新运行投影。"));
                    return;
                }
                if (!m_modelingController->commitReconstructedCompound(candidate)) {
                    QMessageBox::warning(report, workspaceTr("应用到 CAD"), m_modelingController->lastError());
                    return;
                }
                apply->setEnabled(false);
                emit statusMessage(workspaceTr("投影结果已应用到 CAD。"), 5000);
            });
    }
    layout->addStretch();
}

void PointCloudWorkspaceController::onCadReconstructionFinished(const CadReconstructionResult& result)
{
    if (result.projectionDebug) {
        showProjectionDebugResult(result);
        return;
    }
    bool applied=false;QString commandError=result.error;
    const auto finishCommand=qScopeGuard([&] {
        if(m_cliOperation=="reconstruct cad")finishCliOperation(applied,
            applied?QString():commandError.isEmpty()?QStringLiteral("CAD result was cancelled, stale or not applied"):commandError,
            {{"partial_reconstruction",result.partialAssembly},{"features",qint64(m_modelingController->features().size())},
             {"document_revision",QString::number(m_document->revision())},{"cache",result.cache}});
    });
    const auto reportStatus = [this](const QString& message) {
        m_ui.relationsPanel->setStatus(message);
        m_ui.modelingPanel->setStatusText(message);
        emit statusMessage(message, 5000);
    };
    if (m_document->revision() != m_cadCloudRevision
        || m_modelingController->revision() != m_cadModelRevision
        || m_ui.viewer->bodyRevision() != m_cadBodyRevision) {
        const QString message = workspaceTr("点云或模型已修改，本轮 CAD 重建结果未应用；可重新重建。");
        reportStatus(message);
        emit logWarning(message);
        return;
    }
    if (result.outcome == CadReconstructionResult::Outcome::Cancelled) {
        reportStatus(workspaceTr("CAD 重建已取消，模型保持原状。"));
        emit logInfo(workspaceTr("CAD reconstruction cancelled"));
        return;
    }
    // Cache only after checking that the cloud/model still match this run.
    if (!result.cache.isEmpty())
        emit logInfo(workspaceTr("CAD cache · %1").arg(QString::fromUtf8(
            QJsonDocument(result.cache).toJson(QJsonDocument::Compact))));
    if (result.analysis.pointFingerprint) m_surfaceAnalysis = result.analysis;
    if (result.preprocessingComplete) {
        const auto& preprocessingReport = result.preprocessing;
        emit logInfo(workspaceTr("Point-cloud preprocessing: %1 → %2 points, removed %3 voxel "
                   "duplicates and %4 outliers, estimated %5 normals")
                .arg(preprocessingReport.inputPointCount)
                .arg(preprocessingReport.outputPointCount)
                .arg(preprocessingReport.removedByVoxel)
                .arg(preprocessingReport.removedAsOutliers)
                .arg(preprocessingReport.normalsEstimated));
        if (preprocessingReport.normalsEstimated) {
            emit logInfo(workspaceTr("补算法向：已确定方向 %1 个，方向未确定 %2 个。")
                .arg(preprocessingReport.estimatedNormalsOriented)
                .arg(preprocessingReport.normalsWithoutOrientation));
        }
    }
    std::vector<DetectedPointCloudSurface> surfaces;
    if (result.outcome == CadReconstructionResult::Outcome::Unsupported) {
        const double sourceScale = cloudUnit(*m_document) == reconstruction::LengthUnit::Meter ? 1000.0 : 1.0;
        const auto& preview = result.sourceSurfacePreview ? *result.sourceSurfacePreview : result.analysis;
        surfaces = describeAnalysisPatches(preview, m_document->sourceCloud().positions.size(),
            sourceScale, result.sourceSurfacePreview.has_value());
    }
    if(result.assembly && !result.assembly->parts.empty() && surfaces.empty()) {
        std::vector<DetectedPointCloudSurface> groups;size_t closed=0;
        const double scale=cloudUnit(*m_document)==reconstruction::LengthUnit::Meter?.001:1.0;
        const auto vector=[&](modeling::Vec3 p){return QJsonArray{p.x*scale,p.y*scale,p.z*scale};};
        for(const auto& part:result.assembly->parts){DetectedPointCloudSurface group;group.surface.geometry="part";
            const bool fitted = part.projection && !part.projection->operations.empty();
            const auto status = part.closed ? workspaceTr("已闭合") :
                fitted ? workspaceTr("投影拟合通过") : workspaceTr("未闭合");
            group.surface.name=workspaceTr("部件候选 %1 · %2").arg(part.id).arg(status);
            group.surface.confidence=part.confidence;group.surface.templateLocked=false;
            group.surface.parameters={{"status",QString::fromStdString(part.status)},{"closed",part.closed},{"assembly_part_id",double(part.id)},{"lower",vector(part.lower)},
                {"upper",vector(part.upper)},{"open_edges",double(part.openEdges)},{"reason",QString::fromStdString(part.reason)}};
            if (part.projection) {
                group.surface.parameters["projection_accepted"]=fitted;
                group.surface.parameters["projection_reason"]=QString::fromStdString(part.projection->reason);
            }
            for (const auto id : part.pointIds)
                if (id > 0 && id <= m_document->sourceCloud().positions.size())
                    group.points.push_back(static_cast<vtkIdType>(id - 1));
            if(!group.points.empty())groups.push_back(std::move(group));if(part.closed)++closed;
        }
        m_document->replacePartCandidates(groups,false);
        m_inferAfterCad = !groups.empty();
        std::map<size_t,int> regionIds;
        for(const auto& region:m_document->selection().regions())if(region.geometry=="part")
            regionIds[size_t(region.parameters["assembly_part_id"].toInteger())]=region.id;
        std::vector<std::array<double,6>> lines;std::vector<bool> closedLines;std::vector<int> lineRegions;
        for(const auto& part:result.assembly->parts)for(auto id:part.patchIds)for(const auto& loop:result.assembly->patches[id-1].loops)
            for(size_t i=0;i<loop.size();++i){const auto a=loop[i],b=loop[(i+1)%loop.size()];lines.push_back({a.x*scale,a.y*scale,a.z*scale,b.x*scale,b.y*scale,b.z*scale});closedLines.push_back(part.closed);lineRegions.push_back(regionIds[part.id]);}
        m_ui.viewer->setAssemblyBoundaries(lines,closedLines,lineRegions);
        emit logInfo(workspaceTr("仅点云分离：%1 个候选，%2 个验证闭合，%3 个保留开放边界；候选不等同于原装配实例。")
            .arg(result.assembly->parts.size()).arg(closed).arg(result.assembly->parts.size()-closed));
        const auto projected = std::count_if(result.assembly->parts.begin(), result.assembly->parts.end(),
            [](const auto& part) { return part.status == "closed_projection"; });
        if (projected) emit logInfo(workspaceTr("投影重建闭合了 %1 个候选。").arg(projected));
        const auto paired = std::count_if(result.assembly->parts.begin(), result.assembly->parts.end(),
            [](const auto& part) {
                return part.status == "closed_inferred_extrusion" || part.status == "closed_inferred_recess";
            });
        if (paired) emit logInfo(workspaceTr("平行轮廓配对闭合了 %1 个候选。").arg(paired));
    }
    if (result.outcome == CadReconstructionResult::Outcome::Unsupported) {
        if (!result.sourceSurfacePreviewError.isEmpty()) emit logWarning(result.sourceSurfacePreviewError);
        if(!surfaces.empty()) {
            m_document->replacePartCandidates(surfaces,false);
            m_inferAfterCad = true;
            if (result.sourceSurfacePreview) {
                const auto report = [](const auto& r) {
                    return QJsonDocument::fromJson(QByteArray::fromStdString(reconstruction::serializeSurfaceSupportReport(r))).object();
                };
                m_surfaceSupportReport = {{"version", 1}, {"scope", "cad_active_source"},
                    {"before_ownership", report(result.analysis.sourceSupportValidation)},
                    {"after_ownership", report(result.sourceSurfacePreview->supportValidation)}};
                m_surfaceSupportReportFingerprint = surfaceSupportFingerprint(*m_document);
            }
            // Assembly part IDs no longer own the displayed surface regions.
            m_ui.viewer->setAssemblyBoundaries({}, {}, {});
            emit logInfo(workspaceTr("CAD 未闭合，已保留并显示 %1 个检测面片。").arg(surfaces.size()));
        }
        QString message = workspaceTr("在 %1 中未能重建受支持的实体：%2")
                                    .arg(QFileInfo(m_cadSourcePath).fileName(), result.error);
        if(const auto* region=m_document->selection().region(result.failedRegionId))
            message=workspaceTr("模板「%1」重建失败，模型保持原状：%2").arg(region->name,result.error);
        message += !surfaces.empty()?workspaceTr(" 已显示检测面片；表面检测成功，实体闭合未通过。"):
            result.assembly?workspaceTr(" 分离结果已显示在点云集合中；未生成新的 CAD，已有模型保持原状。"):
            workspaceTr(" 未生成新的 CAD，已有模型保持原状。");
        reportStatus(message);
        emit logWarning(message);
        return;
    }
    if (result.outcome != CadReconstructionResult::Outcome::Success) {
        const QString message = result.outcome == CadReconstructionResult::Outcome::PreprocessingFailed
            ? workspaceTr("点云预处理失败：%1").arg(result.error)
            : workspaceTr("CAD 重建失败：%1").arg(result.error);
        reportStatus(message);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("重建 CAD"), message);
        return;
    }
    const auto& candidate = result.candidate;
    if (!m_modelingController->commitReconstructedCompound(candidate, m_cadOverwriteManual)) {
        const QString message = m_modelingController->lastError();
        commandError=message;
        reportStatus(message);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("重建 CAD"), message);
        return;
    }
    if(!candidate.unresolvedPointIds.empty())
        emit logWarning(workspaceTr("已重建有充分支持的独立部件；另有 %1 个观测点所属部件尚未重建。")
            .arg(candidate.unresolvedPointIds.size()));
    // Derived results share the user's edit history instead of adding undo
    // steps that an automatic reconstruction would immediately recreate.
    for (const auto& fit : result.templateFits)
        m_document->updateRegionFit(fit.regionId,
            describeTemplateFit(fit, result.preprocessing.appliedScaleToMillimeters), false);
    if (!result.templateFits.empty()) {
        m_document->inferRelations(m_ui.relationsPanel->relationTolerance(),
                                  m_ui.relationsPanel->angleTolerance(), false);
        emit logInfo(workspaceTr("已按 %1 个手动模板约束完成整体重建。原始点归属保持不变。").arg(result.templateFits.size()));
    }

    emit logInfo((result.assembly?workspaceTr("重建结果：%1 个长方体，%2 个棱柱，%3 个通孔，闭合候选所属观测点比例 %4，%5 个解析曲面实体，%6 个有限边界实体"):
                workspaceTr("重建实体：%1 个长方体，%2 个多边形棱柱，%3 个通孔，表面覆盖率 %4，%5 个曲面实体，%6 个有限壳体"))
                .arg(candidate.boxes.size()).arg(candidate.prisms.size())
                .arg(candidate.throughHoles.size()).arg(candidate.surfaceCoverage, 0, 'f', 3).arg(candidate.curvedSolids.size()).arg(candidate.profileBodies.size()));
    QString curvedName;
    if (!candidate.curvedSolids.empty()) {
        curvedName=std::visit([&](const auto& params)->QString {
            using P=std::decay_t<decltype(params)>;
            if constexpr(std::is_same_v<P,modeling::SpherePrimitiveParams>)return workspaceTr("球");
            else if constexpr(std::is_same_v<P,modeling::CylinderPrimitiveParams>)return workspaceTr("圆柱");
            else if constexpr(std::is_same_v<P,modeling::ConePrimitiveParams>)return workspaceTr("圆锥 / 圆台");
            else return workspaceTr("圆环");
        },candidate.curvedSolids.front().primitive);
    }
    for (const reconstruction::ThroughHoleCandidate& hole : candidate.throughHoles) {
        emit logInfo(workspaceTr("Recognized through hole: diameter %1, thickness %2, confidence %3")
                    .arg(hole.primitive.radius * 2.0, 0, 'f', 3)
                    .arg(hole.bodyThickness, 0, 'f', 3)
                    .arg(hole.confidence, 0, 'f', 3));
    }
    emit statusTextChanged(curvedName.isEmpty() ? workspaceTr("就绪 | 模型：重建实体（%1 个长方体，%2 个棱柱） | 相机：%3")
                               .arg(candidate.boxes.size()).arg(candidate.prisms.size())
                               .arg(m_ui.viewer->cameraParameters().parallelProjection
                                         ? workspaceTr("正交投影") : workspaceTr("透视投影")) :
                          workspaceTr("就绪 | 模型：%1 | 相机：%2").arg(curvedName).arg(m_ui.viewer->cameraParameters().parallelProjection ? workspaceTr("正交投影") : workspaceTr("透视投影")));
    reportStatus(!candidate.unresolvedPointIds.empty()?workspaceTr("已重建部分独立部件；不确定部件仍保留在点云中。"):
        result.assembly?workspaceTr("已显示部件候选并更新可闭合部分的 CAD；未闭合部分保留在点云集合中。"):
        workspaceTr("CAD 重建完成，已更新参数化模型。"));
    m_analyseAfterCad = !result.assembly && m_document->selection().regionPointCount(0) >= 20;
    applied=true;
}

void PointCloudWorkspaceController::startPointCloudAnalysis(int regionId, bool recordUndo)
{
    if (m_analysisThread || m_downsampleThread || m_startingCad || m_cadTask->isRunning() || !m_document->hasCloud()) return;
    std::vector<vtkIdType> ids;
    auto cloud = m_document->workingCloud(&ids, regionId == 0, regionId == 0 ? -1 : regionId);
    const auto* region = m_document->selection().region(regionId);
    const QString geometry = region ? region->geometry : QString();
    if (regionId && (!region || geometry == "cuboid")) return;
    if (cloud.positions.size() < 20) { m_ui.relationsPanel->setStatus(workspaceTr("至少需要 20 个有效点进行识别。")); return; }
    const quint64 revision = m_document->revision();
    const double tolerance = m_ui.relationsPanel->surfaceTolerance();
    auto result = std::make_shared<PointCloudAnalysisResult>();
    m_ui.relationsPanel->setStatus(regionId ? workspaceTr("正在后台重拟合集合…") : workspaceTr("正在后台识别表面，并映射回原始点…"));
    const auto sourceUnit=cloudUnit(*m_document);
    const auto cloudCount = cloud.positions.size();
    m_analysisThread = QThread::create([cloud = std::move(cloud), ids = std::move(ids), geometry, tolerance, result, sourceUnit] {
        try { *result = analysePointCloud(cloud, ids, tolerance, geometry,sourceUnit); }
        catch (const std::exception& error) { result->error = QString::fromUtf8(error.what()); }
    });
    auto* thread = m_analysisThread;
    thread->setParent(this);
    updateCadActions();
    connect(thread, &QThread::finished, this, [this, thread, result, revision, regionId, recordUndo, count = cloudCount] {
        m_analysisThread = nullptr;
        bool continuing=false;
        const auto finishCommand=qScopeGuard([&] {
            if(m_cliOperation=="detect"&&!continuing)finishCliOperation(false,
                result->error.isEmpty()?QStringLiteral("Surface analysis was cancelled, stale or unsupported"):result->error);
        });
        updateCadActions();
        if (thread->isInterruptionRequested()) {
            m_ui.relationsPanel->setStatus(workspaceTr("识别已取消。"));return;
        }
        if (m_document->revision() != revision) {
            m_ui.relationsPanel->setStatus(workspaceTr("工作点云或关系已被修改，本轮识别结果未应用；可重新识别。")); return;
        }
        if (!result->error.isEmpty()) { m_ui.relationsPanel->setStatus(result->error); emit logWarning(result->error); return; }
        if (!regionId) m_surfaceAnalysis = result->analysis;
        if(!regionId) {
            m_document->addSurfaces(result->surfaces, recordUndo);
            const auto report=[](const auto& r){return QJsonDocument::fromJson(QByteArray::fromStdString(reconstruction::serializeSurfaceSupportReport(r))).object();};
            m_surfaceSupportReport={{"version",1},{"scope","detection_active_subset"},
                {"before_ownership",report(result->sourceSupportValidation)},{"after_ownership",report(result->finalSupportValidation)}};
            m_surfaceSupportReportFingerprint=surfaceSupportFingerprint(*m_document);
        }
        if (result->surfaces.empty()) {
            m_ui.relationsPanel->setStatus(workspaceTr("没有识别到受支持的表面；点保持原有归属，可调整容差或手动分组。")); return;
        }
        if (regionId) {
            const auto best = std::max_element(result->surfaces.begin(), result->surfaces.end(),
                [](const auto& a, const auto& b) { return a.points.size() < b.points.size(); });
            if (best->points.size() < count * 0.7) {
                m_ui.relationsPanel->setStatus(workspaceTr("拟合表面覆盖不足 70%，保留原集合；可修正点归属后重试。")); return;
            }
            m_document->updateRegionFit(regionId, best->surface, recordUndo);
        }
        m_ui.relationsPanel->setStatus(workspaceTr("识别完成：%1 个表面，归属 %2 / %3 个原始点。未归属点已保留。")
            .arg(result->surfaces.size()).arg(result->assignedPoints).arg(count));
        emit logInfo(workspaceTr("Surface analysis completed: %1 surfaces, %2 source points assigned")
            .arg(result->surfaces.size()).arg(result->assignedPoints));
        if (!result->cache.isEmpty())
            emit logInfo(workspaceTr("Surface cache · %1").arg(QString::fromUtf8(
                QJsonDocument(result->cache).toJson(QJsonDocument::Compact))));
        continuing=true;
        startRelationInference(recordUndo);
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    thread->start();
}

void PointCloudWorkspaceController::startRelationInference(bool recordUndo)
{
    if (m_analysisThread || m_downsampleThread || m_startingCad || m_cadTask->isRunning() || !m_document->hasCloud()) return;
    const auto revision = m_document->revision();
    const double tolerance = m_ui.relationsPanel->relationTolerance();
    const double angle = m_ui.relationsPanel->angleTolerance();
    auto snapshot = m_document->selection().relationSnapshot();
    struct Result { std::vector<reconstruction::SurfaceRelation> relations; std::string error; };
    auto result = std::make_shared<Result>();
    m_analysisThread = QThread::create([snapshot = std::move(snapshot), tolerance, angle, result] {
        try { PointCloudSelection::computeRelations(snapshot, tolerance, angle, result->relations, result->error); }
        catch (const std::exception& error) { result->error = error.what(); }
    });
    auto* thread = m_analysisThread;
    thread->setParent(this);
    connect(thread, &QThread::finished, this, [this, thread, revision, tolerance, angle, result, recordUndo] {
        m_analysisThread = nullptr;updateCadActions();
        bool applied=false;
        const auto finishCommand=qScopeGuard([&] {
            if(m_cliOperation=="detect")finishCliOperation(applied,
                applied?QString():result->error.empty()?QStringLiteral("Relation result was cancelled or stale"):QString::fromStdString(result->error),
                {{"surface_count",qint64(m_document->selection().regions().size())},
                 {"relation_count",qint64(m_document->selection().relations().size())}});
        });
        if (thread->isInterruptionRequested() || m_document->revision()!=revision ||
            tolerance!=m_ui.relationsPanel->relationTolerance() || angle!=m_ui.relationsPanel->angleTolerance()) {
            m_ui.relationsPanel->setStatus(workspaceTr("点云或容差已修改，本轮自动关系未应用。"));return;
        }
        if (!result->error.empty()) {
            m_ui.relationsPanel->setStatus(QString::fromStdString(result->error));return;
        }
        m_document->applyInferredRelations(result->relations, recordUndo);
        applied=true;
        m_ui.relationsPanel->setStatus(workspaceTr("已按当前容差更新自动关系。手动关系与已删除的关系记录保持有效。"));
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    m_ui.relationsPanel->setStatus(workspaceTr("正在后台推断曲面关系…"));
    updateCadActions();thread->start();
}
