#include "app/MainWindow.h"
#include "ParamCADBuildInfo.h"
#include "app/CaptureController.h"
#include "capture/QualityEvaluation.h"
#include "capture/CaptureSessionController.h"
#include "app/CaptureWorkspaceController.h"
#include "app/ModelingController.h"
#include "app/ModelViewController.h"
#include "app/ParametricModelingTab.h"
#include "app/PlanarJunctionPanel.h"
#include "app/ProjectController.h"
#include "app/PointCloudWorkspaceController.h"
#include "app/ReconstructionController.h"
#include "app/SketchWorkspaceController.h"
#include "app/ShortcutSettings.h"
#include "app/UiLanguage.h"
#include "app/FeatureEditingController.h"
#include "core/CameraController.h"
#include "core/PointCloudDocument.h"
#include "io/ReconstructionCache.h"
#include "ui/HyperparameterSettings.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/FeatureDetailsPanel.h"
#include "ui/panels/TextParameterPanel.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/panels/SketchPanel.h"
#include "ui/panels/SketchToolPalette.h"
#include "ui/viewport/VTKViewer.h"

#include <QAction>
#include <QActionGroup>
#include <QAbstractItemView>
#include <QAbstractSlider>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QDir>
#include <QEventLoop>
#include <QScopeGuard>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QFileInfo>
#include <QLabel>
#include <QLineEdit>
#include <QKeySequence>
#include <QKeyEvent>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QPlainTextEdit>
#include <QComboBox>
#include <QStatusBar>
#include <QStyle>
#include <QTabBar>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QTextEdit>
#include <QUrl>

#include <algorithm>

namespace {
QString droppedAssetPath(const QMimeData* mimeData)
{
    if (!mimeData || !mimeData->hasUrls()) return {};
    const auto urls = mimeData->urls();
    // The scene owns one model and one cloud; do not silently replace assets
    // several times when a whole folder or a multi-file selection is dropped.
    if (urls.size() != 1 || !urls.first().isLocalFile()) return {};
    const QFileInfo file(urls.first().toLocalFile());
    const auto suffix = file.suffix().toLower();
    if (!file.isFile() || !file.isReadable()
        || (suffix != "step" && suffix != "stp" && suffix != "ply")) return {};
    return file.absoluteFilePath();
}

bool consumesArrowKeys(QWidget* widget)
{
    for (auto* current = widget; current; current = current->parentWidget()) {
        if (qobject_cast<QLineEdit*>(current) || qobject_cast<QAbstractSpinBox*>(current)
            || qobject_cast<QTextEdit*>(current) || qobject_cast<QPlainTextEdit*>(current)
            || qobject_cast<QComboBox*>(current) || qobject_cast<QAbstractItemView*>(current)
            || qobject_cast<QAbstractSlider*>(current)) return true;
    }
    return false;
}

VTKViewer* viewportOf(QWidget* page)
{
    if (!page) return nullptr;
    if (auto* viewer = qobject_cast<VTKViewer*>(page)) return viewer;
    return page->findChild<VTKViewer*>();
}
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("CAD 与点云工作台"));
    setAcceptDrops(true);
    resize(1500, 920);
    setDockNestingEnabled(true);
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    createActions();
    createMenus();
    createToolBar();
    createCentralArea();
    createDockPanels();
    createStatusBar();
    createWorkspaceControllers();
    connectUi();
    applyDarkTheme();
    qApp->installEventFilter(this);

    m_scenePanel->setScene(m_scene);
    m_scenePanel->setNodeState(SceneNodeType::Model, m_viewer->hasBody(), m_viewer->objectVisible());
    m_scenePanel->setNodeState(SceneNodeType::PointCloud, m_viewer->hasPointCloud(), m_viewer->pointCloudVisible());
    m_propertyPanel->cameraPanel()->setParameters(m_viewer->cameraParameters());

    logInfo(tr("Application started"));
    logInfo(tr("VTK renderer initialized"));

    // An empty document has no body to fit, so the camera is framed on the datum
    // planes that are what the user picks a sketch plane from.
    m_viewer->resetCamera();
}

MainWindow::~MainWindow()
{
    qApp->removeEventFilter(this);
    disconnect(m_viewportTabs, nullptr, this, nullptr);
    delete m_planarJunctionPanel;
    for (auto* tab : m_viewportTabs->findChildren<ParametricModelingTab*>()) delete tab;
    // Workspaces borrow widgets: destroy them (and join analysis) before Qt tears
    // down the window's child widgets and command controllers.
    delete m_pointCloudWorkspace;
    delete m_featureEditing;
    delete m_sketchWorkspace;
    delete m_captureWorkspace;
    delete m_modelViewController;
    // The view borrows the document; release it after workers and before data.
    delete m_viewer;
    delete m_pointCloudDocument;
    delete m_projectController;
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
    auto* widget = qobject_cast<QWidget*>(watched);
    if (!widget || widget->window() != this) return QMainWindow::eventFilter(watched, event);

    // Handle file drops across the whole window, including text and VTK widgets.
    if (event->type() == QEvent::DragEnter || event->type() == QEvent::DragMove
        || event->type() == QEvent::Drop) {
        auto* drop = static_cast<QDropEvent*>(event);
        if (drop->mimeData()->hasUrls()) {
            if (event->type() == QEvent::Drop) {
                dropEvent(drop);
            } else if (!droppedAssetPath(drop->mimeData()).isEmpty()
                       && drop->possibleActions().testFlag(Qt::CopyAction)) {
                drop->setDropAction(Qt::CopyAction);
                drop->accept();
            } else {
                drop->ignore();
            }
            return true;
        }
    }

    if ((event->type() == QEvent::KeyPress || event->type() == QEvent::ShortcutOverride)
        && !QApplication::activePopupWidget() && !consumesArrowKeys(widget)
        && !consumesArrowKeys(QApplication::focusWidget())) {
        auto* key = static_cast<QKeyEvent*>(event);
        const bool arrow = key->key() == Qt::Key_Left || key->key() == Qt::Key_Right
                           || key->key() == Qt::Key_Up || key->key() == Qt::Key_Down;
        const bool adjust = key->modifiers() == Qt::ControlModifier
                            && (key->key() == Qt::Key_Up || key->key() == Qt::Key_Down);
        if (arrow && (key->modifiers() == Qt::NoModifier || adjust)) {
            key->accept();
            if (event->type() == QEvent::ShortcutOverride) return true;
            // Windows key repeat supplies repeated presses while a key is held.
            if (adjust) {
                m_rotationStepDegrees = std::clamp(m_rotationStepDegrees
                    * (key->key() == Qt::Key_Up ? 1.25 : 0.8), 0.1, 45.0);
                updateRotationStep();
            } else {
                const double yaw = key->key() == Qt::Key_Left ? m_rotationStepDegrees
                                   : key->key() == Qt::Key_Right ? -m_rotationStepDegrees : 0.0;
                const double pitch = key->key() == Qt::Key_Up ? m_rotationStepDegrees
                                     : key->key() == Qt::Key_Down ? -m_rotationStepDegrees : 0.0;
                activeViewport()->rotateCamera(yaw, pitch);
            }
            return true;
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event)
{
    if (!droppedAssetPath(event->mimeData()).isEmpty()
        && event->possibleActions().testFlag(Qt::CopyAction)) {
        event->setDropAction(Qt::CopyAction);
        event->accept();
    } else {
        event->ignore();
    }
}

void MainWindow::dropEvent(QDropEvent* event)
{
    const auto path = droppedAssetPath(event->mimeData());
    if (path.isEmpty() || !event->possibleActions().testFlag(Qt::CopyAction)) {
        event->ignore();
        statusBar()->showMessage(tr("请一次拖入一个本地 STEP/STP 或 PLY 文件。"), 4000);
        return;
    }
    event->setDropAction(Qt::CopyAction);
    event->accept();
    showDocumentViewport();
    if (QFileInfo(path).suffix().compare("ply", Qt::CaseInsensitive) == 0)
        m_pointCloudWorkspace->importPointCloudFromFile(path);
    else
        m_modelViewController->loadModelFromFile(path);
}

void MainWindow::updateRotationStep()
{
    m_rotationStepLabel->setText(tr("旋转步长：%1°").arg(m_rotationStepDegrees, 0, 'f', 2));
}

bool MainWindow::runPerformanceCapture(const QString& modelPath,
                                      const QString& outputDirectory,
                                      const QString& texturePath,
                                      QString& errorMessage)
{
    showDocumentViewport();
    return m_modelViewController->runPerformanceCapture(
        modelPath, outputDirectory, texturePath, errorMessage);
}

bool MainWindow::runStructuredCapture(const QString& modelPath,const QString& outputDirectory,
    const capture::ScanOptions& options,bool reconstruct,QString& cloudPath,QString& error)
{
    showDocumentViewport();
    const QVariant previousBatch=property("paramcadBatchCapture");
    setProperty("paramcadBatchCapture",true);
    const auto restoreBatch=qScopeGuard([&]{setProperty("paramcadBatchCapture",previousBatch);});
    if(!m_modelViewController->prepareStructuredCaptureModel(modelPath,error))return false;
    m_propertyPanel->capturePanel()->setStructuredOptions(options);
    // Material scalars belong to the model page now, so a batch run has to put
    // its command-line material there as well; the scan reads it from the panel.
    auto material=m_propertyPanel->presentation();
    material.metallic=options.material.metallic;
    material.roughness=options.material.roughness;
    m_propertyPanel->setPresentation(material);
    m_viewer->setObjectMetallic(material.metallic);
    m_viewer->setObjectRoughness(material.roughness);
    QEventLoop loop;bool succeeded=false;
    const auto complete=connect(m_captureWorkspace,&CaptureWorkspaceController::structuredCaptureCompleted,&loop,
        [&](const QString&){succeeded=true;loop.quit();});
    const auto failed=connect(m_captureWorkspace,&CaptureWorkspaceController::structuredCaptureFailed,&loop,
        [&](const QString& message){error=message;loop.quit();});
    const auto cancelled=connect(m_captureWorkspace,&CaptureWorkspaceController::structuredCaptureCancelled,&loop,
        [&](const QString&){error=tr("结构光采集已取消");loop.quit();});
    if(!m_captureWorkspace->startStructuredScan(true,QDir(outputDirectory).filePath("capture"))) {
        disconnect(complete);disconnect(failed);disconnect(cancelled);error=tr("无法启动结构光采集，请检查输出目录和扫描参数");return false;
    }
    loop.exec();disconnect(complete);disconnect(failed);disconnect(cancelled);
    if(!succeeded || !reconstruct)return succeeded;
    auto* controller=findChild<ReconstructionController*>();
    succeeded=false;
    const auto reconstructed=connect(controller,&ReconstructionController::finished,&loop,
        [&](const QString& path,int,const QString&,bool){cloudPath=path;succeeded=true;loop.quit();});
    const auto reconstructionFailed=connect(controller,&ReconstructionController::failed,&loop,
        [&](const QString& message){error=message;loop.quit();});
    ReconstructRequest request;request.engine="structured_light";
    request.imageDirectory=QDir(outputDirectory).filePath("capture");request.outputRoot=QDir(outputDirectory).filePath("reconstructions");
    if(controller->start(request))loop.exec();else error=controller->lastError();
    disconnect(reconstructed);disconnect(reconstructionFailed);
    if(succeeded) {
        const auto readObject=[](const QString& path) {
            QFile file(path);if(!file.open(QIODevice::ReadOnly))return QJsonObject{};
            return QJsonDocument::fromJson(file.readAll()).object();
        };
        std::vector<std::array<double,3>> points;
        for(const auto& p:m_pointCloudDocument->sourceCloud().positions)points.push_back({p[0],p[1],p[2]});
        const auto evaluation=capture::evaluateSurface(m_viewer->captureGeometry(),points,
            readObject(outputDirectory+"/capture/capture.json"),readObject(outputDirectory+"/capture/calibration.json"));
        QString evaluationError;
        if(!capture::saveJson(outputDirectory+"/evaluation.json",evaluation,evaluationError))logWarning(evaluationError);
    }
    return succeeded;
}

void MainWindow::createWorkspaceControllers()
{
    auto* modeling = new ModelingController(this);
    m_modelingController=modeling;
    auto* capture = new CaptureController(m_viewer, this);
    auto* reconstruction = new ReconstructionController(this);

    m_sketchWorkspace = new SketchWorkspaceController(modeling,
        {m_viewer, m_toolPalette, m_modelingPanel, m_propertyPanel, m_sketchPanel,
         m_bottomTabs, m_console, m_newSketchAction, m_datumPlanesAction}, this, this);
    m_captureWorkspace = new CaptureWorkspaceController(capture,
        {m_viewer, m_propertyPanel, m_captureToolAction}, this, this);
    m_propertyPanel->reconstructPanel()->setCaptureBaseDirectory(capture->baseDirectory());
    m_pointCloudWorkspace = new PointCloudWorkspaceController(m_pointCloudDocument, modeling, reconstruction,
        {m_viewer, m_modelingPanel, m_relationsPanel, m_scenePanel,
          m_propertyPanel, m_analyzeCadSurfacesAction, m_reconstructCadAction, m_cancelCadAction, m_reconstructToolAction,
          m_projectionDebugAction}, this, this);
    m_planarJunctionPanel = new PlanarJunctionPanel(m_pointCloudDocument, m_viewer, m_relationsPanel);
    m_relationsPanel->setJunctionPanel(m_planarJunctionPanel);
    connect(m_planarJunctionPanel, &PlanarJunctionPanel::boundaryInspectionRequested,
            m_relationsPanel, &PointCloudRelationsPanel::boundaryInspectionRequested);
    connect(m_planarJunctionPanel, &PlanarJunctionPanel::viewportRequested, this, [this](VTKViewer* viewer) {
        showViewportTab(viewer, tr("面片交线与边界"));
    });
    connect(m_pointCloudWorkspace, &PointCloudWorkspaceController::contourSketchCreated,
            this, &MainWindow::showDocumentViewport);
    connect(m_pointCloudWorkspace, &PointCloudWorkspaceController::contourSketchCreated,
            m_sketchWorkspace, &SketchWorkspaceController::openSketch);
    connect(m_pointCloudWorkspace,&PointCloudWorkspaceController::cliOperationFinished,this,[this](QJsonObject result) {
        result["id"]=m_lastCliResult["id"];m_lastCliResult=result;
        auto callback=std::move(m_cliReply);m_cliReply={};if(callback)callback(result);
    });
    m_modelViewController = new ModelViewController(capture,
        {m_viewer, m_propertyPanel, m_scenePanel, m_exportStepAction}, this, this);
    m_featureEditing = new FeatureEditingController(modeling,
        {m_viewer, m_modelingPanel, m_propertyPanel->featureDetailsPanel(), m_undoCadAction, m_redoCadAction},
        this, this);
    connect(m_textEditAction, &QAction::triggered, this, [this] {
        m_propertyPanel->showFeatureDetails();
        if (auto* dock = qobject_cast<QDockWidget*>(m_propertyPanel->parentWidget())) { dock->show(); dock->raise(); }
        m_propertyPanel->featureDetailsPanel()->textEditor()->findChild<QLineEdit*>("textParameterInput")->setFocus();
    });
    connect(m_scenePanel, &ScenePanel::nodeDeleteRequested, this, [this, modeling](SceneNodeType type) {
        if (type != SceneNodeType::Model) return;
        m_pointCloudWorkspace->prepareProjectReplacement();
        m_sketchWorkspace->resetForProject();
        modeling->adoptProject(modeling::ModelingCore{});
        m_modelViewController->deleteModel();
    });

    m_projectController = new ProjectController(modeling, m_pointCloudDocument, m_modelViewController,
        reconstruction, {m_viewer, m_propertyPanel, m_scenePanel, m_openProjectAction, m_saveAction,
        m_saveAsAction, m_datumPlanesAction}, this, this);
    connect(m_projectController, &ProjectController::projectAboutToReplace,
            m_sketchWorkspace, &SketchWorkspaceController::resetForProject);
    connect(m_projectController, &ProjectController::projectAboutToReplace,
            m_pointCloudWorkspace, &PointCloudWorkspaceController::prepareProjectReplacement);
    connect(m_projectController, &ProjectController::projectAboutToReplace,
            this, &MainWindow::showDocumentViewport);
    connect(m_projectController, &ProjectController::projectPathChanged, this, [this](const QString& path) {
        setWindowTitle(tr("CAD 与点云工作台 — %1").arg(QFileInfo(path).fileName()));
    });
    connect(capture, &CaptureController::batchStateChanged, m_projectController, &ProjectController::setCaptureBusy);
    connect(capture, &CaptureController::batchStateChanged, this, [this](bool busy) {
        m_captureBusy = busy;
        if (busy) showDocumentViewport();
        m_viewportTabs->tabBar()->setEnabled(!busy);
    });
    connect(m_captureWorkspace,&CaptureWorkspaceController::captureBusyChanged,this,[this](bool busy){m_captureBusy=busy;});
    connect(m_captureWorkspace,&CaptureWorkspaceController::captureBusyChanged,m_projectController,&ProjectController::setCaptureBusy);

    WorkspaceController* workspaces[] = {
        m_sketchWorkspace, m_captureWorkspace, m_pointCloudWorkspace, m_modelViewController, m_projectController, m_featureEditing};
    for (auto* workspace : workspaces) {
        connect(workspace, &WorkspaceController::logInfo, this, &MainWindow::logInfo);
        connect(workspace, &WorkspaceController::logWarning, this, &MainWindow::logWarning);
        connect(workspace, &WorkspaceController::logError, this, &MainWindow::logError);
        connect(workspace, &WorkspaceController::statusTextChanged, m_statusLabel, &QLabel::setText);
        connect(workspace, &WorkspaceController::statusMessage, statusBar(), &QStatusBar::showMessage);
        connect(workspace, &WorkspaceController::sceneFramed,
                m_captureWorkspace, &CaptureWorkspaceController::syncCaptureDistance);
    }
}

void MainWindow::logInfo(const QString& message)
{
    m_console->appendPlainText(QStringLiteral("[INFO] %1").arg(message));
}

void MainWindow::logWarning(const QString& message)
{
    m_console->appendPlainText(QStringLiteral("[WARNING] %1").arg(message));
}

void MainWindow::logError(const QString& message)
{
    m_console->appendPlainText(QStringLiteral("[ERROR] %1").arg(message));
}

void MainWindow::createActions()
{
    m_undoCadAction = new QAction(tr("撤销 CAD 编辑"),this);
    m_undoCadAction->setObjectName("undoCadAction"); m_undoCadAction->setShortcut(QKeySequence::Undo);
    m_redoCadAction = new QAction(tr("重做 CAD 编辑"),this);
    m_redoCadAction->setObjectName("redoCadAction"); m_redoCadAction->setShortcut(QKeySequence::Redo);
    m_undoCadAction->setEnabled(false); m_redoCadAction->setEnabled(false);
    m_openAction = new QAction(style()->standardIcon(QStyle::SP_DialogOpenButton), tr("打开模型"), this);
    m_openAction->setShortcut(QKeySequence::Open);
    // A cloud produced elsewhere (or by an earlier reconstruction run) can be
    // inspected without going through the pipeline again.
    m_openPointCloudAction = new QAction(tr("打开点云…"), this);
    m_openPointCloudAction->setToolTip(tr("将 .ply 点云载入视口"));
    m_analyzeCadSurfacesAction = new QAction(tr("识别曲面与关系"), this);
    m_analyzeCadSurfacesAction->setObjectName(QStringLiteral("analyzeCadSurfacesAction"));
    m_analyzeCadSurfacesAction->setToolTip(tr("识别点云曲面并推断关系，供检查和编辑。"));
    m_analyzeCadSurfacesAction->setEnabled(false);
    m_reconstructCadAction = new QAction(tr("生成参数化 CAD"), this);
    m_reconstructCadAction->setObjectName(QStringLiteral("reconstructCadAction"));
    m_reconstructCadAction->setToolTip(
        tr("识别已载入点云中的几何并重建参数化 CAD 模型"));
    m_reconstructCadAction->setEnabled(false);
    m_openModelingTabAction = new QAction(tr("参数化建模"), this);
    m_openModelingTabAction->setObjectName(QStringLiteral("openModelingTabAction"));
    m_openModelingTabAction->setToolTip(tr("在新标签页中查看当前实体并调整特征参数"));
    m_openModelingTabAction->setEnabled(false);
    m_cadWithoutProjectionAction = new QAction(tr("重建 CAD（不使用投影回退）"), this);
    m_cadWithoutProjectionAction->setObjectName(QStringLiteral("cadWithoutProjectionAction"));
    m_cadWithoutProjectionAction->setEnabled(false);
    connect(m_reconstructCadAction,&QAction::changed,this,[this] {
        m_cadWithoutProjectionAction->setEnabled(m_reconstructCadAction->isEnabled());
    });
    m_projectionDebugAction = new QAction(tr("在导入的点云上拟合投影…"), this);
    m_projectionDebugAction->setObjectName(QStringLiteral("projectionDebugAction"));
    m_projectionDebugAction->setToolTip(tr("将原始导入点云作为单个部件拟合"));
    m_projectionDebugAction->setEnabled(false);
    m_cancelCadAction = new QAction(tr("取消 CAD 重建"), this);
    m_cancelCadAction->setObjectName(QStringLiteral("cancelCadReconstructionAction"));
    m_cancelCadAction->setEnabled(false);
    // Texturing is a presentation tool, available once a body exists.
    m_textureAction = new QAction(tr("应用纹理…"), this);
    m_textureAction->setToolTip(
        tr("将图像直接投影到模型上，不做展开"));
    // Capturing the current viewport is the data-generation step of the
    // pipeline, so it sits right next to Open Model in the toolbar.
    m_captureAction = new QAction(tr("拍摄图像"), this);
    m_captureAction->setToolTip(tr("将当前视口保存为 PNG"));
    // The screenshot command writes straight to outputs/screenshots so it can be
    // bound to a key without interrupting the user with a save dialog.
    m_screenshotAction = new QAction(tr("视图截图"), this);
    m_screenshotAction->setObjectName(QStringLiteral("screenshotAction"));
    m_screenshotAction->setToolTip(tr("将当前视口截图保存到 outputs/screenshots"));
    m_screenshotAction->setShortcut(shortcuts::screenshotSequence());
    // Starting a sketch is the entry point of manual modeling, and the plane it
    // will use is picked by clicking a datum plane or a planar face in the
    // viewport, so the action stays inert until such a selection exists.
    m_newSketchAction = new QAction(tr("新建草图"), this);
    m_newSketchAction->setObjectName(QStringLiteral("newSketchAction"));
    m_newSketchAction->setToolTip(
        tr("在选定的基准面或平面上创建草图"));
    m_newSketchAction->setEnabled(false);
    m_closeAction = new QAction(tr("关闭模型"), this);
    m_closeAction->setObjectName(QStringLiteral("closeModelAction"));
    m_saveAction = new QAction(style()->standardIcon(QStyle::SP_DialogSaveButton), tr("保存"), this);
    m_saveAction->setShortcut(QKeySequence::Save);
    m_saveAction->setObjectName(QStringLiteral("saveProjectAction"));
    m_openProjectAction = new QAction(tr("打开项目…"), this);
    m_openProjectAction->setObjectName(QStringLiteral("openProjectAction"));
    m_openProjectAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+O")));
    m_saveAsAction = new QAction(tr("项目另存为…"), this);
    m_saveAsAction->setObjectName(QStringLiteral("saveProjectAsAction"));
    m_saveAsAction->setShortcut(QKeySequence::SaveAs);
    m_exportStepAction = new QAction(tr("导出 STEP…"), this);
    m_exportStepAction->setObjectName(QStringLiteral("exportStepAction"));
    m_exportStepAction->setToolTip(tr("以毫米为单位导出当前 CAD 几何"));
    m_exportStepAction->setEnabled(false);
    m_exitAction = new QAction(tr("退出"), this);
    m_exitAction->setShortcut(QKeySequence::Quit);

    m_resetCameraAction = new QAction(style()->standardIcon(QStyle::SP_BrowserReload), tr("重置相机"), this);
    m_frontViewAction = new QAction(tr("前视图"), this);
    m_backViewAction = new QAction(tr("后视图"), this);
    m_leftViewAction = new QAction(tr("左视图"), this);
    m_rightViewAction = new QAction(tr("右视图"), this);
    m_topViewAction = new QAction(tr("俯视图"), this);
    m_bottomViewAction = new QAction(tr("底视图"), this);

    m_datumPlanesAction = new QAction(tr("基准面"), this);
    m_datumPlanesAction->setObjectName(QStringLiteral("datumPlanesAction"));
    m_datumPlanesAction->setCheckable(true);
    // The planes are drawn from the first frame of the 3D view, so the entry
    // starts checked and always reflects what the viewport shows.
    m_datumPlanesAction->setChecked(true);
    m_datumPlanesAction->setToolTip(tr("显示 XY、YZ 和 XZ 基准面"));

    m_captureToolAction = new QAction(tr("拍摄"), this);
    m_captureToolAction->setCheckable(true);
    m_captureToolAction->setToolTip(tr("在属性面板中显示拍摄页"));

    m_reconstructToolAction = new QAction(tr("重建点云"), this);
    m_reconstructToolAction->setCheckable(true);
    m_reconstructToolAction->setToolTip(tr("由多视角图像重建点云"));
    m_textEditAction = new QAction(tr("文字修改参数"), this);
    m_textEditAction->setObjectName("textParameterEditAction");
    m_textEditAction->setToolTip(tr("打开中文尺寸修改与预览"));
    m_textEditAction->setShortcut(QKeySequence("Ctrl+Shift+E"));
}

void MainWindow::createMenus()
{
    QMenu* fileMenu = menuBar()->addMenu(tr("文件(&F)"));
    fileMenu->addAction(m_openAction);
    fileMenu->addAction(m_openPointCloudAction);
    fileMenu->addAction(m_closeAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_openProjectAction);
    fileMenu->addAction(m_saveAction);
    fileMenu->addAction(m_saveAsAction);
    fileMenu->addAction(m_exportStepAction);
    fileMenu->addSeparator();
    fileMenu->addAction(m_exitAction);
    auto* editMenu=menuBar()->addMenu(tr("编辑(&E)"));
    editMenu->addAction(m_undoCadAction); editMenu->addAction(m_redoCadAction);
    editMenu->addSeparator(); editMenu->addAction(m_textEditAction);

    auto* settingsMenu = menuBar()->addMenu(tr("设置(&S)"));
    settingsMenu->setObjectName(QStringLiteral("settingsMenu"));
    // The language picker is checkable and mutually exclusive; switching only
    // takes effect after a restart because catalogs are installed at startup.
    auto* languageMenu = settingsMenu->addMenu(tr("语言"));
    languageMenu->setObjectName(QStringLiteral("languageMenu"));
    auto* languageGroup = new QActionGroup(languageMenu);
    languageGroup->setExclusive(true);
    const auto addLanguage = [this, languageMenu, languageGroup](const char* code, const QString& label) {
        auto* action = languageMenu->addAction(label);
        action->setObjectName(QStringLiteral("language_") + QString::fromLatin1(code));
        action->setCheckable(true);
        action->setChecked(i18n::currentLanguage() == QLatin1String(code));
        action->setData(QString::fromLatin1(code));
        languageGroup->addAction(action);
        connect(action, &QAction::triggered, this, [this, code] {
            if (i18n::currentLanguage() == QLatin1String(code)) return;
            i18n::storeLanguage(QString::fromLatin1(code));
            const auto answer = QMessageBox::question(
                this, tr("语言"),
                tr("语言已更改，重启 ParamCAD Studio 后生效。是否立即重启？"),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
            if (answer == QMessageBox::Yes) i18n::restartApplication();
        });
    };
    addLanguage("zh", tr("中文"));
    addLanguage("en", tr("English"));
    settingsMenu->addSeparator();
    auto* shortcutSettings = settingsMenu->addAction(tr("快捷键…"));
    shortcutSettings->setObjectName(QStringLiteral("shortcutSettingsAction"));
    connect(shortcutSettings, &QAction::triggered, this, &MainWindow::openShortcutSettings);
    settingsMenu->addSeparator();
    auto* hyperparameters = settingsMenu->addAction(tr("超参数"));
    hyperparameters->setObjectName(QStringLiteral("hyperparametersAction"));
    connect(hyperparameters,&QAction::triggered,this,[this] {
        showHyperparameterSettings(this,[this] {m_viewer->refreshPointCloudSurfaceSettings();});
    });
    auto* clearCache = settingsMenu->addAction(tr("清理点云缓存…"));
    clearCache->setObjectName(QStringLiteral("clearReconstructionCacheAction"));
    connect(clearCache, &QAction::triggered, this, &MainWindow::clearReconstructionCache);

    auto* debugMenu = menuBar()->addMenu(tr("调试(&D)"));
    debugMenu->setObjectName(QStringLiteral("debugMenu"));
    debugMenu->addAction(m_projectionDebugAction);

    QMenu* viewMenu = menuBar()->addMenu(tr("视图(&V)"));
    viewMenu->addAction(m_resetCameraAction);
    viewMenu->addSeparator();
    viewMenu->addActions({m_frontViewAction, m_backViewAction, m_leftViewAction,
                          m_rightViewAction, m_topViewAction, m_bottomViewAction});
    viewMenu->addSeparator();
    viewMenu->addAction(m_datumPlanesAction);

    QMenu* toolsMenu = menuBar()->addMenu(tr("工具(&T)"));
    toolsMenu->addAction(m_textureAction);
    toolsMenu->addSeparator();
    toolsMenu->addAction(m_screenshotAction);
    toolsMenu->addAction(m_captureToolAction);
    toolsMenu->addAction(m_reconstructToolAction);
    toolsMenu->addAction(m_analyzeCadSurfacesAction);
    toolsMenu->addAction(m_reconstructCadAction);
    toolsMenu->addAction(m_cadWithoutProjectionAction);
    toolsMenu->addAction(m_cancelCadAction);
    toolsMenu->addSeparator();
    toolsMenu->addAction(m_openModelingTabAction);

    QMenu* helpMenu = menuBar()->addMenu(tr("帮助(&H)"));
    QAction* aboutAction = helpMenu->addAction(tr("关于"));
    connect(aboutAction, &QAction::triggered, this, &MainWindow::showAbout);
}

void MainWindow::createToolBar()
{
    QToolBar* toolBar = addToolBar(tr("主工具栏"));
    toolBar->setObjectName(QStringLiteral("mainToolBar"));
    toolBar->setMovable(true);
    toolBar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    // The toolbar carries loading, image capture, cloud reconstruction and CAD reconstruction.
    toolBar->addAction(m_openAction);
    toolBar->addSeparator();
    // The toolbar button is created explicitly so it keeps a stable object name
    // for the UI automation checks, which a QAction alone does not provide.
    auto* newSketchButton = new QToolButton(toolBar);
    newSketchButton->setObjectName(QStringLiteral("newSketchButton"));
    newSketchButton->setDefaultAction(m_newSketchAction);
    newSketchButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    toolBar->addWidget(newSketchButton);
    toolBar->addSeparator();
    toolBar->addAction(m_captureAction);
    toolBar->addAction(m_reconstructToolAction);
    toolBar->addAction(m_analyzeCadSurfacesAction);
    auto* reconstructCadButton = new QToolButton(toolBar);
    reconstructCadButton->setObjectName(QStringLiteral("reconstructCadButton"));
    reconstructCadButton->setDefaultAction(m_reconstructCadAction);
    reconstructCadButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    toolBar->addWidget(reconstructCadButton);
    toolBar->addAction(m_openModelingTabAction);

    auto* textEditButton = new QToolButton(toolBar);
    textEditButton->setObjectName("textParameterEditButton");
    textEditButton->setDefaultAction(m_textEditAction);
    textEditButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    toolBar->addWidget(textEditButton);
}

void MainWindow::createCentralArea()
{
    m_pointCloudDocument = new PointCloudDocument(this);
    m_viewer = new VTKViewer(this, m_pointCloudDocument);
    m_toolPalette = new SketchToolPalette(this);

    // The tool strip sits against the left edge of the viewport rather than
    // floating over it: the VTK surface is a native window, so an overlay widget
    // would be painted behind it. It only carries the sketch tools, so it stays
    // hidden until a sketch is open - the model mode picks its plane on the
    // datum planes drawn in the viewport itself.
    m_toolPalette->setVisible(false);

    m_viewportTabs = new QTabWidget(this);
    m_viewportTabs->setObjectName(QStringLiteral("viewportTabs"));
    m_viewportTabs->setDocumentMode(true);
    m_viewportTabs->setTabsClosable(true);
    m_viewportTabs->setMovable(true);
    m_viewportTabs->tabBar()->setExpanding(false);
    m_documentViewport = new QWidget(m_viewportTabs);
    auto* layout = new QHBoxLayout(m_documentViewport);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(m_toolPalette);
    layout->addWidget(m_viewer, 1);
    const auto index = m_viewportTabs->addTab(m_documentViewport, tr("主视图"));
    for (const auto side : {QTabBar::LeftSide, QTabBar::RightSide}) {
        auto* button = m_viewportTabs->tabBar()->tabButton(index, side);
        m_viewportTabs->tabBar()->setTabButton(index, side, nullptr);
        if (button) button->deleteLater();
    }
    setCentralWidget(m_viewportTabs);
    connect(m_viewportTabs, &QTabWidget::currentChanged, this, &MainWindow::updateActiveViewport);
    connect(m_viewportTabs, &QTabWidget::tabCloseRequested, this, [this](int tab) {
        auto* page = m_viewportTabs->widget(tab);
        if (!page || page == m_documentViewport) return;
        if (auto* viewer = viewportOf(page)) viewer->setRenderingActive(false);
        m_viewportTabs->removeTab(tab);
        page->hide();
        if (!m_planarJunctionPanel || page != m_planarJunctionPanel->viewport()) page->deleteLater();
    });
}

VTKViewer* MainWindow::activeViewport() const
{
    auto* viewer = viewportOf(m_viewportTabs->currentWidget());
    return viewer ? viewer : m_viewer;
}

void MainWindow::showDocumentViewport()
{
    m_viewportTabs->setCurrentWidget(m_documentViewport);
}

void MainWindow::showViewportTab(QWidget* page, const QString& title)
{
    if (auto* viewer = viewportOf(page)) {
        connect(viewer, &VTKViewer::cameraChanged, this, &MainWindow::updateViewportCamera, Qt::UniqueConnection);
        connect(viewer, &VTKViewer::cameraPanned, this, &MainWindow::releaseViewportOriginLock, Qt::UniqueConnection);
    }
    if (m_viewportTabs->indexOf(page) < 0) m_viewportTabs->addTab(page, title);
    if (m_viewportTabs->tabBar()->isEnabled()) m_viewportTabs->setCurrentWidget(page);
}

void MainWindow::updateActiveViewport()
{
    auto* current = activeViewport();
    for (int i = 0; i < m_viewportTabs->count(); ++i) {
        if (auto* viewer = viewportOf(m_viewportTabs->widget(i)); viewer && viewer != current)
            viewer->setRenderingActive(false);
    }
    current->setRenderingActive(true);
    auto* cameraPanel = m_propertyPanel->cameraPanel();
    cameraPanel->releaseOriginLock();
    const auto parameters = current->cameraParameters();
    cameraPanel->setParameters(parameters);
    updateCameraStatus(parameters.parallelProjection);
    if (m_planarJunctionPanel && current == m_planarJunctionPanel->viewport()) {
        m_propertyPanel->showPointCloudProperties();
        m_relationsPanel->showJunctionPage();
    }
}

void MainWindow::updateViewportCamera(const CameraParameters& parameters)
{
    if (sender() != activeViewport()) return;
    m_propertyPanel->cameraPanel()->setParameters(parameters);
    updateCameraStatus(parameters.parallelProjection);
}

void MainWindow::releaseViewportOriginLock()
{
    if (sender() == activeViewport()) m_propertyPanel->cameraPanel()->releaseOriginLock();
}

void MainWindow::createDockPanels()
{
    m_scenePanel = new ScenePanel(this);
    auto* sceneDock = new QDockWidget(tr("场景"), this);
    sceneDock->setObjectName(QStringLiteral("sceneDock"));
    sceneDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    sceneDock->setWidget(m_scenePanel);
    sceneDock->setMinimumWidth(210);
    addDockWidget(Qt::LeftDockWidgetArea, sceneDock);

    m_propertyPanel = new PropertyPanel(this);
    auto* propertyDock = new QDockWidget(tr("属性"), this);
    propertyDock->setObjectName(QStringLiteral("propertyDock"));
    propertyDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    propertyDock->setWidget(m_propertyPanel);
    propertyDock->setMinimumWidth(400);
    addDockWidget(Qt::RightDockWidgetArea, propertyDock);
    connect(m_projectionDebugAction, &QAction::triggered, propertyDock, &QWidget::show);

    m_console = new QPlainTextEdit(this);
    m_console->setReadOnly(true);
    m_console->setMaximumBlockCount(2000);
    m_console->setObjectName(QStringLiteral("console"));

    m_sketchPanel = new SketchPanel(this);
    // Built detached from the tab widget, so it has to stay hidden until a sketch
    // is edited; a child that is never laid out would float over the tab bar.
    m_sketchPanel->hide();

    // The console and the sketch workspace share one panel as two pages, exactly
    // like Properties groups Camera and Render, so the tabs sit on top and no
    // title is repeated above them.
    m_bottomTabs = new QTabWidget(this);
    m_bottomTabs->addTab(m_console, tr("控制台 / 日志"));

    m_bottomDock = new QDockWidget(tr("输出"), this);
    m_bottomDock->setObjectName(QStringLiteral("outputDock"));
    m_bottomDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    m_bottomDock->setWidget(m_bottomTabs);
    m_bottomDock->setMinimumHeight(150);
    // An empty title bar collapses the dock's own header, so the tab row is the
    // only label above the panel and "Output" never appears.
    m_bottomDock->setTitleBarWidget(new QWidget(m_bottomDock));
    addDockWidget(Qt::BottomDockWidgetArea, m_bottomDock);

    resizeDocks({sceneDock, propertyDock}, {235, 380}, Qt::Horizontal);
    resizeDocks({m_bottomDock}, {170}, Qt::Vertical);

    m_modelingPanel = new ModelingPanel(this);
    m_relationsPanel = m_propertyPanel->pointCloudRelationsPanel();
    auto* modelingDock = new QDockWidget(tr("CAD 特征"), this);
    modelingDock->setObjectName(QStringLiteral("modelingDock"));
    modelingDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    modelingDock->setWidget(m_modelingPanel);
    modelingDock->setMinimumWidth(235);
    addDockWidget(Qt::LeftDockWidgetArea, modelingDock);
    splitDockWidget(sceneDock, modelingDock, Qt::Vertical);
}

void MainWindow::createStatusBar()
{
    m_statusLabel = new QLabel(tr("就绪 | 模型：无 | 相机：透视投影"), this);
    statusBar()->addWidget(m_statusLabel, 1);
    m_rotationStepLabel = new QLabel(this);
    m_rotationStepLabel->setObjectName("keyboardRotationStep");
    m_rotationStepLabel->setToolTip(tr("方向键旋转视角，按住连续旋转；Ctrl+↑/↓ 调整步长（0.1°–45°）。输入框内保持正常键盘操作。"));
    statusBar()->addPermanentWidget(m_rotationStepLabel);
    updateRotationStep();
}

void MainWindow::connectUi()
{
    m_featureEditing->connectUi();
    connect(m_openAction, &QAction::triggered, this, &MainWindow::showDocumentViewport);
    connect(m_openAction, &QAction::triggered, m_modelViewController, &ModelViewController::openModel);
    connect(m_openPointCloudAction, &QAction::triggered, this, &MainWindow::showDocumentViewport);
    connect(m_openPointCloudAction, &QAction::triggered, m_pointCloudWorkspace, &PointCloudWorkspaceController::openPointCloud);
    connect(m_analyzeCadSurfacesAction, &QAction::triggered, this, &MainWindow::showDocumentViewport);
    connect(m_analyzeCadSurfacesAction, &QAction::triggered, m_pointCloudWorkspace,
            &PointCloudWorkspaceController::reconstructCadSurfaces);
    connect(m_reconstructCadAction, &QAction::triggered, this, &MainWindow::showDocumentViewport);
    connect(m_reconstructCadAction, &QAction::triggered, m_pointCloudWorkspace, &PointCloudWorkspaceController::reconstructCad);
    connect(m_viewer, &VTKViewer::bodyShapeChanged, this, [this](const TopoDS_Shape& shape) {
        m_openModelingTabAction->setEnabled(!shape.IsNull() && shape.IsSame(m_modelingController->bodyShape()));
    });
    connect(m_openModelingTabAction, &QAction::triggered, this, [this] {
        auto* tab = new ParametricModelingTab(m_modelingController, m_viewer, m_viewportTabs);
        showViewportTab(tab, tr("参数化建模"));
    });
    connect(m_cadWithoutProjectionAction,&QAction::triggered,m_pointCloudWorkspace,
            &PointCloudWorkspaceController::reconstructCadWithoutProjection);
    connect(m_projectionDebugAction, &QAction::triggered, this, &MainWindow::showDocumentViewport);
    connect(m_projectionDebugAction, &QAction::triggered, m_pointCloudWorkspace,
            &PointCloudWorkspaceController::runProjectionDebug);
    connect(m_cancelCadAction, &QAction::triggered,
            m_pointCloudWorkspace, &PointCloudWorkspaceController::cancelCadReconstruction);
    connect(m_textureAction, &QAction::triggered, m_modelViewController, &ModelViewController::chooseTexture);
    connect(m_captureAction, &QAction::triggered, m_captureWorkspace, &CaptureWorkspaceController::captureImage);
    connect(m_screenshotAction, &QAction::triggered, this, [this] {
        m_captureWorkspace->captureViewport(activeViewport());
    });
    connect(m_captureWorkspace, &CaptureWorkspaceController::documentViewportRequested,
            this, &MainWindow::showDocumentViewport);
    connect(m_closeAction, &QAction::triggered, m_modelViewController, &ModelViewController::closeModel);
    connect(m_closeAction, &QAction::triggered,
            m_pointCloudWorkspace, &PointCloudWorkspaceController::cancelCadReconstruction);
    connect(m_exportStepAction, &QAction::triggered, m_modelViewController, &ModelViewController::exportModel);
    connect(m_exitAction, &QAction::triggered, qApp, &QApplication::closeAllWindows);

    connect(m_resetCameraAction, &QAction::triggered, this, [this]() {
        auto* viewer = activeViewport();
        viewer->resetCamera();
        // Reset Camera is the framing the user has just chosen, so the orbit
        // distance follows it instead of keeping whatever it held before.
        if (viewer == m_viewer) m_captureWorkspace->syncCaptureDistance();
        logInfo(tr("Camera reset"));
    });
    connect(m_frontViewAction, &QAction::triggered, this, [this]() {
        activeViewport()->setStandardView(CameraController::StandardView::Front);
        logInfo(tr("Front view selected"));
    });
    connect(m_backViewAction, &QAction::triggered, this, [this]() {
        activeViewport()->setStandardView(CameraController::StandardView::Back);
        logInfo(tr("Back view selected"));
    });
    connect(m_leftViewAction, &QAction::triggered, this, [this]() {
        activeViewport()->setStandardView(CameraController::StandardView::Left);
        logInfo(tr("Left view selected"));
    });
    connect(m_rightViewAction, &QAction::triggered, this, [this]() {
        activeViewport()->setStandardView(CameraController::StandardView::Right);
        logInfo(tr("Right view selected"));
    });
    connect(m_topViewAction, &QAction::triggered, this, [this]() {
        activeViewport()->setStandardView(CameraController::StandardView::Top);
        logInfo(tr("Top view selected"));
    });
    connect(m_bottomViewAction, &QAction::triggered, this, [this]() {
        activeViewport()->setStandardView(CameraController::StandardView::Bottom);
        logInfo(tr("Bottom view selected"));
    });
    connect(m_datumPlanesAction, &QAction::toggled, this, [this](bool visible) {
        m_viewer->setDatumPlanesVisible(visible);
        m_viewer->renderNow();
        logInfo(visible ? tr("Datum planes shown") : tr("Datum planes hidden"));
    });
    connect(m_newSketchAction, &QAction::triggered, this, &MainWindow::showDocumentViewport);
    connect(m_modelingPanel, &ModelingPanel::editSketchRequested, this, &MainWindow::showDocumentViewport);
    m_sketchWorkspace->connectUi();

    connect(m_scenePanel, &ScenePanel::nodeSelected, this, [this](SceneNodeType type) {
        showDocumentViewport();
        if (type == SceneNodeType::Model) {
            m_propertyPanel->showObjectProperties();
        } else if (type == SceneNodeType::Camera) {
            m_propertyPanel->showCameraProperties();
        } else if (type == SceneNodeType::PointCloud) {
            m_propertyPanel->showPointCloudProperties();
        }
    });

    connect(m_relationsPanel, &PointCloudRelationsPanel::boundaryInspectionRequested,
            this, &MainWindow::showDocumentViewport);
    m_pointCloudWorkspace->connectUi();

    connect(m_propertyPanel, &PropertyPanel::objectTransformChanged,
            m_viewer, &VTKViewer::setObjectTransform);
    connect(m_propertyPanel, &PropertyPanel::objectVisibilityChanged,
            m_viewer, &VTKViewer::setObjectVisible);
    connect(m_propertyPanel, &PropertyPanel::objectRepresentationChanged,
            m_viewer, &VTKViewer::setObjectRepresentation);
    connect(m_propertyPanel, &PropertyPanel::objectOpacityChanged,
            m_viewer, &VTKViewer::setObjectOpacity);
    connect(m_propertyPanel, &PropertyPanel::objectColorChanged,
            m_viewer, &VTKViewer::setObjectColor);
    connect(m_propertyPanel, &PropertyPanel::objectMetallicChanged,
            m_viewer, &VTKViewer::setObjectMetallic);
    connect(m_propertyPanel, &PropertyPanel::objectRoughnessChanged,
            m_viewer, &VTKViewer::setObjectRoughness);
    connect(m_propertyPanel, &PropertyPanel::objectMaterialPreviewChanged,
            m_viewer, &VTKViewer::setObjectMaterialPreview);

    CameraPanel* cameraPanel = m_propertyPanel->cameraPanel();
    connect(cameraPanel, &CameraPanel::applyRequested, this, [this](const CameraParameters& parameters) {
        activeViewport()->applyCamera(parameters);
    });
    connect(cameraPanel, &CameraPanel::resetRequested, m_resetCameraAction, &QAction::trigger);
    connect(m_viewer, &VTKViewer::cameraChanged, this, &MainWindow::updateViewportCamera);
    // A pan has to be allowed to leave the origin behind, so the panel drops its
    // "look at origin" pin rather than pulling the camera back onto it.
    connect(m_viewer, &VTKViewer::cameraPanned, this, &MainWindow::releaseViewportOriginLock);

    RenderPanel* renderPanel = m_propertyPanel->renderPanel();
    connect(renderPanel, &RenderPanel::backgroundColorChanged,
            m_viewer, &VTKViewer::setBackgroundColor);
    connect(renderPanel, &RenderPanel::lightingChanged,
            m_viewer, &VTKViewer::setLightingEnabled);
    connect(renderPanel, &RenderPanel::captureRequested, m_captureWorkspace, &CaptureWorkspaceController::captureImage);
    m_modelViewController->connectUi();

    m_captureWorkspace->connectUi();
    connect(m_propertyPanel->capturePanel(), &CapturePanel::baseDirectoryChanged,
            m_propertyPanel->reconstructPanel(), &ReconstructPanel::setCaptureBaseDirectory);
}

void MainWindow::openShortcutSettings()
{
    QList<QKeySequence> reserved;
    for (const QAction* action : findChildren<QAction*>()) {
        if (action == m_screenshotAction) continue;
        for (const auto& sequence : action->shortcuts()) {
            if (!sequence.isEmpty() && !reserved.contains(sequence)) reserved.append(sequence);
        }
    }
    // The screenshot shortcut must not fire while the editor is recording it.
    const bool wasEnabled = m_screenshotAction->isEnabled();
    m_screenshotAction->setEnabled(false);
    const bool accepted = shortcuts::showShortcutSettings(this, reserved);
    m_screenshotAction->setEnabled(wasEnabled);
    if (accepted) m_screenshotAction->setShortcut(shortcuts::screenshotSequence());
}

void MainWindow::clearReconstructionCache()
{
    const QString title = tr("清理点云缓存");
    const auto answer = QMessageBox::question(this, title,
        tr("要删除点云识别和 CAD 重建的磁盘缓存吗？下次需要时会重新计算。"),
        QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) return;

    if (m_pointCloudWorkspace->reconstructionCacheInUse()) {
        QMessageBox::warning(this, title, tr("正在识别或重建点云，请等待任务结束后重试。"));
        return;
    }
    if (!ReconstructionCache::clearDiskEntries()) {
        QMessageBox::warning(this, title, tr("未能清理全部点云缓存。请检查缓存目录权限或关闭正在使用缓存的进程，然后重试。"));
        return;
    }
    statusBar()->showMessage(tr("点云缓存已清理。"), 6000);
}

void MainWindow::showAbout()
{
    using namespace pcad::build;
    const auto state = QLatin1String(sourceState);
    const QString source = state == "dirty" ? tr("有未提交改动")
        : state == "clean" ? tr("与提交一致") : tr("未知");
    const QString details = tr("CAD 与点云工作台\n\n"
                               "版本 %1\n构建配置 %2\n构建时间（UTC） %3\n"
                               "Git 提交 %4\n工作区 %5\n改动指纹 %6\n程序路径 %7")
        .arg(QApplication::applicationVersion(), QString::fromLatin1(configuration),
             QString::fromLatin1(builtAtUtc), *gitCommit ? QString::fromLatin1(gitCommit) : tr("未知"),
             source, *worktreeFingerprint ? QString::fromLatin1(worktreeFingerprint).left(12) : QStringLiteral("—"),
             QDir::toNativeSeparators(QApplication::applicationFilePath()));
    QMessageBox dialog(QMessageBox::Information, tr("关于"), details, QMessageBox::Ok, this);
    dialog.setTextFormat(Qt::PlainText);
    dialog.setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
    dialog.exec();
}

void MainWindow::updateCameraStatus(bool parallelProjection)
{
    const QString current = m_statusLabel->text();
    const int cameraSeparator = current.lastIndexOf(QStringLiteral(" | Camera:"));
    const QString prefix = cameraSeparator >= 0 ? current.left(cameraSeparator) : tr("就绪 | 模型：无");
    m_statusLabel->setText(prefix + QStringLiteral(" | Camera: ")
                           + (parallelProjection ? tr("正交投影") : tr("透视投影")));
}

void MainWindow::applyDarkTheme()
{
    qApp->setStyle(QStringLiteral("Fusion"));
    qApp->setStyleSheet(QStringLiteral(R"(
        QMainWindow, QWidget { background: #25272b; color: #d7d9dc; }
        QMenuBar { background: #202226; border-bottom: 1px solid #3b3e44; }
        QMenuBar::item:selected, QMenu::item:selected { background: #3b5878; }
        QMenu { background: #292c31; border: 1px solid #454951; }
        QToolBar { background: #2b2e33; border-bottom: 1px solid #454951; spacing: 3px; }
        QToolButton, QPushButton { background: #34373d; border: 1px solid #50545c; padding: 5px 8px; }
        QToolButton:hover, QPushButton:hover { background: #41454c; }
        QToolButton:pressed, QPushButton:pressed { background: #2d638f; }
        QToolButton:checked { background: #315f86; border-color: #4f91c7; }
        QToolButton:disabled { color: #777b82; background: #292b2f; }
        QDockWidget { color: #e2e3e5; }
        QDockWidget::title { background: #303339; padding: 6px; border-bottom: 1px solid #474b52; }
        QTreeWidget, QPlainTextEdit, QTabWidget::pane { background: #202226; border: 1px solid #41444a; }
        QTreeWidget::item:selected { background: #315f86; }
        QTabBar::tab { background: #303339; border: 1px solid #44484f; padding: 6px 10px; }
        QTabBar::tab:selected { background: #3a3e45; border-bottom-color: #4f91c7; }
        QGroupBox { border: 1px solid #44484f; margin-top: 10px; padding-top: 8px; }
        QGroupBox::title { subcontrol-origin: margin; left: 7px; padding: 0 4px; }
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
            background: #1f2125; border: 1px solid #494d54; padding: 3px; selection-background-color: #315f86;
        }
        QStatusBar { background: #202226; border-top: 1px solid #41444a; }
        #console { color: #b8c7d9; font-family: Consolas, monospace; }
        #vtkViewport { border: 0; }
    )"));
}
