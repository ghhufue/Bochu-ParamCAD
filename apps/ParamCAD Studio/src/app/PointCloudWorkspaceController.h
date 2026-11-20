#pragma once

#include "app/WorkspaceController.h"
#include <modeling/ModelPatch.h>
#include <reconstruction/data/PointStore.h>
#include <reconstruction/surface/SurfaceAnalysis.h>
#include <QPointer>
#include <QStringList>
#include <QJsonObject>
#include <memory>

class QAction;
class QLabel;
class QPushButton;
class QComboBox;
class CadReconstructionTask;
struct CadReconstructionResult;
class ModelingController;
class ModelingPanel;
class PointCloudDocument;
class PointCloudRelationsPanel;
class PropertyPanel;
class ReconstructionController;
struct ReconstructRequest;
class ScenePanel;
class QThread;
class VTKViewer;
struct PointCloudContourInput;
namespace reconstruction { enum class ProjectionPath; }

struct PointCloudWorkspaceUi
{
    VTKViewer* viewer;
    ModelingPanel* modelingPanel;
    PointCloudRelationsPanel* relationsPanel;
    ScenePanel* scenePanel;
    PropertyPanel* propertyPanel;
    QAction* reconstructCadAction;
    QAction* cancelCadAction;
    QAction* reconstructToolAction;
    QAction* projectionDebugAction;
};

// Owns cloud editing/analysis and the UI side of photo and CAD reconstruction.
class PointCloudWorkspaceController final : public WorkspaceController
{
    Q_OBJECT
public:
    PointCloudWorkspaceController(PointCloudDocument* document,
                                  ModelingController* modelingController,
                                  ReconstructionController* reconstructionController,
                                  const PointCloudWorkspaceUi& ui,
                                  QWidget* dialogParent, QObject* parent = nullptr);
    ~PointCloudWorkspaceController() override;
    void connectUi();
    void openPointCloud();
    void importPointCloudFromFile(const QString& filePath);
    void loadPointCloudFromFile(const QString& filePath);
    void reconstructCad();
    void reconstructCadWithoutProjection();
    void runProjectionDebug();
    void cancelCadReconstruction();
    void prepareProjectReplacement();
    bool commandBusy() const;
    bool dispatchCliOperation(const QString& command,const QStringList& arguments,QString& error);
signals:
    // Emitted only after the ordinary UI result handlers apply/reject the result.
    void cliOperationFinished(const QJsonObject& result);
    void contourSketchCreated(modeling::FeatureId id);

private:
    void startCadReconstruction(bool projectionFallback);
    void startProjectionDebug(reconstruction::ProjectionPath path, bool inspectOnly);
    void clearProjectionDebugResult();
    std::optional<reconstruction::LengthUnit> confirmCloudUnit();
    void showProjectionDebugResult(const CadReconstructionResult& result);
    void updateCadActions();
    void inspectCloudBoundary(int regionId);
    void invalidateContourPreview();
    void closeContourInspection();
    void startContourFit(double tolerance, int maximumEdges, bool recognizeCircles);
    void createContourSketch();
    void frameImportedCloud(const QString& filePath);
    void reportCloudLoadError(const QString& error);
    void onCadReconstructionFinished(const CadReconstructionResult& result);
    void startPointCloudAnalysis(int regionId = 0, bool recordUndo = true);
    void startRelationInference(bool recordUndo = true);
    void startAnalysisExport();
    void startAnalysisExportTo(const QString& path);
    void finishCliOperation(bool success,const QString& error={},QJsonObject details={});
    void startReconstruction(const ReconstructRequest& request);
    void onReconstructionFinished(const QString& pointCloudPath, int pointCount,
                                  const QString& cameraMode, bool dense);
    void refreshDownsampleControls();
    void applyDownsample(double voxelSizeMm);
    void finishDownsample(const QString& root, const QString& target, double voxelSizeMm);
    bool switchActiveCloud(const QString& path, QString& error);

    PointCloudDocument* const m_document;
    ModelingController* const m_modelingController;
    ReconstructionController* const m_reconstructionController;
    const PointCloudWorkspaceUi m_ui;
    QThread* m_analysisThread = nullptr;
    QThread* m_downsampleThread = nullptr;
    QThread* m_exportThread = nullptr;
    QThread* m_importThread = nullptr;
    QThread* m_contourFitThread = nullptr;
    struct ContourPreview;
    std::shared_ptr<const PointCloudContourInput> m_contourInput;
    std::shared_ptr<const ContourPreview> m_contourPreview;
    std::optional<double> m_contourUnitScale;
    quint64 m_contourFitRevision = 0;
    QString m_pendingImport;
    quint64 m_importGeneration = 0;
    CadReconstructionTask* m_cadTask;
    QPointer<QWidget> m_projectionPage;
    QPointer<QWidget> m_projectionReport;
    QPointer<QLabel> m_projectionStatus;
    QPointer<QPushButton> m_runProjection;
    QPointer<QPushButton> m_inspectProjection;
    QPointer<QComboBox> m_projectionMode;
    bool m_startingCad = false;
    bool m_cadOverwriteManual = false;
    bool m_analyseAfterCad = false;
    bool m_inferAfterCad = false;
    quint64 m_cadCloudRevision = 0;
    quint64 m_cadBodyRevision = 0;
    modeling::ModelRevision m_cadModelRevision = 0;
    QString m_cadSourcePath;
    double m_lastDownsampleSpacing = 10.0;
    std::optional<reconstruction::SurfaceAnalysisResult> m_surfaceAnalysis;
    QJsonObject m_surfaceSupportReport;
    QByteArray m_surfaceSupportReportFingerprint;
    QString m_cliOperation;
};
