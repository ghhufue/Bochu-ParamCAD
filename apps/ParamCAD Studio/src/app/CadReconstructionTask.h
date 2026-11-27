#pragma once

#include "io/PlyReader.h"

#include <reconstruction/recognition/CompoundRecognition.h>
#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/pipeline/GuidedSolidRecognizer.h>
#include <reconstruction/parts/AssemblyReconstruction.h>
#include <reconstruction/brep_closure/ClosedBrepReconstructor.h>

#include <QObject>
#include <QJsonObject>
#include <TopoDS_Shape.hxx>

#include <atomic>
#include <memory>

class QThread;
struct PointCloudSurfaceMesh;

struct CadReconstructionPreview {
    bool resetSurfaces = false;
    std::vector<std::shared_ptr<const PointCloudSurfaceMesh>> surfaces;
    std::optional<TopoDS_Shape> solid;
};

struct CadProjectionDebugResult
{
    reconstruction::ProjectionExtrusionResult fit;
    reconstruction::ProjectionPath path = reconstruction::ProjectionPath::Automatic;
    bool inspectOnly = false;
    TopoDS_Shape previewShape;
    reconstruction::LengthUnit sourceUnit = reconstruction::LengthUnit::Arbitrary;
    std::size_t inputPoints = 0;
    std::size_t validPoints = 0;
    qint64 elapsedMilliseconds = 0;
};

struct CadReconstructionResult
{
    enum class Outcome { Success, PreprocessingFailed, Unsupported, Failed, Cancelled };
    enum class Branch { None, Conventional, Guided, Assembly, ClosedBrep };
    Outcome outcome = Outcome::Failed;
    Branch branch = Branch::None;
    bool partialAssembly = false;
    bool preprocessingComplete = false;
    reconstruction::PointCloudPreprocessingReport preprocessing;
    reconstruction::CompoundCandidate candidate;
    reconstruction::SurfaceAnalysisResult analysis;
    std::optional<reconstruction::SurfaceAnalysisResult> sourceSurfacePreview;
    QString sourceSurfacePreviewError;
    bool analysisReused = false;
    QJsonObject cache;
    std::vector<reconstruction::TemplateFit> templateFits;
    int failedRegionId = 0;
    QString error;
    std::optional<reconstruction::AssemblyReconstructionResult> assembly;
    std::optional<reconstruction::ClosedBrepResult> closedBrep;
    std::optional<CadProjectionDebugResult> projectionDebug;
    std::size_t skippedInvalidObservations = 0;
};

QJsonObject closedBrepReport(const reconstruction::ClosedBrepResult& result);

// Computes a candidate from an owned input snapshot. No view or modeling document
// is accessed by the worker; the workspace decides whether to commit the result.
class CadReconstructionTask final : public QObject
{
    Q_OBJECT
public:
    enum class Stage { Preprocessing, FittingTemplates, Recognizing, SeparatingParts, Projecting, ClosingBrep, Cancelling };
    Q_ENUM(Stage)

    explicit CadReconstructionTask(QObject* parent = nullptr);
    ~CadReconstructionTask() override;
    using PreviewMesher = std::function<std::shared_ptr<const PointCloudSurfaceMesh>(const PlyCloud&,
        double, const reconstruction::PrimitiveEvidence&, const std::vector<reconstruction::PointId>&,
        const reconstruction::SurfaceFootprint*)>;

    bool start(PlyCloud cloud, reconstruction::LengthUnit unit,
               std::vector<reconstruction::TemplateHint> hints = {}, double sourceTolerance = 0,
               std::optional<reconstruction::SurfaceAnalysisResult> analysis = {},
               std::vector<reconstruction::SurfaceRelationHint> relations = {},
               bool projectionFallback = true, bool specializedRecovery = true,
               std::optional<reconstruction::ProjectionExtrusionOptions> projectionOptions = {},
               bool prepareSurfacePreview = true, bool legacyAssembly = false, PreviewMesher previewMesher = {},
               std::optional<reconstruction::BrepClosureOptions> brepOptions = {});
    bool startProjection(PlyCloud cloud, reconstruction::LengthUnit unit,
                         reconstruction::ProjectionPath path = reconstruction::ProjectionPath::Automatic,
                         bool inspectOnly = false,
                         std::optional<reconstruction::ProjectionExtrusionOptions> projectionOptions = {});
    bool isRunning() const noexcept;
    bool isCancellationRequested() const noexcept;
    void cancel();

signals:
    void runningChanged(bool running);
    void stageChanged(CadReconstructionTask::Stage stage);
    void previewChanged(const CadReconstructionPreview& preview);
    // Delivered on this object's owning thread, while isRunning() is still true.
    void finished(const CadReconstructionResult& result);

private:
    using ReportPreview = std::function<void(CadReconstructionPreview)>;
    using Work = std::function<void(CadReconstructionResult&, const std::function<bool()>&,
                                    const std::function<void(Stage)>&, const ReportPreview&)>;
    bool startWork(Work work, Stage initialStage, bool livePreview = false);

    QThread* m_thread = nullptr;
    bool m_running = false;
    std::shared_ptr<std::atomic_bool> m_cancelled;
};
