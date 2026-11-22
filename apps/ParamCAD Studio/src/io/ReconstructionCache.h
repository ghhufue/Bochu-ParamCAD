#pragma once

#include <reconstruction/parts/AssemblyReconstruction.h>
#include <reconstruction/pipeline/GuidedSolidRecognizer.h>
#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/surface/SurfaceAnalysis.h>

#include <QByteArray>
#include <QJsonObject>
#include <QString>

namespace reconstruction {
struct GuidedSolidResult;
struct RecognitionSettings;
struct TemplateHint;
}

// Studio and pcad use the same store. Numerical modules never depend on it.
class ReconstructionCache {
public:
    using Stop = std::function<bool()>;

    explicit ReconstructionCache(Stop cancelled = {});
    static bool clearDiskEntries();
    bool enabled() const { return !m_root.isEmpty(); }
    const QJsonObject& report() const { return m_report; }

    bool preprocess(const reconstruction::ObservationSnapshot& source,
        const reconstruction::PointCloudPreprocessingOptions& options,
        reconstruction::PointStore& working, reconstruction::PointCloudPreprocessingReport& report,
        std::string& error);
    bool analyze(const reconstruction::PointStore& working, const reconstruction::SurfaceAnalysisOptions& options,
        reconstruction::SurfaceAnalysisResult& analysis, std::string& error,
        const reconstruction::SurfaceAnalysisProgress& progress = {});
    bool detect(const reconstruction::PointStore& working, const reconstruction::PrimitiveDetectionOptions& options,
        reconstruction::PrimitiveDetectionResult& evidence, std::string& error);
    bool analyzeSource(const reconstruction::PointStore& working, const reconstruction::PointStore& source,
        const reconstruction::SurfaceAnalysisOptions& options, reconstruction::SurfaceAnalysisResult& result,
        std::string& error, bool& reused, const reconstruction::SurfaceAnalysisProgress& progress = {});
    bool recognize(const reconstruction::PointStore& working, const reconstruction::PointStore& source,
        const reconstruction::PlaneDetectionOptions& planes, const reconstruction::CylinderDetectionOptions& cylinders,
        const std::vector<reconstruction::TemplateHint>& hints,
        const std::vector<reconstruction::SurfaceRelationHint>& relations,
        const reconstruction::RecognitionSettings& settings,
        const reconstruction::SurfaceAnalysisResult* sharedAnalysis, reconstruction::GuidedSolidResult& result,
        std::string& error, const reconstruction::GuidedRecognitionProgress& progress = {});
    bool assemble(const reconstruction::PointStore& source, const reconstruction::AssemblyOptions& options,
        reconstruction::AssemblyReconstructionResult& result, std::string& error);

    QByteArray previewKey(const reconstruction::PointStore& source,
        const reconstruction::SurfaceAnalysisResult& analysis, double radius, bool useNormals);
    bool readPreview(const QByteArray& key, reconstruction::SurfaceAnalysisResult& preview);
    void writePreview(const QByteArray& key, const reconstruction::SurfaceAnalysisResult& preview);

    // The document owns publication and resolves region IDs before it applies a snapshot.

private:
    QByteArray read(const QString& stage, const QByteArray& key);
    void write(const QString& stage, const QByteArray& key, const QByteArray& payload);
    template<class... T> bool load(const QString& stage, const QByteArray& key, T&... output);
    void note(const QString& stage, const char* status, const char* reason = nullptr);
    bool cancelled() const { return m_cancelled && m_cancelled(); }

    QString m_root;
    Stop m_cancelled;
    QJsonObject m_report;
};
