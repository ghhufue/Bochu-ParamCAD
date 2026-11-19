#include "io/ReconstructionCache.h"
#include "io/ReconstructionCacheCodec.h"

#include "ParamCADBuildInfo.h"
#include "ReconstructionCacheVersions.h"

#include <reconstruction/data/StageProfile.h>
#include <reconstruction/parts/ProjectionSettings.h>
#include <reconstruction/recognition/RecognitionAnalysisOptions.h>
#include <reconstruction/settings/ModuleConfig.h>
#include <reconstruction/surface/SurfaceSupportValidation.h>

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QSaveFile>
#include <QStandardPaths>

#include <algorithm>
#include <unordered_set>

namespace {
using namespace reconstruction;
using namespace reconstruction_cache;
using diagnostics::profileStage;

constexpr qint64 maxPayloadBytes = 512 * 1024 * 1024;
constexpr int formatVersion = 1;

QByteArray digest(const QByteArray& bytes)
{
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex();
}

template<class... T> QByteArray key(const char* algorithm, const T&... inputs)
{
    return digest(encode(formatVersion, std::string(algorithm), inputs...));
}

QByteArray pointsKey(const PointStore& points)
{
    return digest(encode(points.unit(), points.points()));
}

QByteArray evidenceKey(const PointStore& working, const SurfaceAnalysisOptions& options)
{
    return key(versions::evidence, pointsKey(working), options.detection, options.independentFamilies,
        options.recoverResidualSurfaces, options.expandPlaneSupport, options.familyDetectionWorkerCount);
}

QByteArray analysisKey(const QByteArray& evidence, const SurfaceAnalysisOptions& options)
{
    return key(versions::analysis, evidence, options);
}

bool reusableAnalysis(const PointStore& working, const PointStore& source,
    const SurfaceAnalysisOptions& options, const SurfaceAnalysisResult& analysis)
{
    const auto fingerprint = pointStoreFingerprint(source);
    return analysisInputMatches(working, analysis) && analysisSettingsMatch(options, analysis.options) &&
        (!analysis.baseFingerprint || analysis.recoverySourceFingerprint == fingerprint) &&
        (!analysis.sourceSupportValidation.pointFingerprint ||
            analysis.sourceSupportValidation.pointFingerprint == fingerprint);
}

bool validAnalysisReferences(const PointStore& working, const SurfaceAnalysisResult& analysis)
{
    std::unordered_set<PointId> pointIds;
    pointIds.reserve(working.size());
    for (const auto& point : working.points()) pointIds.insert(point.id);
    const auto knownPoints = [&](const auto& ids) {
        return std::all_of(ids.begin(), ids.end(), [&](auto id) { return pointIds.count(id) != 0; });
    };
    std::unordered_set<SurfaceId> surfaceIds;
    for (const auto& evidence : analysis.surfaces) {
        if (!std::visit([&](const auto& surface) {
            return surface.id != kInvalidSurfaceId && surfaceIds.insert(surface.id).second &&
                knownPoints(surface.supportPointIds);
        }, evidence)) return false;
    }
    std::unordered_set<PatchId> patchIds;
    for (const auto& patch : analysis.patches)
        if (!patch.id || !patchIds.insert(patch.id).second || !surfaceIds.count(patch.surfaceId) ||
            !knownPoints(patch.supportPointIds)) return false;
    return knownPoints(analysis.unassignedPointIds) &&
        std::all_of(analysis.relations.begin(), analysis.relations.end(), [&](const auto& relation) {
            return relation.first != relation.second && patchIds.count(relation.first) &&
                patchIds.count(relation.second);
        });
}

QByteArray recognitionInputKey(const SurfaceAnalysisResult& analysis)
{
    // Local failures are diagnostics from the previous invocation. Repairs,
    // equations and constraints are inputs to the next one.
    return digest(encode(analysis.options, analysis.surfaces, analysis.supportProposals,
        analysis.supportProposalIds, analysis.patches, analysis.relations,
        analysis.supportValidation, analysis.sourceSupportValidation, analysis.sourceValidatedSurfaces,
        analysis.unassignedPointIds, analysis.baseFingerprint, analysis.recoverySourceFingerprint,
        analysis.supplementalPoints, analysis.repairMappings, analysis.repairPatchMappings));
}

QByteArray assemblyGroupingKey(const QByteArray& source, const AssemblyOptions& o)
{
    return key(versions::assemblyGrouping, source, o.distanceTolerance, o.maximumWorkingPoints,
        o.neighbors, o.minimumPatchPoints, o.normalAngleRadians, o.connectivityFactor,
        o.patchSpacingToleranceMultiplier, o.connectionToleranceMultiplier, o.minimumConvexLinkFraction,
        o.minimumLinkCount);
}

QByteArray assemblyClosureKey(const QByteArray& preparation, AssemblyOptions options)
{
    // Projection parameters first enter after this intermediate result.
    options.projectionOptions.reset();
    options.maximumProjectionParts = 0;
    options.minimumProjectionPartPoints = 0;
    return key(versions::assemblyClosure, preparation, options);
}
}

ReconstructionCache::ReconstructionCache(Stop cancelled) : m_cancelled(std::move(cancelled))
{
    if (qEnvironmentVariableIntValue("PCAD_DISABLE_CACHE") != 0) return;
    const auto overrideRoot = qEnvironmentVariable("PCAD_CACHE_DIR");
    const auto root = overrideRoot.isEmpty()
        ? QStandardPaths::writableLocation(QStandardPaths::CacheLocation) : overrideRoot;
    if (!root.isEmpty()) m_root = QDir(root).absoluteFilePath("reconstruction/v1");
}

void ReconstructionCache::note(const QString& stage, const char* status, const char* reason)
{
    QJsonObject entry{{"status", QString::fromLatin1(status)}};
    if (reason) entry["reason"] = QString::fromLatin1(reason);
    m_report[stage] = entry;
}

QByteArray ReconstructionCache::read(const QString& stage, const QByteArray& cacheKey)
{
    if (!enabled() || cancelled() || cacheKey.isEmpty()) {
        note(stage, "bypass", cancelled() ? "cancelled" : "disabled");
        return {};
    }
    QFile file(QDir(m_root).filePath(stage + "/" + QString::fromLatin1(cacheKey) + ".bin"));
    if (!file.open(QIODevice::ReadOnly)) {
        note(stage, "miss", file.exists() ? "unreadable" : "no_matching_entry");
        return {};
    }
    const auto header = QJsonDocument::fromJson(file.readLine(2048)).object();
    const auto size = header["bytes"].toInteger(-1);
    if (header["key"].toString().toLatin1() != cacheKey || header["version"] != formatVersion ||
        size <= 0 || size > maxPayloadBytes || size != file.size() - file.pos()) {
        note(stage, "miss", "invalid_header");
        return {};
    }
    auto payload = file.read(size);
    if (payload.size() != size || digest(payload) != header["sha256"].toString().toLatin1()) {
        note(stage, "miss", "invalid_checksum");
        return {};
    }
    note(stage, "hit");
    return payload;
}

void ReconstructionCache::write(const QString& stage, const QByteArray& cacheKey, const QByteArray& payload)
{
    if (!enabled() || cancelled() || cacheKey.isEmpty()) return;
    if (payload.isEmpty() || payload.size() > maxPayloadBytes) {
        note(stage, "miss", "payload_size_limit");
        return;
    }
    const QDir root(m_root);
    if (!root.mkpath(stage)) {
        note(stage, "miss", "directory_unavailable");
        return;
    }
    const auto header = QJsonDocument(QJsonObject{
        {"version", formatVersion}, {"key", QString::fromLatin1(cacheKey)},
        {"bytes", qint64(payload.size())}, {"sha256", QString::fromLatin1(digest(payload))},
        {"client_version", pcad::build::version}, {"commit", pcad::build::gitCommit},
        {"worktree_fingerprint", pcad::build::worktreeFingerprint}
    }).toJson(QJsonDocument::Compact) + '\n';
    QSaveFile file(root.filePath(stage + "/" + QString::fromLatin1(cacheKey) + ".bin"));
    if (!file.open(QIODevice::WriteOnly) || file.write(header) != header.size() ||
        file.write(payload) != payload.size()) {
        note(stage, "miss", "write_failed");
        return;
    }
    if (cancelled()) return;
    if (!file.commit()) note(stage, "miss", "write_failed");
}

template<class... T> bool ReconstructionCache::load(const QString& stage, const QByteArray& cacheKey, T&... output)
{
    const auto payload = read(stage, cacheKey);
    if (payload.isEmpty()) return false;
    if (decode(payload, output...)) return true;
    note(stage, "miss", "invalid_payload");
    return false;
}

bool ReconstructionCache::preprocess(const ObservationSnapshot& source,
    const PointCloudPreprocessingOptions& options, PointStore& working, PointCloudPreprocessingReport& report,
    std::string& error)
{
    const QString stage = "preprocessing";
    const auto cacheKey = enabled() ? key(versions::preprocessing, pointsKey(source.observations()),
        source.sourceUnit(), source.sourceToWorkingScale(), options) : QByteArray{};
    LengthUnit unit = LengthUnit::Arbitrary;
    std::vector<PointSample> samples;
    PointCloudPreprocessingReport storedReport;
    if (load(stage, cacheKey, unit, samples, storedReport)) {
        if (unit == source.observations().unit() && samples.size() == storedReport.outputPointCount &&
            storedReport.inputPointCount == source.observations().size()) {
            try {
                working = PointStore(std::move(samples), unit);
                report = storedReport;
                return true;
            } catch (const std::invalid_argument&) {
                // A malformed snapshot is a miss even when its envelope is intact.
            }
        }
        note(stage, "miss", "invalid_working_snapshot");
    }
    if (!profileStage("cad.preprocess", [&] { return preprocessPointCloud(source, options, working, report, error); }))
        return false;
    if (enabled() && !cancelled()) write(stage, cacheKey, encode(working.unit(), working.points(), report));
    return true;
}

bool ReconstructionCache::analyze(const PointStore& working, const SurfaceAnalysisOptions& options,
    SurfaceAnalysisResult& analysis, std::string& error)
{
    if (!enabled()) {
        note("working_analysis", "bypass", "disabled");
        return analyzeSurfaces(working, options, analysis, error, {}, m_cancelled);
    }
    const auto proposalKey = evidenceKey(working, options);
    const auto snapshotKey = analysisKey(proposalKey, options);
    const QString stage = "working_analysis";
    SurfaceAnalysisResult stored;
    if (load(stage, snapshotKey, stored)) {
        if (analysisMatches(working, stored) && validAnalysisReferences(working, stored)) {
            analysis = std::move(stored);
            return true;
        }
        note(stage, "miss", "invalid_analysis_snapshot");
    }
    const QString evidenceStage = "surface_evidence";
    SurfaceEvidenceCollection proposals;
    if (!load(evidenceStage, proposalKey, proposals)) {
        if (!profileStage("surface.collect_evidence",
                [&] { return collectSurfaceEvidence(working, options, proposals, error, {}, m_cancelled); }))
            return false;
        if (!cancelled()) write(evidenceStage, proposalKey, encode(proposals));
    }
    if (!profileStage("surface.validate_and_index", [&] {
        return indexSurfaceEvidence(working, proposals.evidence, options, analysis, error, {}, m_cancelled);
    })) return false;
    analysis.detectionPasses = proposals.detectionPasses;
    if (!cancelled()) write(stage, snapshotKey, encode(analysis));
    return true;
}

bool ReconstructionCache::detect(const PointStore& working, const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& evidence, std::string& error)
{
    const QString stage = "primitive_detection";
    const auto cacheKey = enabled() ? key(versions::evidence, pointsKey(working), options) : QByteArray{};
    PrimitiveDetectionResult stored;
    if (load(stage, cacheKey, stored)) {
        evidence = std::move(stored);
        return true;
    }
    if (!detectPrimitiveEvidence(working, options, evidence, error)) return false;
    if (enabled() && !cancelled()) write(stage, cacheKey, encode(evidence));
    return true;
}

bool ReconstructionCache::analyzeSource(const PointStore& working, const PointStore& source,
    const SurfaceAnalysisOptions& options, SurfaceAnalysisResult& result, std::string& error, bool& reused)
{
    const QString stage = "source_analysis";
    const auto cacheKey = enabled() ? key(versions::analysis,
        analysisKey(evidenceKey(working, options), options), pointsKey(source)) : QByteArray{};
    SurfaceAnalysisResult stored;
    reused = false;
    if (load(stage, cacheKey, stored)) {
        if (reusableAnalysis(working, source, options, stored) && validAnalysisReferences(working, stored) &&
            (options.supportValidation.mode == SupportValidationMode::Off ||
             (stored.sourceSupportValidation.pointFingerprint == pointStoreFingerprint(source) &&
              stored.sourceSupportValidation.settingsSignature == surfaceSupportSettingsSignature(options)))) {
            result = std::move(stored);
            reused = true;
            return true;
        }
        note(stage, "miss", "invalid_source_snapshot");
    }
    if (!profileStage("recognition.surface_analysis", [&] { return analyze(working, options, result, error); }) ||
        !profileStage("recognition.source_revalidation", [&] {
            return revalidateSurfaceSupportOnSource(working, source, result, error, m_cancelled);
        })) return false;
    if (enabled() && !cancelled()) write(stage, cacheKey, encode(result));
    return true;
}

bool ReconstructionCache::recognize(const PointStore& working, const PointStore& source,
    const PlaneDetectionOptions& planes, const CylinderDetectionOptions& cylinders,
    const std::vector<TemplateHint>& hints, const std::vector<SurfaceRelationHint>& relations,
    const RecognitionSettings& settings, const SurfaceAnalysisResult* sharedAnalysis,
    GuidedSolidResult& result, std::string& error)
{
    if (!enabled()) {
        note("recognition", "bypass", "disabled");
        return reconstructGuidedSolid(working, planes, cylinders, hints, result, error, m_cancelled,
            sharedAnalysis, relations, &source, &settings);
    }

    std::optional<SurfaceAnalysisOptions> options;
    SurfaceAnalysisResult validated;
    bool reused = false;
    if (hints.empty()) {
        options = recognitionAnalysisOptions(working, planes, cylinders, settings.analysis);
        if (sharedAnalysis && reusableAnalysis(working, source, *options, *sharedAnalysis)) {
            validated = *sharedAnalysis;
            reused = true;
            note("source_analysis", "memory");
        } else if (!analyzeSource(working, source, *options, validated, error, reused)) {
            result.analysis = std::move(validated);
            return false;
        }
        sharedAnalysis = &validated;
    }
    const QString stage = "recognition";
    const auto cacheKey = key(versions::recognition, pointsKey(working), pointsKey(source), planes,
        cylinders, hints, relations, serializeRecognitionSettings(settings),
        options ? recognitionInputKey(validated) : digest(QByteArray::fromStdString(serializeAnalysisSettings(loadAnalysisSettings()))));
    bool recognized = false;
    GuidedSolidResult restored;
    std::string storedError;
    if (load(stage, cacheKey, recognized, restored, storedError)) {
        restored.analysisReused = !hints.empty() ? restored.analysisReused : true;
        result = std::move(restored);
        error = std::move(storedError);
        return recognized;
    }
    recognized = reconstructGuidedSolid(working, planes, cylinders, hints, result, error,
        m_cancelled, sharedAnalysis, relations, &source, &settings, options ? &*options : nullptr);
    if (options) result.analysisReused = reused;
    // A completed geometric rejection can be replayed. A failed analysis,
    // inconsistent snapshot or cancelled operation has no completed candidate.
    if (!cancelled() && (recognized || (options && result.analysis.pointFingerprint != 0 &&
        error != "Cancelled" && error != "Stale surface analysis")))
        write(stage, cacheKey, encode(recognized, result, error));
    return recognized;
}

bool ReconstructionCache::assemble(const PointStore& source, const AssemblyOptions& options,
    AssemblyReconstructionResult& result, std::string& error)
{
    if (!enabled()) {
        note("assembly", "bypass", "disabled");
        return reconstructAssembly(source, options, result, error, m_cancelled);
    }
    const diagnostics::StageProfile profile("assembly.reconstruct", source.size());
    const auto groupedKey = assemblyGroupingKey(pointsKey(source), options);
    const auto preparedKey = key(versions::assemblyPreparation, groupedKey, options.alphaFactor);
    const auto closureKey = assemblyClosureKey(preparedKey, options);
    const QString closureStage = "assembly_closure";
    AssemblyReconstructionResult closed;
    if (!load(closureStage, closureKey, closed)) {
        const QString stage = "assembly_preparation";
        PreparedAssembly prepared;
        if (!load(stage, preparedKey, prepared)) {
            const QString groupingStage = "assembly_grouping";
            GroupedAssembly grouped;
            if (!load(groupingStage, groupedKey, grouped)) {
                if (!groupAssembly(source, options, grouped, error, m_cancelled)) return false;
                if (!cancelled()) write(groupingStage, groupedKey, encode(grouped));
            }
            if (!prepareAssemblyBoundaries(source, options, std::move(grouped), prepared, error, m_cancelled))
                return false;
            if (!cancelled()) write(stage, preparedKey, encode(prepared));
        }
        if (!closePreparedAssembly(source, options, prepared, closed, error, m_cancelled)) return false;
        if (!cancelled()) write(closureStage, closureKey, encode(closed));
    }
    if (cancelled()) return false;
    if (!options.buildSolids || !options.maximumProjectionParts ||
        std::none_of(closed.parts.begin(), closed.parts.end(), [&](const auto& part) {
            return !part.closed && part.patchIds.size() >= 2 && part.pointIds.size() >= options.minimumProjectionPartPoints;
        })) {
        result = std::move(closed);
        return true;
    }
    auto projection = options;
    try {
        if (!projection.projectionOptions) projection.projectionOptions = loadProjectionOptions();
    } catch (const std::exception& failure) {
        error = failure.what();
        return false;
    }
    const QString projectionStage = "assembly_projection";
    const auto projectionKey = key(versions::projection, closureKey, *projection.projectionOptions,
        options.maximumProjectionParts, options.minimumProjectionPartPoints);
    AssemblyReconstructionResult projected;
    if (!load(projectionStage, projectionKey, projected)) {
        if (!projectOpenAssembly(source, projection, closed, error, m_cancelled)) return false;
        projected = std::move(closed);
        if (!cancelled()) write(projectionStage, projectionKey, encode(projected));
    }
    result = std::move(projected);
    return true;
}

QByteArray ReconstructionCache::previewKey(const PointStore& source, const SurfaceAnalysisResult& analysis,
    double radius, bool useNormals)
{
    if (!enabled()) return {};
    return key(versions::presentation, pointsKey(source), recognitionInputKey(analysis), radius, useNormals);
}

bool ReconstructionCache::readPreview(const QByteArray& cacheKey, SurfaceAnalysisResult& preview)
{
    return load("source_surface_preview", cacheKey, preview);
}

void ReconstructionCache::writePreview(const QByteArray& cacheKey, const SurfaceAnalysisResult& preview)
{
    if (!cacheKey.isEmpty() && !cancelled()) write("source_surface_preview", cacheKey, encode(preview));
}

QByteArray ReconstructionCache::readPresentation(const QByteArray& cacheKey)
{
    return read("presentation", cacheKey);
}

void ReconstructionCache::writePresentation(const QByteArray& cacheKey, const QByteArray& snapshot)
{
    write("presentation", cacheKey, snapshot);
}

QByteArray ReconstructionCache::presentationKey(const PointStore& source, const QByteArray& context,
    bool assembly)
{
    return key(versions::presentation, pointsKey(source), context,
        std::string(assembly ? versions::assemblyClosure : versions::analysis),
        std::string(assembly ? versions::projection : ""));
}
