#pragma once

#include "reconstruction/surface/SurfaceAnalysis.h"

namespace reconstruction {

enum class SceneBoundaryStatus { ObservedClosed, ObservedOpen, Ambiguous };
enum class SceneLoopRole { Outer, Hole, Island, Unknown };

struct SceneHeader {
    std::uint32_t schemaVersion = 1;
    LengthUnit unit = LengthUnit::Arbitrary;
    std::uint64_t analysisFingerprint = 0;
    std::uint64_t sourceFingerprint = 0;
    std::uint64_t settingsFingerprint = 0;
    double distanceToleranceMm = 0;
    double angularToleranceRad = 0;
};

struct BoundaryLoop3d {
    SceneLoopRole role = SceneLoopRole::Unknown;
    std::vector<modeling::Vec3> vertices;
    bool closed = false;
    double confidence = 0;
};

struct ScenePatch {
    PatchId id = 0;
    SurfaceId surfaceId = 0;
    std::vector<PointId> supportPointIds;
    modeling::Vec3 centroid{}, materialNormal{};
    bool orientationKnown = false;
    SceneBoundaryStatus boundaryStatus = SceneBoundaryStatus::Ambiguous;
    std::vector<BoundaryLoop3d> loops;
    double supportConfidence = 0;
    double residualMm = 0;
    double spacingMm = 0;
};

struct SceneRelation {
    PatchId first = 0, second = 0;
    SurfaceRelationKind kind = SurfaceRelationKind::Adjacent;
    double error = 0, confidence = 0;
    bool manualConfirmed = false;
};

struct SurfaceScene {
    SceneHeader header;
    std::vector<PrimitiveEvidence> surfaces;
    std::vector<ScenePatch> patches;
    std::vector<SceneRelation> relations;
    std::vector<PointId> unassignedPointIds;
};

enum class SurfaceSceneError {
    None, WrongUnit, SnapshotMismatch, InvalidSurfaceReference,
    MissingBoundary, InvalidLoop, UnknownOrientation, MappingAmbiguous
};

struct SurfaceSceneValidation {
    SurfaceSceneError code = SurfaceSceneError::None;
    PatchId patchId = 0;
    std::string message;
    explicit operator bool() const noexcept { return code == SurfaceSceneError::None; }
};

bool exportSurfaceScene(const PointStore& points, const SurfaceAnalysisResult& analysis,
                        SurfaceScene& output, std::string& error,
                        const PointStore* sourceObservations = nullptr);
SurfaceSceneValidation validateSurfaceScene(const SurfaceScene& scene,
                                             std::uint64_t expectedAnalysisFingerprint = 0);
std::uint64_t surfaceSceneSettingsFingerprint(const SurfaceAnalysisOptions& options) noexcept;

} // namespace reconstruction
