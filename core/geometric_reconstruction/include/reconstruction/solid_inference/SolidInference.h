#pragma once

#include "reconstruction/solid_inference/SurfaceScene.h"
#include "modeling/FeatureParams.h"
#include "modeling/ProfileBody.h"
#include "modeling/ModelingCore.h"

namespace reconstruction {

enum class SolidCandidateStatus { Proposed, Accepted, Ambiguous, Rejected };

struct SolidFeatureOperation {
    modeling::SketchPlaneReference plane;
    std::vector<modeling::SketchEntity> entities;
    double depth = 0;
    bool reverse = false;
    modeling::ExtrudeOperation operation = modeling::ExtrudeOperation::NewBody;
};

struct InferredSurfaceRecord {
    PrimitiveKind kind = PrimitiveKind::Plane;
    std::vector<PatchId> sourcePatches;
    std::vector<modeling::Vec3> boundaryVertices;
    double confidence = 0;
    std::string reason;
};

struct SolidCandidate {
    std::uint64_t analysisFingerprint = 0;
    std::vector<PatchId> supportingPatches;
    std::vector<PointId> supportingPoints;
    std::vector<PatchId> unresolvedPatches;
    std::vector<PatchId> inferredPatches;
    std::vector<InferredSurfaceRecord> inferredSurfaces;
    std::vector<PointId> unresolvedPoints;
    bool inferredSideWalls = false;
    std::string depthEvidence;
    std::vector<SolidFeatureOperation> featureProgram;
    std::optional<modeling::ProfileBodyParams> boundaryBody;
    double score = 0;
    double observationCoverage = 0;
    double distanceError = 0;
    double distanceMedianMm = 0;
    double distanceP95Mm = 0;
    double normalError = 0;
    std::size_t normalComparisons = 0;
    std::size_t occupancyConflictCount = 0;
    double relationError = 0;
    std::size_t relationComparisons = 0;
    double complexityPenalty = 0;
    SolidCandidateStatus status = SolidCandidateStatus::Rejected;
    std::vector<std::string> diagnostics;
};

// Proposes a direct shell closure for an explicit part hypothesis. The input
// patches must carry observed closed planar boundaries and known orientation.
bool proposeObservedShell(const SurfaceScene& scene, const std::vector<PatchId>& partPatches,
                          SolidCandidate& output, std::string& error);

// Builds a prism hypothesis from two supported, parallel planar patches with
// matching finite boundary rings. Its end caps are explicitly inferred.
bool proposeParallelCapExtrusion(const SurfaceScene& scene, PatchId first, PatchId second,
                                SolidCandidate& output, std::string& error);

struct SolidInferenceResult {
    std::vector<SolidCandidate> candidates;
    std::vector<PatchId> unresolvedPatches;
};

struct SolidInferenceOptions {
    std::size_t beamWidth = 4;
    std::size_t maximumFeatureOperations = 3;
    std::size_t maximumCandidates = 64;
    std::size_t maximumSearchSamples = 256;
};

bool inferSolids(const SurfaceScene& scene, SolidInferenceResult& output, std::string& error,
                 const SolidInferenceOptions& options = {},
                 const PointStore* workingObservations = nullptr);

struct SolidScoringOptions {
    double minimumObservationCoverage = 0.9;
    double maximumP95ToleranceMultiplier = 4.0;
    std::size_t maximumOccupancyConflicts = 2;
    double maximumOccupancyConflictFraction = 0.005;
};

bool scoreSolidCandidate(const SolidCandidate& candidate, const SurfaceScene& scene,
                         const PointStore& observations, double tolerance,
                         SolidCandidate& scored, std::string& error,
                         const SolidScoringOptions& options = {});

// A near tie is ambiguous only when the candidates describe distinct material.
const SolidCandidate* selectSolidCandidate(const std::vector<SolidCandidate>& scored,
                                           std::string& error);

} // namespace reconstruction
