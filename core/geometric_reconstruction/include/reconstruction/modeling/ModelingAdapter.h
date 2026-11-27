#pragma once

#include "modeling/ModelingCore.h"
#include "reconstruction/recognition/FeatureCandidate.h"
#include "reconstruction/recognition/CompoundRecognition.h"
#include "reconstruction/solid_inference/SolidInference.h"

#include <optional>

namespace reconstruction {

// Replays one operation on a staging model and records its extrusion ID for
// later face and body references. When supplied, patch records the same commands.
bool appendFeatureProgramOperation(const ProjectionOperation& operation,
    std::vector<modeling::FeatureId>& operationIds, modeling::ModelingCore& staging,
    std::string& error, modeling::ModelPatch* patch = nullptr);

// Resolves candidate dependencies on a staging copy. The resulting patch
// is bound to the snapshot revision and can be inspected before applying it.
std::optional<modeling::ModelPatch> planCompoundCandidate(
    const CompoundCandidate& candidate, const modeling::ModelingCore& snapshot,
    std::string& error,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision,
    bool replacePrevious = false, bool overwriteManual = false);

modeling::ModelPatchResult commitCompoundCandidate(
    const CompoundCandidate& candidate, modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision,
    bool replacePrevious = false, bool overwriteManual = false);

std::optional<modeling::ModelPatch> planSolidCandidate(
    const SolidCandidate& candidate, const modeling::ModelingCore& snapshot,
    std::string& error,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

modeling::ModelPatchResult commitSolidCandidate(
    const SolidCandidate& candidate, modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

modeling::ModelPatchResult commitBoxCandidate(
    const BoxCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

modeling::ModelPatchResult commitBoxWithThroughHolesCandidate(
    const BoxWithThroughHolesCandidate& candidate,
    modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision = modeling::kAnyModelRevision);

}  // namespace reconstruction
