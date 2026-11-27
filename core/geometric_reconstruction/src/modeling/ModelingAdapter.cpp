#include "reconstruction/modeling/ModelingAdapter.h"

#include <cmath>

namespace reconstruction {

bool appendFeatureProgramOperation(const ProjectionOperation& operation,
    std::vector<modeling::FeatureId>& operationIds, modeling::ModelingCore& staging,
    std::string& error, modeling::ModelPatch* patch) {
    using modeling::ExtrudeOperation;
    const auto target = operation.operation == ExtrudeOperation::NewBody || operationIds.empty()
        ? modeling::kInvalidFeatureId : operationIds.back();
    if (operation.sketch.entities.empty() || !std::isfinite(operation.depth) ||
        (!operation.throughAll && operation.depth <= 0)) {
        error = "Feature program contains an invalid sketch or extrusion depth";
        return false;
    }
    if (operation.throughAll && (operation.operation != ExtrudeOperation::Cut || !target)) {
        error = "Through-all feature program operation requires a scoped cut";
        return false;
    }

    auto sketch = operation.sketch;
    if (operation.sketchFace) {
        const auto& reference = *operation.sketchFace;
        const bool cap = reference.role == modeling::FaceRole::StartFace ||
                         reference.role == modeling::FaceRole::EndFace;
        const bool side = reference.role == modeling::FaceRole::ExtrusionSide;
        if (reference.operationIndex >= operationIds.size() || !std::isfinite(reference.normalOffset) ||
            (!cap && !side) || side != (reference.profileEdge != modeling::kInvalidSketchEntityId)) {
            error = "Feature program sketch must reference a prior extrusion cap or profile edge";
            return false;
        }
        sketch.plane = modeling::featureFace(operationIds[reference.operationIndex], reference.role,
            reference.normalOffset, reference.profileEdge);
    }

    const auto append = [&](const modeling::FeatureParams& params) {
        modeling::AddFeatureCommand command{params, true};
        if (patch) patch->commands.push_back(command);
        return staging.execute(command);
    };
    const auto sketchResult = append(sketch);
    if (!sketchResult.success) {
        error = sketchResult.error;
        return false;
    }
    const auto extrusionResult = append(modeling::ExtrudeFeatureParams{
        sketchResult.featureId, operation.depth, operation.reverse, operation.operation, target, operation.throughAll});
    if (!extrusionResult.success) {
        error = extrusionResult.error;
        return false;
    }
    operationIds.push_back(extrusionResult.featureId);
    return true;
}

std::optional<modeling::ModelPatch> planSolidCandidate(
    const SolidCandidate& candidate, const modeling::ModelingCore& snapshot,
    std::string& error, modeling::ModelRevision expectedRevision) {
    error.clear();
    if (expectedRevision != modeling::kAnyModelRevision && expectedRevision != snapshot.revision()) {
        error = "Model revision mismatch: expected " + std::to_string(expectedRevision) +
            ", current " + std::to_string(snapshot.revision());
        return std::nullopt;
    }
    if (candidate.status != SolidCandidateStatus::Accepted ||
        (candidate.featureProgram.empty() && !candidate.boundaryBody)) {
        error = "Solid candidate is not accepted or has no rebuildable feature program or boundary body";
        return std::nullopt;
    }
    modeling::ModelPatch patch;
    patch.expectedRevision = snapshot.revision();
    patch.rebuild = true;
    auto staged = snapshot;
    if (!candidate.featureProgram.empty()) {
        if (candidate.featureProgram.front().operation != modeling::ExtrudeOperation::NewBody) {
            error = "Solid candidate feature program must begin with NewBody";
            return std::nullopt;
        }
        std::vector<modeling::FeatureId> operationIds;
        for (const auto& operation : candidate.featureProgram)
            if (!appendFeatureProgramOperation(operation, operationIds, staged, error, &patch))
                return std::nullopt;
    } else {
        modeling::AddFeatureCommand addBody{*candidate.boundaryBody, true};
        patch.commands.push_back(addBody);
        if (!staged.apply(patch).success) {
            error = staged.lastError();
            if (error.empty()) error = "Solid candidate failed staged parametric rebuild";
            return std::nullopt;
        }
    }
    return patch;
}

modeling::ModelPatchResult commitSolidCandidate(
    const SolidCandidate& candidate, modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision) {
    std::string error;
    const auto patch = planSolidCandidate(candidate, core, error, expectedRevision);
    if (!patch) return {false, core.revision(), {}, std::move(error)};
    return core.apply(*patch);
}

std::optional<modeling::ModelPatch> planCompoundCandidate(
    const CompoundCandidate& candidate, const modeling::ModelingCore& snapshot,
    std::string& error, modeling::ModelRevision expectedRevision,
    bool replacePrevious, bool overwriteManual) {
    modeling::ModelPatch patch;
    patch.expectedRevision = snapshot.revision();
    error.clear();
    auto reject = [&](const std::string& reason) -> std::optional<modeling::ModelPatch> {
        error = reason;
        return std::nullopt;
    };
    if (expectedRevision != modeling::kAnyModelRevision && expectedRevision != snapshot.revision()) {
        return reject("Model revision mismatch: expected " + std::to_string(expectedRevision) +
            ", current " + std::to_string(snapshot.revision()));
    }
    // Allocate sketch IDs on a private copy; the final patch applies atomically.
    auto staging = snapshot;
    if (replacePrevious) {
        for (const auto& feature : snapshot.features()) {
            if (feature.reconstructed && feature.manuallyEdited && !overwriteManual)
                return reject("Manually edited CAD features are protected; explicit replacement is required");
        }
        // Remove only this workflow's features, in dependency order. Authored
        // dependents make replacement fail atomically rather than being deleted.
        const auto features = snapshot.features();
        for (auto i = features.rbegin(); i != features.rend(); ++i) if (i->reconstructed) {
            patch.commands.push_back(modeling::RemoveFeatureCommand{i->id, false});
            const auto removed = staging.execute(patch.commands.back());
            if (!removed.success) return reject(removed.error);
        }
    }
    auto append = [&](const modeling::FeatureParams &params) {
        patch.commands.push_back(modeling::AddFeatureCommand{params, true});
        return staging.execute(patch.commands.back());
    };
    for (const auto &box : candidate.boxes) {
        auto params = box.primitive;
        params.join = true;
        const auto result = append(params);
        if (!result.success)
            return reject(result.error);
    }
    for (const auto &prism : candidate.prisms) {
        modeling::SketchFeatureParams sketch;
        sketch.plane = prism.plane;
        for (std::size_t i = 0; i < prism.polygon.size(); ++i) {
            const auto a = prism.polygon[i], b = prism.polygon[(i + 1) % prism.polygon.size()];
            sketch.entities.push_back({static_cast<modeling::SketchEntityId>(i + 1),
                                       modeling::Line2D{a.x, a.y, b.x, b.y}});
        }
        const auto result = append(sketch);
        if (!result.success)
            return reject(result.error);
        modeling::ExtrudeFeatureParams extrusion;
        extrusion.sketchId = result.featureId;
        extrusion.depth = prism.depth;
        extrusion.operation = prism.subtract ? modeling::ExtrudeOperation::Cut :
            (staging.features().size() > 1 ? modeling::ExtrudeOperation::Join : modeling::ExtrudeOperation::NewBody);
        const auto added = append(extrusion);
        if (!added.success)
            return reject(added.error);
    }
    for (const auto &hole : candidate.throughHoles) {
        const auto added = append(hole.primitive);
        if (!added.success) return reject(added.error);
    }
    for (const auto& solid : candidate.curvedSolids) {
        const auto added = std::visit([&](const auto& params) { return append(params); }, solid.primitive);
        if (!added.success) return reject(added.error);
    }
    for(const auto& body:candidate.profileBodies) {
        const auto added=append(body);
        if(!added.success)return reject(added.error);
    }
    for (const auto& program : candidate.projectionPrograms) {
        if (program.empty() || program.front().operation != modeling::ExtrudeOperation::NewBody)
            return reject("Projection program has no base extrusion");
        std::vector<modeling::FeatureId> operationIds;
        for (const auto& operation : program)
            if (!appendFeatureProgramOperation(operation, operationIds, staging, error, &patch))
                return std::nullopt;
    }
    return patch;
}

modeling::ModelPatchResult commitCompoundCandidate(
    const CompoundCandidate& candidate, modeling::ModelingCore& core,
    modeling::ModelRevision expectedRevision, bool replacePrevious, bool overwriteManual) {
    std::string error;
    const auto patch = planCompoundCandidate(candidate, core, error, expectedRevision,
        replacePrevious, overwriteManual);
    if (!patch) return {false, core.revision(), {}, std::move(error)};
    return core.apply(*patch);
}

modeling::ModelPatchResult commitBoxCandidate(const BoxCandidate &candidate,
                                              modeling::ModelingCore &core,
                                              modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    patch.commands.push_back(modeling::AddFeatureCommand{candidate.primitive, true});
    patch.rebuild = true;
    return core.apply(patch);
}

modeling::ModelPatchResult
commitBoxWithThroughHolesCandidate(const BoxWithThroughHolesCandidate &candidate,
                                   modeling::ModelingCore &core,
                                   modeling::ModelRevision expectedRevision) {
    modeling::ModelPatch patch;
    patch.expectedRevision = expectedRevision;
    patch.commands.push_back(modeling::AddFeatureCommand{candidate.box.primitive, true});
    for (const ThroughHoleCandidate &hole : candidate.throughHoles) {
        patch.commands.push_back(modeling::AddFeatureCommand{hole.primitive, true});
    }
    patch.rebuild = true;
    return core.apply(patch);
}

} // namespace reconstruction
