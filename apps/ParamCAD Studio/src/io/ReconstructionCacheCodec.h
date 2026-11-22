#pragma once

#include <reconstruction/parts/AssemblyReconstruction.h>
#include <reconstruction/pipeline/GuidedSolidRecognizer.h>
#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <reconstruction/surface/SurfaceFootprint.h>

#include <QByteArray>
#include <QDataStream>
#include <QIODevice>

#include <limits>
#include <tuple>
#include <type_traits>

namespace reconstruction_cache {

// This field order is the cache wire format. Update the format version when changing it.
#define CACHE_FIELDS(Type, ...) \
    template<class Archive> void transfer(Archive& archive, Type& v) { archive(__VA_ARGS__); } \
    template<class Archive> void transfer(Archive& archive, const Type& v) { archive(__VA_ARGS__); }

CACHE_FIELDS(reconstruction::SurfaceRelation, v.first, v.second, v.kind, v.error, v.confidence, v.automatic)

CACHE_FIELDS(reconstruction::SurfacePatch, v.id, v.surfaceId, v.supportPointIds, v.lower, v.upper, v.centroid,
    v.materialNormal, v.orientationKnown)

CACHE_FIELDS(reconstruction::SurfaceRelationOptions, v.distanceTolerance, v.angularToleranceRadians,
    v.adjacentConfidence, v.maximumConfidence, v.derivedConfidenceFraction, v.manualPatchOverlapFraction,
    v.optimizationMaximumAttempts, v.optimizationMaximumActiveSurfaces, v.optimizationResidualSampleTarget,
    v.constraintWeight, v.optimizationInlierFraction, v.maximumMeanErrorIncreaseMultiplier,
    v.validationToleranceFraction, v.optimizationIterations, v.initialDamping, v.convergenceStep,
    v.supportNormalCosine)

CACHE_FIELDS(reconstruction::SurfaceRelationHint, v.firstPoints, v.secondPoints, v.kind)

CACHE_FIELDS(reconstruction::SurfaceAnalysisOptions, v.detection, v.relations, v.supportValidation,
    v.supportMeshing, v.connectivityDistance, v.independentFamilies, v.familyDetectionWorkerCount,
    v.recoverResidualSurfaces, v.expandPlaneSupport, v.minimumRecoveryRegionPoints,
    v.recoveryHaloToleranceMultiplier, v.recoveryHaloRelationMultiplier, v.stablePlaneOverlapFraction,
    v.stablePlaneNormalCosine, v.recoveryConnectivityToleranceMultiplier,
    v.recoveryConnectivityAdaptiveMultiplier)

CACHE_FIELDS(reconstruction::LocalAnalysisFailure, v.reason, v.message, v.pointIds, v.recoveredPointIds,
    v.candidates, v.repairedCandidates, v.rejectedRelations, v.repairedRelations, v.repairedPatches,
    v.localFingerprint, v.sourceFingerprint, v.detectionPasses, v.observations, v.repaired)

CACHE_FIELDS(reconstruction::SurfaceIdMapping, v.localFingerprint, v.local, v.global)

CACHE_FIELDS(reconstruction::SurfaceAnalysisResult, v.pointFingerprint, v.unit, v.options, v.surfaces,
    v.supportProposals, v.supportProposalIds, v.supportValidation, v.sourceSupportValidation,
    v.sourceValidatedSurfaces, v.patches, v.relations, v.unassignedPointIds, v.detectionPasses,
    v.localFailures, v.baseFingerprint, v.recoverySourceFingerprint, v.supplementalPoints, v.repairMappings,
    v.repairPatchMappings)

CACHE_FIELDS(reconstruction::SurfaceEvidenceCollection, v.evidence, v.unassignedPointIds, v.detectionPasses)
CACHE_FIELDS(reconstruction::PrimitiveDetectionResult, v.evidence, v.unassignedPointIds)

CACHE_FIELDS(reconstruction::SurfaceSupportValidationOptions, v.mode, v.planeDensityRatioMax,
    v.sphereMinSolidAngle, v.sphereMinMinorSpanRadians, v.cylinderMinAngularSpanRadians,
    v.minimumSagittaNoiseRatio, v.referenceMinConfidence, v.referenceMinPoints, v.referenceMinPlanes,
    v.referenceAreaQuantile, v.referenceMaxLogSpread, v.minimumExpectedPoints, v.gridSpacingMultiplier,
    v.minimumOccupiedCellRatio, v.noiseFloorToleranceFraction, v.modelComplexityWeight, v.minimumModelGain,
    v.planePreferenceMinReferenceAreaFraction, v.workerCount)

CACHE_FIELDS(reconstruction::SurfaceSamplingReference, v.available, v.density, v.spacing, v.logSpread,
    v.noise, v.reason, v.patchIds)

CACHE_FIELDS(reconstruction::SurfaceSupportDecision, v.proposalId, v.patchId, v.kind, v.verdict,
    v.densityEvaluated, v.occupancyEvaluated, v.protectedManual, v.pointCount, v.coveredPoints,
    v.effectivePoints, v.finalCoveredPoints, v.area, v.finalArea, v.density, v.densityRatio,
    v.occupiedCellRatio, v.solidAngle, v.minorSpanRadians, v.angularSpanRadians, v.sagitta, v.noise,
    v.planeCost, v.curvedCost, v.noiseSource, v.parametersJson, v.footprint, v.reasons, v.supportPointIds)

CACHE_FIELDS(reconstruction::SurfaceSupportValidationReport, v.pointFingerprint, v.unit, v.mode, v.options,
    v.settingsSignature, v.reference, v.planeHoleRecoveredPoints, v.planeRelaxedRecoveredPoints, v.decisions)

CACHE_FIELDS(reconstruction::PlanarDomainSupportCheck, v.observedPoints, v.outerBoundaryPoints,
    v.holeBoundaryPoints, v.interiorPoints, v.occupiedInteriorCells, v.boundaryBandWidth, v.area,
    v.interiorArea, v.expectedInteriorPoints, v.interiorAreaMeasured, v.rejected)

CACHE_FIELDS(reconstruction::PlanarFootprintHole, v.boundary, v.area, v.spacing)

CACHE_FIELDS(reconstruction::SurfaceFootprint, v.origin, v.axisU, v.axisV, v.positions, v.triangles,
    v.sourcePointIds, v.coveredPointIds, v.sampleAnchors, v.effectiveCoveredPoints, v.domainSupportChecks,
    v.holes)

CACHE_FIELDS(reconstruction::SurfaceMeshingSettings, v.minimumSurfacePoints, v.meshPointBudget,
    v.densityNeighborCount, v.localSupportRadiusMultiplier, v.connectivityRadiusMultiplier,
    v.minimumTransverseVarianceRatio, v.regularSpacingTolerance, v.regularSpacingLimitMultiplier,
    v.densityGapMultiplier, v.coverageRadiusMultiplier, v.planarAlphaRadiusMultiplier,
    v.curvedAlphaRadiusMultiplier, v.expectedHolePoints, v.smallDensityScale, v.holeDensityRatio,
    v.boundaryBandSpacingMultiplier, v.boundaryDominanceRatio, v.deepInteriorDistanceMultiplier,
    v.minimumDeepInteriorAreaSpacings, v.minimumExpectedInteriorPoints, v.minimumInteriorSupportRatio,
    v.slotMinimumLengthSpacings, v.slotMaximumWidthSpacings, v.slotMinimumSideSpacings, v.slotEdgeAlignment,
    v.slotPerimeterSupportRatio)

CACHE_FIELDS(reconstruction::PointSample, v.id, v.position, v.normal, v.confidence, v.curvature,
    v.normalOrientationKnown)

CACHE_FIELDS(reconstruction::PlaneEquation, v.normal, v.offset)

CACHE_FIELDS(reconstruction::PlaneEvidence, v.id, v.plane, v.supportPointIds, v.meanAbsoluteResidual,
    v.maxAbsoluteResidual, v.coverage, v.confidence)

CACHE_FIELDS(reconstruction::CylinderEquation, v.axisPoint, v.axisDirection, v.radius)

CACHE_FIELDS(reconstruction::CylinderEvidence, v.id, v.cylinder, v.supportPointIds, v.axialMinimum,
    v.axialMaximum, v.angularCoverage, v.meanAbsoluteResidual, v.maxAbsoluteResidual, v.coverage,
    v.confidence)

CACHE_FIELDS(reconstruction::SphereEquation, v.center, v.radius)

CACHE_FIELDS(reconstruction::SphereEvidence, v.id, v.sphere, v.supportPointIds, v.meanAbsoluteResidual,
    v.maxAbsoluteResidual, v.coverage, v.confidence)

CACHE_FIELDS(reconstruction::ConeEquation, v.apex, v.axisDirection, v.openingAngleRadians)

CACHE_FIELDS(reconstruction::ConeEvidence, v.id, v.cone, v.supportPointIds, v.axialMinimum, v.axialMaximum,
    v.angularCoverage, v.meanAbsoluteResidual, v.maxAbsoluteResidual, v.coverage, v.confidence)

CACHE_FIELDS(reconstruction::TorusEquation, v.center, v.axisDirection, v.majorRadius, v.minorRadius)

CACHE_FIELDS(reconstruction::TorusEvidence, v.id, v.torus, v.supportPointIds, v.meanAbsoluteResidual,
    v.maxAbsoluteResidual, v.coverage, v.confidence)

CACHE_FIELDS(reconstruction::PlaneDetectionOptions, v.distanceThreshold, v.minimumSupportPoints,
    v.maximumPlanes, v.probability, v.minimumNormalAlignment, v.clusterEpsilon,
    v.duplicatePlaneAngularToleranceRadians, v.duplicatePlaneDistanceMultiplier,
    v.mergeAngularToleranceRadians, v.mergeResidualToleranceMultiplier, v.randomSeed)

CACHE_FIELDS(reconstruction::CylinderDetectionOptions, v.distanceThreshold, v.minimumNormalAlignment,
    v.minimumSupportPoints, v.maximumCylinders, v.probability, v.clusterEpsilon, v.minimumRadius,
    v.maximumRadius, v.randomSeed)

CACHE_FIELDS(reconstruction::SphereDetectionOptions, v.distanceThreshold, v.minimumNormalAlignment,
    v.minimumSupportPoints, v.maximumSpheres, v.probability, v.clusterEpsilon, v.minimumRadius,
    v.maximumRadius)

CACHE_FIELDS(reconstruction::ConeDetectionOptions, v.distanceThreshold, v.minimumNormalAlignment,
    v.minimumSupportPoints, v.maximumCones, v.probability, v.clusterEpsilon, v.minimumOpeningAngleRadians,
    v.maximumOpeningAngleRadians)

CACHE_FIELDS(reconstruction::TorusDetectionOptions, v.distanceThreshold, v.minimumNormalAlignment,
    v.minimumSupportPoints, v.maximumTori, v.probability, v.clusterEpsilon, v.minimumMajorRadius,
    v.maximumMajorRadius, v.minimumMinorRadius, v.maximumMinorRadius)

CACHE_FIELDS(reconstruction::PrimitiveDetectionOptions, v.randomSeed, v.detectPlanes, v.detectCylinders,
    v.detectSpheres, v.detectCones, v.detectTori, v.multiScaleRounds, v.multiScaleInitialAlignment,
    v.multiScaleRecoveryAlignment, v.multiScaleNeighborCount, v.multiScaleMinimumSupportFloor, v.plane,
    v.cylinder, v.sphere, v.cone, v.torus)

CACHE_FIELDS(reconstruction::PointCloudPreprocessingOptions, v.workerCount, v.voxelSize,
    v.removeStatisticalOutliers, v.outlierNeighborCount, v.outlierStandardDeviationMultiplier,
    v.outlierRobustMedianMultiplier, v.estimateMissingNormals, v.normalNeighborCount,
    v.normalOrientationMinimumAlignment, v.normalOrientationMaximumTangentDeviation,
    v.normalOrientationMaximumSpacingRatio, v.normalOrientationDistanceCostWeight,
    v.convertKnownUnitsToMillimeters, v.protectedPointIds)

CACHE_FIELDS(reconstruction::PointCloudPreprocessingReport, v.inputPointCount, v.voxelPointCount,
    v.outputPointCount, v.removedByVoxel, v.removedAsOutliers, v.normalsFromInput, v.normalsEstimated,
    v.estimatedNormalsOriented, v.normalsWithoutOrientation, v.degenerateNeighborhoods, v.pointsWithCurvature,
    v.appliedScaleToMillimeters, v.outlierMeanNeighborDistance, v.outlierMedianNeighborDistance,
    v.outlierDistanceThreshold)

CACHE_FIELDS(reconstruction::PrismCandidate, v.plane, v.polygon, v.depth, v.subtract)

CACHE_FIELDS(reconstruction::CompoundCandidate, v.boxes, v.prisms, v.throughHoles, v.curvedSolids,
    v.profileBodies, v.projectionPrograms, v.occupancyError, v.surfaceCoverage, v.confidence, v.gridCells,
    v.searchNodes, v.minimumBoxCountProven, v.unresolvedPointIds)

CACHE_FIELDS(reconstruction::CurvedSolidCandidate, v.primitive, v.surfaceCoverage, v.boundaryCoverage,
    v.meanResidual)

CACHE_FIELDS(reconstruction::BoxCandidate, v.primitive, v.sourceSurfaces, v.confidence)

CACHE_FIELDS(reconstruction::ThroughHoleCandidate, v.primitive, v.sourceSurface, v.bodyThickness,
    v.confidence)

CACHE_FIELDS(reconstruction::TemplateHint, v.regionId, v.kind, v.points)

CACHE_FIELDS(reconstruction::TemplateFit, v.regionId, v.surface, v.box, v.solid)

CACHE_FIELDS(reconstruction::GuidedSolidResult, v.candidate, v.analysis, v.analysisReused, v.fits,
    v.failedRegionId)

CACHE_FIELDS(reconstruction::AssemblyOptions, v.distanceTolerance, v.maximumWorkingPoints, v.neighbors,
    v.minimumPatchPoints, v.maximumPrimitiveAttempts, v.normalAngleRadians, v.connectivityFactor,
    v.alphaFactor, v.buildSolids, v.maximumProjectionParts, v.maximumLocalProfileAttempts,
    v.enableSpecializedRecovery, v.projectionOptions, v.minimumPartPatches, v.maximumPartPatches,
    v.maximumClosurePatches, v.maximumClosureVertices, v.weldToleranceMultiplier, v.minimumPartSpanMultiplier,
    v.minimumClosureSamples, v.maximumClosureSamples, v.closureSampleToleranceMultiplier,
    v.curvedClosureCoverage, v.minimumClosureConflicts, v.closureConflictPointDivisor,
    v.minimumProjectionPartPoints, v.patchSpacingToleranceMultiplier, v.connectionToleranceMultiplier,
    v.minimumConvexLinkFraction, v.minimumLinkCount, v.minimumPartPointsForLocalRecovery,
    v.minimumProfileSplitPoints, v.minimumParentSplitPoints, v.minimumLargeParentPoints,
    v.minimumHugeParentPoints, v.minimumProfileSamples, v.minimumLargeProfileSamples,
    v.minimumRetainedFraction, v.minimumResidualSamples, v.minimumParentSupportFraction, v.minimumPlaneCount,
    v.maximumPlaneCount, v.maximumBroadPairs, v.maximumProfilePairs)

CACHE_FIELDS(reconstruction::BoundaryRecoveryDiagnostics, v.tracedLoops, v.nonSimpleAlphaLoops,
    v.nonSimpleSimplifiedLoops, v.nonSimpleRecoveredLoops, v.discardedConstraintLoops, v.splitAlphaLoops,
    v.rejectedAlphaLoops, v.simplificationFallbacks, v.constraintFallbacks, v.snappingFallbacks,
    v.topologyFallbacks)

CACHE_FIELDS(reconstruction::ObservedPatchBoundary, v.normal, v.loops, v.snappedVertices, v.revisionSource)

CACHE_FIELDS(reconstruction::FiniteSurfacePatch, v.id, v.origin, v.normal, v.xDirection, v.lower, v.upper,
    v.spacing, v.residual, v.planar, v.orientationKnown, v.pointIds, v.loops, v.snappedVertices,
    v.boundaryDiagnostics, v.boundaryBeforeClosure, v.workingPointIds)

CACHE_FIELDS(reconstruction::PatchConnection, v.first, v.second, v.convex, v.observations, v.distance,
    v.accepted, v.orientationKnown)

CACHE_FIELDS(reconstruction::AssemblyPart, v.id, v.patchIds, v.pointIds, v.lower, v.upper, v.closed,
    v.orientationKnown, v.normalOrientationInferred, v.openEdges, v.nonmanifoldEdges, v.confidence, v.status,
    v.reason, v.projection, v.workingPointIds)

CACHE_FIELDS(reconstruction::AssemblyReconstructionResult, v.inputPoints, v.workingPoints,
    v.distanceTolerance, v.patches, v.connections, v.parts, v.unassignedPointIds, v.candidate,
    v.unassignedWorkingPointIds)

CACHE_FIELDS(reconstruction::PreparedAssembly, v.sourceFingerprint, v.workingPoints, v.groups, v.observed)

CACHE_FIELDS(reconstruction::GroupedAssembly, v.preparation, v.spacing, v.labels, v.patchParts)

CACHE_FIELDS(reconstruction::ProjectionRegionSupport, v.minimumPoints, v.minimumFraction)

CACHE_FIELDS(reconstruction::ProjectionExtrusionOptions, v.distanceTolerance, v.maximumAxes,
    v.maximumFitPoints, v.maximumScorePoints, v.maximumOperations, v.includeAreaDirections,
    v.retainRejectedProgram, v.path, v.collectSketchInputs, v.intersectionRegionSupport,
    v.relativeDistanceTolerance, v.scoreVoxelScale, v.maximumEvidencePlanes, v.maximumEvidenceCylinders,
    v.evidenceSupportDivisor, v.axisAlignmentCosine, v.sideMomentNormalCosine, v.sideMomentEigenGap,
    v.areaSamplePoints, v.areaSampleDirections, v.maximumAreaDirections, v.extentTrimFraction,
    v.minimumCapPoints, v.capBandMultiplier, v.capNormalCosine, v.minimumDepthMultiplier,
    v.capAlphaSpacingMultiplier, v.capAlphaToleranceMultiplier, v.capSimplificationTolerance,
    v.capSpacingNeighbors, v.capSpacingSamples, v.maximumSketchEdges, v.minimumCircleVertices,
    v.minimumCircleRadiusMultiplier, v.maximumCircleGapDegrees, v.sideSnapNormalCosine,
    v.sideSnapReachMultiplier, v.intersectionAlphaRadius, v.intersectionAlphaScale,
    v.intersectionSimplificationTolerance, v.intersectionMaximumAxisCosine, v.innerWallNormalCosine,
    v.innerWallEndTrimFraction, v.innerWallDepthBands, v.innerWallMinimumBands,
    v.innerWallMinimumPointsPerBand, v.innerWallReachMultiplier, v.capWallSpacingMultiplier,
    v.cappedCylinderCoverage, v.cutCylinderCoverage, v.cylinderEndBandMultiplier,
    v.cylinderRadialNormalCosine, v.cylinderOpposingNormalFraction, v.exteriorCircleOutsideFraction,
    v.minimumOpeningPoints, v.blindHoleNormalCosine, v.throughHoleOpeningNormalCosine,
    v.throughHoleToleranceSpanFraction, v.throughHoleOpeningBandMultiplier, v.throughHoleOpeningBandFraction,
    v.throughHoleCutMarginToleranceFraction, v.throughHoleCutMarginDepthFraction, v.slotWallNormalCosine,
    v.slotWallMinimumDepthFraction, v.slotWallMinimumPoints, v.slotWallDistanceMultiplier,
    v.minimumSlotSupportedEdges, v.maximumInteriorSections, v.interiorSectionEndMultiplier,
    v.interiorSectionMergeMultiplier, v.searchBeamWidth, v.minimumScoreImprovement, v.minimumInlierFraction,
    v.maximumP95Multiplier, v.maximumUnsupportedFaceFraction, v.scoreMaximumDistanceLoss, v.scoreNormalWeight,
    v.scoreFaceSupportNormalCosine, v.scoreMinimumFacePoints, v.scoreOperationPenalty, v.scoreCurvePenalty,
    v.minimumRingAreaMultiplier, v.cylinderOpeningClearanceMultiplier, v.cylinderMergeToleranceMultiplier,
    v.cylinderMergeSpacingMultiplier, v.throughHoleMinimumRadiusMultiplier,
    v.throughHoleMaximumResidualMultiplier, v.holeOpeningOuterRadiusMultiplier, v.cutOvertravelMultiplier,
    v.projectionDepthPaddingMultiplier, v.minimumBooleanVolumeMultiplier)

CACHE_FIELDS(reconstruction::ProjectionSketchInput, v.plane, v.section, v.points, v.alphaRadius,
    v.simplificationTolerance, v.firstPointId, v.regionSupport, v.automaticAlphaScale)

CACHE_FIELDS(reconstruction::ProjectionOperation, v.sketch, v.operation, v.depth, v.reverse,
    v.supportPointIds)

CACHE_FIELDS(reconstruction::ProjectionFitQuality, v.scoredPoints, v.medianDistance, v.p95Distance,
    v.surfaceInlierFraction, v.unsupportedFaceAreaFraction, v.meanNormalErrorDegrees, v.normalComparisons,
    v.score)

CACHE_FIELDS(reconstruction::ProjectionAxisReport, v.axis, v.source, v.convexProjectionArea,
    v.convexEnvelopeVolume, v.status, v.bestBaseQuality)

CACHE_FIELDS(reconstruction::ProjectionExtrusionResult, v.distanceTolerance, v.axes, v.operations, v.quality,
    v.residualPointIds, v.reason, v.bestAxisIndex, v.rejectedOperations, v.sketchInputs)

CACHE_FIELDS(modeling::Pose3d, v.origin, v.xDirection, v.zDirection)

CACHE_FIELDS(modeling::BoxPrimitiveParams, v.pose, v.sizeX, v.sizeY, v.sizeZ, v.join)

CACHE_FIELDS(modeling::ThroughHolePrimitiveParams, v.axisPoint, v.axisDirection, v.radius)

CACHE_FIELDS(modeling::SpherePrimitiveParams, v.center, v.radius, v.join)

CACHE_FIELDS(modeling::CylinderPrimitiveParams, v.baseCenter, v.axisDirection, v.radius, v.height, v.join)

CACHE_FIELDS(modeling::ConePrimitiveParams, v.baseCenter, v.axisDirection, v.baseRadius, v.topRadius,
    v.height, v.join)

CACHE_FIELDS(modeling::TorusPrimitiveParams, v.center, v.axisDirection, v.majorRadius, v.minorRadius, v.join)

CACHE_FIELDS(modeling::FaceReference, v.ownerFeature, v.role, v.normal, v.centroid, v.area,
    v.transientFaceIndexHint)

CACHE_FIELDS(modeling::DatumPlaneReference, v.plane)

CACHE_FIELDS(modeling::OffsetDatumPlane, v.base, v.offset)

CACHE_FIELDS(modeling::Plane3d, v.origin, v.normal, v.xDirection)

CACHE_FIELDS(modeling::Vec3, v.x, v.y, v.z)

CACHE_FIELDS(modeling::Point2D, v.x, v.y)

CACHE_FIELDS(modeling::Line2D, v.x1, v.y1, v.x2, v.y2)

CACHE_FIELDS(modeling::Rectangle2D, v.x, v.y, v.width, v.height)

CACHE_FIELDS(modeling::Circle2D, v.x, v.y, v.radius)
CACHE_FIELDS(modeling::Arc2D, v.start, v.middle, v.end)

CACHE_FIELDS(modeling::SketchEntity, v.id, v.geometry)

CACHE_FIELDS(modeling::SketchFeatureParams, v.plane, v.entities)

CACHE_FIELDS(modeling::SplineCurve2D, v.degree, v.poles, v.weights, v.knots, v.multiplicities)

CACHE_FIELDS(modeling::BoundaryEdgeUse, v.edgeId, v.reversed, v.curve)

CACHE_FIELDS(modeling::BoundarySurface, v.kind, v.pose, v.radius, v.minorRadius, v.semiAngle, v.uDegree,
    v.vDegree, v.uCount, v.vCount, v.poles, v.weights, v.uKnots, v.vKnots, v.uMultiplicities,
    v.vMultiplicities)

CACHE_FIELDS(modeling::BoundaryFace, v.surface, v.reversed, v.wires)

CACHE_FIELDS(modeling::BoundaryBody, v.faces, v.tolerance)

CACHE_FIELDS(modeling::ProfileCurve2D, v.circular, v.x1, v.y1, v.x2, v.y2, v.centerX, v.centerY, v.radius,
    v.startAngle, v.sweepAngle)

CACHE_FIELDS(modeling::AxialSection, v.centerX, v.centerY, v.start, v.end, v.startRadius, v.endRadius,
    v.innerStartRadius, v.innerEndRadius, v.centerZ, v.axis)

CACHE_FIELDS(modeling::ProfileEndBevel, v.depth, v.inset)

CACHE_FIELDS(modeling::RoundedEndProfile, v.center, v.straightLength, v.radius, v.bottom, v.top)

CACHE_FIELDS(modeling::ProfileRimRound, v.curveIndex, v.radius, v.top, v.trimPlane)

CACHE_FIELDS(modeling::ProfileTrimPlane, v.point, v.outward)

CACHE_FIELDS(modeling::ProfileBodyParams, v.pose, v.outline, v.profileVoids, v.envelope, v.cavities, v.depth,
    v.referenceSizeX, v.referenceSizeY, v.sizeX, v.sizeY, v.sizeZ, v.boundary, v.roundedEnd, v.rimRounds,
    v.trimPlanes)

#undef CACHE_FIELDS

template<class T> struct Vector : std::false_type {};
template<class T> struct Vector<std::vector<T>> : std::true_type {};
template<class T> struct Optional : std::false_type {};
template<class T> struct Optional<std::optional<T>> : std::true_type {};
template<class T> struct Shared : std::false_type {};
template<class T> struct Shared<std::shared_ptr<T>> : std::true_type {};
template<class T> struct Variant : std::false_type {};
template<class... T> struct Variant<std::variant<T...>> : std::true_type {};
template<class T, class = void> struct Tuple : std::false_type {};
template<class T> struct Tuple<T, std::void_t<decltype(std::tuple_size<T>::value)>> : std::true_type {};

class Writer {
public:
    explicit Writer(QByteArray& bytes) : m_stream(&bytes, QIODevice::WriteOnly) {
        m_stream.setVersion(QDataStream::Qt_6_0);
        m_stream.setByteOrder(QDataStream::LittleEndian);
    }

    template<class... T> void operator()(const T&... value) { (write(value), ...); }

private:
    template<class T> void write(const T& value) {
        if constexpr (std::is_same_v<T, bool>)
            m_stream << quint8(value);
        else if constexpr (std::is_integral_v<T> || std::is_enum_v<T>)
            m_stream << quint64(value);
        else if constexpr (std::is_floating_point_v<T>)
            m_stream << double(value);
        else if constexpr (std::is_same_v<T, QByteArray>) {
            m_stream << quint64(value.size());
            m_stream.writeRawData(value.constData(), value.size());
        } else if constexpr (std::is_same_v<T, std::string>) {
            m_stream << quint64(value.size());
            m_stream.writeRawData(value.data(), value.size());
        } else if constexpr (Vector<T>::value) {
            m_stream << quint64(value.size());
            for (const auto& item : value) write(item);
        } else if constexpr (Optional<T>::value || Shared<T>::value) {
            write(bool(value));
            if (value) write(*value);
        } else if constexpr (Variant<T>::value) {
            m_stream << quint64(value.index());
            std::visit([&](const auto& item) { write(item); }, value);
        } else if constexpr (Tuple<T>::value) {
            std::apply([&](const auto&... item) { (*this)(item...); }, value);
        } else transfer(*this, value);
    }

    QDataStream m_stream;
};

class Reader {
public:
    explicit Reader(const QByteArray& bytes) : m_stream(bytes) {
        m_stream.setVersion(QDataStream::Qt_6_0);
        m_stream.setByteOrder(QDataStream::LittleEndian);
    }

    template<class... T> void operator()(T&... value) { (read(value), ...); }
    bool complete() const { return m_stream.status() == QDataStream::Ok && m_stream.atEnd(); }

private:
    void corrupt() { m_stream.setStatus(QDataStream::ReadCorruptData); }

    quint64 count() {
        quint64 n = 0;
        m_stream >> n;
        if (n > quint64(m_stream.device()->bytesAvailable())) {
            corrupt();
            return 0;
        }
        return n;
    }

    template<std::size_t Index = 0, class T> void readVariant(quint64 type, T& value) {
        if constexpr (Index == std::variant_size_v<T>) corrupt();
        else if (type == Index) read(value.template emplace<Index>());
        else readVariant<Index + 1>(type, value);
    }

    template<class T> void read(T& value) {
        if (m_stream.status() != QDataStream::Ok) return;
        if constexpr (std::is_same_v<T, bool>) {
            quint8 flag = 0;
            m_stream >> flag;
            if (flag > 1) corrupt();
            value = flag != 0;
        } else if constexpr (std::is_integral_v<T> || std::is_enum_v<T>) {
            quint64 bits = 0;
            m_stream >> bits;
            value = static_cast<T>(bits);
        } else if constexpr (std::is_floating_point_v<T>) {
            double number = 0;
            m_stream >> number;
            value = static_cast<T>(number);
        } else if constexpr (std::is_same_v<T, QByteArray> || std::is_same_v<T, std::string>) {
            const auto size = count();
            if (m_stream.status() != QDataStream::Ok) return;
            value.resize(size);
            if (size && m_stream.readRawData(value.data(), size) != qint64(size)) corrupt();
        } else if constexpr (Vector<T>::value) {
            const auto size = count();
            value.clear();
            for (quint64 i = 0; i < size && m_stream.status() == QDataStream::Ok; ++i)
                read(value.emplace_back());
        } else if constexpr (Optional<T>::value || Shared<T>::value) {
            bool present = false;
            read(present);
            value.reset();
            if (!present) return;
            if constexpr (Optional<T>::value) read(value.emplace());
            else {
                auto object = std::make_shared<std::remove_const_t<typename T::element_type>>();
                read(*object);
                value = std::move(object);
            }
        } else if constexpr (Variant<T>::value) {
            quint64 type = 0;
            m_stream >> type;
            readVariant(type, value);
        } else if constexpr (Tuple<T>::value) {
            std::apply([&](auto&... item) { (*this)(item...); }, value);
        } else transfer(*this, value);
    }

    QDataStream m_stream;
};

template<class... T> QByteArray encode(const T&... value) {
    QByteArray bytes;
    Writer writer(bytes);
    writer(value...);
    return bytes;
}

template<class... T> bool decode(const QByteArray& bytes, T&... value) {
    Reader reader(bytes);
    reader(value...);
    return reader.complete();
}

} // namespace reconstruction_cache
