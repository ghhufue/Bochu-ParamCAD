#include "reconstruction/recognition/RecognitionAnalysisOptions.h"
#include "io/PointCloudAnalysis.h"
#include <reconstruction/surface/SurfaceSupportValidation.h>
#include <reconstruction/surface/SurfaceOwnership.h>
#include "io/PointCloudAdapter.h"
#include "io/ReconstructionCache.h"
#include <reconstruction/surface/PlaneSupportRecovery.h>

#include <QJsonArray>
#include <QThread>
#include <reconstruction/segmentation/PrimitiveDetector.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <QCoreApplication>

namespace {
using Vec = modeling::Vec3;
QJsonArray array(const Vec& p) { return {p.x, p.y, p.z}; }
PointCloudRegion describe(const reconstruction::PrimitiveEvidence& evidence, int index)
{
    PointCloudRegion region;
    region.source = "detected";
    std::visit([&](const auto& surface) {
        using T = std::decay_t<decltype(surface)>;
        region.residual = surface.meanAbsoluteResidual;
        region.confidence = std::clamp(surface.confidence, 0.0, 1.0);
        if constexpr (std::is_same_v<T, reconstruction::PlaneEvidence>) {
            region.geometry = "plane"; region.name = QCoreApplication::translate("PointCloudAnalysis", "平面 %1").arg(index);
            region.parameters = {{"normal", array(surface.plane.normal)}, {"offset", surface.plane.offset}};
        } else if constexpr (std::is_same_v<T, reconstruction::SphereEvidence>) {
            region.geometry = "sphere"; region.name = QCoreApplication::translate("PointCloudAnalysis", "球面 %1").arg(index);
            region.parameters = {{"center", array(surface.sphere.center)}, {"radius", surface.sphere.radius}};
        } else if constexpr (std::is_same_v<T, reconstruction::CylinderEvidence>) {
            region.geometry = "cylinder"; region.name = QCoreApplication::translate("PointCloudAnalysis", "圆柱面 %1").arg(index);
            region.parameters = {{"center", array(surface.cylinder.axisPoint)}, {"axis", array(surface.cylinder.axisDirection)},
                {"radius", surface.cylinder.radius}, {"axial_min", surface.axialMinimum}, {"axial_max", surface.axialMaximum}};
        } else if constexpr (std::is_same_v<T, reconstruction::ConeEvidence>) {
            region.geometry = "cone"; region.name = QCoreApplication::translate("PointCloudAnalysis", "圆锥面 %1").arg(index);
            region.parameters = {{"center", array(surface.cone.apex)}, {"axis", array(surface.cone.axisDirection)},
                {"angle", surface.cone.openingAngleRadians}, {"axial_min", surface.axialMinimum}, {"axial_max", surface.axialMaximum}};
        } else {
            region.geometry = "ring"; region.name = QCoreApplication::translate("PointCloudAnalysis", "环面 %1").arg(index);
            region.parameters = {{"center", array(surface.torus.center)}, {"axis", array(surface.torus.axisDirection)},
                {"major_radius", surface.torus.majorRadius}, {"minor_radius", surface.torus.minorRadius}};
        }
    }, evidence);
    return region;
}

template<class SourceRow>
std::shared_ptr<const SphereSourceTriangles> validatedSphereTriangles(
    const reconstruction::SurfaceSupportValidationReport& report, reconstruction::SurfaceId proposal,
    SourceRow sourceRow)
{
    std::shared_ptr<SphereSourceTriangles> triangles;
    for (const auto& decision : report.decisions) {
        if (decision.proposalId != proposal || decision.kind != reconstruction::PrimitiveKind::Sphere ||
            decision.verdict != reconstruction::SupportVerdict::Accepted || !decision.footprint) continue;
        if (!triangles) triangles = std::make_shared<SphereSourceTriangles>();
        const auto& footprint = *decision.footprint;
        for (const auto& indices : footprint.triangles) {
            if (std::any_of(indices.begin(), indices.end(), [&](auto index) {
                return index >= footprint.sourcePointIds.size();
            })) continue;
            const std::array<vtkIdType, 3> rows{sourceRow(footprint.sourcePointIds[indices[0]]),
                sourceRow(footprint.sourcePointIds[indices[1]]), sourceRow(footprint.sourcePointIds[indices[2]])};
            if (rows[0] >= 0 && rows[1] >= 0 && rows[2] >= 0 &&
                rows[0] != rows[1] && rows[1] != rows[2] && rows[2] != rows[0]) triangles->push_back(rows);
        }
    }
    return triangles;
}
}

PointCloudAnalysisResult analysePointCloud(const PlyCloud& cloud,
    const std::vector<vtkIdType>& originalIds, double distanceTolerance, const QString& geometry,
    reconstruction::LengthUnit unit)
{
    PointCloudAnalysisResult result;
    if (cloud.positions.size() < 20 || cloud.positions.size() != originalIds.size()) {
        result.error = QCoreApplication::translate("PointCloudAnalysis", "至少需要 20 个有效点进行表面识别。"); return result;
    }
    if (!std::isfinite(distanceTolerance) || distanceTolerance <= 0) {
        result.error = QCoreApplication::translate("PointCloudAnalysis", "表面识别距离容差必须大于零。"); return result;
    }
    const auto source = PointCloudAdapter::toPointStore(cloud, unit);
    const auto cancelled=[] { return QThread::currentThread()->isInterruptionRequested(); };
    ReconstructionCache cache(cancelled);
    auto preprocessing = PointCloudAdapter::preprocessingOptionsFor(cloud);
    preprocessing.convertKnownUnitsToMillimeters = false;
    // A scene-wide voxel can exceed a small hole's diameter. Analysis resolution
    // follows the requested fit tolerance, and density differences between faces
    // are not evidence of outliers. RANSAC still requires supported surfaces.
    preprocessing.voxelSize=std::min(preprocessing.voxelSize,distanceTolerance*0.5);
    if(geometry.isEmpty())preprocessing.removeStatisticalOutliers=false;
    if (cloud.positions.size() < 50000) preprocessing.voxelSize = 0;
    reconstruction::PointStore points({}, unit);
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    if (!cache.preprocess(reconstruction::ObservationSnapshot(source, false), preprocessing, points, report, error)) {
        result.error = QString::fromStdString(error); return result;
    }
    result.processedPoints = points.size();
    if (QThread::currentThread()->isInterruptionRequested()) { result.error = QCoreApplication::translate("PointCloudAnalysis", "识别已取消。"); return result; }
    const auto detectionSettings = reconstruction::loadDetectionSettings();
    const auto analysisSettings = reconstruction::loadAnalysisSettings();
    const auto recognitionSettings = reconstruction::loadRecognitionSettings();
    reconstruction::PrimitiveDetectionOptions options = detectionSettings.options;
    options.plane = PointCloudAdapter::planeOptionsFor(points, detectionSettings);
    options.cylinder = PointCloudAdapter::cylinderOptionsFor(points, detectionSettings);
    const size_t curvedSupport=std::max(detectionSettings.curvedSupportFloor,
        std::min(detectionSettings.curvedSupportCeiling,points.size()/std::max<size_t>(1,detectionSettings.curvedSupportDivisor)));
    options.plane.distanceThreshold = distanceTolerance;
    options.plane.maximumPlanes = geometry.isEmpty() ? options.plane.maximumPlanes : 1;
    options.cylinder.distanceThreshold = distanceTolerance;
    options.cylinder.minimumRadius = distanceTolerance * detectionSettings.cylinderMinimumRadiusToleranceMultiplier;
    options.cylinder.maximumCylinders = geometry.isEmpty() ? options.cylinder.maximumCylinders : 1;
    options.sphere.distanceThreshold = distanceTolerance;
    if(!options.sphere.minimumSupportPoints)options.sphere.minimumSupportPoints=curvedSupport;
    options.sphere.maximumSpheres = geometry.isEmpty() ? options.sphere.maximumSpheres : 1;
    options.cone.distanceThreshold = distanceTolerance;
    if(!options.cone.minimumSupportPoints)options.cone.minimumSupportPoints=curvedSupport;
    options.cone.maximumCones = geometry.isEmpty() ? options.cone.maximumCones : 1;
    options.torus.distanceThreshold = distanceTolerance;
    if(!options.torus.minimumSupportPoints)options.torus.minimumSupportPoints=curvedSupport;
    options.torus.maximumTori = geometry.isEmpty() ? options.torus.maximumTori : 1;
    if (!geometry.isEmpty()) {
        options.detectPlanes = geometry == "plane"; options.detectCylinders = geometry == "cylinder";
        options.detectSpheres = geometry == "sphere"; options.detectCones = geometry == "cone";
        options.detectTori = geometry == "ring";
    } else options = reconstruction::recognitionAnalysisOptions(
        points,options.plane,options.cylinder,recognitionSettings.analysis).detection;
    // An explicit detector minimum must survive the automatic analysis quotas.
    const auto minimum=[&](auto& family,const auto& configured) {
        if(configured.minimumSupportPoints)family.minimumSupportPoints=configured.minimumSupportPoints;
        family.minimumSupportPoints=std::max(family.minimumSupportPoints,analysisSettings.meshing.minimumSurfacePoints);
    };
    minimum(options.plane,detectionSettings.options.plane);minimum(options.cylinder,detectionSettings.options.cylinder);
    minimum(options.sphere,detectionSettings.options.sphere);minimum(options.cone,detectionSettings.options.cone);
    minimum(options.torus,detectionSettings.options.torus);
    const auto seedText=qEnvironmentVariable("PCAD_RANSAC_SEED");
    if(!seedText.isEmpty()) {
        bool valid=false;options.randomSeed=seedText.toUInt(&valid);
        if(!valid){result.error="Invalid PCAD_RANSAC_SEED";return result;}
    }
    reconstruction::SurfaceAnalysisOptions analysisOptions;
    analysisOptions.detection=options;
    analysisOptions.supportValidation=analysisSettings.supportValidation;
    analysisOptions.supportMeshing=analysisSettings.meshing;
    analysisOptions.recoverResidualSurfaces=geometry.isEmpty() || analysisSettings.recoverResidualSurfaces;
    analysisOptions.connectivityDistance=analysisSettings.connectivityDistance;
    analysisOptions.independentFamilies=analysisSettings.independentFamilies;
    analysisOptions.familyDetectionWorkerCount=analysisSettings.familyDetectionWorkerCount;
    analysisOptions.relations.distanceTolerance=distanceTolerance*analysisSettings.relationDistanceToleranceMultiplier;
    analysisOptions.relations.adjacentConfidence=analysisSettings.adjacentConfidence;
    analysisOptions.relations.maximumConfidence=analysisSettings.maximumConfidence;
    analysisOptions.relations.derivedConfidenceFraction=analysisSettings.derivedConfidenceFraction;
    analysisOptions.relations.manualPatchOverlapFraction=analysisSettings.manualPatchOverlapFraction;
    analysisOptions.relations.optimizationMaximumAttempts=analysisSettings.optimizationMaximumAttempts;
    analysisOptions.relations.optimizationMaximumActiveSurfaces=analysisSettings.optimizationMaximumActiveSurfaces;
    analysisOptions.relations.optimizationResidualSampleTarget=analysisSettings.optimizationResidualSampleTarget;
    analysisOptions.relations.constraintWeight=analysisSettings.constraintWeight;
    analysisOptions.relations.optimizationInlierFraction=analysisSettings.optimizationInlierFraction;
    analysisOptions.relations.maximumMeanErrorIncreaseMultiplier=analysisSettings.maximumMeanErrorIncreaseMultiplier;
    analysisOptions.relations.validationToleranceFraction=analysisSettings.validationToleranceFraction;
    analysisOptions.relations.optimizationIterations=analysisSettings.optimizationIterations;
    analysisOptions.relations.initialDamping=analysisSettings.initialDamping;
    analysisOptions.relations.convergenceStep=analysisSettings.convergenceStep;
    analysisOptions.relations.supportNormalCosine=analysisSettings.supportNormalCosine;
    analysisOptions.minimumRecoveryRegionPoints=analysisSettings.minimumRecoveryRegionPoints;
    analysisOptions.recoveryHaloToleranceMultiplier=analysisSettings.recoveryHaloToleranceMultiplier;
    analysisOptions.recoveryHaloRelationMultiplier=analysisSettings.recoveryHaloRelationMultiplier;
    analysisOptions.stablePlaneOverlapFraction=analysisSettings.stablePlaneOverlapFraction;
    analysisOptions.stablePlaneNormalCosine=analysisSettings.stablePlaneNormalCosine;
    analysisOptions.recoveryConnectivityToleranceMultiplier=analysisSettings.recoveryConnectivityToleranceMultiplier;
    analysisOptions.recoveryConnectivityAdaptiveMultiplier=analysisSettings.recoveryConnectivityAdaptiveMultiplier;
    reconstruction::SurfaceAnalysisResult analysis;
    if (!cache.analyze(points, analysisOptions, analysis, error)) {
        result.error = QString::fromStdString(error); return result;
    }
    result.analysis = analysis;
    // Grow on the original observations as well: a one-shot halo around a
    // downsampled seed cannot recover a long connected face. Keep the processed
    // analysis fingerprint intact for subsequent CAD cache validation.
    auto displayEvidence=analysis.supportProposals.empty()?analysis.surfaces:analysis.supportProposals;
    if(!reconstruction::recoverPlaneSupport(source,options.plane,displayEvidence,error,{},cancelled)) {
        result.error=QString::fromStdString(error);return result;
    }
    reconstruction::SurfaceAnalysisResult displayAnalysis;
    if(!reconstruction::indexDetectedSurfaceEvidence(source,displayEvidence,analysisOptions,displayAnalysis,error,{},cancelled)) {
        result.error=QString::fromStdString(error);return result;
    }
    result.sourceSupportValidation=displayAnalysis.supportValidation;
    if(!reconstruction::revalidateSurfaceSupportOnSource(points,source,result.analysis,error,cancelled,&displayAnalysis)) {
        result.error=QString::fromStdString(error);return result;
    }
    reconstruction::SurfaceOwnershipOptions ownershipOptions;
    ownershipOptions.distanceTolerance=distanceTolerance;
    ownershipOptions.supportRadius=std::max(preprocessing.voxelSize*3.0,distanceTolerance*4.0);
    ownershipOptions.useSourceNormals=cloud.normals.size()==cloud.positions.size();
    reconstruction::SurfaceOwnershipResult ownership;
    if(!reconstruction::resolveSurfaceOwnership(source,displayAnalysis,ownershipOptions,ownership,error,cancelled)) {
        result.error=QString::fromStdString(error);return result;
    }
    result.finalSupportValidation=std::move(ownership.finalSupportValidation);
    result.assignedPoints=ownership.assignedPoints;
    result.preferredPlaneCount=ownership.preferredPlaneCount;
    if(ownership.requiresFreshAnalysis)result.analysis.pointFingerprint=0;
    std::unordered_map<reconstruction::PointId,vtkIdType> originalBySource;
    for(size_t i=0;i<cloud.positions.size();++i) {
        const auto id=cloud.sourceRows.empty()?reconstruction::PointId(i+1):reconstruction::PointId(cloud.sourceRows[i]+1);
        originalBySource.emplace(id,originalIds[i]);
    }
    const auto& finalReport=result.finalSupportValidation;
    for(size_t j=0;j<ownership.surfaces.size();++j) {
        const auto& evidence=ownership.surfaces[j];
        const auto label=analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Enforce?j+1:
            std::visit([](const auto& e){return std::size_t(e.id);},evidence);
        auto region=describe(evidence,static_cast<int>(label));
        if(analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Enforce) {
            region.supportValidation={{"verdict","accepted"},
                {"reference_spacing",finalReport.reference.available?finalReport.reference.spacing:0},
                {"density_ratio_max",analysisOptions.supportValidation.planeDensityRatioMax}};
            const auto proposalId=std::visit([](const auto& e){return e.id;},evidence);
            if (region.geometry == "sphere") {
                region.validatedSphereTriangles = validatedSphereTriangles(finalReport, proposalId, [&](auto id) {
                    const auto point = originalBySource.find(id);
                    return point == originalBySource.end() ? vtkIdType(-1) : point->second;
                });
            }
            const reconstruction::SurfaceSupportDecision* dominant=nullptr;
            for(const auto& d:finalReport.decisions)
                if(d.proposalId==proposalId && d.verdict==reconstruction::SupportVerdict::Accepted && (!dominant || d.area>dominant->area))dominant=&d;
            if(dominant) {
                const auto& d=*dominant;
                region.supportValidation["proposal_id"]=qint64(d.proposalId);region.supportValidation["patch_id"]=qint64(d.patchId);
                region.supportValidation["density_ratio"]=d.densityRatio;
                region.supportValidation["density_evaluated"]=d.densityEvaluated;
                region.supportValidation["solid_angle_sr"]=d.solidAngle;
                region.supportValidation["minor_span_deg"]=d.minorSpanRadians*180/std::acos(-1.);
                region.supportValidation["angular_span_deg"]=d.angularSpanRadians*180/std::acos(-1.);
            }
        }
        std::vector<vtkIdType> members;
        std::visit([&](const auto& e){for(auto id:e.supportPointIds)members.push_back(originalBySource.at(id));},evidence);
        result.surfaces.push_back({std::move(region),std::move(members)});
    }
    result.cache = cache.report();
    return result;
}

PointCloudRegion describeTemplateFit(const reconstruction::TemplateFit& fit, double scale)
{
    PointCloudRegion region;
    if (fit.surface) region=describe(*fit.surface,fit.regionId);
    if (fit.box) {
        const auto& p=fit.box->primitive;
        region.geometry="cuboid"; region.confidence=fit.box->confidence;
        region.parameters={{"origin",array(p.pose.origin)},{"x_direction",array(p.pose.xDirection)},
            {"z_direction",array(p.pose.zDirection)},{"size_x",p.sizeX},{"size_y",p.sizeY},{"size_z",p.sizeZ}};
    }
    // Fits run in CAD millimetres; annotation equations stay in PLY units.
    for (const auto& key : {"origin","center"}) if(region.parameters.contains(key)) {
        auto values=region.parameters[key].toArray();
        for(int i=0;i<values.size();++i) values[i]=values[i].toDouble()/scale;
        region.parameters[key]=values;
    }
    for (const auto& key : {"offset","radius","major_radius","minor_radius","axial_min","axial_max",
                           "size_x","size_y","size_z"}) if(region.parameters.contains(key))
        region.parameters[key]=region.parameters[key].toDouble()/scale;
    region.residual/=scale;
    return region;
}

std::vector<DetectedPointCloudSurface> describeAnalysisPatches(
    const reconstruction::SurfaceAnalysisResult& analysis, std::size_t sourcePointCount, double scale,
    bool ownedPreview)
{
    std::vector<DetectedPointCloudSurface> result;
    const auto minimum=reconstruction::loadAnalysisSettings().meshing.minimumSurfacePoints;
    for (const auto& patch : analysis.patches) {
        if (!patch.surfaceId || patch.surfaceId > analysis.surfaces.size()) continue;
        reconstruction::TemplateFit fit;
        fit.regionId = int(patch.id);
        fit.surface = analysis.surfaces[patch.surfaceId - 1];
        DetectedPointCloudSurface surface;
        surface.surface = describeTemplateFit(fit, scale);
        surface.surface.parameters["orientation_known"] = patch.orientationKnown;
        for (auto id : patch.supportPointIds)
            if (id > 0 && id <= sourcePointCount) surface.points.push_back(vtkIdType(id - 1));
        std::sort(surface.points.begin(),surface.points.end());
        surface.points.erase(std::unique(surface.points.begin(),surface.points.end()),surface.points.end());
        if (surface.points.size() < minimum) continue;
        if (ownedPreview && surface.surface.geometry == "sphere" && patch.surfaceId <= analysis.supportProposalIds.size()) {
            const auto proposal = analysis.supportProposalIds[patch.surfaceId - 1];
            surface.surface.validatedSphereTriangles = validatedSphereTriangles(analysis.supportValidation, proposal, [&](auto id) {
                if (id == 0 || id > sourcePointCount) return vtkIdType(-1);
                const auto row = static_cast<vtkIdType>(id - 1);
                return std::binary_search(surface.points.begin(), surface.points.end(), row) ? row : vtkIdType(-1);
            });
        }
        result.push_back(std::move(surface));
    }
    std::stable_sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        if ((a.surface.geometry == "plane") != (b.surface.geometry == "plane")) return a.surface.geometry == "plane";
        return a.points.size() > b.points.size();
    });
    return result;
}
