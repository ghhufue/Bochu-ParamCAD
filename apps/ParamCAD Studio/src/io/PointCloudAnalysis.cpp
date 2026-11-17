#include "io/PointCloudAnalysis.h"
#include <reconstruction/analysis/SurfaceSupportValidation.h>
#include "io/PointCloudAdapter.h"
#include <reconstruction/detection/PlaneSupportRecovery.h>

#include <QJsonArray>
#include <QThread>
#include <reconstruction/detection/PrimitiveDetector.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStaticPointLocator.h>

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
    if (!reconstruction::preprocessPointCloud(source, preprocessing, points, report, error)) {
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
    if (!reconstruction::analyzeSurfaces(points, analysisOptions, analysis, error, {}, [] {
            return QThread::currentThread()->isInterruptionRequested(); })) {
        result.error = QString::fromStdString(error); return result;
    }
    result.analysis = analysis;
    // Grow on the original observations as well: a one-shot halo around a
    // downsampled seed cannot recover a long connected face. Keep the processed
    // analysis fingerprint intact for subsequent CAD cache validation.
    auto displayEvidence=analysis.supportProposals.empty()?analysis.surfaces:analysis.supportProposals;
    const auto cancelled=[] { return QThread::currentThread()->isInterruptionRequested(); };
    if(!reconstruction::recoverPlaneSupport(source,options.plane,displayEvidence,error,{},cancelled)) {
        result.error=QString::fromStdString(error);return result;
    }
    reconstruction::SurfaceAnalysisResult displayAnalysis;
    if(!reconstruction::indexSurfaceEvidence(source,displayEvidence,analysisOptions,displayAnalysis,error,{},cancelled)) {
        result.error=QString::fromStdString(error);return result;
    }
    result.sourceSupportValidation=displayAnalysis.supportValidation;
    if(!reconstruction::revalidateSurfaceSupportOnSource(points,source,result.analysis,error,cancelled,&displayAnalysis)) {
        result.error=QString::fromStdString(error);return result;
    }
    // Display connected patches independently, even when they share an equation.
    reconstruction::PrimitiveDetectionResult detection;
    for (const auto& patch : displayAnalysis.patches) {
        auto evidence=displayAnalysis.surfaces.at(static_cast<size_t>(patch.surfaceId-1));
        std::visit([&](auto& surface) { surface.supportPointIds=patch.supportPointIds; },evidence);
        detection.evidence.push_back(std::move(evidence));
    }
    std::unordered_map<reconstruction::PointId, const reconstruction::PointSample*> positionById;
    positionById.reserve(source.size());
    for (const auto& point : source.points()) positionById.emplace(point.id, &point);
    std::vector<vtkSmartPointer<vtkStaticPointLocator>> supports;
    std::vector<std::unordered_set<reconstruction::PointId>> planeMembers(detection.evidence.size());
    for(size_t j=0;j<detection.evidence.size();++j)
        if(const auto* plane=std::get_if<reconstruction::PlaneEvidence>(&detection.evidence[j]))
            planeMembers[j].insert(plane->supportPointIds.begin(),plane->supportPointIds.end());
    // Protect supported broad planes as regions before per-point residual competition.
    // A curved proposal spanning several faces cannot steal each face's interior.
    std::vector<bool> preferredPlane(detection.evidence.size());
    std::vector<std::vector<std::unique_ptr<reconstruction::SurfaceFootprintQuery>>> planeDomains(detection.evidence.size());
    if(analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Enforce && displayAnalysis.supportValidation.reference.available) {
        const auto& validation=displayAnalysis.supportValidation;
        std::vector<double> areas;
        for(auto id:validation.reference.patchIds)for(const auto& d:validation.decisions)if(d.patchId==id)areas.push_back(d.area);
        std::sort(areas.begin(),areas.end());
        const double areaFloor=areas.empty()?INFINITY:areas[areas.size()/2]*analysisOptions.supportValidation.planePreferenceMinReferenceAreaFraction;
        for(size_t j=0;j<detection.evidence.size();++j)if(!planeMembers[j].empty()) {
            const auto surfaceId=displayAnalysis.patches[j].surfaceId;
            if(!surfaceId || surfaceId>displayAnalysis.supportProposalIds.size())continue;
            const auto origin=displayAnalysis.supportProposalIds[size_t(surfaceId-1)];
            for(const auto& d:validation.decisions)if(d.proposalId==origin && d.verdict==reconstruction::SupportVerdict::Accepted &&
                d.densityEvaluated && d.effectivePoints>=analysisOptions.supportValidation.referenceMinPoints && d.finalArea>=areaFloor) {
                size_t shared=0;for(auto id:d.supportPointIds)shared+=planeMembers[j].count(id);
                if(shared>=planeMembers[j].size()*.9 && d.footprint){
                    preferredPlane[j]=true;
                    planeDomains[j].push_back(std::make_unique<reconstruction::SurfaceFootprintQuery>(*d.footprint,
                        std::max(1e-12,validation.reference.spacing*1e-9)));
                }
            }
        }
    }
    result.preferredPlaneCount=std::count(preferredPlane.begin(),preferredPlane.end(),true);
    const auto supportedByPlane=[&](size_t index,reconstruction::PointId id,const Vec& point) {
        if(planeMembers[index].count(id))return true;
        if(!preferredPlane[index] || reconstruction::surfaceResidual(detection.evidence[index],point)>distanceTolerance)return false;
        const auto sample=positionById.find(id);
        const auto material=displayAnalysis.patches[index].materialNormal;
        const double materialLength=std::hypot(material.x,material.y,material.z);
        if(sample!=positionById.end() && sample->second->normal && sample->second->normalOrientationKnown && sample->second->confidence>=.5 && materialLength>.25) {
            const auto n=*sample->second->normal;const double magnitude=std::hypot(n.x,n.y,n.z);
            if(magnitude>1e-12 && (n.x*material.x+n.y*material.y+n.z*material.z)/(magnitude*materialLength)<-.5)return false;
        }
        for(const auto& domain:planeDomains[index])if(domain->containsProjection(point))return true;
        return false;
    };
    // Retain typed evidence in the core, but resolve equivalent regions before
    // assigning individual points. A complete plane must not lose its interior
    // to smaller coplanar or curved alternatives with slightly lower residuals.
    std::vector<bool> redundant(detection.evidence.size());
    for(size_t j=0;j<detection.evidence.size();++j) {
        const auto& ids=std::visit([](const auto& s)->const std::vector<reconstruction::PointId>& {return s.supportPointIds;},detection.evidence[j]);
        for(size_t k=0;k<planeMembers.size();++k) {
            const auto& plane=planeMembers[k];
            if(k==j || plane.empty() || ids.empty() || plane.size()<ids.size()) continue;
            if(!planeMembers[j].empty()) {
                if(plane.size()==ids.size() && k>j) continue;
                const auto a=std::get<reconstruction::PlaneEvidence>(detection.evidence[j]).plane.normal;
                const auto b=std::get<reconstruction::PlaneEvidence>(detection.evidence[k]).plane.normal;
                if(std::abs(a.x*b.x+a.y*b.y+a.z*b.z)<std::cos(.03490658503988659)) continue;
            }
            size_t shared=0;for(auto id:ids) shared+=plane.count(id);
            if(shared>=ids.size()*.98) {redundant[j]=true;break;}
        }
    }
    std::vector<std::vector<vtkIdType>> owners(detection.evidence.size());
    for (const auto& evidence : detection.evidence) {
        vtkNew<vtkPoints> coordinates;
        std::visit([&](const auto& surface) {
            for (const auto id : surface.supportPointIds) {
                const auto found = positionById.find(id);
                if (found == positionById.end()) continue;
                const auto& p = found->second->position;
                coordinates->InsertNextPoint(p.x, p.y, p.z);
            }
        }, evidence);
        vtkNew<vtkPolyData> data; data->SetPoints(coordinates);
        auto locator = vtkSmartPointer<vtkStaticPointLocator>::New();
        locator->SetDataSet(data);
        if (coordinates->GetNumberOfPoints()) locator->BuildLocator();
        supports.push_back(locator);
    }
    // Recover every eligible original observation, constrained to the observed
    // patch so distant coplanar surfaces aren't swallowed by an infinite plane.
    const double supportRadius = std::max(preprocessing.voxelSize*3.0, distanceTolerance*4.0);
    for (size_t i = 0; i < cloud.positions.size(); ++i) {
        if (i % 4096 == 0 && QThread::currentThread()->isInterruptionRequested()) {
            result.error = QCoreApplication::translate("PointCloudAnalysis", "识别已取消。"); return result;
        }
        const auto& coordinate = cloud.positions[i];
        if(!std::isfinite(coordinate[0]) || !std::isfinite(coordinate[1]) || !std::isfinite(coordinate[2])) continue;
        Vec point{coordinate[0], coordinate[1], coordinate[2]};
        const auto pointId=cloud.sourceRows.empty()?reconstruction::PointId(i+1):reconstruction::PointId(cloud.sourceRows[i]+1);
        double best = distanceTolerance*1.5;
        size_t chosen = owners.size();
        for (size_t j = 0; j < detection.evidence.size(); ++j) {
            if(redundant[j]) continue;
            const bool isPlane=!planeMembers[j].empty();
            if(isPlane && !supportedByPlane(j,pointId,point)) continue;
            const auto& patch=displayAnalysis.patches[j];
            if(point.x<patch.lower.x-supportRadius || point.x>patch.upper.x+supportRadius ||
               point.y<patch.lower.y-supportRadius || point.y>patch.upper.y+supportRadius ||
               point.z<patch.lower.z-supportRadius || point.z>patch.upper.z+supportRadius)continue;
            double distance = reconstruction::surfaceResidual(detection.evidence[j], point);
            if(isPlane && analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Enforce && distance>distanceTolerance)continue;
            if(!isPlane && cloud.normals.size()==cloud.positions.size()) {
                const auto& normal=cloud.normals[i];const Vec n{normal[0],normal[1],normal[2]};
                const double magnitude=std::hypot(std::hypot(n.x,n.y),n.z);
                if(std::isfinite(magnitude) && magnitude>1e-12) {
                    const double alignment=reconstruction::surfaceNormalAlignment(detection.evidence[j],point,n);
                    if(alignment<0.9)continue;
                    distance+=distanceTolerance*(1-alignment);
                }
            }
            const bool preferCurrent=chosen!=owners.size() && preferredPlane[chosen] && best<=distanceTolerance;
            const bool preferCandidate=preferredPlane[j] && distance<=distanceTolerance;
            if(preferCurrent && !preferCandidate)continue;
            const bool replaceAlternative=preferCandidate && chosen!=owners.size() && !preferredPlane[chosen];
            if ((distance > best && !replaceAlternative) || supports[j]->GetDataSet()->GetNumberOfPoints() == 0) continue;
            // Near-identical candidates remain in the core pool, but a UI label
            // must not fragment a perfect plane into accidental curved patches.
            if (chosen != owners.size() && !replaceAlternative) {
                const bool bothPlanes=isPlane && !planeMembers[chosen].empty();
                const double slack=distanceTolerance*(bothPlanes?.1:1e-4);
                if(bothPlanes && std::abs(distance-best)<=slack) {
                    if(planeMembers[j].size()<=planeMembers[chosen].size()) continue;
                } else if(distance>=best-slack) continue;
            }
            if(isPlane) {best=distance;chosen=j;continue;}
            double p[3]{point.x,point.y,point.z}, q[3];
            const auto nearest = supports[j]->FindClosestPoint(p);
            supports[j]->GetDataSet()->GetPoint(nearest, q);
            if (std::hypot(std::hypot(p[0]-q[0],p[1]-q[1]),p[2]-q[2]) > supportRadius) continue;
            best = distance; chosen = j;
        }
        if (chosen != owners.size()) owners[chosen].push_back(originalIds[i]);
    }
    if(analysisOptions.supportValidation.mode!=reconstruction::SupportValidationMode::Off) {
        std::unordered_map<vtkIdType,reconstruction::PointId> toSource;
        std::unordered_map<reconstruction::PointId,vtkIdType> toOriginal;
        std::unordered_map<reconstruction::PointId,size_t> toCloud;
        for(size_t i=0;i<cloud.positions.size();++i) {
            const auto id=cloud.sourceRows.empty()?reconstruction::PointId(i+1):reconstruction::PointId(cloud.sourceRows[i]+1);
            toSource.emplace(originalIds[i],id);toOriginal.emplace(id,originalIds[i]);toCloud.emplace(id,i);
        }
        std::vector<reconstruction::PrimitiveEvidence> owned;
        std::vector<size_t> ownedToDetection;
        for(size_t j=0;j<owners.size();++j)if(owners[j].size()>=analysisSettings.meshing.minimumSurfacePoints) {
            auto e=detection.evidence[j];std::visit([&](auto& q){q.supportPointIds.clear();for(auto id:owners[j])q.supportPointIds.push_back(toSource.at(id));},e);owned.push_back(std::move(e));ownedToDetection.push_back(j);
        }
        std::vector<reconstruction::PrimitiveEvidence> accepted;reconstruction::SurfaceSupportValidationReport finalReport;
        if(!reconstruction::validateSurfaceSupport(source,owned,analysisOptions,accepted,finalReport,error,
            &displayAnalysis.supportValidation.reference,cancelled)) {result.error=QString::fromStdString(error);return result;}
        if(analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Report) {
            result.finalSupportValidation=std::move(finalReport);
            for(size_t j=0;j<owners.size();++j)if(owners[j].size()>=analysisSettings.meshing.minimumSurfacePoints) {
                result.assignedPoints+=owners[j].size();result.surfaces.push_back({describe(detection.evidence[j],static_cast<int>(j+1)),std::move(owners[j])});
            }
            return result;
        }
        // One bounded reassignment of observations released by the final gate.
        std::unordered_set<reconstruction::PointId> retained;
        for(const auto& e:accepted)std::visit([&](const auto& q){retained.insert(q.supportPointIds.begin(),q.supportPointIds.end());},e);
        bool reassigned=false;
        for(size_t j=0;j<owners.size();++j)for(auto original:owners[j]) {
            const auto id=toSource.at(original);if(retained.count(id))continue;
            const auto found=positionById.find(id);if(found==positionById.end())continue;const auto p=found->second->position;
            size_t chosen=accepted.size();double best=distanceTolerance;
            for(size_t k=0;k<accepted.size();++k) {
                const auto proposal=std::visit([](const auto& e){return e.id;},accepted[k]);
                if(!proposal || proposal>ownedToDetection.size())continue;
                const auto old=ownedToDetection[size_t(proposal-1)];
                const bool plane=!planeMembers[old].empty();if(plane && !supportedByPlane(old,id,p))continue;
                const bool protectedPlane=plane && preferredPlane[old];
                if(chosen!=accepted.size()) {
                    const auto previous=std::visit([](const auto& e){return e.id;},accepted[chosen]);
                    const auto oldPrevious=ownedToDetection[size_t(previous-1)];
                    if(preferredPlane[oldPrevious] && !protectedPlane)continue;
                }
                const double distance=reconstruction::surfaceResidual(accepted[k],p);
                if(distance>distanceTolerance)continue;
                const bool replaceAlternative=protectedPlane && chosen!=accepted.size() &&
                    !preferredPlane[ownedToDetection[size_t(std::visit([](const auto& e){return e.id;},accepted[chosen])-1)]];
                if(distance>=best && !replaceAlternative)continue;
                const auto& patch=displayAnalysis.patches[old];
                if(p.x<patch.lower.x-supportRadius || p.x>patch.upper.x+supportRadius || p.y<patch.lower.y-supportRadius || p.y>patch.upper.y+supportRadius || p.z<patch.lower.z-supportRadius || p.z>patch.upper.z+supportRadius)continue;
                if(!plane) {
                    if(cloud.normals.size()==cloud.positions.size()) {
                        const auto& n=cloud.normals[toCloud.at(id)];const Vec normal{n[0],n[1],n[2]};
                        if(std::hypot(normal.x,normal.y,normal.z)>1e-12 && reconstruction::surfaceNormalAlignment(accepted[k],p,normal)<.9)continue;
                    }
                    double q[3],raw[3]{p.x,p.y,p.z};const auto near=supports[old]->FindClosestPoint(raw);
                    if(near<0)continue;supports[old]->GetDataSet()->GetPoint(near,q);
                    if(std::hypot(p.x-q[0],p.y-q[1],p.z-q[2])>supportRadius)continue;
                }
                chosen=k;best=distance;
            }
            if(chosen!=accepted.size()){std::visit([&](auto& e){e.supportPointIds.push_back(id);},accepted[chosen]);retained.insert(id);reassigned=true;}
        }
        if(reassigned) {
            std::vector<reconstruction::PrimitiveEvidence> checked;reconstruction::SurfaceSupportValidationReport checkedReport;
            if(!reconstruction::validateSurfaceSupport(source,accepted,analysisOptions,checked,checkedReport,error,
                &displayAnalysis.supportValidation.reference,cancelled)){result.error=QString::fromStdString(error);return result;}
            accepted=std::move(checked);finalReport=std::move(checkedReport);
        }
        for(auto& d:finalReport.decisions)if(d.verdict!=reconstruction::SupportVerdict::Accepted)d.reasons.emplace_back("FinalSupportLost");
        result.finalSupportValidation=finalReport;
        const auto reference=finalReport.reference;
        for(size_t j=0;j<accepted.size();++j) {
            auto region=describe(accepted[j],static_cast<int>(j+1));
            region.supportValidation={{"verdict","accepted"},{"reference_spacing",reference.available?reference.spacing:0},
                {"density_ratio_max",analysisOptions.supportValidation.planeDensityRatioMax}};
            const auto proposalId=std::visit([](const auto& e){return e.id;},accepted[j]);
            const reconstruction::SurfaceSupportDecision* dominant=nullptr;
            for(const auto& d:finalReport.decisions)if(d.proposalId==proposalId && d.verdict==reconstruction::SupportVerdict::Accepted && (!dominant || d.area>dominant->area))dominant=&d;
            if(dominant) {
                const auto& d=*dominant;
                region.supportValidation["proposal_id"]=qint64(d.proposalId);region.supportValidation["patch_id"]=qint64(d.patchId);
                region.supportValidation["density_ratio"]=d.densityRatio;
                region.supportValidation["density_evaluated"]=d.densityEvaluated;
                region.supportValidation["solid_angle_sr"]=d.solidAngle;
                region.supportValidation["minor_span_deg"]=d.minorSpanRadians*180/std::acos(-1.);
                region.supportValidation["angular_span_deg"]=d.angularSpanRadians*180/std::acos(-1.);
            }
            std::vector<vtkIdType> members;
            std::visit([&](const auto& q){for(auto id:q.supportPointIds)members.push_back(toOriginal.at(id));},accepted[j]);
            result.assignedPoints+=members.size();result.surfaces.push_back({std::move(region),std::move(members)});
        }
        // Final unique ownership has a different validation scope than the processed cache.
        // Force a fresh CAD analysis if publication removed any source-accepted patch.
        if(accepted.size()!=owned.size())result.analysis.pointFingerprint=0;
        return result;
    }
    for (size_t j = 0; j < owners.size(); ++j) {
        if(owners[j].size()<analysisSettings.meshing.minimumSurfacePoints)continue;
        result.assignedPoints+=owners[j].size();
        result.surfaces.push_back({describe(detection.evidence[j], static_cast<int>(j+1)), std::move(owners[j])});
    }
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
    const reconstruction::SurfaceAnalysisResult& analysis, std::size_t sourcePointCount, double scale)
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
        if (surface.points.size()>=minimum) result.push_back(std::move(surface));
    }
    std::stable_sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        if ((a.surface.geometry == "plane") != (b.surface.geometry == "plane")) return a.surface.geometry == "plane";
        return a.points.size() > b.points.size();
    });
    return result;
}
