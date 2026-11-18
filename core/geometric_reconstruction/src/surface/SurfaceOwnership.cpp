#include "reconstruction/surface/SurfaceOwnership.h"
#include "reconstruction/surface/SurfaceSupportValidation.h"
#include "common/spatial/KdTree.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace reconstruction {
namespace {
using Vec = modeling::Vec3;
struct WorkResult : SurfaceOwnershipResult { std::string error; };
struct SupportIndex {
    std::vector<PointSample> points;
    detail::spatial::KdTree tree;
    explicit SupportIndex(std::vector<PointSample> samples)
        :points(std::move(samples)),tree(points) {}
    double distanceSquared(Vec point) const {
        const auto nearest=tree.nearest(point,1);
        return nearest.empty()?INFINITY:nearest.front().first;
    }
};
WorkResult resolve(const PointStore& source,const SurfaceAnalysisResult& displayAnalysis,
    const SurfaceOwnershipOptions& options,const std::function<bool()>& cancelled) {
    WorkResult result;
    std::string error;
    const auto& analysisOptions=displayAnalysis.options;
    const double distanceTolerance=options.distanceTolerance,supportRadius=options.supportRadius;
    const bool useSourceNormals=options.useSourceNormals;
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
    std::vector<std::unique_ptr<SupportIndex>> supports;
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
    std::vector<std::vector<PointId>> owners(detection.evidence.size());
    for (const auto& evidence : detection.evidence) {
        std::vector<PointSample> samples;
        std::visit([&](const auto& surface) {
            for (const auto id : surface.supportPointIds) {
                const auto found = positionById.find(id);
                if (found != positionById.end()) samples.push_back(*found->second);
            }
        }, evidence);
        supports.push_back(std::make_unique<SupportIndex>(std::move(samples)));
    }
    // Recover every eligible original observation, constrained to the observed
    // patch so distant coplanar surfaces aren't swallowed by an infinite plane.
    for (size_t i = 0; i < source.size(); ++i) {
        if (i % 4096 == 0 && (cancelled && cancelled())) {
            result.error = "Cancelled"; return result;
        }
        const auto& sample = source.points()[i];
        const Vec point = sample.position;
        const auto pointId = sample.id;
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
            if(!isPlane && useSourceNormals && sample.normal.has_value()) {
                const Vec n=*sample.normal;
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
            if ((distance > best && !replaceAlternative) || supports[j]->points.empty()) continue;
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
            if (supports[j]->distanceSquared(point) > supportRadius*supportRadius) continue;
            best = distance; chosen = j;
        }
        if (chosen != owners.size()) owners[chosen].push_back(pointId);
    }
    if(analysisOptions.supportValidation.mode!=reconstruction::SupportValidationMode::Off) {
        std::vector<reconstruction::PrimitiveEvidence> owned;
        std::vector<size_t> ownedToDetection;
        for(size_t j=0;j<owners.size();++j)if(owners[j].size()>=analysisOptions.supportMeshing.minimumSurfacePoints) {
            auto e=detection.evidence[j];std::visit([&](auto& q){q.supportPointIds=owners[j];},e);owned.push_back(std::move(e));ownedToDetection.push_back(j);
        }
        std::vector<reconstruction::PrimitiveEvidence> accepted;reconstruction::SurfaceSupportValidationReport finalReport;
        if(!reconstruction::validateSurfaceSupport(source,owned,analysisOptions,accepted,finalReport,error,
            &displayAnalysis.supportValidation.reference,cancelled)) {result.error=error;return result;}
        if(analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Report) {
            result.finalSupportValidation=std::move(finalReport);
            for(size_t j=0;j<owners.size();++j)if(owners[j].size()>=analysisOptions.supportMeshing.minimumSurfacePoints) {
                result.assignedPoints+=owners[j].size();
                auto evidence=detection.evidence[j];
                std::visit([&](auto& e){e.supportPointIds=std::move(owners[j]);e.id=j+1;},evidence);
                result.surfaces.push_back(std::move(evidence));
            }
            return result;
        }
        // One bounded reassignment of observations released by the final gate.
        std::unordered_set<reconstruction::PointId> retained;
        for(const auto& e:accepted)std::visit([&](const auto& q){retained.insert(q.supportPointIds.begin(),q.supportPointIds.end());},e);
        bool reassigned=false;
        for(size_t j=0;j<owners.size();++j)for(auto id:owners[j]) {
            if(retained.count(id))continue;
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
                    if(useSourceNormals && found->second->normal) {
                        const Vec normal=*found->second->normal;
                        if(std::hypot(normal.x,normal.y,normal.z)>1e-12 && reconstruction::surfaceNormalAlignment(accepted[k],p,normal)<.9)continue;
                    }
                    if(supports[old]->distanceSquared(p)>supportRadius*supportRadius)continue;
                }
                chosen=k;best=distance;
            }
            if(chosen!=accepted.size()){std::visit([&](auto& e){e.supportPointIds.push_back(id);},accepted[chosen]);retained.insert(id);reassigned=true;}
        }
        if(reassigned) {
            std::vector<reconstruction::PrimitiveEvidence> checked;reconstruction::SurfaceSupportValidationReport checkedReport;
            if(!reconstruction::validateSurfaceSupport(source,accepted,analysisOptions,checked,checkedReport,error,
                &displayAnalysis.supportValidation.reference,cancelled)){result.error=error;return result;}
            accepted=std::move(checked);finalReport=std::move(checkedReport);
        }
        for(auto& d:finalReport.decisions)if(d.verdict!=reconstruction::SupportVerdict::Accepted)d.reasons.emplace_back("FinalSupportLost");
        result.finalSupportValidation=finalReport;
        for(const auto& evidence:accepted)
            result.assignedPoints+=std::visit([](const auto& e){return e.supportPointIds.size();},evidence);
        result.surfaces=accepted;
        // Final unique ownership has a different validation scope than the processed cache.
        // Force a fresh CAD analysis if publication removed any source-accepted patch.
        if(accepted.size()!=owned.size())result.requiresFreshAnalysis=true;
        return result;
    }
    for (size_t j = 0; j < owners.size(); ++j) {
        if(owners[j].size()<analysisOptions.supportMeshing.minimumSurfacePoints)continue;
        result.assignedPoints+=owners[j].size();
        auto evidence=detection.evidence[j];
        std::visit([&](auto& e){e.supportPointIds=std::move(owners[j]);e.id=j+1;},evidence);
        result.surfaces.push_back(std::move(evidence));
    }
    return result;
}
} // namespace

bool resolveSurfaceOwnership(const PointStore& source,const SurfaceAnalysisResult& analysis,
    const SurfaceOwnershipOptions& options,SurfaceOwnershipResult& output,
    std::string& error,const std::function<bool()>& cancelled) {
    error.clear();
    try {
        if(!std::isfinite(options.distanceTolerance) || options.distanceTolerance<=0 ||
           !std::isfinite(options.supportRadius) || options.supportRadius<=0) {
            error="Invalid surface ownership distances";return false;
        }
        if(!analysisMatches(source,analysis)){error="Stale source surface analysis";return false;}
        if(analysis.options.supportValidation.mode!=SupportValidationMode::Off &&
           (analysis.supportValidation.pointFingerprint!=analysis.pointFingerprint ||
            analysis.supportValidation.mode!=analysis.options.supportValidation.mode ||
            analysis.supportValidation.settingsSignature!=surfaceSupportSettingsSignature(analysis.options))) {
            error="Surface ownership requires current support validation";return false;
        }
        if(cancelled && cancelled()){error="Cancelled";return false;}
        auto staged=resolve(source,analysis,options,cancelled);
        if(!staged.error.empty()){error=std::move(staged.error);return false;}
        if(cancelled && cancelled()){error="Cancelled";return false;}
        output=std::move(staged);
        return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
} // namespace reconstruction
