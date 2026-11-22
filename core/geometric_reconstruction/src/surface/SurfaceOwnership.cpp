#include "reconstruction/surface/SurfaceOwnership.h"
#include "reconstruction/surface/SurfaceFootprint.h"
#include "reconstruction/surface/SurfaceSupportValidation.h"
#include "common/Math3.h"
#include "common/spatial/KdTree.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace reconstruction {
namespace {
using Vec = modeling::Vec3;
using XY = std::array<double,2>;
using namespace math3;
using PointLookup = std::unordered_map<PointId,const PointSample*>;
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

bool planeNormalCompatible(const PointSample& sample, Vec planeNormal, Vec material) {
    if (!sample.normal || sample.confidence<.5 ||
        (sample.curvature && *sample.curvature>=.02)) return true;
    const double normalLength=length(*sample.normal),materialLength=length(material);
    if (normalLength<=1e-12) return true;
    if (std::abs(dot(*sample.normal,planeNormal))<.5*normalLength*length(planeNormal)) return false;
    return !sample.normalOrientationKnown || materialLength<=.25 ||
        dot(*sample.normal,material)>=-.5*normalLength*materialLength;
}

bool insideHole(XY point,const std::vector<XY>& boundary) {
    bool inside=false;
    for (size_t i=0,j=boundary.size()-1;i<boundary.size();j=i++) {
        const auto a=boundary[j],b=boundary[i];
        if ((a[1]>point[1])!=(b[1]>point[1]) &&
            point[0]<a[0]+(b[0]-a[0])*(point[1]-a[1])/(b[1]-a[1])) inside=!inside;
    }
    return inside;
}

std::vector<PointId> holeObservations(const detail::spatial::KdTree& tree,
    const SurfaceFootprint& domain,const PlanarFootprintHole& hole,const PlaneEquation& plane,
    Vec material,double tolerance,const SurfaceMeshingSettings& settings,
    const std::unordered_set<PointId>& retained) {
    if (hole.spacing<=0) return {};
    const auto normal=normalized(plane.normal);
    const auto origin=subtract(domain.origin,scale(normal,(dot(plane.normal,domain.origin)+plane.offset)/length(plane.normal)));
    Vec lower{INFINITY,INFINITY,INFINITY},upper{-INFINITY,-INFINITY,-INFINITY};
    for (const auto p:hole.boundary) {
        const auto vertex=add(origin,add(scale(domain.axisU,p[0]),scale(domain.axisV,p[1])));
        lower={std::min(lower.x,vertex.x),std::min(lower.y,vertex.y),std::min(lower.z,vertex.z)};
        upper={std::max(upper.x,vertex.x),std::max(upper.y,vertex.y),std::max(upper.z,vertex.z)};
    }
    struct Observation { PointId id; XY position; double residual; };
    std::vector<Observation> observations;
    const Vec padding{tolerance,tolerance,tolerance};
    tree.visitBox(subtract(lower,padding),add(upper,padding),[&](const PointSample& point) {
        if (retained.count(point.id)) return;
        const double residual=dot(normal,subtract(point.position,origin));
        if (std::abs(residual)>tolerance || !planeNormalCompatible(point,normal,material)) return;
        const auto q=subtract(point.position,origin);
        const XY uv{dot(q,domain.axisU),dot(q,domain.axisV)};
        if (insideHole(uv,hole.boundary)) observations.push_back({point.id,uv,residual});
    });
    const double expected=hole.area/(hole.spacing*hole.spacing);
    const double minimum=std::max(settings.minimumExpectedInteriorPoints,expected*settings.holeDensityRatio);
    if (observations.size()<minimum) return {};

    // Keep only observations connected to the measured rim at the local spacing.
    std::vector<PointSample> planar;
    planar.reserve(observations.size()+hole.boundary.size());
    const auto planarSample=[&](XY p) {
        PointSample sample;sample.id=planar.size()+1;sample.position={p[0],p[1],0};
        planar.push_back(sample);
    };
    for (const auto p:hole.boundary) planarSample(p);
    for (const auto& p:observations) planarSample(p.position);
    const detail::spatial::KdTree planarTree(planar);
    std::vector<bool> seen(planar.size());
    std::vector<std::size_t> connected;
    for (size_t i=0;i<hole.boundary.size();++i) {seen[i]=true;connected.push_back(i);}
    const double reach=settings.connectivityRadiusMultiplier*hole.spacing;
    const Vec window{reach,reach,0};
    for (size_t i=0;i<connected.size();++i) {
        const auto p=planar[connected[i]].position;
        planarTree.visitBox(subtract(p,window),add(p,window),[&](const PointSample& neighbor) {
            const auto index=std::size_t(neighbor.id-1);
            if (seen[index] || detail::spatial::squaredDistance(p,neighbor.position)>reach*reach) return;
            seen[index]=true;connected.push_back(index);
        });
    }
    std::set<std::pair<long long,long long>> cells;
    std::vector<double> residuals;
    std::vector<PointId> ids;
    double u=0,v=0,uu=0,uv=0,vv=0;
    for (size_t i=0;i<observations.size();++i) {
        if (!seen[i+hole.boundary.size()]) continue;
        const auto& p=observations[i];
        const double x=p.position[0]-hole.boundary.front()[0],y=p.position[1]-hole.boundary.front()[1];
        cells.emplace(static_cast<long long>(std::floor(x/hole.spacing)),
                      static_cast<long long>(std::floor(y/hole.spacing)));
        u+=x;v+=y;uu+=x*x;uv+=x*y;vv+=y*y;
        ids.push_back(p.id);residuals.push_back(p.residual);
    }
    if (cells.size()<minimum) return {};
    const auto count=double(ids.size());
    uu=uu/count-u*u/(count*count);uv=uv/count-u*v/(count*count);vv=vv/count-v*v/(count*count);
    const double minorVariance=.5*(uu+vv-std::hypot(uu-vv,2*uv));
    if (minorVariance<.25*hole.spacing*hole.spacing) return {};
    const auto median=residuals.begin()+residuals.size()/2;
    std::nth_element(residuals.begin(),median,residuals.end());
    // A coherent offset is evidence of a separate layer in the opening.
    if (std::abs(*median)>.6*tolerance) return {};
    return ids;
}

struct RecoveryClaim {
    size_t surface=0;
    double distance=INFINITY, competitor=INFINITY;
    void consider(size_t index,double residual) {
        if (index==surface) {distance=std::min(residual,distance);return;}
        if (residual<distance) {
            competitor=distance;distance=residual;surface=index;
        } else competitor=std::min(competitor,residual);
    }
    bool unique(double margin) const { return competitor-distance>margin; }
};

Vec footprintPosition(const SurfaceFootprint& domain,XY position) {
    return add(domain.origin,add(scale(domain.axisU,position[0]),scale(domain.axisV,position[1])));
}

std::size_t recoverRelaxedPlanePoints(const PointLookup& points,const SurfaceSupportValidationReport& initial,
    const SurfaceSupportValidationReport* beforeReassignment,const SurfaceOwnershipOptions& options,double fitTolerance,
    const SurfaceSupportValidationOptions& validation,double areaFloor,
    const std::function<bool()>& cancelled,SurfaceSupportValidationReport& report,
    std::vector<PrimitiveEvidence>& accepted) {
    if (!report.reference.available) return 0;
    std::set<PointId> released;
    const auto collect=[&](const auto& decisions) {
        for (const auto& d:decisions) {
            if (d.verdict==SupportVerdict::Accepted || d.protectedManual ||
                d.kind==PrimitiveKind::Plane || d.kind==PrimitiveKind::Torus) continue;
            released.insert(d.supportPointIds.begin(),d.supportPointIds.end());
        }
    };
    collect(initial.decisions);collect(report.decisions);
    if (beforeReassignment) collect(beforeReassignment->decisions);
    const auto retain=[&](const auto& evidence) {
        std::visit([&](const auto& e){for (auto id:e.supportPointIds) released.erase(id);},evidence);
    };
    for (const auto& e:accepted) retain(e);
    // A surviving alternative's inliers are excluded by the accepted ownership.
    // Keep original released points even when the final validation is run twice.
    if (released.empty()) return 0;
    const double spacing=report.reference.spacing;
    const double tolerance=std::min(2*fitTolerance,1.25*spacing);
    const double reach=std::min(options.supportRadius,1.5*spacing);
    if (tolerance<=fitTolerance) return 0;
    std::vector<PointSample> releasedSamples;
    releasedSamples.reserve(released.size());
    for (auto id:released) releasedSamples.push_back(*points.at(id));
    const detail::spatial::KdTree releasedTree(releasedSamples);
    std::unordered_map<PointId,RecoveryClaim> claims;
    for (size_t index=0;index<accepted.size();++index) {
        if (cancelled && cancelled()) return 0;
        const auto* plane=std::get_if<PlaneEvidence>(&accepted[index]);
        if (!plane) continue;
        const auto n=normalized(plane->plane.normal);
        for (const auto& d:report.decisions) {
            if (d.proposalId!=plane->id || d.verdict!=SupportVerdict::Accepted || !d.footprint) continue;
            const bool stable=d.densityEvaluated && d.effectivePoints>=validation.referenceMinPoints && d.finalArea>=areaFloor;
            const auto& domain=*d.footprint;
            Vec lower{INFINITY,INFINITY,INFINITY},upper{-INFINITY,-INFINITY,-INFINITY};
            for (const auto& uv:domain.positions) {
                const auto p=footprintPosition(domain,{uv[0],uv[1]});
                lower={std::min(lower.x,p.x),std::min(lower.y,p.y),std::min(lower.z,p.z)};
                upper={std::max(upper.x,p.x),std::max(upper.y,p.y),std::max(upper.z,p.z)};
            }
            // The footprint origin is itself a measured point within the fit slab.
            const double pad=tolerance+fitTolerance;
            const Vec padding{pad,pad,pad};
            std::vector<const PointSample*> nearby;
            releasedTree.visitBox(subtract(lower,padding),add(upper,padding),
                [&](const PointSample& point){nearby.push_back(&point);});
            if (nearby.empty()) continue;
            const SurfaceFootprintQuery query(domain,std::max(1e-12,spacing*1e-9));
            std::vector<PointSample> samples;
            Vec material{};
            for (auto id:domain.coveredPointIds) {
                const auto& point=*points.at(id);
                samples.push_back(point);
                if (point.normal && point.normalOrientationKnown && point.confidence>=.5) material=add(material,*point.normal);
            }
            if (!samples.empty())material=scale(material,1./samples.size());
            const SupportIndex support(std::move(samples));
            for (const auto* sample:nearby) {
                const auto& point=*sample;
                const double signedDistance=(dot(plane->plane.normal,point.position)+plane->plane.offset)/length(plane->plane.normal);
                const double residual=std::abs(signedDistance);
                if (residual>tolerance || !planeNormalCompatible(point,n,material) ||
                    !query.containsProjection(point.position)) continue;
                // A closer accepted plane can veto relaxation even if that
                // plane is too small to claim observations outside its fit.
                if (residual<=fitTolerance || !stable ||
                    support.distanceSquared(subtract(point.position,scale(n,signedDistance)))>reach*reach) {
                    claims[point.id].consider(accepted.size(),residual);
                    continue;
                }
                claims[point.id].consider(index,residual);
            }
        }
    }
    std::size_t recovered=0;
    for (auto id:released) {
        const auto found=claims.find(id);
        if (found==claims.end() || found->second.surface==accepted.size() || !found->second.unique(.1*fitTolerance)) continue;
        std::get<PlaneEvidence>(accepted[found->second.surface]).supportPointIds.push_back(id);
        ++recovered;
    }
    return recovered;
}
WorkResult resolve(const PointStore& source,const SurfaceAnalysisResult& displayAnalysis,
    const SurfaceOwnershipOptions& options,const std::function<bool()>& cancelled) {
    WorkResult result;
    std::string error;
    const auto& analysisOptions=displayAnalysis.options;
    const double distanceTolerance=options.distanceTolerance,supportRadius=options.supportRadius;
    const bool useSourceNormals=options.useSourceNormals;
    // Display connected patches independently, even when they share an equation.
    reconstruction::PrimitiveDetectionResult detection;
    std::vector<SurfaceId> detectionRecoveryProposals;
    for (const auto& patch : displayAnalysis.patches) {
        auto evidence=displayAnalysis.surfaces.at(static_cast<size_t>(patch.surfaceId-1));
        std::visit([&](auto& surface){surface.id=detection.evidence.size()+1;},evidence);
        std::visit([&](auto& surface) { surface.supportPointIds=patch.supportPointIds; },evidence);
        detection.evidence.push_back(std::move(evidence));
        const auto surfaceId=patch.surfaceId;
        detectionRecoveryProposals.push_back(surfaceId && surfaceId<=displayAnalysis.supportProposalIds.size()
            ? displayAnalysis.supportProposalIds[static_cast<size_t>(surfaceId-1)] : 0);
    }
    PointLookup positionById;
    positionById.reserve(source.size());
    for (const auto& point : source.points()) positionById.emplace(point.id, &point);
    std::vector<std::unique_ptr<SupportIndex>> supports;
    std::vector<std::unordered_set<PointId>> planeMembers(detection.evidence.size());
    std::vector<std::unordered_set<PointId>> sphereMembers(detection.evidence.size());
    for (size_t j = 0; j < detection.evidence.size(); ++j) {
        if (const auto* plane = std::get_if<PlaneEvidence>(&detection.evidence[j]))
            planeMembers[j].insert(plane->supportPointIds.begin(), plane->supportPointIds.end());
        if (analysisOptions.supportValidation.mode == SupportValidationMode::Enforce) {
            if (const auto* sphere = std::get_if<SphereEvidence>(&detection.evidence[j]))
                sphereMembers[j].insert(sphere->supportPointIds.begin(), sphere->supportPointIds.end());
        }
    }
    // Protect supported broad planes as regions before per-point residual competition.
    // A curved proposal spanning several faces cannot steal each face's interior.
    std::vector<bool> preferredPlane(detection.evidence.size());
    std::vector<std::vector<std::unique_ptr<reconstruction::SurfaceFootprintQuery>>> planeDomains(detection.evidence.size());
    double areaFloor=INFINITY;
    if(analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Enforce && displayAnalysis.supportValidation.reference.available) {
        const auto& validation=displayAnalysis.supportValidation;
        std::vector<double> areas;
        for(auto id:validation.reference.patchIds)for(const auto& d:validation.decisions)if(d.patchId==id)areas.push_back(d.area);
        std::sort(areas.begin(),areas.end());
        if (!areas.empty()) areaFloor=areas[areas.size()/2]*analysisOptions.supportValidation.planePreferenceMinReferenceAreaFraction;
        for(size_t j=0;j<detection.evidence.size();++j)if(!planeMembers[j].empty()) {
            const auto surfaceId=displayAnalysis.patches[j].surfaceId;
            if(!surfaceId || surfaceId>displayAnalysis.supportProposalIds.size())continue;
            const auto origin=surfaceId;
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
            const bool another = chosen != owners.size();
            // Validated spherical support stays coherent across tangent-plane
            // approximations. Broad observed planes keep their existing priority.
            const bool supportedSphere = distance <= distanceTolerance && sphereMembers[j].count(pointId);
            const bool priorSphere = another && best <= distanceTolerance && sphereMembers[chosen].count(pointId);
            if (isPlane && !preferCandidate && priorSphere) continue;
            const bool replaceAlternative = another &&
                ((preferCandidate && !preferredPlane[chosen]) ||
                 (supportedSphere && !preferCurrent && !planeMembers[chosen].empty()));
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
        std::vector<size_t> acceptedToDetection;
        acceptedToDetection.reserve(accepted.size());
        for(const auto& evidence:accepted) {
            const auto proposal=std::visit([](const auto& surface){return surface.id;},evidence);
            acceptedToDetection.push_back(proposal && proposal<=ownedToDetection.size()
                ? ownedToDetection[static_cast<size_t>(proposal-1)] : owners.size());
        }
        if(analysisOptions.supportValidation.mode==reconstruction::SupportValidationMode::Report) {
            result.finalSupportValidation=std::move(finalReport);
            for(size_t j=0;j<owners.size();++j)if(owners[j].size()>=analysisOptions.supportMeshing.minimumSurfacePoints) {
                result.assignedPoints+=owners[j].size();
                auto evidence=detection.evidence[j];
                std::visit([&](auto& e){e.supportPointIds=std::move(owners[j]);e.id=j+1;},evidence);
                result.surfaces.push_back(std::move(evidence));
                result.sourceRecoveryProposalIds.push_back(detectionRecoveryProposals[j]);
            }
            return result;
        }
        // One bounded reassignment of observations released by the final gate.
        std::unordered_set<reconstruction::PointId> retained;
        for(const auto& e:accepted)std::visit([&](const auto& q){retained.insert(q.supportPointIds.begin(),q.supportPointIds.end());},e);
        bool reassigned=false;
        const double fitTolerance=std::min(distanceTolerance,analysisOptions.detection.plane.distanceThreshold);
        std::map<PointId,RecoveryClaim> holeClaims;
        std::unique_ptr<detail::spatial::KdTree> sourceTree;
        std::unordered_map<SurfaceId,std::size_t> acceptedIndex;
        for (size_t k=0;k<accepted.size();++k)
            if (const auto* plane=std::get_if<PlaneEvidence>(&accepted[k])) acceptedIndex.emplace(plane->id,k);
        for (const auto& decision:finalReport.decisions) {
            if (!decision.footprint || decision.footprint->holes.empty() ||
                decision.verdict!=SupportVerdict::Accepted || !acceptedIndex.count(decision.proposalId)) continue;
            const auto index=acceptedIndex.at(decision.proposalId);
            const auto& plane=std::get<PlaneEvidence>(accepted[index]);
            const auto patch=ownedToDetection.at(size_t(plane.id-1));
            const auto material=displayAnalysis.patches[patch].materialNormal;
            if (!sourceTree) sourceTree=std::make_unique<detail::spatial::KdTree>(source.points());
            for (const auto& hole:decision.footprint->holes) {
                if (cancelled && cancelled()) {result.error="Cancelled";return result;}
                for (auto id:holeObservations(*sourceTree,*decision.footprint,hole,plane.plane,material,
                        fitTolerance,analysisOptions.supportMeshing,retained)) {
                    holeClaims[id].consider(index,surfaceResidual(accepted[index],positionById.at(id)->position));
                }
            }
        }
        for (const auto& [id,claim]:holeClaims) {
            if (!claim.unique(.1*fitTolerance)) continue;
            std::get<PlaneEvidence>(accepted[claim.surface]).supportPointIds.push_back(id);
            retained.insert(id);
            reassigned=true;
        }
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
        std::optional<SurfaceSupportValidationReport> beforeReassignment;
        if(reassigned) {
            std::vector<reconstruction::PrimitiveEvidence> checked;reconstruction::SurfaceSupportValidationReport checkedReport;
            if(!reconstruction::validateSurfaceSupport(source,accepted,analysisOptions,checked,checkedReport,error,
                &displayAnalysis.supportValidation.reference,cancelled)){result.error=error;return result;}
            std::vector<size_t> checkedToDetection;checkedToDetection.reserve(checked.size());
            for(const auto& evidence:checked) {
                const auto proposal=std::visit([](const auto& surface){return surface.id;},evidence);
                checkedToDetection.push_back(proposal && proposal<=acceptedToDetection.size()
                    ? acceptedToDetection[static_cast<size_t>(proposal-1)] : owners.size());
            }
            accepted=std::move(checked);beforeReassignment=std::move(finalReport);finalReport=std::move(checkedReport);
            acceptedToDetection=std::move(checkedToDetection);
        }
        for (const auto& e:accepted) {
            const auto* plane=std::get_if<PlaneEvidence>(&e);
            if (!plane) continue;
            for (auto id:plane->supportPointIds) {
                const auto claim=holeClaims.find(id);
                if (claim!=holeClaims.end() && claim->second.unique(.1*fitTolerance))
                    ++finalReport.planeHoleRecoveredPoints;
            }
        }
        finalReport.planeRelaxedRecoveredPoints=recoverRelaxedPlanePoints(positionById,displayAnalysis.supportValidation,
            beforeReassignment?&*beforeReassignment:nullptr,options,fitTolerance,analysisOptions.supportValidation,
            areaFloor,cancelled,finalReport,accepted);
        for(auto& d:finalReport.decisions)if(d.verdict!=reconstruction::SupportVerdict::Accepted)d.reasons.emplace_back("FinalSupportLost");
        result.finalSupportValidation=finalReport;
        for(const auto& evidence:accepted)
            result.assignedPoints+=std::visit([](const auto& e){return e.supportPointIds.size();},evidence);
        result.surfaces=accepted;
        result.sourceRecoveryProposalIds.reserve(accepted.size());
        for(size_t i=0;i<accepted.size();++i) {
            const auto patchIndex=acceptedToDetection[i];
            if(patchIndex>=displayAnalysis.patches.size()) {result.sourceRecoveryProposalIds.push_back(0);continue;}
            const auto surfaceId=displayAnalysis.patches[patchIndex].surfaceId;
            result.sourceRecoveryProposalIds.push_back(surfaceId && surfaceId<=displayAnalysis.supportProposalIds.size()
                ? displayAnalysis.supportProposalIds[static_cast<size_t>(surfaceId-1)] : 0);
        }
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
        result.sourceRecoveryProposalIds.push_back(detectionRecoveryProposals[j]);
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
