#include "reconstruction/surface/PlaneSupportRecovery.h"
#include "common/spatial/KdTree.h"
#include "common/Math3.h"
#include <unordered_map>
#include <unordered_set>
#include <numeric>

namespace reconstruction {
namespace {
using namespace math3;
using Index = std::size_t;
using Neighbors = std::vector<std::pair<double, Index>>;
constexpr double angularLimit = 0.03490658503988659; // two degrees for growing one fitted region
constexpr double mergeInlierFraction = 0.95;

double residual(const PlaneEquation& plane, Vec3 p) { return std::abs(dot(plane.normal,p)+plane.offset); }

bool fit(const PointStore& points, const std::vector<Index>& support, PlaneEquation& plane,double* span=nullptr) {
    if(support.size()<3) return false;
    Vec3 center{};
    for(auto i:support) center=add(center,points.points()[i].position);
    center=scale(center,1.0/support.size());
    std::array<std::array<double,3>,3> matrix{};
    for(auto i:support) {
        const auto d=subtract(points.points()[i].position,center);
        const double v[]{d.x,d.y,d.z};
        for(int a=0;a<3;++a) for(int b=0;b<3;++b) matrix[a][b]+=v[a]*v[b];
    }
    const auto eigen=symmetricEigenDecomposition(matrix);
    if(eigen.values[1]<=std::max(1e-18,eigen.values[2]*1e-8)) return false;
    if(span) *span=2*std::sqrt(eigen.values[1]/support.size());
    auto normal=eigen.vectors[0];
    if(dot(normal,plane.normal)<0) normal=scale(normal,-1);
    plane={normal,-dot(normal,center)};
    return true;
}

struct Region {
    PlaneEvidence evidence;
    std::vector<Index> support;
    // Keep seed and growth supports separate so every source patch has its own merge gate.
    std::vector<std::vector<Index>> mergeComponents;
    PlaneEquation anchor;
    std::optional<Vec3> material;
    double angularBudget=angularLimit;
};

class Recovery {
    const PointStore& points;
    const PlaneDetectionOptions& options;
    detail::spatial::KdTree tree;
    std::vector<Neighbors> neighbors;
    std::vector<double> radii;
    std::vector<bool> blocked;
    const std::function<bool()>& cancelled;
public:
    bool stopped=false;
    Recovery(const PointStore& p,const PlaneDetectionOptions& o,const std::vector<PointId>& protectedIds,
             const std::function<bool()>& cancel):points(p),options(o),tree(p.points()),neighbors(p.size()),
             radii(p.size()),blocked(p.size()),cancelled(cancel) {
        const std::unordered_set<PointId> protectedSet(protectedIds.begin(),protectedIds.end());
        for(Index i=0;i<p.size();++i) blocked[i]=protectedSet.count(p.points()[i].id)!=0;
    }
    bool check() { return stopped=cancelled && cancelled(); }
    void blockSeedSupports(const std::vector<Region>& regions) {
        for (const auto& region : regions)
            for (const auto index : region.support) blocked[index] = true;
    }
    const Neighbors& near(Index i) {
        if(neighbors[i].empty()) {
            neighbors[i]=tree.nearest(i,std::min<Index>(24,points.size()-1));
            std::optional<Vec3> tangentNormal;
            const auto& sample=points.points()[i];
            if(sample.normal && std::isfinite(length(*sample.normal)) && length(*sample.normal)>1e-12)
                tangentNormal=normalized(*sample.normal);
            else {
                std::vector<Index> local{i};for(const auto& n:neighbors[i]) local.push_back(n.second);
                PlaneEquation geometry;
                if(fit(points,local,geometry)) tangentNormal=geometry.normal;
            }
            std::vector<double> distances;
            for(const auto& n:neighbors[i]) if(n.first>std::max(1e-20,options.distanceThreshold*options.distanceThreshold/16)) {
                const double distance=std::sqrt(n.first);
                const auto delta=subtract(points.points()[n.second].position,sample.position);
                if(tangentNormal && std::abs(dot(delta,*tangentNormal))>std::max(options.distanceThreshold*2,distance*.3)) continue;
                distances.push_back(distance);
            }
            // k-nearest queries on a small isolated island also return distant
            // islands. A discontinuity in spacing must not enlarge its reach.
            for(Index j=1;j<distances.size();++j) if(distances[j]>distances[j-1]*3) {
                distances.resize(j);break;
            }
            const double spacing=distances.empty()?options.distanceThreshold:
                distances[std::min<Index>(5,distances.size()-1)];
            radii[i]=std::max(options.distanceThreshold*2,spacing*2);
        }
        return neighbors[i];
    }
    bool adjacent(Index a,Index b,double squared) {
        near(a);near(b);
        const double radius=std::min(radii[a],radii[b]);
        return squared<=radius*radius;
    }
    bool eligible(const Region& region,Index i) {
        const auto& p=points.points()[i];
        if(blocked[i] || residual(region.evidence.plane,p.position)>options.distanceThreshold) return false;
        if(p.normal && std::isfinite(length(*p.normal)) && length(*p.normal)>1e-12) {
            const auto n=normalized(*p.normal);
            // Curvature identifies normals blurred across an edge. Missing or
            // unreliable directions are not allowed to punch holes in a plane.
            bool reliable=p.confidence>=.5 && (!p.curvature || *p.curvature<.02);
            if(reliable && std::abs(dot(n,region.evidence.plane.normal))<options.minimumNormalAlignment) {
                // A measured normal can be wrong even on a geometrically flat
                // neighborhood. Only a stable, independently fitted local
                // plane may override that normal for geometric membership.
                std::vector<Index> local{i};for(const auto& neighbor:near(i)) local.push_back(neighbor.second);
                auto geometry=region.evidence.plane;
                if(!fit(points,local,geometry) || std::abs(dot(geometry.normal,region.evidence.plane.normal))<.98) return false;
                double squared=0,spread=0;
                for(auto j:local) {
                    const double d=residual(geometry,points.points()[j].position);squared+=d*d;
                    const auto delta=subtract(points.points()[j].position,p.position);spread+=dot(delta,delta);
                }
                if(squared>spread*.005) return false;
                reliable=false;
            }
            if(reliable && p.normalOrientationKnown && region.material && dot(n,*region.material)<0) return false;
        }
        return true;
    }
    bool refit(Region& r) const {
        auto proposed=r.evidence.plane;
        if(!fit(points,r.support,proposed) || std::abs(dot(proposed.normal,r.anchor.normal))<std::cos(r.angularBudget)) return false;
        for(auto i:r.support) if(residual(proposed,points.points()[i].position)>options.distanceThreshold) return false;
        const bool changed=std::abs(dot(proposed.normal,r.evidence.plane.normal))<1-1e-12 ||
            std::abs(proposed.offset-r.evidence.plane.offset)>options.distanceThreshold*1e-4;
        r.evidence.plane=proposed;
        return changed;
    }
    void describe(Region& r) const {
        auto& e=r.evidence;e.supportPointIds.clear();e.meanAbsoluteResidual=0;e.maxAbsoluteResidual=0;
        for(auto i:r.support) {
            e.supportPointIds.push_back(points.points()[i].id);
            const auto d=residual(e.plane,points.points()[i].position);
            e.meanAbsoluteResidual+=d;e.maxAbsoluteResidual=std::max(e.maxAbsoluteResidual,d);
        }
        std::sort(e.supportPointIds.begin(),e.supportPointIds.end());
        if(!r.support.empty()) e.meanAbsoluteResidual/=r.support.size();
        e.coverage=double(r.support.size())/points.size();
        e.confidence=std::clamp(1-e.meanAbsoluteResidual/options.distanceThreshold,0.0,1.0);
    }
    bool merge(Region& a,const Region& b) {
        if(std::abs(dot(a.evidence.plane.normal,b.evidence.plane.normal))<
                std::cos(options.mergeAngularToleranceRadians) ||
            (a.material && b.material && dot(*a.material,*b.material)<0)) return false;
        std::unordered_set<Index> included(a.support.begin(),a.support.end());
        bool connected=false;
        for(auto i:b.support) {
            if(included.count(i)) {connected=true;break;}
            for(const auto& n:near(i)) if(included.count(n.second) && adjacent(i,n.second,n.first)) {connected=true;break;}
            if(connected) break;
        }
        if(!connected) return false;
        auto combined=a;
        for(auto i:b.support) if(included.insert(i).second) combined.support.push_back(i);
        combined.mergeComponents.insert(combined.mergeComponents.end(), b.mergeComponents.begin(), b.mergeComponents.end());
        auto proposed=combined.evidence.plane;
        const double minimumAlignment = std::cos(options.mergeAngularToleranceRadians);
        if(!fit(points,combined.support,proposed) ||
            std::abs(dot(proposed.normal,a.anchor.normal))<minimumAlignment ||
            std::abs(dot(proposed.normal,b.anchor.normal))<minimumAlignment) return false;
        const double maximumResidual = options.distanceThreshold * options.mergeResidualToleranceMultiplier;
        for (const auto& component : combined.mergeComponents) {
            if (component.empty()) continue;
            std::vector<double> residuals;
            residuals.reserve(component.size());
            for (auto i : component) residuals.push_back(residual(proposed, points.points()[i].position));
            const auto index = static_cast<std::size_t>(std::ceil(mergeInlierFraction * residuals.size())) - 1;
            std::nth_element(residuals.begin(), residuals.begin() + index, residuals.end());
            if (residuals[index] > maximumResidual) return false;
        }
        combined.evidence.plane=proposed;
        combined.anchor=proposed;
        if(!combined.material) combined.material=b.material;
        a=std::move(combined);
        return true;
    }
    bool grow(Region& r) {
        const auto initialSupportSize = r.support.size();
        std::vector<bool> accepted(points.size()),deferredFlag(points.size());
        for(auto i:r.support) accepted[i]=true;
        auto frontier=r.support;
        std::vector<Index> deferred;
        Index sinceFit=0;
        while(!frontier.empty()) {
            if(check()) return false;
            std::vector<Index> next;
            for(auto i:frontier) {
                if((i&255)==0 && check()) return false;
                for(const auto& neighbor:near(i)) {
                    const auto j=neighbor.second;
                    if(accepted[j] || blocked[j] || deferredFlag[j] || !adjacent(i,j,neighbor.first)) continue;
                    if(eligible(r,j)) {accepted[j]=true;r.support.push_back(j);next.push_back(j);++sinceFit;}
                    else {deferredFlag[j]=true;deferred.push_back(j);}
                }
            }
            if(sinceFit && (next.empty() || sinceFit>=std::max<Index>(32,r.support.size()/4))) {
                const bool changed=refit(r);sinceFit=0;
                if(changed) {
                    deferred.erase(std::remove_if(deferred.begin(),deferred.end(),[&](auto j) {
                        if(!eligible(r,j)) return false;
                        accepted[j]=true;deferredFlag[j]=false;r.support.push_back(j);next.push_back(j);++sinceFit;
                        return true;
                    }),deferred.end());
                }
            }
            frontier=std::move(next);
        }
        refit(r);describe(r);
        if (r.support.size() > initialSupportSize)
            r.mergeComponents.emplace_back(r.support.begin() + initialSupportSize, r.support.end());
        return true;
    }
};
}

bool recoverPlaneSupport(const PointStore& points,const PlaneDetectionOptions& options,
    std::vector<PrimitiveEvidence>& evidence,std::string& error,const std::vector<PointId>& protectedIds,
    const std::function<bool()>& cancelled) {
    error.clear();
    if(cancelled && cancelled()) {error="Cancelled";return false;}
    if(points.empty() || std::none_of(evidence.begin(),evidence.end(),[](const auto& e){return std::holds_alternative<PlaneEvidence>(e);})) return true;
    if(!std::isfinite(options.distanceThreshold) || options.distanceThreshold<=0 ||
       !std::isfinite(options.mergeAngularToleranceRadians) || options.mergeAngularToleranceRadians<=0 ||
       options.mergeAngularToleranceRadians>10*std::acos(-1.0)/180 ||
       !std::isfinite(options.mergeResidualToleranceMultiplier) ||
       options.mergeResidualToleranceMultiplier<1 || options.mergeResidualToleranceMultiplier>3) {
        error="Invalid plane support or merge tolerance";
        return false;
    }
    std::unordered_map<PointId,Index> indices;
    for(Index i=0;i<points.size();++i) indices.emplace(points.points()[i].id,i);
    const std::unordered_set<PointId> protectedSet(protectedIds.begin(),protectedIds.end());
    std::vector<Region> regions;
    for(const auto& e:evidence) if(const auto* plane=std::get_if<PlaneEvidence>(&e)) {
        Region r;r.evidence=*plane;r.anchor=plane->plane;
        std::unordered_set<Index> used;Vec3 material{};
        for(auto id:plane->supportPointIds) {
            const auto found=indices.find(id);
            if(found==indices.end()) {error="Plane support references a different point snapshot";return false;}
            if(protectedSet.count(id) || !used.insert(found->second).second) continue;
            r.support.push_back(found->second);
            const auto& p=points.points()[found->second];
            if(p.normal && p.normalOrientationKnown && std::isfinite(length(*p.normal)) && length(*p.normal)>1e-12)
                material=add(material,normalized(*p.normal));
        }
        if(r.support.size()<3) continue;
        if(length(material)>r.support.size()*.5) r.material=normalized(material);
        // Robust RANSAC supports supply the geometric frame; a single point's
        // measured normal never becomes the plane used for a whole large face.
        auto fitted=r.evidence.plane;
        double span=0;
        if(fit(points,r.support,fitted,&span)) {
            bool valid=true;for(auto i:r.support) valid &= residual(fitted,points.points()[i].position)<=options.distanceThreshold;
            if(valid) {
                r.evidence.plane=r.anchor=fitted;
                // A small seed under a finite measurement tolerance has greater
                // directional uncertainty than an already broad supported face.
                r.angularBudget=std::clamp(std::atan2(options.distanceThreshold*2,span),angularLimit,.17453292519943295);
            }
        }
        r.mergeComponents.push_back(r.support);
        regions.push_back(std::move(r));
    }
    std::stable_sort(regions.begin(),regions.end(),[](const auto& a,const auto& b){return a.support.size()>b.support.size();});
    Recovery recovery(points,options,protectedIds,cancelled);
    recovery.blockSeedSupports(regions);
    const auto consolidate=[&]() {
        for(Index i=0;i<regions.size();++i) {
            if(recovery.check()) return false;
            for(Index j=i+1;j<regions.size();) {
                if(recovery.merge(regions[i],regions[j])) regions.erase(regions.begin()+j);
                else ++j;
            }
        }
        return true;
    };
    if(!consolidate()) {error="Cancelled";return false;}
    for(auto& r:regions) if(!recovery.grow(r)) {error="Cancelled";return false;}
    if(!consolidate()) {error="Cancelled";return false;}
    std::vector<PrimitiveEvidence> output;output.reserve(evidence.size());
    for(auto& r:regions) {recovery.describe(r);output.emplace_back(std::move(r.evidence));}
    for(const auto& e:evidence) if(!std::holds_alternative<PlaneEvidence>(e)) output.push_back(e);
    SurfaceId id=0;for(auto& e:output) std::visit([&](auto& s){s.id=++id;},e);
    evidence=std::move(output);
    return true;
}
}
