#include "reconstruction/solid_inference/SolidInference.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace reconstruction {
namespace {
using Vec3 = modeling::Vec3;
Vec3 add(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3 subtract(Vec3 a, Vec3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vec3 scale(Vec3 a, double s) { return {a.x*s,a.y*s,a.z*s}; }
double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
double length(Vec3 a) { return std::sqrt(dot(a,a)); }
Vec3 normalized(Vec3 a) { return scale(a,1.0/length(a)); }

struct PlaneFrame { Vec3 origin, x, y, normal; };
struct FaceInput { const ScenePatch* patch; PlaneFrame frame; std::vector<std::vector<std::size_t>> loops; };

bool fail(SolidCandidate& output, const std::string& message, std::string& error) {
    output.status = SolidCandidateStatus::Rejected;
    output.diagnostics.push_back(message);
    error = message;
    return false;
}
}

bool proposeObservedShell(const SurfaceScene& scene, const std::vector<PatchId>& partPatches,
                          SolidCandidate& output, std::string& error, const SolidPreview& preview) {
    output = {};
    output.analysisFingerprint = scene.header.analysisFingerprint;
    output.supportingPatches = partPatches;
    if (const auto validation = validateSurfaceScene(scene); !validation)
        return fail(output, validation.message, error);
    if (partPatches.size() < 4) return fail(output,"Observed shell needs at least four patches",error);
    const double tolerance = scene.header.distanceToleranceMm;
    std::map<PatchId,const ScenePatch*> byId;
    for (const auto& patch : scene.patches) byId.emplace(patch.id,&patch);
    std::set<PatchId> unique(partPatches.begin(),partPatches.end());
    if (unique.size()!=partPatches.size()) return fail(output,"Part hypothesis contains duplicate patch IDs",error);

    std::vector<Vec3> vertices;
    std::vector<FaceInput> faces;
    std::map<std::pair<std::size_t,std::size_t>,int> edgeIds;
    std::map<std::pair<std::size_t,std::size_t>,int> edgeUses;
    auto vertexId = [&](Vec3 p) -> std::size_t {
        for (std::size_t i=0;i<vertices.size();++i)
            if (length(subtract(vertices[i],p))<=tolerance) return i;
        vertices.push_back(p);
        return vertices.size()-1;
    };

    for (const auto id : partPatches) {
        const auto found=byId.find(id);
        if(found==byId.end()) return fail(output,"Part hypothesis references missing patch "+std::to_string(id),error);
        const auto& patch=*found->second;
        if(patch.boundaryStatus!=SceneBoundaryStatus::ObservedClosed || patch.loops.empty())
            return fail(output,"Patch "+std::to_string(id)+" has no observed closed boundary",error);
        if(!patch.orientationKnown || length(patch.materialNormal)<1e-9)
            return fail(output,"Patch "+std::to_string(id)+" has unknown material orientation",error);
        if(!std::holds_alternative<PlaneEvidence>(scene.surfaces[std::size_t(patch.surfaceId-1)]))
            return fail(output,"Observed shell closure currently supports planar patches only",error);
        const auto& plane=std::get<PlaneEvidence>(scene.surfaces[std::size_t(patch.surfaceId-1)]).plane;
        const double normalLength=length(plane.normal);
        if(!std::isfinite(normalLength)||normalLength<1e-9||!std::isfinite(plane.offset))
            return fail(output,"Patch has invalid plane coefficients",error);
        const auto n=normalized(plane.normal);
        const auto origin=scale(n,-plane.offset/normalLength);
        const auto reference=std::abs(n.x)<.8?Vec3{1,0,0}:Vec3{0,1,0};
        const auto x=normalized(cross(reference,n));
        const PlaneFrame frame{origin,x,cross(n,x),n};
        FaceInput face{&patch,frame,{}};
        if(std::any_of(patch.loops.begin(),patch.loops.end(),[](const auto& loop) {
            return loop.role!=SceneLoopRole::Outer&&loop.role!=SceneLoopRole::Hole;
        })) return fail(output,"Patch boundary loop has unresolved role",error);
        bool hasOuter=false;
        for(const auto role:{SceneLoopRole::Outer,SceneLoopRole::Hole}) for(const auto& loop:patch.loops) {
            if(loop.role!=role) continue;
            if(!loop.closed||loop.vertices.size()<3)
                return fail(output,"Patch boundary loop is not closed",error);
            if(role==SceneLoopRole::Outer) {
                if(hasOuter) return fail(output,"Patch has more than one outer loop",error);
                hasOuter=true;
            }
            std::vector<std::size_t> ids;
            for(const auto p:loop.vertices) {
                if(std::abs(dot(frame.normal,subtract(p,frame.origin)))>tolerance*2)
                    return fail(output,"Boundary loop is inconsistent with its fitted plane",error);
                const auto index=vertexId(p);
                if(ids.empty()||ids.back()!=index)ids.push_back(index);
            }
            if(ids.size()>1&&ids.front()==ids.back())ids.pop_back();
            if(ids.size()<3)return fail(output,"Boundary loop collapses after vertex welding",error);
            face.loops.push_back(std::move(ids));
        }
        if(!hasOuter)return fail(output,"Patch boundary has no explicitly identified outer loop",error);
        faces.push_back(std::move(face));
    }

    for(const auto& face:faces)for(const auto& loop:face.loops)for(std::size_t i=0;i<loop.size();++i) {
        const auto a=loop[i],b=loop[(i+1)%loop.size()];
        if(a==b)return fail(output,"Boundary contains a zero-length edge",error);
        const auto edge=std::minmax(a,b);
        ++edgeUses[edge];
        if(!edgeIds.count(edge))edgeIds.emplace(edge,int(edgeIds.size()+1));
    }
    for(const auto& edge:edgeUses)if(edge.second!=2)
        return fail(output,"Observed boundary graph has an open or non-manifold edge",error);

    Vec3 low=vertices.front(),high=vertices.front();
    for(const auto p:vertices) {
        low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};
        high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)};
    }
    const Vec3 span=subtract(high,low);
    const double minimumSpan=std::min({span.x,span.y,span.z});
    if(minimumSpan<=tolerance)return fail(output,"Observed shell has no three-dimensional extent",error);

    modeling::ProfileBodyParams body;
    body.referenceSizeX=body.sizeX=span.x;
    body.referenceSizeY=body.sizeY=span.y;
    body.depth=body.sizeZ=span.z;
    body.boundary.emplace();
    body.boundary->tolerance=std::min(tolerance,minimumSpan*.01);
    for(const auto& input:faces) {
        modeling::BoundaryFace face;
        const auto relativeOrigin=subtract(input.frame.origin,body.pose.origin);
        face.surface.kind=modeling::BoundarySurface::Kind::Plane;
        face.surface.pose={relativeOrigin,input.frame.x,input.frame.normal};
        face.reversed=dot(input.patch->materialNormal,input.frame.normal)<0;
        for(const auto& loop:input.loops) {
            std::vector<modeling::BoundaryEdgeUse> wire;
            for(std::size_t i=0;i<loop.size();++i) {
                const auto a=loop[i],b=loop[(i+1)%loop.size()];
                const auto edge=std::minmax(a,b);
                modeling::BoundaryEdgeUse use;
                use.edgeId=edgeIds.at(edge);
                use.reversed=a>b;
                use.curve.degree=1;
                use.curve.knots={0,1};
                use.curve.multiplicities={2,2};
                use.curve.weights={1,1};
                for(const auto index:{edge.first,edge.second}) {
                    const auto d=subtract(vertices[index],input.frame.origin);
                    use.curve.poles.push_back({dot(d,input.frame.x),dot(d,input.frame.y),0});
                }
                wire.push_back(std::move(use));
            }
            face.wires.push_back(std::move(wire));
        }
        body.boundary->faces.push_back(std::move(face));
    }
    TopoDS_Shape shape;
    if(!modeling::buildProfileBody(body,shape,error)) {
        output.diagnostics.push_back("OpenCASCADE rejected shell: "+error);
        output.status=SolidCandidateStatus::Rejected;
        return false;
    }
    output.boundaryBody=std::move(body);
    output.status=SolidCandidateStatus::Accepted;
    if (preview) preview(shape);
    error.clear();
    return true;
}
} // namespace reconstruction
