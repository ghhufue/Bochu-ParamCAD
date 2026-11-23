#include "reconstruction/solid_inference/SolidInference.h"

#include "reconstruction/modeling/ModelingAdapter.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace reconstruction {
namespace {
using Vec3=modeling::Vec3;
Vec3 add(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 mul(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
double norm(Vec3 a){return std::sqrt(dot(a,a));}
Vec3 unit(Vec3 a){return mul(a,1/norm(a));}
Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}

const ScenePatch* findPatch(const SurfaceScene& scene,PatchId id){
    const auto it=std::find_if(scene.patches.begin(),scene.patches.end(),[&](const auto& p){return p.id==id;});
    return it==scene.patches.end()?nullptr:&*it;
}
bool explicitProfile(const ScenePatch& patch,const BoundaryLoop3d*& outer,
                     std::vector<const BoundaryLoop3d*>& holes){
    outer=nullptr;
    for(const auto& loop:patch.loops) {
        if(!loop.closed||loop.vertices.size()<3||loop.role==SceneLoopRole::Unknown||loop.role==SceneLoopRole::Island)return false;
        if(loop.role==SceneLoopRole::Outer) {if(outer)return false;outer=&loop;}
        if(loop.role==SceneLoopRole::Hole)holes.push_back(&loop);
    }
    return outer!=nullptr;
}
bool matchLoops(const BoundaryLoop3d& a,const BoundaryLoop3d& b,Vec3 normal,double separation,double tolerance,
                double* maximumDeviationOutput=nullptr) {
    if(!a.closed||!b.closed||a.vertices.size()<3||b.vertices.size()<3)return false;
    const auto distanceToLoop=[](Vec3 point,const std::vector<Vec3>& loop) {
        double nearest=INFINITY;
        for(std::size_t index=0;index<loop.size();++index) {
            const auto first=loop[index],edge=sub(loop[(index+1)%loop.size()],first);
            const double edgeLengthSquared=dot(edge,edge);
            if(edgeLengthSquared<=1e-18)continue;
            const double parameter=std::clamp(dot(sub(point,first),edge)/edgeLengthSquared,0.0,1.0);
            nearest=std::min(nearest,norm(sub(point,add(first,mul(edge,parameter)))));
        }
        return nearest;
    };
    const auto translatedB=[&] {
        auto vertices=b.vertices;
        for(auto& point:vertices)point=sub(point,mul(normal,separation));
        return vertices;
    }();
    std::vector<double> deviations;
    for(const auto point:a.vertices)
        deviations.push_back(distanceToLoop(add(point,mul(normal,separation)),b.vertices));
    for(const auto point:translatedB)
        deviations.push_back(distanceToLoop(point,a.vertices));
    std::sort(deviations.begin(),deviations.end());
    const double maximumDeviation=deviations.back();
    const double percentile95=deviations[std::size_t(.95*(deviations.size()-1))];
    if(maximumDeviationOutput)*maximumDeviationOutput=maximumDeviation;
    if(percentile95>tolerance*3||maximumDeviation>tolerance*6)return false;
    return true;
}

struct CircularBoundary {
    Vec3 center;
    double radius=0;
};

std::optional<CircularBoundary> fitCircularBoundary(const BoundaryLoop3d& loop,double tolerance) {
    Vec3 center{};
    for(const auto point:loop.vertices)center=add(center,point);
    center=mul(center,1.0/loop.vertices.size());
    double radius=0;
    for(const auto point:loop.vertices)radius+=norm(sub(point,center));
    radius/=loop.vertices.size();
    if(!std::isfinite(radius)||radius<=tolerance)return {};
    for(const auto point:loop.vertices)
        if(std::abs(norm(sub(point,center))-radius)>std::max(3*tolerance,.03*radius))return {};
    return CircularBoundary{center,radius};
}

std::optional<PatchId> supportingCylinder(const SurfaceScene& scene,const CircularBoundary& hole,
    Vec3 extrusionAxis,Vec3 firstCenter,Vec3 secondCenter,double tolerance,
    const std::set<PatchId>& usedCylinders) {
    std::optional<PatchId> match;
    for(const auto& patch:scene.patches) {
        if(usedCylinders.count(patch.id)||patch.supportPointIds.empty()||!patch.supportConfidence)continue;
        const auto& evidence=scene.surfaces[std::size_t(patch.surfaceId-1)];
        const auto* cylinder=std::get_if<CylinderEvidence>(&evidence);
        if(!cylinder||cylinder->angularCoverage<.7)continue;
        const auto axisLength=norm(cylinder->cylinder.axisDirection);
        if(axisLength<1e-9)continue;
        const auto axis=unit(cylinder->cylinder.axisDirection);
        if(std::abs(dot(axis,extrusionAxis))<std::cos(scene.header.angularToleranceRad))continue;
        if(std::abs(cylinder->cylinder.radius-hole.radius)>std::max(3*tolerance,.03*hole.radius))continue;
        const auto fromAxis=sub(hole.center,cylinder->cylinder.axisPoint);
        const auto radial=sub(fromAxis,mul(axis,dot(fromAxis,axis)));
        if(norm(radial)>3*tolerance)continue;
        const double a=dot(sub(firstCenter,cylinder->cylinder.axisPoint),axis);
        const double b=dot(sub(secondCenter,cylinder->cylinder.axisPoint),axis);
        const double low=std::min(a,b),high=std::max(a,b);
        if(cylinder->axialMinimum>low+2*tolerance||cylinder->axialMaximum<high-2*tolerance)continue;
        if(match)return {};
        match=patch.id;
    }
    return match;
}

struct HoleCut {
    const BoundaryLoop3d* opening;
    CircularBoundary circle;
    double start;
    double end;
    std::optional<PatchId> floor;
    PatchId wall;
};

std::optional<HoleCut> matchBlindHole(const SurfaceScene& scene,const BoundaryLoop3d& opening,
    const ScenePatch& cap,Vec3 axis,double direction,double openingDepth,double hostDepth,
    double tolerance,const std::set<PatchId>& usedCylinders) {
    const double parallelCosine=std::cos(scene.header.angularToleranceRad);
    const auto intoHost=mul(axis,direction);
    const auto circle=fitCircularBoundary(opening,tolerance);
    if(!circle||!cap.orientationKnown||dot(cap.materialNormal,intoHost)>=-parallelCosine)
        return {};
    std::optional<HoleCut> match;
    for(const auto& patch:scene.patches) {
        if(!patch.orientationKnown||patch.boundaryStatus!=SceneBoundaryStatus::ObservedClosed||
           patch.supportPointIds.empty()||dot(patch.materialNormal,intoHost)>=-parallelCosine)
            continue;
        if(!std::holds_alternative<PlaneEvidence>(scene.surfaces[std::size_t(patch.surfaceId-1)]))
            continue;
        const BoundaryLoop3d* floor=nullptr;
        std::vector<const BoundaryLoop3d*> floorHoles;
        if(!explicitProfile(patch,floor,floorHoles)||!floorHoles.empty())continue;
        const double cutDepth=dot(sub(floor->vertices.front(),circle->center),intoHost);
        if(cutDepth<=2*tolerance||cutDepth>=hostDepth-2*tolerance||
           !matchLoops(opening,*floor,intoHost,cutDepth,tolerance))continue;
        const auto floorCenter=add(circle->center,mul(intoHost,cutDepth));
        const auto wall=supportingCylinder(scene,*circle,axis,circle->center,floorCenter,tolerance,usedCylinders);
        if(!wall)continue;
        if(match)return {};
        match=HoleCut{&opening,*circle,openingDepth,openingDepth+direction*cutDepth,patch.id,*wall};
    }
    return match;
}

std::vector<modeling::ProfileCurve2D> profileCurves(const BoundaryLoop3d& loop,Vec3 origin,Vec3 x,Vec3 y) {
    std::vector<modeling::Point2D> points;
    points.reserve(loop.vertices.size());
    for(const auto point:loop.vertices) {
        const auto relative=sub(point,origin);
        points.push_back({dot(relative,x),dot(relative,y)});
    }
    std::vector<modeling::ProfileCurve2D> curves;
    curves.reserve(points.size());
    for(std::size_t i=0;i<points.size();++i) {
        const auto first=points[i],second=points[(i+1)%points.size()];
        modeling::ProfileCurve2D edge;
        edge.x1=first.x;edge.y1=first.y;edge.x2=second.x;edge.y2=second.y;
        curves.push_back(edge);
    }
    return curves;
}

std::vector<modeling::SketchEntity> loopSketch(const BoundaryLoop3d& loop,Vec3 origin,Vec3 x,Vec3 y) {
    std::vector<modeling::SketchEntity> entities;
    entities.reserve(loop.vertices.size());
    for(std::size_t i=0;i<loop.vertices.size();++i) {
        const auto first=sub(loop.vertices[i],origin);
        const auto second=sub(loop.vertices[(i+1)%loop.vertices.size()],origin);
        entities.push_back({static_cast<modeling::SketchEntityId>(i+1),modeling::Line2D{
            dot(first,x),dot(first,y),dot(second,x),dot(second,y)}});
    }
    return entities;
}

bool isSampledRectangle(const std::vector<modeling::Point2D>& polygon) {
    if (polygon.size() < 4) return false;
    double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
    for (const auto& point:polygon) {
        xmin=std::min(xmin,point.x); xmax=std::max(xmax,point.x);
        ymin=std::min(ymin,point.y); ymax=std::max(ymax,point.y);
    }
    const double scale=std::max({1.0,xmax-xmin,ymax-ymin});
    const double tolerance=1e-8*scale;
    const auto onBoundary=[&](const modeling::Point2D& point) {
        return std::abs(point.x-xmin)<=tolerance||std::abs(point.x-xmax)<=tolerance||
               std::abs(point.y-ymin)<=tolerance||std::abs(point.y-ymax)<=tolerance;
    };
    if (std::any_of(polygon.begin(),polygon.end(),[&](const auto& point) { return !onBoundary(point); }))
        return false;
    const auto hasCorner=[&](double x,double y) {
        return std::any_of(polygon.begin(),polygon.end(),[&](const auto& point) {
            return std::abs(point.x-x)<=tolerance&&std::abs(point.y-y)<=tolerance;
        });
    };
    return hasCorner(xmin,ymin)&&hasCorner(xmax,ymin)&&hasCorner(xmax,ymax)&&hasCorner(xmin,ymax);
}
}

bool proposeParallelCapExtrusion(const SurfaceScene& scene,PatchId firstId,PatchId secondId,
                                SolidCandidate& output,std::string& error, const SolidPreview& preview) {
    output={};output.analysisFingerprint=scene.header.analysisFingerprint;
    output.supportingPatches={firstId,secondId};
    if(const auto valid=validateSurfaceScene(scene);!valid){error=valid.message;output.diagnostics.push_back(error);return false;}
    const auto* first=findPatch(scene,firstId);const auto* second=findPatch(scene,secondId);
    if(!first||!second||firstId==secondId){error="Parallel-cap hypothesis references invalid patches";output.diagnostics.push_back(error);return false;}
    const double tolerance=std::max({scene.header.distanceToleranceMm,first->spacingMm,second->spacingMm});
    if(!std::holds_alternative<PlaneEvidence>(scene.surfaces[std::size_t(first->surfaceId-1)])||
       !std::holds_alternative<PlaneEvidence>(scene.surfaces[std::size_t(second->surfaceId-1)])) {
        error="Parallel-cap extrusion requires two planar patches";output.diagnostics.push_back(error);return false;
    }
    const auto& p1=std::get<PlaneEvidence>(scene.surfaces[std::size_t(first->surfaceId-1)]).plane;
    const auto& p2=std::get<PlaneEvidence>(scene.surfaces[std::size_t(second->surfaceId-1)]).plane;
    const auto n1=unit(p1.normal),n2=unit(p2.normal);
    if(std::abs(dot(n1,n2))<std::cos(scene.header.angularToleranceRad)) {
        error="Candidate patches are not parallel";output.diagnostics.push_back(error);return false;
    }
    const double direction=dot(n1,n2)>=0?1:-1;
    const double distance=-(direction*p2.offset/norm(p2.normal)-p1.offset/norm(p1.normal));
    const double depth=std::abs(distance);
    if(!std::isfinite(depth)||depth<=tolerance) {
        error="Parallel patches have no positive finite separation";output.diagnostics.push_back(error);return false;
    }
    const auto axis=distance>=0?n1:mul(n1,-1);
    const BoundaryLoop3d *outer1=nullptr,*outer2=nullptr;
    std::vector<const BoundaryLoop3d*> holes1,holes2;
    if(!explicitProfile(*first,outer1,holes1)||!explicitProfile(*second,outer2,holes2)) {
        error="Cap boundary loops lack closed, classified outer profiles";output.diagnostics.push_back(error);return false;
    }
    const auto origin=outer1->vertices.front();
    const double projectedSeparation=dot(sub(outer2->vertices.front(),origin),axis);
    double outerDeviation=0;
    if(!matchLoops(*outer1,*outer2,axis,projectedSeparation,tolerance,&outerDeviation)||
       std::abs(projectedSeparation-depth)>tolerance*3) {
        error="Outer cap loops do not match within the sampling-aware scene tolerance (vertices="+
            std::to_string(outer1->vertices.size())+"/"+std::to_string(outer2->vertices.size())+
            ", deviation="+std::to_string(outerDeviation)+", tolerance="+std::to_string(tolerance)+")";
        output.diagnostics.push_back(error);return false;
    }
    std::vector<HoleCut> cuts;
    std::set<const BoundaryLoop3d*> matchedSecondHoles;
    std::set<PatchId> holeWallPatches;
    for(const auto* firstHole:holes1) {
        const BoundaryLoop3d* match=nullptr;
        for(const auto* secondHole:holes2) {
            if(matchedSecondHoles.count(secondHole)||!matchLoops(*firstHole,*secondHole,axis,
                projectedSeparation,tolerance))continue;
            if(match) {
                error="Parallel cap hole boundary mapping is ambiguous";
                output.diagnostics.push_back(error);return false;
            }
            match=secondHole;
        }
        if(!match) {
            auto blind=matchBlindHole(scene,*firstHole,*first,axis,1,0,depth,tolerance,holeWallPatches);
            if(!blind) {
                error="Unmatched cap opening lacks a supported blind-hole floor and cylinder wall";
                output.diagnostics.push_back(error);return false;
            }
            holeWallPatches.insert(blind->wall);
            cuts.push_back(*blind);
            continue;
        }
        matchedSecondHoles.insert(match);
        const auto circle=fitCircularBoundary(*firstHole,tolerance);
        if(!circle) {
            error="Matched through-hole boundary is not a supported circular profile";
            output.diagnostics.push_back(error);return false;
        }
        const auto wall=supportingCylinder(scene,*circle,axis,circle->center,
            add(circle->center,mul(axis,projectedSeparation)),tolerance,holeWallPatches);
        if(!wall) {
            error="Matched cap openings lack a coaxial observed cylinder wall across the extrusion depth";
            output.diagnostics.push_back(error);return false;
        }
        holeWallPatches.insert(*wall);
        cuts.push_back({firstHole,*circle,0,depth,{},*wall});
    }
    for(const auto* secondHole:holes2) {
        if(matchedSecondHoles.count(secondHole))continue;
        auto blind=matchBlindHole(scene,*secondHole,*second,axis,-1,depth,depth,tolerance,holeWallPatches);
        if(!blind) {
            error="Unmatched cap opening lacks a supported blind-hole floor and cylinder wall";
            output.diagnostics.push_back(error);return false;
        }
        holeWallPatches.insert(blind->wall);
        cuts.push_back(*blind);
    }
    const auto reference=std::abs(axis.x)<.8?Vec3{1,0,0}:Vec3{0,1,0};
    const auto x=unit(cross(reference,axis));
    const auto y=cross(axis,x);
    modeling::ProfileBodyParams body;
    body.pose.origin=origin;
    body.pose.xDirection=x;body.pose.zDirection=axis;
    body.depth=body.sizeZ=depth;
    double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
    std::vector<modeling::Point2D> polygon;
    for(const auto p:outer1->vertices) {
        const auto d=sub(p,origin);const double u=dot(d,x),v=dot(d,y);
        polygon.push_back({u,v});xmin=std::min(xmin,u);xmax=std::max(xmax,u);ymin=std::min(ymin,v);ymax=std::max(ymax,v);
        modeling::ProfileCurve2D edge;edge.x1=u;edge.y1=v;edge.x2=0;edge.y2=0;
        body.outline.push_back(edge);
    }
    if (cuts.empty() && polygon.size() > 4 && isSampledRectangle(polygon)) {
        auto medianGap = [](std::vector<double> values) {
            std::sort(values.begin(), values.end());
            values.erase(std::unique(values.begin(), values.end(), [](double a,double b) {
                return std::abs(a-b)<1e-9;
            }), values.end());
            std::vector<double> gaps;
            for (std::size_t i=1;i<values.size();++i) if (values[i]-values[i-1]>1e-9)
                gaps.push_back(values[i]-values[i-1]);
            return gaps.empty()?0.0:gaps[gaps.size()/2];
        };
        std::vector<double> us,vs;
        for (const auto& point:polygon) { us.push_back(point.x); vs.push_back(point.y); }
        const double spacing=std::min(medianGap(us),medianGap(vs));
        if (std::isfinite(spacing)&&spacing>0) {
            xmin-=spacing*.5;xmax+=spacing*.5;ymin-=spacing*.5;ymax+=spacing*.5;
            polygon={{xmin,ymin},{xmax,ymin},{xmax,ymax},{xmin,ymax}};
            body.outline.clear();body.outline.resize(4);
        }
    }
    const auto profileOrigin=cuts.empty()?add(origin,add(mul(x,xmin),mul(y,ymin))):origin;
    if (cuts.empty()) {
        for (auto& point:polygon) { point.x-=xmin; point.y-=ymin; }
        xmin=0;xmax=body.sizeX;ymin=0;ymax=body.sizeY;
    }
    if (cuts.empty()) body.pose.origin=profileOrigin;
    for(std::size_t i=0;i<body.outline.size();++i) {
        const auto start=polygon[i], next=polygon[(i+1)%polygon.size()];
        body.outline[i].x1=start.x;body.outline[i].y1=start.y;
        body.outline[i].x2=next.x;body.outline[i].y2=next.y;
    }
    body.referenceSizeX=body.sizeX=std::max(xmax-xmin,scene.header.distanceToleranceMm);
    body.referenceSizeY=body.sizeY=std::max(ymax-ymin,scene.header.distanceToleranceMm);
    SolidFeatureOperation baseOperation;
    baseOperation.plane=modeling::explicitPlane(profileOrigin,axis,x);
    baseOperation.entities.reserve(polygon.size());
    for(std::size_t i=0;i<polygon.size();++i) {
        const auto startPoint=polygon[i],endPoint=polygon[(i+1)%polygon.size()];
        baseOperation.entities.push_back({static_cast<modeling::SketchEntityId>(i+1),
            modeling::Line2D{startPoint.x,startPoint.y,endPoint.x,endPoint.y}});
    }
    baseOperation.depth=depth;
    baseOperation.operation=modeling::ExtrudeOperation::NewBody;
    output.featureProgram.push_back(std::move(baseOperation));
    const double inferredWallConfidence = std::clamp(
        std::min(outer1->confidence, outer2->confidence), 0.0, 1.0);
    output.inferredSurfaces.reserve(outer1->vertices.size());
    for (std::size_t i = 0; i < outer1->vertices.size(); ++i) {
        const auto firstVertex = outer1->vertices[i];
        const auto secondVertex = outer1->vertices[(i + 1) % outer1->vertices.size()];
        output.inferredSurfaces.push_back({
            PrimitiveKind::Plane,
            {firstId, secondId},
            {firstVertex, secondVertex, add(secondVertex, mul(axis, depth)),
             add(firstVertex, mul(axis, depth))},
            inferredWallConfidence,
            "Side wall inferred by extruding a matched observed cap edge"});
    }
    for(const auto& cut:cuts) {
        SolidFeatureOperation cutOperation;
        const auto cutOrigin=add(profileOrigin,mul(axis,cut.start));
        cutOperation.plane=modeling::explicitPlane(cutOrigin,axis,x);
        cutOperation.depth=std::abs(cut.end-cut.start);
        cutOperation.reverse=cut.end<cut.start;
        if(cut.floor) {
            const auto center=sub(cut.circle.center,origin);
            const double centerX=dot(center,x),centerY=dot(center,y);
            body.cavities.push_back({centerX,centerY,std::min(cut.start,cut.end),
                std::max(cut.start,cut.end),cut.circle.radius,cut.circle.radius});
            cutOperation.entities.push_back({1,modeling::Circle2D{centerX,centerY,cut.circle.radius}});
            output.supportingPatches.push_back(*cut.floor);
        } else {
            body.profileVoids.push_back(profileCurves(*cut.opening,origin,x,y));
            cutOperation.entities=loopSketch(*cut.opening,cutOrigin,x,y);
        }
        cutOperation.operation=modeling::ExtrudeOperation::Cut;
        output.featureProgram.push_back(std::move(cutOperation));
    }
    TopoDS_Shape shape;
    if(!modeling::buildProfileBody(body,shape,error)) {
        output.diagnostics.push_back("OCCT rejected parallel-cap extrusion: "+error);return false;
    }
    output.boundaryBody=std::move(body);
    output.supportingPatches.insert(output.supportingPatches.end(),holeWallPatches.begin(),holeWallPatches.end());
    output.inferredSideWalls=true;
    output.depthEvidence="distance between parallel fitted plane equations";
    output.status=SolidCandidateStatus::Proposed;
    output.relationError=std::abs(std::abs(dot(n1,n2))-1);
    output.distanceError=0;
    output.complexityPenalty=.05;
    output.score=output.relationError+output.complexityPenalty;
    output.diagnostics.push_back(cuts.empty()
        ? "Extrusion depth is supported by parallel patch separation; side walls are inferred"
        : "Cuts are supported by cap openings, observed cylinder walls and a matching exit or floor; outer side walls are inferred");
    if (preview) preview(shape);
    error.clear();
    return true;
}
} // namespace reconstruction
