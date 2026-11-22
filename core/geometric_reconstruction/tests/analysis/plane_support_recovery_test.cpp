#include "reconstruction/surface/PlaneSupportRecovery.h"
#include "reconstruction/surface/SurfaceAnalysis.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_set>

using namespace reconstruction;
namespace {
void require(bool value,const char* message) {if(!value) throw std::runtime_error(message);}
std::vector<PointSample> grid(int nx,int ny,double origin=0,double z=0) {
    std::vector<PointSample> result;
    for(int x=0;x<nx;++x) for(int y=0;y<ny;++y) {
        PointSample p;p.id=result.size()+1;p.position={origin+x*10.0,y*10.0,z};
        p.normal=modeling::Vec3{0,0,1};result.push_back(p);
    }
    return result;
}
PlaneEvidence seed(const std::vector<PointSample>& points,int first,int count,int stride) {
    PlaneEvidence result;result.plane={{0,0,1},0};
    for(int x=first;x<first+count;++x) for(int y=0;y<5;++y) result.supportPointIds.push_back(points.at(x*stride+y).id);
    return result;
}
PlaneDetectionOptions options() {PlaneDetectionOptions o;o.distanceThreshold=.05;o.minimumSupportPoints=10;return o;}
void largeFaceAndDuplicateSeeds() {
    auto samples=grid(120,60);
    for(int x=0;x<5;++x) for(int y=0;y<5;++y) samples[x*60+y].position.z=x*.005;
    for(size_t i=0;i<samples.size();++i) {
        if(i%5==0) {samples[i].normal.reset();samples[i].normalOrientationKnown=false;}
        else if(i%7==0) samples[i].normal=modeling::Vec3{0,1,0};
    }
    auto a=seed(samples,0,5,60),b=seed(samples,110,5,60);
    // A slightly tilted proposal is re-established from actual seed positions.
    a.plane.normal={.0174524064,0,.999847695};
    PointStore points(samples);const auto fingerprint=pointStoreFingerprint(points);
    std::vector<PrimitiveEvidence> evidence{a};std::string error;
    require(recoverPlaneSupport(points,options(),evidence,error),error.c_str());
    require(std::get<PlaneEvidence>(evidence[0]).supportPointIds.size()==samples.size(),
        "periodic refitting removes local seed bias instead of clipping the far end of a long plane");
    evidence={a,b};
    require(recoverPlaneSupport(points,options(),evidence,error),error.c_str());
    require(evidence.size()==1,"overlapping grown seeds consolidate into one whole plane");
    const auto& plane=std::get<PlaneEvidence>(evidence[0]);
    require(plane.supportPointIds.size()==samples.size(),"a small seed covers a large face despite missing and inconsistent normals");
    require(plane.maxAbsoluteResidual<=.05 && pointStoreFingerprint(points)==fingerprint,
        "expansion respects distance tolerance without changing observations or material orientation");
    const auto expected=plane.supportPointIds;
    std::reverse(samples.begin(),samples.end());evidence={a,b};
    require(recoverPlaneSupport(PointStore(samples),options(),evidence,error),error.c_str());
    require(std::get<PlaneEvidence>(evidence[0]).supportPointIds==expected,"input order does not change geometric ownership");
}
void changingDensity() {
    auto samples=grid(80,30);
    for(int x=0;x<80;++x) for(int y=0;y<30;++y)
        samples[x*30+y].position.x=x<30?x*5.0:150+(x-30)*10.0;
    std::vector<PrimitiveEvidence> evidence{seed(samples,0,5,30)};std::string error;
    require(recoverPlaneSupport(PointStore(samples),options(),evidence,error),error.c_str());
    require(std::get<PlaneEvidence>(evidence[0]).supportPointIds.size()==samples.size(),
        "local neighbor spacing allows expansion across a measured density change");
}
void uncertainSmallSeedCanCorrectItsTilt() {
    auto samples=grid(100,30);
    for(int x=0;x<5;++x) for(int y=0;y<5;++y)
        samples[x*30+y].position.z=std::tan(.05235987755982989)*x*10;
    auto settings=options();settings.distanceThreshold=3.4;
    std::vector<PrimitiveEvidence> evidence{seed(samples,0,5,30)};std::string error;
    require(recoverPlaneSupport(PointStore(samples),settings,evidence,error),error.c_str());
    const auto& plane=std::get<PlaneEvidence>(evidence[0]);
    require(plane.supportPointIds.size()==samples.size() && plane.maxAbsoluteResidual<=settings.distanceThreshold,
        "seed direction uncertainty must not prevent correcting a small biased seed into a supported large plane");
}
void thinWallsAndEdges() {
    auto samples=grid(50,30);
    const auto frontCount=samples.size();
    auto back=grid(50,30,0,1);
    for(auto p:back) {p.id=samples.size()+1;p.normal=modeling::Vec3{0,0,-1};samples.push_back(p);}
    for(int y=0;y<30;++y) for(int z=0;z<30;++z) {
        PointSample p;p.id=samples.size()+1;p.position={490.0,y*10.0,z*10.0};p.normal=modeling::Vec3{1,0,0};samples.push_back(p);
    }
    std::vector<PrimitiveEvidence> evidence{seed(samples,0,5,30)};std::string error;
    require(recoverPlaneSupport(PointStore(samples),options(),evidence,error),error.c_str());
    const auto& support=std::get<PlaneEvidence>(evidence[0]).supportPointIds;
    require(support.size()==frontCount && support.back()==frontCount,
        "growth does not swallow an opposite thin wall or cross a sharp edge");
}
void disconnectedAndProtected() {
    auto samples=grid(40,20);const auto firstCount=samples.size();
    auto other=grid(40,20,2000);
    for(auto p:other) {p.id=samples.size()+1;samples.push_back(p);}
    std::vector<PointId> protectedIds;
    for(int y=0;y<20;++y) protectedIds.push_back(samples[15*20+y].id);
    std::vector<PrimitiveEvidence> evidence{seed(samples,0,5,20)};std::string error;
    require(recoverPlaneSupport(PointStore(samples),options(),evidence,error,protectedIds),error.c_str());
    const auto& support=std::get<PlaneEvidence>(evidence[0]).supportPointIds;
    require(support.size()==firstCount-protectedIds.size() && support.back()<=firstCount,
        "a face grows around protected observations without bridging an unsupported spatial gap");
    for(auto id:protectedIds) require(!std::binary_search(support.begin(),support.end(),id),"manual ownership is preserved");
}
std::vector<PrimitiveEvidence> tiltedAdjacentPlanes(const std::vector<PointSample>& samples, double span)
{
    PlaneEvidence first, second;
    first.plane={{0,0,1},0};
    const double slope=std::tan(2.3*std::acos(-1.0)/180.0);
    const double cosine=1/std::sqrt(1+slope*slope);
    const double sine=slope*cosine;
    second.plane={{-sine,0,cosine},sine*span};
    const auto columns=std::size_t(span/10)+1;
    for(std::size_t i=0;i<samples.size();++i) {
        (i/11<=columns-1?first.supportPointIds:second.supportPointIds).push_back(samples[i].id);
    }
    return {first,second};
}
void relaxedAdjacentPlaneMerge()
{
    constexpr double span=400;
    std::vector<PointSample> samples;
    const double slope=std::tan(2.3*std::acos(-1.0)/180.0);
    for(int x=0;x<=80;++x) for(int y=0;y<=10;++y) {
        PointSample p;
        p.id=samples.size()+1;
        const double positionX=x*10.0;
        p.position={positionX,y*10.0,positionX<=span?0:slope*(positionX-span)};
        p.normal=positionX<=span?modeling::Vec3{0,0,1}:
            modeling::Vec3{-slope/std::sqrt(1+slope*slope),0,1/std::sqrt(1+slope*slope)};
        samples.push_back(p);
    }
    PointStore points(samples);
    std::string error;
    auto planes=tiltedAdjacentPlanes(samples,span);
    auto strictAngle=options();
    strictAngle.distanceThreshold=0.05;
    strictAngle.mergeAngularToleranceRadians=2*std::acos(-1.0)/180;
    require(recoverPlaneSupport(points,strictAngle,planes,error),error.c_str());
    require(planes.size()==2,"the plane merge angle independently gates tilted regions");

    planes=tiltedAdjacentPlanes(samples,span);
    auto strictDistance=options();
    strictDistance.distanceThreshold=3.4;
    strictDistance.mergeResidualToleranceMultiplier=1.0;
    require(recoverPlaneSupport(points,strictDistance,planes,error),error.c_str());
    require(planes.size()==2,"the per-region merge residual independently gates offset planes");

    planes=tiltedAdjacentPlanes(samples,span);
    auto permissive=options();
    permissive.distanceThreshold=3.4;
    permissive.mergeAngularToleranceRadians=2.5*std::acos(-1.0)/180;
    permissive.mergeResidualToleranceMultiplier=1.4;
    require(recoverPlaneSupport(points,permissive,planes,error),error.c_str());
    require(planes.size()==1,"adjacent tilted patches merge after a shared plane fit passes each patch's 95th-percentile residual");
    const auto& merged=std::get<PlaneEvidence>(planes.front());
    require(merged.supportPointIds.size()==samples.size() && merged.maxAbsoluteResidual>permissive.distanceThreshold,
        "merge keeps full patch support while its bounded outliers do not widen detection tolerance");
}
void curvedSurfaceCannotDrift() {
    std::vector<PointSample> samples;PlaneEvidence initial;
    for(int x=-60;x<=60;++x) for(int y=0;y<20;++y) {
        const double angle=x*.005;
        PointSample p;p.id=samples.size()+1;p.position={2000*std::sin(angle),y*10.0,2000*(1-std::cos(angle))};
        p.normal=modeling::Vec3{-std::sin(angle),0,std::cos(angle)};samples.push_back(p);
        if(std::abs(x)<=1) initial.supportPointIds.push_back(p.id);
    }
    std::vector<PrimitiveEvidence> evidence{initial};std::string error;
    require(recoverPlaneSupport(PointStore(samples),options(),evidence,error),error.c_str());
    const auto& plane=std::get<PlaneEvidence>(evidence[0]);
    require(plane.supportPointIds.size()<samples.size()/5 && plane.maxAbsoluteResidual<=.05,
        "periodic refitting cannot walk around a slowly curved surface");
}
void cancellationAndSnapshots() {
    auto samples=grid(40,20);const auto initial=seed(samples,0,5,20);
    std::vector<PrimitiveEvidence> evidence{initial};std::string error;int checks=0;
    require(!recoverPlaneSupport(PointStore(samples),options(),evidence,error,{},[&]{return ++checks>=3;})
        && error=="Cancelled" && std::get<PlaneEvidence>(evidence[0]).supportPointIds==initial.supportPointIds,
        "cancellation publishes no partially expanded evidence");
    auto wrong=initial;wrong.supportPointIds.push_back(999999);evidence={wrong};
    require(!recoverPlaneSupport(PointStore(samples),options(),evidence,error) && evidence.size()==1,
        "foreign point IDs are rejected before publication");
}
void sparseIslandsAndCachePolicy() {
    std::vector<PointSample> samples;
    for(double offset:{0.0,1000.0}) for(auto p:std::vector<modeling::Vec3>{{0,0,0},{1,0,0},{0,1,0}}) {
        p.x+=offset;samples.push_back({PointId(samples.size()+1),p,modeling::Vec3{0,0,1}});
    }
    PlaneEvidence initial;initial.supportPointIds={1,2,3};
    std::vector<PrimitiveEvidence> evidence{initial};std::string error;
    require(recoverPlaneSupport(PointStore(samples),options(),evidence,error),error.c_str());
    require(std::get<PlaneEvidence>(evidence[0]).supportPointIds==initial.supportPointIds,
        "a sparse seed island cannot use distant k-nearest results to bridge empty space");
    SurfaceAnalysisOptions expanded,baseline;baseline.expandPlaneSupport=false;
    require(!analysisSettingsMatch(expanded,baseline),"support recovery policy is part of analysis cache validity");
}
}
int main() {
    try {largeFaceAndDuplicateSeeds();changingDensity();uncertainSmallSeedCanCorrectItsTilt();thinWallsAndEdges();disconnectedAndProtected();relaxedAdjacentPlaneMerge();curvedSurfaceCannotDrift();cancellationAndSnapshots();sparseIslandsAndCachePolicy();
        std::cout<<"Plane support recovery tests passed\n";return 0;}
    catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
