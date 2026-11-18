#include "reconstruction/surface/SurfaceOwnership.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>

namespace {
using namespace reconstruction;
void require(bool value,const char* message) { if(!value)throw std::runtime_error(message); }

struct Fixture {
    std::vector<PointSample> points;
    std::vector<PrimitiveEvidence> evidence;
    Fixture() {
        for(int face=0;face<3;++face) {
            PlaneEvidence plane;plane.plane={{0,0,1},-face*50.};plane.confidence=.99;
            for(int x=0;x<20;++x)for(int y=0;y<20;++y) {
                PointSample p;p.id=points.size()*3+17;p.position={double(x),double(y),face*50.};
                p.normal=modeling::Vec3{0,0,1};points.push_back(p);plane.supportPointIds.push_back(p.id);
            }
            evidence.push_back(plane);
        }
        auto duplicate=std::get<PlaneEvidence>(evidence.front());duplicate.supportPointIds.resize(100);
        evidence.push_back(duplicate);
        PointSample isolated;isolated.id=9000000;isolated.position={1000,1000,1000};points.push_back(isolated);
    }
};

void ownershipAndModes() {
    Fixture fixture;const PointStore source(fixture.points);
    std::set<PointId> baseline;
    for(const auto mode:{SupportValidationMode::Enforce,SupportValidationMode::Report,SupportValidationMode::Off}) {
        SurfaceAnalysisOptions settings;settings.supportValidation.mode=mode;
        settings.detection.plane.distanceThreshold=.05;
        SurfaceAnalysisResult analysis;std::string error;
        require(indexSurfaceEvidence(source,fixture.evidence,settings,analysis,error),error.c_str());
        SurfaceOwnershipOptions options;options.distanceTolerance=.05;options.supportRadius=.2;
        SurfaceOwnershipResult result;
        require(resolveSurfaceOwnership(source,analysis,options,result,error),error.c_str());
        require(result.surfaces.size()==3 && result.assignedPoints==1200,"complete planes survive duplicate proposals in every mode");
        std::set<PointId> owned;
        for(const auto& evidence:result.surfaces)std::visit([&](const auto& e){
            for(auto id:e.supportPointIds)require(id!=9000000 && owned.insert(id).second,"unique source IDs without absorbing distant observations");
        },evidence);
        if(baseline.empty())baseline=owned;else require(baseline==owned,"report/off retain the same eligible observations");
        require(mode!=SupportValidationMode::Enforce || result.preferredPlaneCount>=3,"broad validated planes retain region priority");
        require(mode!=SupportValidationMode::Off || result.finalSupportValidation.decisions.empty(),"off bypasses final validation");
        require(mode!=SupportValidationMode::Report || !result.finalSupportValidation.decisions.empty(),"report computes final diagnostics");

        SurfaceOwnershipResult unchanged;unchanged.assignedPoints=17;
        int callbacks=0;
        require(!resolveSurfaceOwnership(source,analysis,options,unchanged,error,[&]{return ++callbacks>2;}),"cancellation rejects publication");
        require(unchanged.assignedPoints==17 && unchanged.surfaces.empty(),"cancellation leaves the output transaction unchanged");
        auto moved=fixture.points;moved.front().position.x+=.001;
        require(!resolveSurfaceOwnership(PointStore(moved),analysis,options,unchanged,error),"stale observations are rejected");
        require(unchanged.assignedPoints==17,"stale identity leaves output unchanged");
        if(mode!=SupportValidationMode::Off) {
            auto changed=analysis;changed.options.supportValidation.planeDensityRatioMax*=2;
            require(!resolveSurfaceOwnership(source,changed,options,unchanged,error),"changed validation settings require a new report");
            require(unchanged.assignedPoints==17,"settings rejection is transactional");
        }
    }
}
void recoverSupportedPlaneHolesAndRejectedConePoints() {
    Fixture fixture;
    fixture.evidence.pop_back();
    std::set<PointId> missing,opposite;
    for (auto& p:fixture.points) {
        const auto x=p.position.x-10,y=p.position.y-10;
        if (x*x+y*y>=36) continue;
        if (p.position.z==0) missing.insert(p.id);
        if (p.position.z==50) {
            opposite.insert(p.id);
            // Even a similarly oriented layer is separate if it sits below the rim.
            if (x<0) {p.position.z-=.01;p.normal=modeling::Vec3{0,0,-1};}
            else p.position.z-=.04;
        }
    }
    for (auto& e:fixture.evidence) {
        auto& ids=std::get<PlaneEvidence>(e).supportPointIds;
        ids.erase(std::remove_if(ids.begin(),ids.end(),[&](auto id){return missing.count(id)||opposite.count(id);}),ids.end());
    }
    ConeEvidence cone;
    cone.cone={{3,3,-1},{0,0,1},std::atan(1.)};
    for (int i=0;i<5;++i) {
        PointSample p;p.id=9000001+i;
        p.position={3+1.08*std::cos(i*.1),3+1.08*std::sin(i*.1),.08};
        if (i==4)p.normal=modeling::Vec3{0,0,-1};
        fixture.points.push_back(p);cone.supportPointIds.push_back(p.id);
    }
    fixture.evidence.push_back(cone);
    const PointStore source(fixture.points);
    SurfaceAnalysisOptions settings;
    settings.detection.plane.distanceThreshold=.05;
    settings.detection.cone.distanceThreshold=.05;
    SurfaceAnalysisResult analysis;
    std::string error;
    require(indexSurfaceEvidence(source,fixture.evidence,settings,analysis,error),error.c_str());
    SurfaceOwnershipOptions options;options.distanceTolerance=.05;options.supportRadius=1;
    SurfaceOwnershipResult result;
    require(resolveSurfaceOwnership(source,analysis,options,result,error),error.c_str());
    require(result.surfaces.size()==3,"sparse circular trace does not become a cone");
    require(result.finalSupportValidation.planeHoleRecoveredPoints==missing.size(),
        "missing plane interior is recovered within the original fit tolerance");
    require(result.finalSupportValidation.planeRelaxedRecoveredPoints==4,
        "rejected cone trace on the host returns without its opposing observation");
    std::set<PointId> used;
    for (const auto& e:result.surfaces) {
        const auto& plane=std::get<PlaneEvidence>(e);
        require(plane.maxAbsoluteResidual<.001,"relaxed ownership does not change the fitted residual");
        for (auto id:plane.supportPointIds) {
            require(!opposite.count(id) && id!=9000005 && used.insert(id).second,
                "an oppositely oriented layer in an opening stays separate");
        }
    }
}
}
int main() {
    try {ownershipAndModes();recoverSupportedPlaneHolesAndRejectedConePoints();std::cout<<"surface_ownership_test passed\n";return 0;}
    catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
