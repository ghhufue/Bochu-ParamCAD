#include "reconstruction/surface/SurfaceSupportValidation.h"
#include "reconstruction/settings/ModuleConfig.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>

using namespace reconstruction;
namespace {
constexpr double pi=3.14159265358979323846;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Fixture {
    std::vector<PointSample> points;
    std::vector<PrimitiveEvidence> evidence;
    void add(PointSample p,std::vector<PointId>& ids){p.id=points.size()+1;ids.push_back(p.id);points.push_back(p);}
    void plane(double z,int count=20,double step=1) {
        PlaneEvidence e;e.plane={{0,0,1},-z};e.confidence=.99;
        for(int i=0;i<count;++i)for(int j=0;j<count;++j){PointSample p;p.position={i*step,j*step,z};p.normal=modeling::Vec3{0,0,1};add(p,e.supportPointIds);}
        evidence.emplace_back(e);
    }
};
SurfaceSupportValidationReport run(const Fixture& f,std::vector<PrimitiveEvidence>& accepted,SurfaceAnalysisOptions options={}) {
    options.detection.plane.distanceThreshold=.02;options.detection.sphere.distanceThreshold=.02;options.detection.cylinder.distanceThreshold=.02;
    options.detection.cone.distanceThreshold=.02;
    PointStore points(f.points);std::string error;SurfaceSupportValidationReport report;
    require(validateSurfaceSupport(points,f.evidence,options,accepted,report,error),error.c_str());return report;
}
void densityAndModes() {
    Fixture f;f.plane(0);f.plane(50);f.plane(100);
    PlaneEvidence bad;bad.plane={{0,0,1},-200};bad.confidence=.99;
    for(int i=0;i<25;++i){PointSample p;p.position={i*10.,0,200};f.add(p,bad.supportPointIds);p.position={240,i*10.,200};f.add(p,bad.supportPointIds);}f.evidence.emplace_back(bad);
    std::vector<PrimitiveEvidence> accepted;auto r=run(f,accepted);
    require(r.reference.available,"three large independent planes establish density reference");
    require(accepted.size()==3,"boundary-only L must not become a plane");
    require(r.decisions.back().verdict!=SupportVerdict::Accepted,"rejected L has a diagnostic");
    SurfaceAnalysisOptions o;o.supportValidation.mode=SupportValidationMode::Report;r=run(f,accepted,o);
    require(accepted.size()==f.evidence.size(),"report mode preserves proposals");
    o.supportValidation.mode=SupportValidationMode::Off;r=run(f,accepted,o);
    require(accepted.size()==f.evidence.size() && r.decisions.empty(),"off is exact bypass");
    const auto signature=surfaceSupportSettingsSignature(o);o.supportValidation.planeDensityRatioMax+=1;
    require(signature!=surfaceSupportSettingsSignature(o),"density threshold invalidates signature");
    auto a=o,b=o;b.supportMeshing.meshPointBudget*=2;
    require(surfaceSupportSettingsSignature(a)==surfaceSupportSettingsSignature(b),"display budget does not change validation");
}
void cylinder(bool narrow) {
    Fixture f;CylinderEvidence c;c.cylinder={{0,0,0},{0,0,1},narrow?3000.:2.};c.confidence=.99;
    for(int ring=0;ring<6;++ring)for(int i=0;i<96;++i){const double t=narrow?-.01+.02*i/95:2*pi*i/96;PointSample p;p.position={c.cylinder.radius*std::cos(t),c.cylinder.radius*std::sin(t),double(ring)};p.normal=modeling::Vec3{std::cos(t),std::sin(t),0};f.add(p,c.supportPointIds);}f.evidence.emplace_back(c);
    std::vector<PrimitiveEvidence> accepted;const auto r=run(f,accepted);
    if(!narrow && accepted.empty())std::cerr<<serializeSurfaceSupportReport(r)<<'\n';
    require(narrow?accepted.empty():!accepted.empty(),narrow?"long shallow cylinder strip is rejected":"full radius-2 cylinder keeps valid support");
    if(!narrow)require(std::get<CylinderEvidence>(accepted.front()).supportPointIds.size()>=570,"anisotropic cylinder sampling retains its rings");
}
void spheres() {
    Fixture f;SphereEvidence s;s.sphere={{0,0,0},3000};s.confidence=.99;
    for(int i=0;i<20;++i)for(int j=0;j<20;++j){const double u=-.02+i*.04/19,v=-.02+j*.04/19,k=3000/std::sqrt(1+u*u+v*v);PointSample p;p.position={k*u,k*v,k};f.add(p,s.supportPointIds);}f.evidence.emplace_back(s);
    std::vector<PrimitiveEvidence> accepted;const auto r=run(f,accepted);
    require(accepted.empty(),"shallow sphere cannot be published as curved evidence");
    require(r.decisions.front().solidAngle<.02,"sphere coverage is solid angle, independent of radius");
    Fixture full;SphereEvidence sphere;sphere.sphere={{0,0,0},2};sphere.confidence=.99;
    for(int i=1;i<20;++i)for(int j=0;j<48;++j){const double t=pi*i/20,a=2*pi*j/48;PointSample p;p.position={2*std::sin(t)*std::cos(a),2*std::sin(t)*std::sin(a),2*std::cos(t)};full.add(p,sphere.supportPointIds);}full.evidence.emplace_back(sphere);
    const auto large=run(full,accepted);
    if(accepted.empty())std::cerr<<serializeSurfaceSupportReport(large)<<'\n';
    require(!accepted.empty(),"full sphere survives multiple charts and pole handling");
    require(large.decisions.front().solidAngle>11 && large.decisions.front().solidAngle<4*pi+.001,"sphere charts cover its surface without duplicate area");
    Fixture centers;SphereEvidence invalid;invalid.sphere={{0,0,0},2};
    for(int i=0;i<20;++i){PointSample p;p.position={0,0,0};centers.add(p,invalid.supportPointIds);}centers.evidence.emplace_back(invalid);
    const auto degenerate=run(centers,accepted);
    require(accepted.empty() && !degenerate.decisions.empty(),"sphere-center observations are rejected without undefined radial normalization");
}
void cones() {
    for (bool shallow:{false,true}) {
        Fixture fixture;
        ConeEvidence cone;
        const double angle=shallow?89*pi/180:pi/6;
        cone.cone={{0,0,0},{0,0,1},angle};
        const double radius=shallow?30:2;
        const double span=shallow?.2:1.6;
        for (int row=0;row<12;++row) for (int column=0;column<24;++column) {
            const double a=span*(column/23.-.5);
            const double slant=radius/std::sin(angle)+row*.2;
            const double r=slant*std::sin(angle);
            PointSample p;
            p.position={r*std::cos(a),r*std::sin(a),slant*std::cos(angle)};
            fixture.add(p,cone.supportPointIds);
        }
        fixture.evidence.emplace_back(cone);
        std::vector<PrimitiveEvidence> accepted;
        const auto report=run(fixture,accepted);
        const auto& decision=report.decisions.front();
        require(decision.effectivePoints>200 && decision.area>0,"cone checks its sampled two-dimensional domain");
        if (shallow) {
            require(accepted.empty(),"near-planar cone is rejected");
            require(std::find(decision.reasons.begin(),decision.reasons.end(),"CurvatureBelowNoise")!=decision.reasons.end(),
                "cone sagitta includes the half opening angle");
        } else {
            require(accepted.size()==1 && decision.planeCost>decision.curvedCost,
                "a small cone across the angular seam keeps its support");
        }
    }
}
void transactionAndSettings() {
    Fixture f;f.plane(0);PointStore points(f.points);SurfaceAnalysisOptions o;
    std::vector<PrimitiveEvidence> output=f.evidence;SurfaceSupportValidationReport report;report.pointFingerprint=42;std::string error;
    require(!validateSurfaceSupport(points,f.evidence,o,output,report,error,nullptr,[]{return true;}),"cancellation fails");
    require(report.pointFingerprint==42 && output.size()==1,"cancellation publishes no partial state");
    AnalysisSettings settings;setAnalysisSetting(settings,"support_validation_mode=report");
    require(settings.supportValidation.mode==SupportValidationMode::Report,"text mode alias");
    bool invalid=false;try{setAnalysisSetting(settings,"plane_density_ratio_max=.5");}catch(const std::exception&){invalid=true;}
    require(invalid,"invalid density ratio is rejected");
}
void heterogeneousReference() {
    Fixture f;f.plane(0);f.plane(50);f.plane(100,20,3);
    std::vector<PrimitiveEvidence> accepted;const auto report=run(f,accepted);
    require(!report.reference.available && report.reference.reason=="UnstableDensityReference","equal majority must not hide a distinct sampling regime");
    require(accepted.size()==3,"heterogeneous reference falls back to local support without deleting real faces");
}
void invalidRing() {
    Fixture f;TorusEvidence torus;torus.torus={{0,0,0},{0,0,1},2,10};torus.confidence=.99;
    for(int i=0;i<20;++i){PointSample p;p.position={double(i),0,0};f.add(p,torus.supportPointIds);}f.evidence.emplace_back(torus);
    std::vector<PrimitiveEvidence> accepted;const auto r=run(f,accepted);
    require(accepted.empty() && r.decisions.front().reasons.front()=="InvalidPrimitiveGeometry","unsupported spindle torus cannot take ownership from a plane");
}
void footprintsKeepHolesAndStrips() {
    std::vector<PlanarSurfaceSample> samples;
    for(int x=0;x<40;++x)for(int y=0;y<40;++y)if((x-20)*(x-20)+(y-20)*(y-20)>=64)
        samples.push_back({{double(x),double(y)},PointId(samples.size()+1)});
    SurfaceMeshingSettings settings;const auto mesh=buildPlanarSurfaceFootprint(samples,settings);
    const auto area=[](const SurfaceFootprint& f){double sum=0;for(const auto& t:f.triangles){const auto a=f.positions[t[0]],b=f.positions[t[1]],c=f.positions[t[2]];sum+=std::abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))*.5;}return sum;};
    require(!mesh.triangles.empty(),"holed plane has a supported finite domain");
    const SurfaceFootprintQuery query(mesh);
    require(query.containsProjection({2.5,2.5,0}) && !query.containsProjection({20,20,0}),"finite-domain membership fills supported interior without crossing a hole");
    auto moved=mesh;moved.origin={1000,2000,3000};moved.axisU={0,1,0};moved.axisV={0,0,1};
    const SurfaceFootprintQuery frame(moved);
    require(frame.containsProjection({1000,2002.5,3002.5}) && !frame.containsProjection({1000,2020,3020}),"domain queries preserve translated and rotated plane frames");
    for(const auto& t:mesh.triangles) {
        const auto a=mesh.positions[t[0]],b=mesh.positions[t[1]],c=mesh.positions[t[2]];
        const auto cross=[](const auto& p,const auto& q){return (q[0]-p[0])*(20-p[1])-(q[1]-p[1])*(20-p[0]);};
        const double x=cross(a,b),y=cross(b,c),z=cross(c,a);
        require(!((x>=0&&y>=0&&z>=0)||(x<=0&&y<=0&&z<=0)),"finite-domain triangulation does not fill the observed hole");
    }
    settings.meshPointBudget=256;const auto thinned=buildPlanarSurfaceFootprint(samples,settings);
    require(std::abs(area(mesh)-area(thinned))<1e-8,"display thinning retains the supported exterior and holes");
    samples.clear();for(int x=0;x<30;++x)for(int y=0;y<2;++y)samples.push_back({{double(x),double(y)},PointId(samples.size()+1)});
    require(!buildPlanarSurfaceFootprint(samples,settings).triangles.empty(),"real narrow strip is not rejected for lacking a deep interior");
}
void disconnectedDomainMembership() {
    Fixture f;PlaneEvidence e;e.plane={{0,0,1},0};e.confidence=.99;
    for(int block=0;block<2;++block)for(int x=0;x<10;++x)for(int y=0;y<10;++y) {
        PointSample p;p.position={block*100.+x,double(y),0};f.add(p,e.supportPointIds);
    }
    PointSample extra;extra.position={4.000001,4.000001,0};f.add(extra,e.supportPointIds);f.evidence.push_back(e);
    SurfaceAnalysisOptions options;options.connectivityDistance=1000;
    std::vector<PrimitiveEvidence> accepted;const auto report=run(f,accepted,options);
    require(accepted.size()==1 && std::get<PlaneEvidence>(accepted.front()).supportPointIds.size()==f.points.size(),"geometrically covered observations retain the correct disconnected domain");
    require(std::count_if(report.decisions.begin(),report.decisions.end(),[](const auto& d){return d.verdict==SupportVerdict::Accepted;})==2,"independent finite domains are checked independently");
}
}
int main(){try{densityAndModes();cylinder(false);cylinder(true);spheres();cones();transactionAndSettings();heterogeneousReference();invalidRing();footprintsKeepHolesAndStrips();disconnectedDomainMembership();std::cout<<"surface_support_validation_test passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
