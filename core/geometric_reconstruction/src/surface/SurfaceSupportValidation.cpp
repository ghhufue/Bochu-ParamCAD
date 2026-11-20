#include "reconstruction/surface/SurfaceSupportValidation.h"
#include "detail/AnalysisInternal.h"
#include "reconstruction/surface/PlaneSupportRecovery.h"
#include "reconstruction/data/StageProfile.h"
#include "common/Parallel.h"
#include "common/spatial/KdTree.h"
#include <Eigen/Dense>
#include <iomanip>
#include <locale>
#include <mutex>
#include <numeric>
#include <sstream>
#include <set>

namespace reconstruction {
namespace {
using namespace math3;
constexpr double pi=3.14159265358979323846;
using XY=std::array<double,2>;
using Cell=std::pair<long long,long long>;
double quantile(std::vector<double> x,double q) {
    if(x.empty())return 0;
    std::sort(x.begin(),x.end());const double t=q*(x.size()-1);
    const auto a=std::size_t(t);return x[a]+(x[std::min(a+1,x.size()-1)]-x[a])*(t-a);
}
double median(const std::vector<double>& x){return quantile(x,.5);}
double noise(const std::vector<double>& x) {
    const double m=median(x);std::vector<double> delta;for(double v:x)delta.push_back(std::abs(v-m));
    return 1.4826*median(delta);
}
Vec3 v3(const std::array<double,3>& p){return {p[0],p[1],p[2]};}
Vec3 tangent(Vec3 n) {const Vec3 a=std::abs(n.x)<.8?Vec3{1,0,0}:Vec3{0,1,0};return normalized(subtract(a,scale(n,dot(a,n))));}
double area2(XY a,XY b,XY c){return std::abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))*.5;}
std::string parametersJson(const PrimitiveEvidence& evidence) {
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
    const auto vector=[&](Vec3 p){out<<'['<<p.x<<','<<p.y<<','<<p.z<<']';};
    std::visit([&](const auto& e){using T=std::decay_t<decltype(e)>;
        if constexpr(std::is_same_v<T,PlaneEvidence>){out<<"{\"normal\":";vector(e.plane.normal);out<<",\"offset\":"<<e.plane.offset;}
        else if constexpr(std::is_same_v<T,SphereEvidence>){out<<"{\"center\":";vector(e.sphere.center);out<<",\"radius\":"<<e.sphere.radius;}
        else if constexpr(std::is_same_v<T,CylinderEvidence>){out<<"{\"center\":";vector(e.cylinder.axisPoint);out<<",\"axis\":";vector(e.cylinder.axisDirection);out<<",\"radius\":"<<e.cylinder.radius;}
        else if constexpr(std::is_same_v<T,ConeEvidence>){out<<"{\"center\":";vector(e.cone.apex);out<<",\"axis\":";vector(e.cone.axisDirection);out<<",\"angle\":"<<e.cone.openingAngleRadians;}
        else {out<<"{\"center\":";vector(e.torus.center);out<<",\"axis\":";vector(e.torus.axisDirection);out<<",\"major_radius\":"<<e.torus.majorRadius<<",\"minor_radius\":"<<e.torus.minorRadius;}
        out<<'}';},evidence);
    return out.str();
}
struct Metrics {
    PrimitiveEvidence evidence;
    SurfaceSupportDecision decision;
    SurfaceFootprint domain;
    std::vector<PlanarSurfaceSample> samples;
    std::unordered_map<PointId,Vec3> positions;
    double planeNoise=0, confidence=0;
    double coneRadius=0;
    PlaneEquation localPlane;
    bool geometryValid=true;
};
bool validGeometry(const PrimitiveEvidence& evidence) {
    const auto finite=[](Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
    const auto direction=[&](Vec3 p){return finite(p)&&length(p)>1e-12;};
    return std::visit([&](const auto& e) {
        using T=std::decay_t<decltype(e)>;
        if constexpr(std::is_same_v<T,PlaneEvidence>)return direction(e.plane.normal)&&std::isfinite(e.plane.offset);
        else if constexpr(std::is_same_v<T,SphereEvidence>)return finite(e.sphere.center)&&std::isfinite(e.sphere.radius)&&e.sphere.radius>0;
        else if constexpr(std::is_same_v<T,CylinderEvidence>)return finite(e.cylinder.axisPoint)&&direction(e.cylinder.axisDirection)&&std::isfinite(e.cylinder.radius)&&e.cylinder.radius>0;
        else if constexpr(std::is_same_v<T,ConeEvidence>)return finite(e.cone.apex)&&direction(e.cone.axisDirection)&&std::isfinite(e.cone.openingAngleRadians)&&e.cone.openingAngleRadians>0&&e.cone.openingAngleRadians<pi*.5;
        else return finite(e.torus.center)&&direction(e.torus.axisDirection)&&std::isfinite(e.torus.majorRadius)&&std::isfinite(e.torus.minorRadius)&&e.torus.minorRadius>0&&e.torus.majorRadius>e.torus.minorRadius;
    },evidence);
}
// Repeated dense samples along one chart axis must not erase a legitimately
// coarser transverse scan step. Only bounded, independently observed steps vote.
double axialChartStretch(const std::vector<PlanarSurfaceSample>& samples) {
    std::vector<PointSample> points;points.reserve(samples.size());
    for(const auto& s:samples){PointSample p;p.id=points.size()+1;p.position={s.position[0],s.position[1],0};points.push_back(p);}
    detail::spatial::KdTree tree(points);std::vector<double> ratios;
    const auto stride=std::max<std::size_t>(1,points.size()/512);
    for(std::size_t i=0;i<points.size();i+=stride) {
        const auto neighbors=tree.nearest(i,32);double horizontal=INFINITY,vertical=INFINITY;
        for(const auto& item:neighbors) {
            const auto q=subtract(points[item.second].position,points[i].position);
            if(std::abs(q.x)>4*std::abs(q.y) && std::abs(q.x)>1e-12)horizontal=std::min(horizontal,std::abs(q.x));
            if(std::abs(q.y)>4*std::abs(q.x) && std::abs(q.y)>1e-12)vertical=std::min(vertical,std::abs(q.y));
        }
        if(std::isfinite(horizontal) && std::isfinite(vertical) && vertical/horizontal>=1./16 && vertical/horizontal<=16)ratios.push_back(vertical/horizontal);
    }
    return ratios.empty()?1:median(ratios);
}
void fail(SurfaceSupportDecision& d,const char* reason) {
    if(std::find(d.reasons.begin(),d.reasons.end(),reason)==d.reasons.end())d.reasons.emplace_back(reason);
    d.verdict=SupportVerdict::Rejected;
}
void planeFit(Metrics& m) {
    Vec3 center{};for(const auto& p:m.positions)center=add(center,p.second);
    if(m.positions.size()<3)return;
    center=scale(center,1./m.positions.size());Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
    for(const auto& p:m.positions) {const auto q=subtract(p.second,center);Eigen::Vector3d v(q.x,q.y,q.z);covariance+=v*v.transpose();}
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    const auto n=eigen.eigenvectors().col(0);m.localPlane={{n.x(),n.y(),n.z()},-dot({n.x(),n.y(),n.z()},center)};
    std::vector<double> residuals;for(const auto& p:m.positions)residuals.push_back(dot(m.localPlane.normal,p.second)+m.localPlane.offset);
    m.planeNoise=noise(residuals);
}
void appendDomain(SurfaceFootprint& to,const SurfaceFootprint& from) {
    const auto offset=to.positions.size();to.positions.insert(to.positions.end(),from.positions.begin(),from.positions.end());
    to.sourcePointIds.insert(to.sourcePointIds.end(),from.sourcePointIds.begin(),from.sourcePointIds.end());
    to.coveredPointIds.insert(to.coveredPointIds.end(),from.coveredPointIds.begin(),from.coveredPointIds.end());
    for(auto t:from.triangles){for(auto& i:t)i+=offset;to.triangles.push_back(t);}
}
double angularSpan(std::vector<double> angles) {
    if(angles.size()<2)return 0;std::sort(angles.begin(),angles.end());
    double gap=angles.front()+2*pi-angles.back();std::vector<double> steps;
    if(gap>1e-8)steps.push_back(gap);
    for(std::size_t i=1;i<angles.size();++i){const double step=angles[i]-angles[i-1];gap=std::max(gap,step);if(step>1e-8)steps.push_back(step);}
    if(steps.size()>=12 && 2*pi-gap>pi && gap<=3*median(steps))return 2*pi;
    return 2*pi-gap;
}
void buildDomain(Metrics& m,const SurfaceMeshingSettings& settings,double maxSpacing) {
    const auto kind=primitiveKind(m.evidence);auto& d=m.decision;
    if(kind==PrimitiveKind::Plane) {
        const auto& p=std::get<PlaneEvidence>(m.evidence).plane;const auto x=tangent(p.normal),y=cross(p.normal,x);
        const auto origin=m.positions.begin()->second;
        for(auto id:d.supportPointIds){const auto q=subtract(m.positions.at(id),origin);m.samples.push_back({{dot(q,x),dot(q,y)},id});}
        m.domain=buildPlanarSurfaceFootprint(m.samples,settings,maxSpacing);
        m.domain.origin=origin;m.domain.axisU=x;m.domain.axisV=y;
        for(const auto& t:m.domain.triangles){const auto a=m.domain.positions[t[0]],b=m.domain.positions[t[1]],c=m.domain.positions[t[2]];d.area+=area2({a[0],a[1]},{b[0],b[1]},{c[0],c[1]});}
    } else if(kind==PrimitiveKind::Cylinder || kind==PrimitiveKind::Cone) {
        const auto* cone=std::get_if<ConeEvidence>(&m.evidence);
        const auto* cylinder=std::get_if<CylinderEvidence>(&m.evidence);
        const auto origin=cone?cone->cone.apex:cylinder->cylinder.axisPoint;
        const auto z=normalized(cone?cone->cone.axisDirection:cylinder->cylinder.axisDirection);
        const auto x=tangent(z),y=cross(z,x);
        const double sine=cone?std::sin(cone->cone.openingAngleRadians):0;
        const double cosine=cone?std::cos(cone->cone.openingAngleRadians):1;
        std::vector<double> angles,radii;
        std::unordered_map<PointId,XY> uv;
        for(auto id:d.supportPointIds) {
            const auto q=subtract(m.positions.at(id),origin);
            const double u=dot(q,x),v=dot(q,y),axial=dot(q,z);
            const double slant=axial*cosine+std::hypot(u,v)*sine;
            double a=std::atan2(v,u);if(a<0)a+=2*pi;
            angles.push_back(a);uv[id]={a,slant};
            if(cone)radii.push_back(slant*sine);
        }
        const double radius=cone?median(radii):cylinder->cylinder.radius;
        if(radius<=1e-12)return;
        std::sort(angles.begin(),angles.end());double gap=angles.front()+2*pi-angles.back(),seam=angles.front();
        for(std::size_t i=1;i<angles.size();++i)if(angles[i]-angles[i-1]>gap){gap=angles[i]-angles[i-1];seam=angles[i];}
        for(auto id:d.supportPointIds){auto q=uv.at(id);double a=q[0]-seam;if(a<0)a+=2*pi;m.samples.push_back({{a*radius,q[1]},id});}
        const double stretch=axialChartStretch(m.samples);
        for(auto& sample:m.samples)sample.position[0]*=stretch;
        m.domain=buildPlanarSurfaceFootprint(m.samples,settings,maxSpacing,SurfaceSampleSpace::UnwrappedSurface);
        for(const auto& t:m.domain.triangles){
            const auto a=m.domain.positions[t[0]],b=m.domain.positions[t[1]],c=m.domain.positions[t[2]];
            // The cone's circumferential metric grows linearly with slant.
            const double areaScale=cone?(a[1]+b[1]+c[1])*sine/(3*radius):1;
            d.area+=area2({a[0],a[1]},{b[0],b[1]},{c[0],c[1]})*areaScale/stretch;
        }
        std::vector<double> supportedAngles,supportedRadii;
        for(auto id:m.domain.coveredPointIds){
            supportedAngles.push_back(uv.at(id)[0]);
            if(cone)supportedRadii.push_back(uv.at(id)[1]*sine);
        }
        d.angularSpanRadians=angularSpan(supportedAngles);
        if(cone)m.coneRadius=median(supportedRadii);
    } else if(kind==PrimitiveKind::Sphere) {
        const auto& sphere=std::get<SphereEvidence>(m.evidence).sphere;
        std::unordered_map<PointId,Vec3> rays;Vec3 mean{};
        for(auto id:d.supportPointIds){const auto q=normalized(subtract(m.positions.at(id),sphere.center));rays[id]=q;mean=add(mean,q);}
        mean=scale(mean,1./rays.size());
        std::vector<Vec3> charts;
        if(length(mean)>.8)charts.push_back(normalized(mean));
        else charts={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
        for(std::size_t chart=0;chart<charts.size();++chart) {
            const auto z=charts[chart],x=tangent(z),y=cross(z,x);
            std::vector<PlanarSurfaceSample> samples;
            for(auto id:d.supportPointIds){const auto q=rays.at(id);const double dz=dot(q,z);if(dz>.3)samples.push_back({{sphere.radius*dot(q,x)/dz,sphere.radius*dot(q,y)/dz},id});}
            auto mesh=buildPlanarSurfaceFootprint(samples,settings,maxSpacing,SurfaceSampleSpace::UnwrappedSurface);
            SurfaceFootprint retained=mesh;retained.triangles.clear();retained.coveredPointIds.clear();
            for(auto t:mesh.triangles) {
                const auto a=rays.at(mesh.sourcePointIds[t[0]]),b=rays.at(mesh.sourcePointIds[t[1]]),c=rays.at(mesh.sourcePointIds[t[2]]);
                const auto mid=add(add(a,b),c);std::size_t owner=0;
                for(std::size_t k=1;k<charts.size();++k)if(dot(mid,charts[k])>dot(mid,charts[owner]))owner=k;
                if(owner!=chart)continue;
                const double omega=2*std::atan2(std::abs(dot(a,cross(b,c))),1+dot(a,b)+dot(b,c)+dot(c,a));
                if(!(omega>0) || omega>pi)continue;
                d.solidAngle+=omega;retained.triangles.push_back(t);
                for(auto i:t)retained.coveredPointIds.push_back(mesh.sourcePointIds[i]);
            }
            appendDomain(m.domain,retained);
        }
        d.area=d.solidAngle*sphere.radius*sphere.radius;
        if(length(mean)>.8) {
            const auto z=normalized(mean),x=tangent(z),y=cross(z,x);Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
            std::vector<Eigen::Vector2d> uv;for(auto id:d.supportPointIds){const auto q=rays.at(id);Eigen::Vector2d v(std::atan2(dot(q,x),dot(q,z)),std::atan2(dot(q,y),dot(q,z)));uv.push_back(v);covariance+=v*v.transpose();}
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> e(covariance);std::vector<double> minor;
            for(const auto& v:uv)minor.push_back(v.dot(e.eigenvectors().col(0)));
            d.minorSpanRadians=quantile(minor,.95)-quantile(minor,.05);
        } else {
            Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
            for(auto id:d.supportPointIds){const auto q=subtract(rays.at(id),mean);Eigen::Vector3d v(q.x,q.y,q.z);covariance+=v*v.transpose()/double(rays.size());}
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(covariance);
            d.minorSpanRadians=2*std::asin(std::min(1.,std::sqrt(3*std::max(0.,e.eigenvalues()[0]))));
        }
    }
    std::sort(m.domain.coveredPointIds.begin(),m.domain.coveredPointIds.end());
    m.domain.coveredPointIds.erase(std::unique(m.domain.coveredPointIds.begin(),m.domain.coveredPointIds.end()),m.domain.coveredPointIds.end());
    d.coveredPoints=m.domain.coveredPointIds.size();
    std::set<std::array<double,3>> uniquePositions;
    for(auto id:m.domain.coveredPointIds){const auto p=m.positions.at(id);uniquePositions.insert({p.x,p.y,p.z});}
    d.effectivePoints=kind==PrimitiveKind::Sphere?uniquePositions.size():m.domain.effectiveCoveredPoints;
    d.density=d.area>0?double(d.effectivePoints)/d.area:0;
    d.finalArea=d.area;d.finalCoveredPoints=d.coveredPoints;
}
std::vector<XY> clip(const std::vector<XY>& input,int axis,double bound,bool lower) {
    std::vector<XY> output;if(input.empty())return output;
    auto inside=[&](XY p){return lower?p[axis]>=bound:p[axis]<=bound;};
    XY a=input.back();bool ia=inside(a);
    for(auto b:input){const bool ib=inside(b);if(ia!=ib){const double f=(bound-a[axis])/(b[axis]-a[axis]);output.push_back({a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])});}if(ib)output.push_back(b);a=b;ia=ib;}
    return output;
}
double occupancy(const Metrics& m,double spacing,double density,const std::function<bool()>& cancelled) {
    std::vector<double> ratios;
    for(const auto offset: {XY{0,0},XY{.5,0},XY{0,.5},XY{.5,.5}}) {
        const double h=spacing;const XY origin{offset[0]*h,offset[1]*h};std::map<Cell,double> cells;
        std::size_t work=0;
        for(auto t:m.domain.triangles) {
            const auto a=m.domain.positions[t[0]],b=m.domain.positions[t[1]],c=m.domain.positions[t[2]];
            const long long loX=static_cast<long long>(std::floor((std::min({a[0],b[0],c[0]})-origin[0])/h));
            const long long hiX=static_cast<long long>(std::floor((std::max({a[0],b[0],c[0]})-origin[0])/h));
            const long long loY=static_cast<long long>(std::floor((std::min({a[1],b[1],c[1]})-origin[1])/h));
            const long long hiY=static_cast<long long>(std::floor((std::max({a[1],b[1],c[1]})-origin[1])/h));
            if(double(hiX-loX+1)*double(hiY-loY+1)>2000000)return 0;
            for(auto x=loX;x<=hiX;++x)for(auto y=loY;y<=hiY;++y) {
                if((++work&4095)==0 && cancelled && cancelled())return -1;
                std::vector<XY> p{{a[0],a[1]},{b[0],b[1]},{c[0],c[1]}};
                p=clip(p,0,origin[0]+x*h,true);p=clip(p,0,origin[0]+(x+1)*h,false);
                p=clip(p,1,origin[1]+y*h,true);p=clip(p,1,origin[1]+(y+1)*h,false);
                double area=0;for(std::size_t i=1;i+1<p.size();++i)area+=area2(p[0],p[i],p[i+1]);
                if(area>h*h*1e-12)cells[{x,y}]+=area;
            }
        }
        std::set<Cell> occupied;
        const std::unordered_set<PointId> covered(m.domain.coveredPointIds.begin(),m.domain.coveredPointIds.end());
        for(const auto& p:m.samples)if(covered.count(p.sourcePointId)) {
            const Cell cell{static_cast<long long>(std::floor((p.position[0]-origin[0])/h)),static_cast<long long>(std::floor((p.position[1]-origin[1])/h))};
            if(cells.count(cell))occupied.insert(cell);
            else { // A sample exactly on the exterior/grid boundary belongs to an adjacent clipped cell.
                for(int dx=-1;dx<=0;++dx)for(int dy=-1;dy<=0;++dy)if(cells.count({cell.first+dx,cell.second+dy})){occupied.insert({cell.first+dx,cell.second+dy});dx=1;break;}
            }
        }
        double expected=0;for(const auto& cell:cells)expected+=-std::expm1(-cell.second*density);
        ratios.push_back(expected>0?occupied.size()/expected:0);
    }
    return median(ratios);
}
double huber(double r){r=std::abs(r);return r<=1.345?.5*r*r:1.345*(r-.5*1.345);}
void splitDomains(Metrics m, const SurfaceMeshingSettings& settings, std::vector<Metrics>& output) {
        std::unordered_map<PointId,PointId> parent;
        for(auto id:m.decision.supportPointIds)parent[id]=id;
        auto root=[&](PointId id){while(parent.at(id)!=id){parent[id]=parent.at(parent.at(id));id=parent.at(id);}return id;};
        for (const auto& [member, anchor] : m.domain.sampleAnchors) parent[root(member)] = root(anchor);
        for(const auto& t:m.domain.triangles) {
            const auto a=m.domain.sourcePointIds[t[0]];
            for(int k=1;k<3;++k)parent[root(m.domain.sourcePointIds[t[k]])]=root(a);
        }
        std::map<std::array<double,3>,PointId> locations;
        for(auto id:m.decision.supportPointIds){const auto p=m.positions.at(id);auto found=locations.emplace(std::array<double,3>{p.x,p.y,p.z},id);if(!found.second)parent[root(id)]=root(found.first->second);}
        std::unordered_set<PointId> domainRoots;
        for(const auto& t:m.domain.triangles)domainRoots.insert(root(m.domain.sourcePointIds[t[0]]));
        if(domainRoots.size()==1) {
            const auto anchor=*domainRoots.begin();for(auto id:m.domain.coveredPointIds)parent[root(id)]=root(anchor);
        } else if(domainRoots.size()>1) {
            const SurfaceFootprintQuery query(m.domain);
            std::unordered_map<PointId,XY> chart;
            if(m.decision.kind==PrimitiveKind::Cylinder || m.decision.kind==PrimitiveKind::Cone)
                for(const auto& p:m.samples)chart.emplace(p.sourcePointId,p.position);
            for(auto id:m.domain.coveredPointIds)if(!domainRoots.count(root(id))) {
                auto point=m.positions.at(m.decision.kind == PrimitiveKind::Plane ? root(id) : id);
                if(!chart.empty()){const auto p=chart.at(id);point={p[0],p[1],0};}
                const auto anchor=query.anchorAtProjection(point);if(anchor)parent[root(id)]=root(*anchor);
            }
        }
        std::map<PointId,std::vector<PointId>> groups;
        for(auto id:m.domain.coveredPointIds)groups[root(id)].push_back(id);
        if(groups.size()<=1){output.push_back(std::move(m));return;}
        std::unordered_set<PointId> assigned;
        for(const auto& group:groups) {
            Metrics child;child.evidence=m.evidence;child.confidence=m.confidence;
            child.decision=m.decision;child.decision.supportPointIds=group.second;
            child.decision.pointCount=group.second.size();child.decision.area=child.decision.solidAngle=0;
            for(auto id:group.second){child.positions.emplace(id,m.positions.at(id));assigned.insert(id);}
            planeFit(child);buildDomain(child,settings,0);
            if(child.decision.area<=0 || child.decision.effectivePoints<settings.minimumSurfacePoints){child.decision.verdict=SupportVerdict::NeedsMoreSupport;child.decision.reasons={"InsufficientTwoDimensionalSupport"};}
            output.push_back(std::move(child));
        }
        std::vector<PointId> tails;for(auto id:m.decision.supportPointIds)if(!assigned.count(id))tails.push_back(id);
        if(!tails.empty()){
            Metrics tail;tail.evidence=m.evidence;tail.confidence=m.confidence;tail.decision.proposalId=m.decision.proposalId;
            tail.decision.kind=m.decision.kind;tail.decision.supportPointIds=std::move(tails);tail.decision.pointCount=tail.decision.supportPointIds.size();
            tail.decision.verdict=SupportVerdict::NeedsMoreSupport;tail.decision.reasons={"InsufficientTwoDimensionalSupport"};
            for(auto id:tail.decision.supportPointIds)tail.positions.emplace(id,m.positions.at(id));if(tail.positions.size()>=3)planeFit(tail);
            output.push_back(std::move(tail));
        }
}
void curvedChecks(Metrics& m,const SurfaceSupportValidationOptions& o,double tolerance,const SurfaceSamplingReference& reference) {
    auto& d=m.decision;const bool sphere=d.kind==PrimitiveKind::Sphere;
    const auto* cone=std::get_if<ConeEvidence>(&m.evidence);
    if(sphere){if(d.solidAngle<o.sphereMinSolidAngle)fail(d,"SphereSolidAngleTooSmall");if(d.minorSpanRadians<o.sphereMinMinorSpanRadians)fail(d,"SphereMinorSpanTooSmall");}
    else if(!cone && d.angularSpanRadians<o.cylinderMinAngularSpanRadians)fail(d,"CylinderAngularSpanTooSmall");
    const double radius=sphere?std::get<SphereEvidence>(m.evidence).sphere.radius:
        cone?m.coneRadius*std::cos(cone->cone.openingAngleRadians):std::get<CylinderEvidence>(m.evidence).cylinder.radius;
    const double theta=std::min(pi,sphere?d.minorSpanRadians:d.angularSpanRadians);
    d.sagitta=2*radius*std::pow(std::sin(theta*.25),2);
    d.noise=std::max(reference.available?reference.noise:tolerance/3,o.noiseFloorToleranceFraction*tolerance);
    d.noise=std::max(d.noise,std::numeric_limits<double>::min());
    d.noiseSource=reference.available?"reference_plane_mad_with_tolerance_floor":"fit_tolerance_divided_by_three";
    if(d.sagitta/d.noise<o.minimumSagittaNoiseRatio)fail(d,"CurvatureBelowNoise");
    // Iteratively reweighted plane on exactly the same observations as the curved score.
    const auto& observations=cone?m.domain.coveredPointIds:d.supportPointIds;
    auto plane=m.localPlane;
    for(int iteration=0;iteration<5;++iteration) {
        Vec3 center{};double total=0;
        for(auto id:observations){const auto p=m.positions.at(id);const double r=std::abs(dot(plane.normal,p)+plane.offset)/d.noise;const double w=r>1.345?1.345/r:1;center=add(center,scale(p,w));total+=w;}
        center=scale(center,1/total);Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
        for(auto id:observations){const auto p=m.positions.at(id);const double r=std::abs(dot(plane.normal,p)+plane.offset)/d.noise,w=r>1.345?1.345/r:1;const auto q=subtract(p,center);Eigen::Vector3d v(q.x,q.y,q.z);covariance+=w*v*v.transpose();}
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(covariance);const auto n=e.eigenvectors().col(0);plane={{n.x(),n.y(),n.z()},-dot({n.x(),n.y(),n.z()},center)};
    }
    for(auto id:observations){const auto p=m.positions.at(id);d.planeCost+=huber((dot(plane.normal,p)+plane.offset)/d.noise);d.curvedCost+=huber(surfaceResidual(m.evidence,p)/d.noise);}
    const double count=double(observations.size()),penalty=o.modelComplexityWeight*std::log(count)/count;
    d.planeCost=d.planeCost/count+3*penalty;d.curvedCost=d.curvedCost/count+(sphere?4:cone?6:5)*penalty;
    if(d.planeCost-d.curvedCost<o.minimumModelGain)fail(d,"PlanarAlternativePreferred");
}
} // namespace

const char* supportVerdictName(SupportVerdict v) noexcept {
    switch(v){case SupportVerdict::Accepted:return "accepted";case SupportVerdict::Rejected:return "rejected";default:return "needs_more_support";}
}
bool validSurfaceSupportOptions(const SurfaceSupportValidationOptions& o) noexcept {
    const double values[]{o.planeDensityRatioMax,o.sphereMinSolidAngle,o.sphereMinMinorSpanRadians,o.cylinderMinAngularSpanRadians,o.minimumSagittaNoiseRatio,o.referenceMinConfidence,o.referenceAreaQuantile,o.referenceMaxLogSpread,o.minimumExpectedPoints,o.gridSpacingMultiplier,o.minimumOccupiedCellRatio,o.noiseFloorToleranceFraction,o.modelComplexityWeight,o.minimumModelGain,o.planePreferenceMinReferenceAreaFraction};
    for(auto x:values)if(!std::isfinite(x) || x<0)return false;
    if (o.workerCount > 32) return false;
    return int(o.mode)>=0 && int(o.mode)<=2 && o.planeDensityRatioMax>=1 && o.sphereMinSolidAngle>0 && o.sphereMinSolidAngle<=4*pi && o.sphereMinMinorSpanRadians>0 && o.sphereMinMinorSpanRadians<=pi && o.cylinderMinAngularSpanRadians>0 && o.cylinderMinAngularSpanRadians<=2*pi && o.referenceMinConfidence<=1 && o.referenceAreaQuantile<=1 && o.referenceMinPoints>=3 && o.referenceMinPlanes>0 && o.referenceMaxLogSpread>0 && o.minimumExpectedPoints>=1 && o.gridSpacingMultiplier>0 && o.minimumOccupiedCellRatio>0 && o.minimumOccupiedCellRatio<=1 && o.noiseFloorToleranceFraction>0 && o.planePreferenceMinReferenceAreaFraction>0;
}
std::vector<double> surfaceSupportSettingsSignature(const SurfaceAnalysisOptions& options) {
    const auto& o=options.supportValidation;const auto& m=options.supportMeshing;
    return {double(SurfaceSupportValidationReport::version),double(o.mode),o.planeDensityRatioMax,o.sphereMinSolidAngle,o.sphereMinMinorSpanRadians,o.cylinderMinAngularSpanRadians,o.minimumSagittaNoiseRatio,o.referenceMinConfidence,double(o.referenceMinPoints),double(o.referenceMinPlanes),o.referenceAreaQuantile,o.referenceMaxLogSpread,o.minimumExpectedPoints,o.gridSpacingMultiplier,o.minimumOccupiedCellRatio,o.noiseFloorToleranceFraction,o.modelComplexityWeight,o.minimumModelGain,o.planePreferenceMinReferenceAreaFraction,
        double(m.minimumSurfacePoints),double(m.densityNeighborCount),m.localSupportRadiusMultiplier,m.connectivityRadiusMultiplier,m.minimumTransverseVarianceRatio,m.regularSpacingTolerance,m.regularSpacingLimitMultiplier,m.densityGapMultiplier,m.coverageRadiusMultiplier,m.planarAlphaRadiusMultiplier,m.expectedHolePoints,m.smallDensityScale,m.holeDensityRatio,m.boundaryBandSpacingMultiplier,m.boundaryDominanceRatio,m.deepInteriorDistanceMultiplier,m.minimumDeepInteriorAreaSpacings,m.minimumExpectedInteriorPoints,m.minimumInteriorSupportRatio,m.slotMinimumLengthSpacings,m.slotMaximumWidthSpacings,m.slotMinimumSideSpacings,m.slotEdgeAlignment,m.slotPerimeterSupportRatio};
}

bool validateSurfaceSupport(const PointStore& points,const std::vector<PrimitiveEvidence>& proposals,
    const SurfaceAnalysisOptions& options,std::vector<PrimitiveEvidence>& accepted,
    SurfaceSupportValidationReport& output,std::string& error,const SurfaceSamplingReference* frozen,
    const std::function<bool()>& cancelled,const std::vector<PointId>& protectedIds) {
    try {
    error.clear();if(!validSurfaceSupportOptions(options.supportValidation)){error="Invalid surface support validation options";return false;}
    if(cancelled && cancelled()){error="Cancelled";return false;}
    SurfaceSupportValidationReport report;report.pointFingerprint=pointStoreFingerprint(points);report.unit=points.unit();report.mode=options.supportValidation.mode;report.options=options.supportValidation;report.settingsSignature=surfaceSupportSettingsSignature(options);
    if(report.mode==SupportValidationMode::Off){accepted=proposals;output=std::move(report);return true;}
    using diagnostics::profileStage;
    const diagnostics::StageProfile profile("support.validate", points.size());
    SurfaceAnalysisResult indexed;
    if (!profileStage("support.index_proposals", [&] {
        return indexSurfacePatches(points, proposals, options, indexed, error);
    })) return false;
    const auto map=detail::analysis::pointMap(points);const auto& o=options.supportValidation;
    const std::unordered_set<PointId> protectedSet(protectedIds.begin(),protectedIds.end());
    const auto workers = points.size() < 4096 && o.workerCount == 0 ? 1 : o.workerCount;
    std::mutex cancellationMutex;
    bool stopped = false;
    const std::function<bool()> checkCancelled = cancelled ? std::function<bool()>([&] {
        const std::lock_guard<std::mutex> lock(cancellationMutex);
        if (!stopped) stopped = cancelled();
        return stopped;
    }) : std::function<bool()>{};
    const auto evaluatePatches = [&](const char* stage, const auto& patches, auto pointCount, auto evaluate) {
        const diagnostics::StageProfile phase(stage);
        std::vector<std::size_t> order(patches.size());
        std::iota(order.begin(), order.end(), 0);
        if (workers != 1) {
            std::stable_sort(order.begin(), order.end(), [&](auto first, auto second) {
                return pointCount(patches[first]) > pointCount(patches[second]);
            });
        }
        detail::parallelTasks(order.size(), workers, [&](std::size_t task) {
            if (!checkCancelled || !checkCancelled()) evaluate(order[task]);
        });
        return !checkCancelled || !checkCancelled();
    };
    std::vector<std::vector<Metrics>> domains(indexed.patches.size());
    auto settings=options.supportMeshing;settings.meshPointBudget=std::numeric_limits<std::size_t>::max()/2;
    if (!evaluatePatches("support.build_domains", indexed.patches,
        [](const auto& patch) { return patch.supportPointIds.size(); }, [&](std::size_t index) {
        const auto& patch = indexed.patches[index];
        Metrics m;m.evidence=indexed.surfaces.at(std::size_t(patch.surfaceId-1));auto& d=m.decision;
        d.proposalId=patch.surfaceId;d.patchId=patch.id;d.kind=primitiveKind(m.evidence);d.supportPointIds=patch.supportPointIds;
        if(validGeometry(m.evidence))d.parametersJson=parametersJson(m.evidence);
        std::sort(d.supportPointIds.begin(),d.supportPointIds.end());d.supportPointIds.erase(std::unique(d.supportPointIds.begin(),d.supportPointIds.end()),d.supportPointIds.end());
        d.pointCount=d.supportPointIds.size();d.protectedManual=!protectedSet.empty() && std::all_of(d.supportPointIds.begin(),d.supportPointIds.end(),[&](auto id){return protectedSet.count(id)!=0;});
        m.confidence=detail::analysis::confidence(m.evidence);
        for(auto id:d.supportPointIds)m.positions.emplace(id,map.at(id)->position);
        if(!validGeometry(m.evidence)){m.geometryValid=false;fail(d,"InvalidPrimitiveGeometry");}
        else {
        if(d.kind==PrimitiveKind::Plane) {
            auto& ids=d.supportPointIds;
            ids.erase(std::remove_if(ids.begin(),ids.end(),[&](auto id){return surfaceResidual(m.evidence,m.positions.at(id))>options.detection.plane.distanceThreshold;}),ids.end());
            m.positions.clear();for(auto id:ids)m.positions.emplace(id,map.at(id)->position);
        }
        if(d.kind==PrimitiveKind::Sphere) {
            const auto center=std::get<SphereEvidence>(m.evidence).sphere.center;auto& ids=d.supportPointIds;
            ids.erase(std::remove_if(ids.begin(),ids.end(),[&](auto id){return length(subtract(m.positions.at(id),center))<=1e-12;}),ids.end());
            m.positions.clear();for(auto id:ids)m.positions.emplace(id,map.at(id)->position);
        }
        if(const auto* cone=std::get_if<ConeEvidence>(&m.evidence)) {
            const auto axis=normalized(cone->cone.axisDirection);
            auto& ids=d.supportPointIds;
            ids.erase(std::remove_if(ids.begin(),ids.end(),[&](auto id){
                const auto point=m.positions.at(id);
                const auto q=subtract(point,cone->cone.apex);
                return dot(q,axis)<=0 || length(cross(q,axis))<=1e-12 ||
                    surfaceResidual(m.evidence,point)>options.detection.cone.distanceThreshold;
            }),ids.end());
            m.positions.clear();for(auto id:ids)m.positions.emplace(id,map.at(id)->position);
        }
        if(d.kind!=PrimitiveKind::Torus) {
            if(d.supportPointIds.size()>=3){planeFit(m);buildDomain(m,settings,0);}
            const auto support=d.kind==PrimitiveKind::Cone?d.effectivePoints:d.coveredPoints;
            if(d.area<=0 || support<settings.minimumSurfacePoints){d.verdict=SupportVerdict::NeedsMoreSupport;d.reasons.emplace_back("InsufficientTwoDimensionalSupport");}
        }
        }
        splitDomains(std::move(m), settings, domains[index]);
    })) {
        error = "Cancelled";
        return false;
    }
    std::vector<Metrics> metrics;
    for (auto& group : domains) for (auto& entry : group) {
        entry.decision.patchId = metrics.size() + 1;
        metrics.push_back(std::move(entry));
    }
    profileStage("support.density_reference", [&] {
    if(frozen)report.reference=*frozen;
    else {
        std::vector<std::size_t> refs;
        for(std::size_t i=0;i<metrics.size();++i){const auto& m=metrics[i];if(m.decision.kind==PrimitiveKind::Plane && m.decision.verdict==SupportVerdict::Accepted && m.confidence>=o.referenceMinConfidence && m.decision.effectivePoints>=o.referenceMinPoints)refs.push_back(i);}
        std::stable_sort(refs.begin(),refs.end(),[&](auto a,auto b){return metrics[a].decision.area>metrics[b].decision.area;});
        std::vector<std::size_t> unique;
        for(auto i:refs){bool duplicate=false;const auto p=std::get<PlaneEvidence>(metrics[i].evidence).plane;
            for(auto j:unique){const auto q=std::get<PlaneEvidence>(metrics[j].evidence).plane;const double sign=dot(p.normal,q.normal)<0?-1.:1.;if(std::abs(dot(p.normal,q.normal))>1-1e-8 && std::abs(p.offset-sign*q.offset)<options.detection.plane.distanceThreshold*.001){duplicate=true;break;}}
            if(!duplicate)unique.push_back(i);
        }
        const auto count=std::min(unique.size(),std::max(o.referenceMinPlanes,std::size_t(std::ceil((1-o.referenceAreaQuantile)*unique.size()))));unique.resize(count);
        if(count>=o.referenceMinPlanes) {
            std::vector<double> logs,noises;for(auto i:unique){logs.push_back(std::log(metrics[i].decision.density));noises.push_back(metrics[i].planeNoise);report.reference.patchIds.push_back(metrics[i].decision.patchId);}
            report.reference.density=std::exp(median(logs));report.reference.spacing=1/std::sqrt(report.reference.density);
            // Three references with two identical densities have MAD=0 even
            // when the third describes a genuinely different sampling regime.
            const double center=median(logs);double maximumDeviation=0;for(double v:logs)maximumDeviation=std::max(maximumDeviation,std::abs(v-center));
            report.reference.logSpread=std::max(noise(logs),maximumDeviation);report.reference.noise=median(noises);
            report.reference.available=report.reference.logSpread<=o.referenceMaxLogSpread;report.reference.reason=report.reference.available?"":"UnstableDensityReference";
        }
    }
    });
    std::vector<std::optional<PrimitiveEvidence>> results(metrics.size());
    if (!evaluatePatches("support.check_domains", metrics,
        [](const auto& entry) { return entry.decision.supportPointIds.size(); }, [&](std::size_t index) {
        auto& m = metrics[index];
        auto& d=m.decision;
        if(d.kind==PrimitiveKind::Plane && d.area>0 && report.reference.available) {
            d.densityRatio=d.density/report.reference.density;
            if(d.area*report.reference.density>=o.minimumExpectedPoints) {
                d.densityEvaluated=true;
                if(d.densityRatio<1/o.planeDensityRatioMax)fail(d,"PlaneDensityTooLow");
                if(d.densityRatio>o.planeDensityRatioMax)fail(d,"PlaneDensityTooHigh");
                // Reference spacing fixes the grid; its occupancy model uses rho_ref.
                d.occupiedCellRatio=occupancy(m,report.reference.spacing*o.gridSpacingMultiplier,report.reference.density,checkCancelled);
                if(d.occupiedCellRatio<0)return;
                d.occupancyEvaluated=true;if(d.occupiedCellRatio<o.minimumOccupiedCellRatio)fail(d,"InsufficientInteriorSupport");
            }
        }
        if(m.geometryValid && (d.kind==PrimitiveKind::Sphere || d.kind==PrimitiveKind::Cylinder) && d.supportPointIds.size()>=3)
            curvedChecks(m,o,d.kind==PrimitiveKind::Sphere?options.detection.sphere.distanceThreshold:options.detection.cylinder.distanceThreshold,report.reference);
        if(d.kind==PrimitiveKind::Cone && d.verdict==SupportVerdict::Accepted)
            curvedChecks(m,o,options.detection.cone.distanceThreshold,report.reference);
        if(d.verdict==SupportVerdict::Accepted) {
            if(d.kind==PrimitiveKind::Plane && report.reference.available) {
                // Freeze rejection metrics above; clipping can only create accepted subdomains.
                m.samples.clear();m.domain={};const auto initial=d;d.area=0;
                buildDomain(m,settings,std::sqrt(o.planeDensityRatioMax)*report.reference.spacing);
                const auto finalArea=d.area;const auto finalCovered=d.coveredPoints;d=initial;d.finalArea=finalArea;d.finalCoveredPoints=finalCovered;
                if(m.domain.coveredPointIds.size()<settings.minimumSurfacePoints){fail(d,"InsufficientTwoDimensionalSupport");}
            }
            if(d.verdict==SupportVerdict::Accepted) {
                auto e=m.evidence;
                const bool finite=d.kind!=PrimitiveKind::Torus;
                std::visit([&](auto& s){s.supportPointIds=finite?m.domain.coveredPointIds:d.supportPointIds;},e);
                results[index] = std::move(e);
            }
        }
        if(d.kind==PrimitiveKind::Plane && d.verdict==SupportVerdict::Accepted)
            d.footprint=std::make_shared<SurfaceFootprint>(std::move(m.domain));
    })) {
        error = "Cancelled";
        return false;
    }
    const diagnostics::StageProfile publish("support.merge_decisions");
    std::vector<PrimitiveEvidence> retained;
    for (std::size_t index = 0; index < metrics.size(); ++index) {
        if (results[index]) retained.push_back(std::move(*results[index]));
        report.decisions.push_back(std::move(metrics[index].decision));
    }
    if(cancelled && cancelled()){error="Cancelled";return false;}
    for(const auto& d:report.decisions)if(d.protectedManual && d.verdict!=SupportVerdict::Accepted && report.mode==SupportValidationMode::Enforce) {
        // A small explicit seed may be completed by a validated candidate of
        // the requested type; the seed itself never certifies a curved face.
        bool completed=false;
        for(const auto& e:retained)if(!d.supportPointIds.empty() && primitiveKind(e)==d.kind) {
            const auto& support=detail::analysis::ids(e);const std::unordered_set<PointId> covered(support.begin(),support.end());
            std::size_t shared=0;for(auto id:d.supportPointIds)shared+=covered.count(id);
            if(shared>=d.supportPointIds.size()*.98){completed=true;break;}
        }
        if(!completed){error="Manual surface failed support validation: "+d.reasons.front();return false;}
    }
    // A finite patch is the validation unit; equation identity remains shared.
    std::map<SurfaceId,PrimitiveEvidence> equations;
    for(auto& e:retained) {
        const auto id=std::visit([](const auto& s){return s.id;},e);
        auto inserted=equations.emplace(id,e);
        if(!inserted.second)std::visit([&](auto& s){const auto& more=detail::analysis::ids(e);s.supportPointIds.insert(s.supportPointIds.end(),more.begin(),more.end());std::sort(s.supportPointIds.begin(),s.supportPointIds.end());s.supportPointIds.erase(std::unique(s.supportPointIds.begin(),s.supportPointIds.end()),s.supportPointIds.end());},inserted.first->second);
    }
    retained.clear();for(auto& item:equations) {
        auto& evidence=item.second;
        if (auto* cone=std::get_if<ConeEvidence>(&evidence)) {
            const auto axis=normalized(cone->cone.axisDirection);
            const auto u=tangent(axis),v=cross(axis,u);
            cone->axialMinimum=INFINITY;cone->axialMaximum=-INFINITY;
            std::vector<double> angles;
            for (auto id:cone->supportPointIds) {
                const auto q=subtract(map.at(id)->position,cone->cone.apex);
                const double axial=dot(q,axis);
                cone->axialMinimum=std::min(cone->axialMinimum,axial);
                cone->axialMaximum=std::max(cone->axialMaximum,axial);
                angles.push_back(std::atan2(dot(q,v),dot(q,u)));
            }
            cone->angularCoverage=angularSpan(std::move(angles))/(2*pi);
        }
        std::visit([&](auto& s){double sum=0;s.maxAbsoluteResidual=0;
            for(auto id:s.supportPointIds){const double r=surfaceResidual(evidence,map.at(id)->position);sum+=r;s.maxAbsoluteResidual=std::max(s.maxAbsoluteResidual,r);}
            s.meanAbsoluteResidual=s.supportPointIds.empty()?0:sum/s.supportPointIds.size();
            s.coverage=double(s.supportPointIds.size())/std::max<std::size_t>(1,points.size());},evidence);
        retained.push_back(std::move(evidence));
    }
    accepted=report.mode==SupportValidationMode::Enforce?std::move(retained):proposals;output=std::move(report);return true;
    } catch(const std::exception& failure){error=failure.what();return false;}
}

std::string serializeSurfaceSupportReport(const SurfaceSupportValidationReport& r) {
    std::ostringstream s;s.imbue(std::locale::classic());s<<std::setprecision(17);
    auto number=[&](double v){s<<(std::isfinite(v)?v:0);};
    auto ids=[&](const auto& values){s<<'[';bool first=true;for(auto v:values){if(!first)s<<',';first=false;s<<v;}s<<']';};
    s<<"{\"version\":"<<SurfaceSupportValidationReport::version<<",\"point_fingerprint\":\""<<r.pointFingerprint<<"\",\"unit\":"<<int(r.unit)<<",\"mode\":"<<int(r.mode)<<",\"settings_signature\":";ids(r.settingsSignature);s<<",\"reference\":{\"available\":"<<(r.reference.available?"true":"false")<<",\"density\":";number(r.reference.density);s<<",\"spacing\":";number(r.reference.spacing);s<<",\"log_spread\":";number(r.reference.logSpread);s<<",\"noise\":";number(r.reference.noise);s<<",\"reason\":\""<<r.reference.reason<<"\",\"patch_ids\":";ids(r.reference.patchIds);s<<"},\"unit_name\":\""<<(r.unit==LengthUnit::Millimeter?"mm":r.unit==LengthUnit::Meter?"m":"arbitrary")<<"\",\"options\":{";
    bool optionFirst=true;
    auto option=[&](const char* key,double value){if(!optionFirst)s<<',';optionFirst=false;s<<'\"'<<key<<"\":";number(value);};
    const auto& o=r.options;
    option("support_validation_mode",int(o.mode));option("plane_density_ratio_max",o.planeDensityRatioMax);
    option("sphere_min_solid_angle_sr",o.sphereMinSolidAngle);option("sphere_min_minor_angular_span_deg",o.sphereMinMinorSpanRadians*180/pi);
    option("cylinder_min_angular_span_deg",o.cylinderMinAngularSpanRadians*180/pi);option("curvature_min_sagitta_noise_ratio",o.minimumSagittaNoiseRatio);
    option("density_reference_min_confidence",o.referenceMinConfidence);option("density_reference_min_points",double(o.referenceMinPoints));option("density_reference_min_planes",double(o.referenceMinPlanes));
    option("density_reference_area_quantile",o.referenceAreaQuantile);option("density_reference_max_log_spread",o.referenceMaxLogSpread);
    option("support_min_expected_points",o.minimumExpectedPoints);option("support_grid_spacing_multiplier",o.gridSpacingMultiplier);option("plane_min_occupied_cell_ratio",o.minimumOccupiedCellRatio);
    option("curvature_noise_floor_tolerance_fraction",o.noiseFloorToleranceFraction);option("curved_model_complexity_weight",o.modelComplexityWeight);option("curved_min_model_gain",o.minimumModelGain);option("plane_preference_min_reference_area_fraction",o.planePreferenceMinReferenceAreaFraction);
    s<<"},\"plane_hole_recovered_points\":"<<r.planeHoleRecoveredPoints
     <<",\"plane_relaxed_recovered_points\":"<<r.planeRelaxedRecoveredPoints<<",\"decisions\":[";
    bool first=true;for(const auto& d:r.decisions){if(!first)s<<',';first=false;
        s<<"{\"proposal_id\":"<<d.proposalId<<",\"patch_id\":"<<d.patchId<<",\"kind\":"<<int(d.kind)<<",\"verdict\":\""<<supportVerdictName(d.verdict)<<"\",\"point_count\":"<<d.pointCount<<",\"covered_points\":"<<d.coveredPoints<<",\"effective_points\":"<<d.effectivePoints<<",\"final_covered_points\":"<<d.finalCoveredPoints;
        auto field=[&](const char* name,double v){s<<",\""<<name<<"\":";number(v);};
        field("area",d.area);field("final_area",d.finalArea);field("density",d.density);field("density_ratio",d.densityRatio);field("occupied_cell_ratio",d.occupiedCellRatio);field("solid_angle_sr",d.solidAngle);field("minor_span_radians",d.minorSpanRadians);field("angular_span_radians",d.angularSpanRadians);field("sagitta",d.sagitta);field("noise",d.noise);field("plane_cost",d.planeCost);field("curved_cost",d.curvedCost);
        s<<",\"parameters\":"<<d.parametersJson<<",\"density_evaluated\":"<<(d.densityEvaluated?"true":"false")<<",\"occupancy_evaluated\":"<<(d.occupancyEvaluated?"true":"false")<<",\"noise_source\":\""<<d.noiseSource<<"\",\"reasons\":[";
        bool f=true;for(const auto& reason:d.reasons){if(!f)s<<',';f=false;s<<'\"'<<reason<<'\"';}s<<"]}";
    }s<<"]}";return s.str();
}

bool revalidateSurfaceSupportOnSource(const PointStore& working,const PointStore& source,
    SurfaceAnalysisResult& analysis,std::string& error,const std::function<bool()>& cancelled,const SurfaceAnalysisResult* validatedSource) {
    if(analysis.options.supportValidation.mode==SupportValidationMode::Off)return true;
    if(working.unit()!=source.unit()){error="Surface validation source unit mismatch";return false;}
    const auto sourceFingerprint=pointStoreFingerprint(source);
    if(analysis.baseFingerprint) {
        if(analysis.recoverySourceFingerprint!=sourceFingerprint){error="Changed recovery source invalidates repaired analysis";return false;}
        PointStore materialized({},working.unit());
        if(!materializeAnalysisPoints(working,analysis,materialized,error))return false;
        if(analysis.supportValidation.pointFingerprint!=analysis.pointFingerprint || analysis.supportValidation.settingsSignature!=surfaceSupportSettingsSignature(analysis.options)) {
            error="Repaired analysis requires validation on its materialized observations";return false;
        }
        return true;
    }
    if(analysis.sourceSupportValidation.pointFingerprint==sourceFingerprint &&
       analysis.sourceSupportValidation.settingsSignature==surfaceSupportSettingsSignature(analysis.options))return true;
    auto proposals=analysis.supportProposals.empty()?analysis.surfaces:analysis.supportProposals;
    std::vector<PrimitiveEvidence> sourceAccepted;SurfaceSupportValidationReport report;
    if(validatedSource && validatedSource->pointFingerprint==sourceFingerprint &&
        validatedSource->supportProposalIds.size()==validatedSource->surfaces.size() &&
       analysisSettingsMatch(analysis.options,validatedSource->options)) {
        sourceAccepted=validatedSource->surfaces;report=validatedSource->supportValidation;
        for (size_t i = 0; i < sourceAccepted.size(); ++i)
            std::visit([&](auto& s) { s.id = validatedSource->supportProposalIds[i]; }, sourceAccepted[i]);
    } else {
        if (!diagnostics::profileStage("source.expand_planes", [&] {
            return recoverPlaneSupport(source, analysis.options.detection.plane, proposals, error, {}, cancelled);
        }) || !diagnostics::profileStage("source.validate", [&] {
            return detail::analysis::validateDetectedSurfaceSupport(source, proposals, analysis.options,
                sourceAccepted, report, error, {}, cancelled);
        })) return false;
    }
    if(analysis.options.supportValidation.mode==SupportValidationMode::Report){analysis.sourceSupportValidation=std::move(report);return true;}
    const auto ids=detail::analysis::pointMap(working);
    std::vector<PrimitiveEvidence> workAccepted;
    for(auto e:sourceAccepted) {
        bool keep=false;
        std::visit([&](auto& s){auto& support=s.supportPointIds;support.erase(std::remove_if(support.begin(),support.end(),[&](auto id){return !ids.count(id);}),support.end());keep=support.size()>=analysis.options.supportMeshing.minimumSurfacePoints;},e);
        if(keep)workAccepted.push_back(std::move(e));
    }
    SurfaceAnalysisResult staged;
    if (!diagnostics::profileStage("source.reindex_working", [&] {
        return indexSurfaceEvidence(working, workAccepted, analysis.options, staged, error, {}, cancelled);
    })) return false;
    staged.sourceSupportValidation=std::move(report);
    staged.sourceValidatedSurfaces=std::move(sourceAccepted);
    staged.detectionPasses=analysis.detectionPasses;
    // A repair snapshot must be materialised before this API is used.
    staged.baseFingerprint=analysis.baseFingerprint;staged.recoverySourceFingerprint=analysis.recoverySourceFingerprint;
    staged.supplementalPoints=analysis.supplementalPoints;staged.localFailures=analysis.localFailures;
    analysis=std::move(staged);return true;
}

bool indexValidatedSourceSurfaces(const PointStore& source, const SurfaceAnalysisResult& workingAnalysis,
    SurfaceAnalysisResult& sourceIndex, std::string& error) {
    const auto& options = workingAnalysis.options;
    const auto& report = workingAnalysis.sourceSupportValidation;
    if (report.mode != SupportValidationMode::Enforce || report.pointFingerprint != pointStoreFingerprint(source) ||
        report.settingsSignature != surfaceSupportSettingsSignature(options)) {
        error = "Surface preview requires current source support validation";
        return false;
    }
    SurfaceAnalysisResult indexed;
    if (!indexSurfacePatches(source, workingAnalysis.sourceValidatedSurfaces, options, indexed, error)) return false;
    indexed.supportValidation = report;
    for (const auto& surface : workingAnalysis.sourceValidatedSurfaces)
        indexed.supportProposalIds.push_back(std::visit([](const auto& s) { return s.id; }, surface));
    sourceIndex = std::move(indexed);
    return true;
}
} // namespace reconstruction
