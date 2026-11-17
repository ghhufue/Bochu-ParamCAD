#include "rendering/PointCloudSurfaceMesh.h"
#include "rendering/PlanarSurfaceDomain.h"
#include "io/SurfaceEvidenceAdapter.h"
#include <reconstruction/settings/ModuleConfig.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <type_traits>
#include <unordered_map>

namespace {
using V = modeling::Vec3;
V add(V a,V b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
V sub(V a,V b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
V mul(V a,double s){return {a.x*s,a.y*s,a.z*s};}
double dot(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
V cross(V a,V b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double norm(V a){return std::sqrt(dot(a,a));}
V unit(V a){return mul(a,1/std::max(norm(a),1e-30));}
constexpr double pi=3.14159265358979323846;
struct UV {double u=0,v=0;};
struct Sample {UV uv;vtkIdType source=0;};
struct Chart {
    QString kind;
    V origin{},x{},y{},z{};
    double radius=1,minor=0,angle=0,metric=1,periodU=0,periodV=0;
    UV project(V p) const {
        const auto d=sub(p,origin);
        const double a=dot(d,x),b=dot(d,y),t=dot(d,z);
        if(kind=="plane")return {a,b};
        double theta=std::atan2(b,a);if(theta<0)theta+=2*pi;
        if(kind=="sphere")return {theta*metric,std::atan2(t,std::hypot(a,b))*radius};
        if(kind=="ring") {
            double phi=std::atan2(t,std::hypot(a,b)-radius);if(phi<0)phi+=2*pi;
            return {theta*metric,phi*minor};
        }
        // Cone distance follows its generator, not its compressed axis. An
        // orthogonal projection also avoids amplifying normal noise by tan(a).
        if(kind=="cone")return {theta*metric,std::hypot(a,b)*std::sin(angle)+t*std::cos(angle)};
        return {theta*metric,t};
    }
    V evaluate(UV p) const {
        if(kind=="plane")return add(origin,add(mul(x,p.u),mul(y,p.v)));
        const double theta=p.u/metric;
        const auto radial=add(mul(x,std::cos(theta)),mul(y,std::sin(theta)));
        if(kind=="sphere")return add(origin,add(mul(radial,radius*std::cos(p.v/radius)),mul(z,radius*std::sin(p.v/radius))));
        if(kind=="ring")return add(origin,add(mul(radial,radius+minor*std::cos(p.v/minor)),mul(z,minor*std::sin(p.v/minor))));
        if(kind=="cone")return add(origin,add(mul(radial,p.v*std::sin(angle)),mul(z,p.v*std::cos(angle))));
        return add(origin,add(mul(radial,radius),mul(z,p.v)));
    }
};
using Kernel=CGAL::Exact_predicates_inexact_constructions_kernel;
struct Info {vtkIdType index=0;double spacing2=0;};
using Vertex=CGAL::Triangulation_vertex_base_with_info_2<Info,Kernel>;
using DT=CGAL::Delaunay_triangulation_2<Kernel,CGAL::Triangulation_data_structure_2<Vertex>>;
}

PointCloudSurfaceMesh meshPointCloudSurface(const PointCloudRegion& region,
    const PlyCloud& cloud,const std::vector<vtkIdType>& members)
{
    return meshPointCloudSurface(region,cloud,members,reconstruction::loadAnalysisSettings().meshing);
}

PointCloudSurfaceMesh meshPointCloudSurface(const PointCloudRegion& region,
    const PlyCloud& cloud,const std::vector<vtkIdType>& members,const reconstruction::SurfaceMeshingSettings& settings)
{
    PointCloudSurfaceMesh mesh;
    if(region.supportValidation.value("verdict").toString()=="rejected")return mesh;
    if(members.size()<std::max<size_t>(3,settings.minimumSurfacePoints))return mesh;
    const auto evidence=regionEvidence(region);
    if(!evidence)return mesh;
    Chart chart;chart.kind=region.geometry;
    std::visit([&](const auto& e) {
        using T=std::decay_t<decltype(e)>;
        if constexpr(std::is_same_v<T,reconstruction::PlaneEvidence>) {
            chart.z=e.plane.normal;
            // Use a nearby origin to retain precision for translated scans.
            const auto& p=cloud.positions[size_t(members.front())];
            chart.origin={p[0],p[1],p[2]};
            chart.origin=sub(chart.origin,mul(chart.z,dot(chart.z,chart.origin)+e.plane.offset));
        } else if constexpr(std::is_same_v<T,reconstruction::CylinderEvidence>) {
            chart.origin=e.cylinder.axisPoint;chart.z=e.cylinder.axisDirection;chart.radius=e.cylinder.radius;
        } else if constexpr(std::is_same_v<T,reconstruction::ConeEvidence>) {
            chart.origin=e.cone.apex;chart.z=e.cone.axisDirection;chart.angle=e.cone.openingAngleRadians;
        } else if constexpr(std::is_same_v<T,reconstruction::SphereEvidence>) {
            chart.origin=e.sphere.center;chart.z={0,0,1};chart.radius=e.sphere.radius;
        } else {
            chart.origin=e.torus.center;chart.z=e.torus.axisDirection;
            chart.radius=e.torus.majorRadius;chart.minor=e.torus.minorRadius;
        }
    },*evidence);
    const V reference=std::abs(chart.z.x)<.8?V{1,0,0}:V{0,1,0};
    chart.x=unit(sub(reference,mul(chart.z,dot(reference,chart.z))));chart.y=cross(chart.z,chart.x);
    chart.metric=chart.radius;
    if(chart.kind=="cone") {
        double total=0;
        for(auto id:members) {
            const auto& p=cloud.positions[size_t(id)];const auto d=sub({p[0],p[1],p[2]},chart.origin);
            const double t=dot(d,chart.z),r=norm(sub(d,mul(chart.z,t)));
            total+=std::max(0.,r*std::sin(chart.angle)+t*std::cos(chart.angle));
        }
        chart.metric=total/members.size()*std::sin(chart.angle);
    }
    if(!(chart.metric>1e-12))return mesh;
    if(chart.kind!="plane")chart.periodU=2*pi*chart.metric;
    if(chart.kind=="ring")chart.periodV=2*pi*chart.minor;
    std::vector<Sample> projected;projected.reserve(members.size());
    for(auto id:members) {
        const auto& p=cloud.positions[size_t(id)];const auto uv=chart.project({p[0],p[1],p[2]});
        if(std::isfinite(uv.u)&&std::isfinite(uv.v) && (chart.kind!="cone" || uv.v>0))projected.push_back({uv,id});
    }
    if(projected.size()<3)return mesh;
    if(chart.kind=="plane") {
        std::vector<PlanarSurfaceSample> samples;samples.reserve(projected.size());
        for(const auto& p:projected)samples.push_back({{p.uv.u,p.uv.v},p.source});
        const double reference=region.supportValidation.value("reference_spacing").toDouble();
        const double ratio=region.supportValidation.value("density_ratio_max").toDouble(1);
        mesh=meshPlanarSurfaceDomain(samples,settings,reference>0?reference*std::sqrt(std::max(1.,ratio)):0);
        for(auto& position:mesh.positions) {
            const auto p=chart.evaluate({position[0],position[1]});position={p.x,p.y,p.z};
        }
        return mesh;
    }
    UV low=projected.front().uv,high=low;
    for(const auto& sample:projected){const auto p=sample.uv;low.u=std::min(low.u,p.u);low.v=std::min(low.v,p.v);high.u=std::max(high.u,p.u);high.v=std::max(high.v,p.v);}
    if(std::max(high.u-low.u,high.v-low.v)<=1e-12)return mesh;
    // Spatial thinning bounds UI work on dense scans without selecting a strip
    // from a source file's point order. Small collections retain every sample.
    const size_t budget=settings.meshPointBudget;
    const auto observed=projected;
    if(projected.size()>budget) {
        double cell=std::max(high.u-low.u,high.v-low.v)/std::sqrt(double(budget));
        for(;;cell*=1.5) {
            std::map<std::pair<long long,long long>,Sample> bins;
            for(auto sample:projected){const auto p=sample.uv;bins.try_emplace({std::llround((p.u-low.u)/cell),std::llround((p.v-low.v)/cell)},sample);}
            if(bins.size()<=budget){projected.clear();for(const auto& entry:bins)projected.push_back(entry.second);break;}
        }
    }
    std::vector<std::pair<Kernel::Point_2,Info>> input;
    for(size_t i=0;i<projected.size();++i) {
        const auto uv=projected[i].uv;const auto p=chart.evaluate(uv);mesh.positions.push_back({p.x,p.y,p.z});
        mesh.sourcePointIds.push_back(projected[i].source);
        for(int u=chart.periodU?-1:0;u<=(chart.periodU?1:0);++u)
            for(int v=chart.periodV?-1:0;v<=(chart.periodV?1:0);++v)
                input.emplace_back(Kernel::Point_2(uv.u+u*chart.periodU,uv.v+v*chart.periodV),Info{vtkIdType(i),0});
    }
    DT triangulation;triangulation.insert(input.begin(),input.end());
    if(triangulation.dimension()!=2)return {};
    // Alpha filtering preserves holes, concavities and separate islands. A
    // convex hull would fill all of them and misrepresent the detected surface.
    for(auto v=triangulation.finite_vertices_begin();v!=triangulation.finite_vertices_end();++v) {
        std::vector<double> distances;auto adjacent=triangulation.incident_vertices(v),end=adjacent;
        if(adjacent!=0)do {
            if(!triangulation.is_infinite(adjacent))distances.push_back(CGAL::to_double(CGAL::squared_distance(v->point(),adjacent->point())));
        }while(++adjacent!=end);
        std::sort(distances.begin(),distances.end());
        v->info().spacing2=distances.front();
    }
    std::set<std::array<vtkIdType,3>> emitted;
    for(auto face=triangulation.finite_faces_begin();face!=triangulation.finite_faces_end();++face) {
        const auto a=face->vertex(0),b=face->vertex(1),c=face->vertex(2);
        const double u=(a->point().x()+b->point().x()+c->point().x())/3;
        const double v=(a->point().y()+b->point().y()+c->point().y())/3;
        if((chart.periodU&&(u<0||u>=chart.periodU))||(chart.periodV&&(v<0||v>=chart.periodV)))continue;
        const double ab=CGAL::to_double(CGAL::squared_distance(a->point(),b->point()));
        const double bc=CGAL::to_double(CGAL::squared_distance(b->point(),c->point()));
        const double ca=CGAL::to_double(CGAL::squared_distance(c->point(),a->point()));
        const double area2=CGAL::to_double(CGAL::area(a->point(),b->point(),c->point()))*2;
        std::array<double,3> local{a->info().spacing2,b->info().spacing2,c->info().spacing2};
        std::sort(local.begin(),local.end());
        const double spacing2=local[0];
        const double alpha=4*settings.curvedAlphaRadiusMultiplier*settings.curvedAlphaRadiusMultiplier;
        if(area2<=0 || ab*bc*ca>alpha*spacing2*area2*area2)continue;
        std::array<vtkIdType,3> ids{a->info().index,b->info().index,c->info().index};
        if(ids[0]==ids[1]||ids[1]==ids[2]||ids[2]==ids[0])continue;
        auto key=ids;std::sort(key.begin(),key.end());
        if(emitted.insert(key).second)mesh.triangles.push_back(ids);
    }
    if(mesh.triangles.empty())return {};
    // A large collection cannot legitimise a separate tiny triangle island.
    std::vector<size_t> parent(mesh.triangles.size());std::iota(parent.begin(),parent.end(),0);
    const auto root=[&](size_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    std::map<std::pair<vtkIdType,vtkIdType>,size_t> edges;
    for(size_t i=0;i<mesh.triangles.size();++i)for(size_t j=0;j<3;++j) {
        auto a=mesh.triangles[i][j],b=mesh.triangles[i][(j+1)%3];if(a>b)std::swap(a,b);
        const auto entry=edges.emplace(std::make_pair(a,b),i);
        if(!entry.second)parent[root(i)]=root(entry.first->second);
    }
    std::map<size_t,std::set<vtkIdType>> componentPoints;
    for(size_t i=0;i<mesh.triangles.size();++i)for(auto vertex:mesh.triangles[i])componentPoints[root(i)].insert(vertex);
    std::vector<std::array<vtkIdType,3>> triangles;emitted.clear();
    for(size_t i=0;i<mesh.triangles.size();++i)if(componentPoints.at(root(i)).size()>=settings.minimumSurfacePoints) {
        const auto triangle=mesh.triangles[i];triangles.push_back(triangle);
        auto key=triangle;std::sort(key.begin(),key.end());emitted.insert(key);
    }
    mesh.triangles=std::move(triangles);if(mesh.triangles.empty())return {};
    // Alpha-rejected tails and isolated observations have no surface coverage.
    // Test the original chart samples, including those omitted by thinning.
    const auto covered=[&](DT::Face_handle face) {
        if(triangulation.is_infinite(face))return false;
        std::array<vtkIdType,3> ids{face->vertex(0)->info().index,face->vertex(1)->info().index,face->vertex(2)->info().index};
        std::sort(ids.begin(),ids.end());return emitted.count(ids)!=0;
    };
    DT::Face_handle hint;
    for(const auto& sample:observed) {
        DT::Locate_type type;int index=0;
        const auto face=triangulation.locate(Kernel::Point_2(sample.uv.u,sample.uv.v),type,index,hint);hint=face;
        bool supported=covered(face);
        if(!supported && type==DT::EDGE)supported=covered(face->neighbor(index));
        if(!supported && type==DT::VERTEX) {
            auto adjacent=triangulation.incident_faces(face->vertex(index)),end=adjacent;
            if(adjacent!=0)do {supported=supported || covered(adjacent);}while(++adjacent!=end);
        }
        if(supported)mesh.coveredPointIds.push_back(sample.source);
    }
    return mesh;
}

PointCloudSurfaceDisplay meshPointCloudSurfaces(const PointCloudSelection& selection)
{
    return meshPointCloudSurfaces(selection,reconstruction::loadAnalysisSettings().meshing);
}

PointCloudSurfaceDisplay meshPointCloudSurfaces(const PointCloudSelection& selection,const reconstruction::SurfaceMeshingSettings& settings)
{
    PointCloudSurfaceDisplay display;
    std::map<int,std::vector<vtkIdType>> members;
    for(const auto& region:selection.regions())
        if(!region.fitStale && !region.parameters.isEmpty())members.emplace(region.id,std::vector<vtkIdType>{});
    for(size_t i=0;i<selection.memberships().size();++i) {
        if(!selection.active(vtkIdType(i)))continue;
        const auto found=members.find(selection.memberships()[i]);
        if(found!=members.end())found->second.push_back(vtkIdType(i));
    }
    // The detector can publish several connected patches of one evidence
    // surface. Reuse its equation rather than refitting or moving that plane.
    std::map<std::array<double,4>,std::vector<const PointCloudRegion*>> planes;
    for(const auto& region:selection.regions()) {
        const auto found=members.find(region.id);
        if(found==members.end() || found->second.size()<settings.minimumSurfacePoints)continue;
        const auto evidence=regionEvidence(region);
        if(!evidence)continue;
        if(!region.supportValidation.isEmpty()) {
            auto mesh=meshPointCloudSurface(region,selection.sourceCloud(),found->second,settings);
            display.planeSupportChecks[region.id]=mesh.domainSupportChecks;
            if(!mesh.triangles.empty())display.meshes.emplace(region.id,std::move(mesh));
            display.colorRegions[region.id]=region.id;
        } else if(const auto* plane=std::get_if<reconstruction::PlaneEvidence>(&*evidence)) {
            auto n=plane->plane.normal;double d=plane->plane.offset;
            const double major=std::abs(n.x)>=std::abs(n.y)&&std::abs(n.x)>=std::abs(n.z)?n.x:
                (std::abs(n.y)>=std::abs(n.z)?n.y:n.z);
            if(major<0){n=mul(n,-1);d=-d;}
            planes[{n.x,n.y,n.z,d}].push_back(&region);
        } else {
            auto mesh=meshPointCloudSurface(region,selection.sourceCloud(),found->second,settings);
            if(!mesh.triangles.empty())display.meshes.emplace(region.id,std::move(mesh));
        }
    }
    for(const auto& family:planes) {
        const auto& regions=family.second;
        const auto* representative=*std::min_element(regions.begin(),regions.end(),[](const auto* a,const auto* b){return a->id<b->id;});
        std::vector<vtkIdType> combined;
        for(const auto* region:regions) {
            const auto& ids=members.at(region->id);combined.insert(combined.end(),ids.begin(),ids.end());
            display.colorRegions[region->id]=representative->id;
        }
        auto mesh=meshPointCloudSurface(*representative,selection.sourceCloud(),combined,settings);
        display.planeSupportChecks[representative->id]=mesh.domainSupportChecks;
        std::map<int,std::unordered_map<vtkIdType,vtkIdType>> remaps;
        for(const auto& triangle:mesh.triangles) {
            std::array<int,3> owners;
            for(size_t i=0;i<3;++i)owners[i]=selection.memberships()[size_t(mesh.sourcePointIds[size_t(triangle[i])])];
            // A majority keeps most of each patch's triangles with its owner.
            // Ties use the lowest collection ID, independent of winding.
            int owner=std::min({owners[0],owners[1],owners[2]});
            if(owners[0]==owners[1]||owners[0]==owners[2])owner=owners[0];
            else if(owners[1]==owners[2])owner=owners[1];
            auto& output=display.meshes[owner];auto& remap=remaps[owner];
            if(output.triangles.empty())output.domainSupportChecks=mesh.domainSupportChecks;
            std::array<vtkIdType,3> ids;
            for(size_t i=0;i<3;++i) {
                const auto old=triangle[i];const auto inserted=remap.emplace(old,vtkIdType(output.positions.size()));
                if(inserted.second){output.positions.push_back(mesh.positions[size_t(old)]);output.sourcePointIds.push_back(mesh.sourcePointIds[size_t(old)]);}
                ids[i]=inserted.first->second;
            }
            output.triangles.push_back(ids);
        }
        for(auto id:mesh.coveredPointIds) {
            const int owner=selection.memberships()[size_t(id)];const auto found=display.meshes.find(owner);
            if(found!=display.meshes.end())found->second.coveredPointIds.push_back(id);
        }
    }
    return display;
}
