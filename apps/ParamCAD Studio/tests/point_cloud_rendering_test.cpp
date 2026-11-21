#include "core/PointCloudSelection.h"
#include "rendering/PointCloudActor.h"
#include "SurfaceMeshMetrics.h"
#include <reconstruction/surface/SurfaceSupportValidation.h>
#include <QCoreApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <vtkActor.h>
#include <vtkCellData.h>
#include <vtkCell.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <random>

void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
void vertexTouchingBoundaries()
{
    PointCloudRegion region;
    region.geometry = "plane";
    region.parameters = {{"normal", QJsonArray{0, 0, 1}}, {"offset", 0}};
    PointCloudSurfaceMesh mesh;
    mesh.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}, {0, -1, 0}};
    mesh.triangles = {{0, 1, 2}, {0, 4, 3}};
    const auto boundary = planarBoundaryOfMesh(region, mesh);
    require(boundary && boundary->regions.size() == 2, "triangles touching at a vertex have separate outer rings");
    for (const auto& component : boundary->regions)
        require(component.outer.size() == 3 && component.holes.empty(), "each triangle retains its own closed ring");
}

void surfaceDisplay()
{
    PlyCloud cloud;
    std::vector<vtkIdType> members;
    // A tilted face with a hole and a separate island on the same plane.
    for(int x=-10;x<=10;++x)for(int y=-10;y<=10;++y) {
        if(std::abs(x)<4 && std::abs(y)<4)continue;
        members.push_back(vtkIdType(cloud.positions.size()));
        cloud.positions.push_back({float(x),float(y),float(3+.5*x+.2*y)});
    }
    for(int x=30;x<=35;++x)for(int y=0;y<=5;++y) {
        members.push_back(vtkIdType(cloud.positions.size()));
        cloud.positions.push_back({float(x),float(y),float(3+.5*x+.2*y)});
    }
    members.push_back(vtkIdType(cloud.positions.size()));
    cloud.positions.push_back({15,50,20.5F}); // Sparse spur must not stretch the face.
    cloud.positions.push_back({100,100,100});
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{-.5,-.2,1}},{"offset",-3}};
    PointCloudSelection selection;selection.setCloud(cloud);
    selection.addSurfaces({{region,members}});
    const auto revision=selection.revision();
    const int id=selection.regions().front().id;
    PointCloudActor actor;actor.setCloud(cloud);actor.setAnnotations(&selection);
    auto* points=vtkPolyDataMapper::SafeDownCast(actor.actor()->GetMapper())->GetInput();
    auto* surface=vtkPolyDataMapper::SafeDownCast(actor.surfaceActor()->GetMapper())->GetInput();
    require(actor.displayedSurfaceRegions()==std::set<int>{id},"open fitted face is displayed by default");
    const auto* retained = actor.surfaceMesh(id);
    const auto boundary = planarBoundaryOfMesh(*selection.region(id), *retained);
    require(boundary && boundary->regions.size() == 2 && boundary->regions.front().holes.size() == 1 &&
            boundary->regions.back().holes.empty(), "the fitted mesh yields independent islands and the observed hole");
    using Edge = std::pair<std::size_t, std::size_t>;
    std::map<Edge, int> incidence;
    for (const auto& triangle : retained->triangles)
        for (std::size_t i = 0; i < 3; ++i)
            ++incidence[std::minmax(std::size_t(triangle[i]), std::size_t(triangle[(i + 1) % 3]))];
    std::set<Edge> meshBoundary, tracedBoundary;
    for (const auto& [edge, count] : incidence)
        if (count == 1) meshBoundary.insert(edge);
    const auto collect = [&](const auto& ring) {
        for (std::size_t i = 0; i < ring.size(); ++i)
            tracedBoundary.insert(std::minmax(ring[i], ring[(i + 1) % ring.size()]));
    };
    for (const auto& component : boundary->regions) {
        collect(component.outer);
        for (const auto& hole : component.holes) collect(hole);
    }
    require(meshBoundary == tracedBoundary, "every exposed mesh edge occurs in the traced rings");
    require(surface->GetNumberOfPolys()>0 && points->GetNumberOfVerts()==2,"surface hides supported samples while retaining an unassigned point and an unsupported spur");
    const auto color=PointCloudActor::regionColor(id);
    auto* colors=surface->GetCellData()->GetScalars();
    for(int i=0;i<3;++i)require(colors->GetComponent(0,i)==color[size_t(i)],"surface uses the point collection palette");
    double p[3];
    for(vtkIdType i=0;i<surface->GetNumberOfPoints();++i) {
        surface->GetPoint(i,p);require(std::abs(p[2]-.5*p[0]-.2*p[1]-3)<1e-10,"render vertices lie on the fitted plane");
    }
    for(vtkIdType i=0;i<surface->GetNumberOfCells();++i) {
        auto* cell=surface->GetCell(i);double x=0,y=0,low=INFINITY,high=-INFINITY,maxY=-INFINITY;
        for(int j=0;j<3;++j){surface->GetPoint(cell->GetPointId(j),p);x+=p[0]/3;y+=p[1]/3;low=std::min(low,p[0]);high=std::max(high,p[0]);maxY=std::max(maxY,p[1]);}
        require(!(std::abs(x)<2 && std::abs(y)<2),"surface preview preserves the observed hole");
        require(!(low<=10 && high>=30),"coplanar islands are not bridged by a convex hull");
        require(maxY<40,"isolated support cannot stretch triangles across unobserved space");
    }
    const auto meshTime=surface->GetMTime();
    selection.selectRegion(id);actor.setAnnotations(&selection);
    require(surface->GetMTime()==meshTime,"selection does not retriangulate surfaces");
    auto* highlight=vtkPolyDataMapper::SafeDownCast(actor.surfaceHighlightActor()->GetMapper())->GetInput();
    require(actor.surfaceActor()->GetPickable() && !actor.surfaceHighlightActor()->GetPickable(),"only the displayed surface participates in ray picking");
    require(actor.surfaceRegionOfCell(0)==id && actor.surfaceRegionOfCell(-1)==0
        && actor.surfaceRegionOfCell(surface->GetNumberOfCells())==0,"surface cells retain their owning collection");
    require(actor.surfaceHighlightActor()->GetVisibility() && highlight->GetNumberOfPolys()==surface->GetNumberOfPolys(),"selecting a collection highlights its entire rendered surface");
    selection.select({0});actor.setAnnotations(&selection);
    require(highlight->GetNumberOfPolys()==surface->GetNumberOfPolys(),"point lookup highlights its owning face in surface mode");
    selection.select({});actor.setAnnotations(&selection);
    require(!actor.surfaceHighlightActor()->GetVisibility(),"clearing selection removes the face highlight");
    selection.selectRegion(id);actor.setAnnotations(&selection);
    auto* selected=vtkPolyDataMapper::SafeDownCast(actor.selectionActor()->GetMapper())->GetInput();
    require(selected->GetNumberOfPoints()==1,"surface mode hides covered point highlights and retains the unsupported spur highlight");
    actor.setRegionSurfaceDisplay(id,false);actor.setAnnotations(&selection);
    require(points->GetNumberOfVerts()==vtkIdType(cloud.positions.size()) && !actor.surfaceActor()->GetVisibility(),"per-collection switch restores original points");
    require(selected->GetNumberOfPoints()==vtkIdType(members.size()),"point mode restores selection highlight");
    actor.setAutomaticSurfaceDisplay(false);
    actor.setRegionSurfaceDisplay(id,true);actor.setAnnotations(&selection);
    require(actor.surfaceActor()->GetVisibility() && points->GetNumberOfVerts()==2,"manual surface choice overrides the automatic default");
    actor.setVisible(false);require(!actor.surfaceActor()->GetVisibility(),"scene visibility hides surfaces too");
    require(!actor.surfaceHighlightActor()->GetVisibility(),"hidden cloud cannot leave a face highlight visible");
    actor.setVisible(true);require(actor.surfaceActor()->GetVisibility(),"scene visibility restores surfaces");
    actor.setAnnotations(nullptr);
    require(!actor.surfaceHighlightActor()->GetVisibility(),"capture excludes face highlights");
    require(!actor.surfaceActor()->GetVisibility() && points->GetNumberOfVerts()==vtkIdType(cloud.positions.size()),"capture restores source points without fitted overlays");
    actor.setAnnotations(&selection);
    require(actor.surfaceActor()->GetVisibility() && selection.revision()==revision,"display switches leave source and document revision unchanged");
    const auto beforeRelations = surface->GetMTime();
    selection.applyInferredRelations({}, false);
    actor.setAnnotations(&selection);
    require(surface->GetMTime() == beforeRelations && actor.surfaceMesh(id) == retained,
            "relation updates retain the existing display mesh");
    require(!selection.replacePartCandidates({{region, members}}, false) && selection.region(id),
            "identical reconstruction candidates preserve the collection and display cache");
    selection.select({0});selection.deleteSelectedPoints();actor.setPointMask(selection.deleted());actor.setAnnotations(&selection);
    require(actor.availableSurfaceRegions().empty() && points->GetNumberOfVerts()==vtkIdType(cloud.positions.size()-1),"stale fit returns to points after membership edits");
    require(selection.undo(),"undo membership edit");actor.setPointMask(selection.deleted());actor.setAnnotations(&selection);
    require(actor.surfaceActor()->GetVisibility(),"undo restores the fitted surface");
    actor.clear();require(actor.availableSurfaceRegions().empty() && !actor.surfaceActor()->GetVisibility(),"clear discards surface cache");
    actor.setCloud(cloud);actor.setAutomaticSurfaceDisplay(false);actor.setAnnotations(&selection);
    require(actor.displayedSurfaceRegions().empty() && points->GetNumberOfVerts()==vtkIdType(cloud.positions.size()),"disabled automatic display retains points on a new cloud");
}

void finalSurfaceMinimum()
{
    PlyCloud cloud;std::vector<vtkIdType> ids;
    for(int x=0;x<5;++x)for(int y=0;y<5;++y){ids.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({float(x),float(y),0});}
    PointCloudRegion plane;plane.geometry="plane";plane.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    reconstruction::SurfaceMeshingSettings settings;
    require(!meshPointCloudSurface(plane,cloud,ids,settings).triangles.empty(),"default support policy accepts a sampled plane");
    settings.minimumSurfacePoints=26;
    require(meshPointCloudSurface(plane,cloud,ids,settings).triangles.empty(),"configured final support minimum controls rendering");
    PointCloudSelection selection;selection.setCloud(cloud);
    std::vector<vtkIdType> seven(ids.begin(),ids.begin()+7);
    selection.addSurfaces({{plane,seven}});
    PointCloudActor actor;actor.setCloud(cloud);actor.setAnnotations(&selection);
    require(actor.availableSurfaceRegions().empty(),"legacy seven-point collection is retained as points without a surface");
    auto* visible=vtkPolyDataMapper::SafeDownCast(actor.actor()->GetMapper())->GetInput();
    require(visible->GetNumberOfVerts()==25,"subminimum collection cannot hide its original observations");
}

void wideConeSupport()
{
    reconstruction::SurfaceMeshingSettings geometrySettings;geometrySettings.minimumSurfacePoints=3;
    const double angle=1.47126159858816;
    const auto point=[&](double slant,double theta,double normalOffset=0.) {
        const double radius=slant*std::sin(angle)+normalOffset*std::cos(angle);
        return std::array<float,3>{float(radius*std::cos(theta)),float(radius*std::sin(theta)),
            float(slant*std::cos(angle)-normalOffset*std::sin(angle))};
    };
    PointCloudRegion cone;cone.geometry="cone";
    cone.parameters={{"center",QJsonArray{0,0,0}},{"axis",QJsonArray{0,0,1}},{"angle",angle}};
    PlyCloud cloud;std::vector<vtkIdType> members;
    for(int i=0;i<6;++i) {
        members.push_back(vtkIdType(cloud.positions.size()));
        cloud.positions.push_back(point(1000+(i%2)*.3,(i-2.5)*.006));
    }
    members.push_back(6);cloud.positions.push_back(point(1200,0));
    auto mesh=meshPointCloudSurface(cone,cloud,members,geometrySettings);
    for(const auto& triangle:mesh.triangles)for(auto vertex:triangle)
        require(mesh.sourcePointIds[size_t(vertex)]!=6,"wide cone must not bridge a boundary strip to a remote observation");
    require(std::find(mesh.coveredPointIds.begin(),mesh.coveredPointIds.end(),6)==mesh.coveredPointIds.end(),
        "unmeshed curved observations must remain visible");
    // A lowered explicit policy can still display a geometrically valid sparse patch.
    cloud.positions.clear();members.clear();
    for(int i=-1;i<=1;++i)for(int j=0;j<3;++j) {
        members.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back(point(100+j*10,i*.1));
    }
    require(meshPointCloudSurface(cone,cloud,members).triangles.empty(),"default final support policy rejects a nine-point cone");
    mesh=meshPointCloudSurface(cone,cloud,members,geometrySettings);
    require(!mesh.triangles.empty(),"a uniformly sparse nine-point cone patch retains its surface");
    cloud.positions.clear();members.clear();
    for(int i=0;i<20;++i)for(int j=0;j<20;++j) {
        members.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back(point(100+j,i*.01,3));
    }
    mesh=meshPointCloudSurface(cone,cloud,members,geometrySettings);
    require(!mesh.triangles.empty(),"noisy wide cone retains valid surface support");
    for(size_t i=0;i<mesh.positions.size();++i) {
        const auto& p=mesh.positions[i];const auto& observation=cloud.positions[size_t(mesh.sourcePointIds[i])];
        double squared=0;for(size_t a=0;a<3;++a)squared+=std::pow(p[a]-observation[a],2);
        require(std::sqrt(squared)<3.001,"cone render vertices use the orthogonal fit projection without radial amplification");
    }
}

void curvedSurfaces()
{
    constexpr double pi=3.14159265358979323846;
    for(const auto& kind: {QString("cylinder"),QString("cone"),QString("sphere"),QString("ring")}) {
        PlyCloud cloud;std::vector<vtkIdType> members;
        for(int i=0;i<72;++i)for(int j=0;j<24;++j) {
            const double u=(i+.5)*2*pi/72;
            double radius=5,z=(j+.5)*10/24;
            if(kind=="cone")radius=z*std::tan(.4);
            if(kind=="sphere"){const double v=-pi/2+(j+.5)*pi/24;radius=5*std::cos(v);z=5*std::sin(v);}
            if(kind=="ring"){const double v=(j+.5)*2*pi/24;radius=5+std::cos(v);z=std::sin(v);}
            members.push_back(vtkIdType(cloud.positions.size()));
            cloud.positions.push_back({float(radius*std::cos(u)),float(radius*std::sin(u)),float(z)});
        }
        PointCloudRegion region;region.geometry=kind;
        region.parameters={{"center",QJsonArray{0,0,0}},{"axis",QJsonArray{0,0,1}},
            {"radius",5},{"angle",.4},{"major_radius",5},{"minor_radius",1}};
        const auto mesh=meshPointCloudSurface(region,cloud,members);
        require(!mesh.triangles.empty(),"analytic curved surface renders without a closed solid");
        require(mesh.coveredPointIds.size()==members.size(),"dense curved support retains coverage across angular seams");
        bool seam=false;
        for(const auto& t:mesh.triangles) {
            double low=INFINITY,high=-INFINITY;
            for(auto id:t){const auto& p=mesh.positions[size_t(id)];const double a=std::atan2(p[1],p[0]);low=std::min(low,a);high=std::max(high,a);}
            seam|=high-low>pi;
        }
        require(seam,"periodic curved surface joins across its angular seam");
        region.fitStale=true;
        require(meshPointCloudSurface(region,cloud,members).triangles.empty(),"stale curved fit is never rendered");
    }
}

double meshArea(const PointCloudSurfaceMesh& mesh)
{
    double result=0;
    for(const auto& t:mesh.triangles) {
        const auto& a=mesh.positions[size_t(t[0])];const auto& b=mesh.positions[size_t(t[1])];const auto& c=mesh.positions[size_t(t[2])];
        const double x=(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]);
        const double y=(b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]);
        const double z=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
        result+=std::sqrt(x*x+y*y+z*z)*.5;
    }
    return result;
}

bool covers(const PointCloudSurfaceMesh& mesh,double x,double y)
{
    for(const auto& triangle:mesh.triangles) {
        std::array<double,3> sides;
        for(size_t i=0;i<3;++i) {
            const auto& a=mesh.positions[size_t(triangle[i])];
            const auto& b=mesh.positions[size_t(triangle[(i+1)%3])];
            sides[i]=(b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]);
        }
        if(*std::min_element(sides.begin(),sides.end())>=-1e-9
            || *std::max_element(sides.begin(),sides.end())<=1e-9)return true;
    }
    return false;
}

void validatedSphereDisplay()
{
    constexpr double radius = 8.;
    PlyCloud cloud;
    reconstruction::SphereEvidence sphere;
    sphere.sphere = {{0, 0, 0}, radius};
    sphere.confidence = .99;
    std::vector<reconstruction::PointSample> samples;
    for (int i = -20; i <= 20; ++i) for (int j = -20; j <= 20; ++j) {
        const double x = i * .2, y = j * .2;
        if (std::hypot(x, y) < 1.5) continue;
        const double z = std::sqrt(radius * radius - x * x - y * y);
        cloud.positions.push_back({float(x), float(y), float(z)});
        reconstruction::PointSample point;
        point.id = cloud.positions.size();
        point.position = {x, y, z};
        samples.push_back(point);
        sphere.supportPointIds.push_back(point.id);
    }
    const reconstruction::PointStore source(std::move(samples), reconstruction::LengthUnit::Millimeter);
    reconstruction::SurfaceAnalysisOptions options;
    options.detection.sphere.distanceThreshold = .02;
    std::vector<reconstruction::PrimitiveEvidence> accepted;
    reconstruction::SurfaceSupportValidationReport report;
    std::string error;
    require(reconstruction::validateSurfaceSupport(source, {sphere}, options, accepted, report, error),
        "validate sphere with an observed opening");
    require(accepted.size() == 1 && report.decisions.size() == 1 && report.decisions.front().footprint,
        "accepted sphere carries its supported triangles");
    const auto& footprint = *report.decisions.front().footprint;
    auto topology = std::make_shared<SphereSourceTriangles>();
    for (const auto& triangle : footprint.triangles)
        topology->push_back({vtkIdType(footprint.sourcePointIds[triangle[0]] - 1),
            vtkIdType(footprint.sourcePointIds[triangle[1]] - 1), vtkIdType(footprint.sourcePointIds[triangle[2]] - 1)});
    DetectedPointCloudSurface annotation;
    annotation.surface.geometry = "sphere";
    annotation.surface.parameters = {{"center", QJsonArray{0, 0, 0}}, {"radius", radius}};
    annotation.surface.validatedSphereTriangles = topology;
    for (const auto id : footprint.coveredPointIds) annotation.points.push_back(vtkIdType(id - 1));

    PointCloudSelection selection;
    selection.setCloud(cloud);
    selection.addSurfaces({annotation});
    auto settings = options.supportMeshing;
    settings.curvedAlphaRadiusMultiplier = .1;
    const auto display = meshPointCloudSurfaces(selection, settings);
    const auto& mesh = display.meshes.at(1);
    require(mesh.triangles.size() == topology->size() && mesh.coveredPointIds == annotation.points,
        "display reuses the validated topology and coverage independently of curved Alpha");
    for (size_t i = 0; i < mesh.triangles.size(); ++i)
        for (size_t j = 0; j < 3; ++j)
            require(mesh.sourcePointIds[mesh.triangles[i][j]] == (*topology)[i][j], "display preserves source triangle vertices");
    require(covers(mesh, 2.6, -1) && !covers(mesh, 0, 0), "validated display preserves the sphere opening");
    for (const auto& p : mesh.positions)
        require(std::abs(std::hypot(p[0], p[1], p[2]) - radius) < 1e-10, "display vertices lie on the fitted sphere");

    PointCloudSelection reopened;
    reopened.setCloud(cloud);
    QString loadError;
    require(reopened.restoreAutomaticAnnotations(selection.automaticAnnotations(), loadError) &&
        reopened.region(1)->validatedSphereTriangles && *reopened.region(1)->validatedSphereTriangles == *topology,
        "saved annotations preserve the validated sphere topology");
    reopened.select({annotation.points.front()});
    reopened.deleteSelectedPoints();
    require(reopened.region(1)->fitStale && !reopened.region(1)->validatedSphereTriangles,
        "membership edits invalidate the topology");
    require(reopened.undo() && reopened.region(1)->validatedSphereTriangles,
        "undo restores the topology with its observations");
}

void relativeDensitySurfaces()
{
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    const auto build=[&](const PlyCloud& cloud) {
        std::vector<vtkIdType> ids;
        for(size_t i=0;i<cloud.positions.size();++i)ids.push_back(vtkIdType(i));
        return meshPointCloudSurface(region,cloud,ids);
    };
    // The same physical empty region is resolved on a dense scan. On a sparse
    // scan it spans too few expected observations to establish an opening.
    for(float spacing:{.5F,2.F}) {
        PlyCloud cloud;
        const int extent=int(10/spacing);
        for(int i=-extent;i<=extent;++i)for(int j=-extent;j<=extent;++j) {
            const float x=i*spacing,y=j*spacing;
            if(x*x+y*y<9)continue;
            cloud.positions.push_back({x,y,0});
        }
        const auto mesh=build(cloud);
        require(!mesh.triangles.empty(),"relative density produces a finite surface");
        require(covers(mesh,0,0)==(spacing==2),"hole resolution follows expected local observations, not a fixed world distance");
        if(spacing==.5F)require(std::any_of(mesh.domainSupportChecks.begin(),mesh.domainSupportChecks.end(),[](const auto& c){return c.holeBoundaryPoints>0;}),
            "inner-hole boundary observations are classified separately from the outer boundary");
    }
    PlyCloud mixed;
    for(int x=0;x<=40;++x)for(int y=0;y<=40;++y)mixed.positions.push_back({x*.5F,y*.5F,0});
    for(int x=22;x<=40;x+=2)for(int y=0;y<=20;y+=2)mixed.positions.push_back({float(x),float(y),0});
    const auto transition=build(mixed);
    std::cout<<"Density transition area: "<<meshArea(transition)<<" / 800, seam "<<covers(transition,21,10)<<'\n';
    require(meshArea(transition)>795 && covers(transition,21,10),"a sixteen-fold density transition stays connected");
    require(surfaceMeshComponentCount(transition)==1,"density transition produces one connected triangle patch");

    PlyCloud sparsePatch;
    for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y) {
        if(std::abs(x)<6 && std::abs(y)<6 && (x%2==0 || y%2==0))continue;
        sparsePatch.positions.push_back({float(x),float(y),0});
    }
    const auto patch=build(sparsePatch);
    require(meshArea(patch)>1590 && covers(patch,0,0),"a locally sparse observed patch is not mistaken for a hole in its dense host");

    for(float scale:{.1F,1.F,20.F}) {
        PlyCloud cloud;
        std::mt19937 random(4517);std::uniform_real_distribution<float> jitter(-.4F,.4F);
        for(int x=0;x<=40;++x)for(int y=0;y<=40;++y)
            cloud.positions.push_back({scale*(x+(x==0||x==40?0:jitter(random))),scale*(y+(y==0||y==40?0:jitter(random))),0});
        const auto mesh=build(cloud);
        require(meshArea(mesh)/(scale*scale)>1580 && covers(mesh,20*scale,20*scale),"uniformly sparse jittered scans retain coverage at every world scale");
        require(surfaceMeshComponentCount(mesh)==1,"jittered complete planes have no disconnected display fragments");
    }
    PlyCloud randomCloud;
    std::mt19937 random(4509);std::uniform_real_distribution<float> position(0,40);
    for(int i=0;i<2400;++i)randomCloud.positions.push_back({position(random),position(random),0});
    for(int i=0;i<=40;++i)for(auto p: {std::array<float,3>{float(i),0,0},{float(i),40,0},{0,float(i),0},{40,float(i),0}})
        randomCloud.positions.push_back(p);
    const auto randomMesh=build(randomCloud);
    std::cout<<"Random sampling area: "<<meshArea(randomMesh)<<" / 1600\n";
    require(meshArea(randomMesh)>1570,"random sampling gaps do not become a network of artificial holes");
    require(surfaceMeshComponentCount(randomMesh)==1,"random complete plane is rendered as one connected patch");

    for(double angle:{0.,.61}) {
        PlyCloud slot;
        for(int x=-20;x<=20;++x)for(int y=-20;y<=20;++y) {
            if(x==0 && std::abs(y)<8)continue;
            slot.positions.push_back({float(1000+x*std::cos(angle)-y*std::sin(angle)),
                float(-2000+x*std::sin(angle)+y*std::cos(angle)),0});
        }
        require(!covers(build(slot),1000,-2000),"stable narrow inner slots retain their observed boundaries under rotation and translation");
    }
    PlyCloud large;
    for(int x=0;x<=160;++x)for(int y=0;y<=160;++y) {
        if(std::abs(x-80)<=10 && std::abs(y-80)<=10)continue;
        large.positions.push_back({float(x),float(y),0});
    }
    const auto thinned=build(large);
    require(meshArea(thinned)>25000 && !covers(thinned,80,80) && covers(thinned,50,50),
        "interior mesh thinning preserves density-approved coverage and the measured hole");
    require(thinned.coveredPointIds.size()==large.positions.size() && surfaceMeshComponentCount(thinned)==1,
        "triangulation budget does not reintroduce fragments or lose covered observations");
}

void boundaryAndInteriorSupport()
{
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    const auto build=[&](const PlyCloud& cloud) {
        std::vector<vtkIdType> members;
        for(size_t i=0;i<cloud.positions.size();++i)members.push_back(vtkIdType(i));
        return meshPointCloudSurface(region,cloud,members);
    };
    PlyCloud solid;
    for(int x=0;x<=20;++x)for(int y=0;y<=20;++y)solid.positions.push_back({float(x),float(y),0});
    const auto regular=build(solid);
    require(std::abs(meshArea(regular)-400)<1e-8 && regular.domainSupportChecks.size()==1,
        "complete face retains one verified domain");
    const auto& check=regular.domainSupportChecks.front();
    require(check.observedPoints==441 && check.outerBoundaryPoints==80 && check.interiorPoints==361 && !check.rejected,
        "boundary diagnostics distinguish outer-rim observations from independent interior observations");

    for(int interior:{0,1,2,5}) {
        PlyCloud frame;
        for(int i=0;i<=40;++i) {
            frame.positions.push_back({float(i),0,0});frame.positions.push_back({float(i),40,0});
            if(i>0 && i<40){frame.positions.push_back({0,float(i),0});frame.positions.push_back({40,float(i),0});}
        }
        for(int i=0;i<interior;++i)for(int repeat=0;repeat<20;++repeat)
            frame.positions.push_back({float(13+i*3),float(17+(i%2)*5),0});
        const auto mesh=build(frame);
        require(!covers(mesh,20,20) && meshArea(mesh)<400,
            "a dense outline with only a few interior points cannot support a large filled face");
    }
    for(int outside:{1,2,5,10,20}) {
        PlyCloud line;
        for(int x=0;x<=200;++x)line.positions.push_back({float(x),float(.05*std::sin(x*.7)),0});
        for(int i=0;i<outside;++i)line.positions.push_back({float(200*(i+.5)/outside),float(40+5*std::sin(i)),0});
        const auto mesh=build(line);
        require(!covers(mesh,100,20),"a line plus several sparse off-line observations cannot stretch a face across empty interior");
    }
    for(float width:{.15F,1.F}) {
        PlyCloud strip;
        for(int x=0;x<=30;++x)for(float y:{0.F,width})strip.positions.push_back({float(x),y,0});
        const auto mesh=build(strip);
        require(std::abs(meshArea(mesh)-30*width)<1e-5 && !mesh.domainSupportChecks.empty(),
            "an unresolved narrow face with all points on its boundary remains valid");
        require(std::none_of(mesh.domainSupportChecks.begin(),mesh.domainSupportChecks.end(),[](const auto& c){return c.rejected;}),
            "all-boundary ratio alone cannot reject a narrow observed strip");
    }
}

void irregularSampling()
{
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    std::array<double,2> areas{};
    for(int paired=0;paired<2;++paired) {
        PlyCloud cloud;std::vector<vtkIdType> members;
        for(int x=-10;x<=10;++x)for(int y=-10;y<=10;++y) {
            if(std::abs(x)<4 && std::abs(y)<4)continue;
            members.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({float(x),float(y),0});
            if(paired){members.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({float(x+.00001),float(y+.00002),0});}
        }
        const auto mesh=meshPointCloudSurface(region,cloud,members);
        areas[size_t(paired)]=meshArea(mesh);
        require(areas[size_t(paired)]>320 && areas[size_t(paired)]<380,
            "sampling gaps are filled while a measured central opening is retained");
    }
    require(std::abs(areas[0]-areas[1])<1,"near-duplicate samples do not punch artificial holes into a plane");
    PlyCloud jittered;std::vector<vtkIdType> ids;
    std::mt19937 random(4500);std::uniform_real_distribution<float> jitter(-.4F,.4F);
    for(int x=0;x<=40;++x)for(int y=0;y<=40;++y) {
        ids.push_back(vtkIdType(jittered.positions.size()));
        jittered.positions.push_back({float(x)+(x==0||x==40?0:jitter(random)),
            float(y)+(y==0||y==40?0:jitter(random)),0});
    }
    const auto jitteredMesh=meshPointCloudSurface(region,jittered,ids);
    std::cout<<"Irregular plane area: "<<meshArea(jitteredMesh)<<" / 1600\n";
    require(meshArea(jitteredMesh)>1550,"ordinary scan spacing variation does not fragment a complete plane");
}

void fragmentedPlanes()
{
    PlyCloud cloud;std::vector<DetectedPointCloudSurface> surfaces(5);
    for(size_t i=0;i<surfaces.size();++i) {
        auto& region=surfaces[i].surface;region.geometry="plane";
        region.parameters={{"normal",QJsonArray{0,0,i==1?-1:1}},{"offset",i==4?-.15:0}};
    }
    const auto append=[&](int owner,float x,float y,float z) {
        surfaces[size_t(owner)].points.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({x,y,z});
    };
    // Interleaved patches cover one continuous plane, plus a separate island.
    for(int x=0;x<=30;++x)for(int y=0;y<=20;++y)append((x/2)%3,float(x),float(y),0);
    for(int x=60;x<=65;++x)for(int y=0;y<=5;++y)append(3,float(x),float(y),0);
    for(int x=0;x<=10;++x)for(int y=0;y<=10;++y)append(4,float(x),float(y),.15F);
    PointCloudSelection selection;selection.setCloud(cloud);selection.addSurfaces(surfaces);
    const auto revision=selection.revision();const auto memberships=selection.memberships();
    const auto display=meshPointCloudSurfaces(selection);
    require(display.meshes.size()==5,"shared meshing retains each collection as a display target");
    double area=0;
    for(int id=1;id<=4;++id) {
        require(display.colorRegions.at(id)==1,"fragments of the same plane share one display color");
        const auto& mesh=display.meshes.at(id);area+=meshArea(mesh);
        for(const auto& t:mesh.triangles) {
            double low=INFINITY,high=-INFINITY;
            for(auto v:t){low=std::min(low,mesh.positions[size_t(v)][0]);high=std::max(high,mesh.positions[size_t(v)][0]);}
            require(!(low<=30 && high>=60),"same-plane grouping keeps physical gaps between islands");
        }
    }
    require(std::abs(area-625)<1e-8,"shared plane meshing closes patch seams without filling the separate island's gap");
    require(display.colorRegions.at(5)==5,"a nearby parallel plane remains a separate surface");
    PointCloudActor actor;actor.setCloud(cloud);actor.setAnnotations(&selection);
    actor.setAutomaticSurfaceDisplay(false);actor.setAnnotations(&selection);
    auto* points=vtkPolyDataMapper::SafeDownCast(actor.actor()->GetMapper())->GetInput();
    const auto color=PointCloudActor::regionColor(1);
    for(auto id:surfaces[1].points)for(int axis=0;axis<3;++axis)
        require(points->GetPointData()->GetScalars()->GetComponent(id,axis)==color[size_t(axis)],"raw points retain the same color as their shared plane");
    actor.setAutomaticSurfaceDisplay(true);actor.setRegionSurfaceDisplay(2,false);actor.setAnnotations(&selection);
    require(!actor.displayedSurfaceRegions().count(2) && actor.displayedSurfaceRegions().size()==4,
        "each fragment can still independently switch back to its original points");
    require(points->GetNumberOfVerts()==vtkIdType(surfaces[1].points.size()),"switching one fragment only restores its own source points");
    require(selection.revision()==revision && selection.memberships()==memberships,"shared display meshing never merges semantic point memberships");
}

void unsupportedPlaneSamples()
{
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    for(int count:{3,4,16,64}) {
        PlyCloud cloud;std::vector<vtkIdType> members;
        for(int i=0;i<count;++i){members.push_back(i);cloud.positions.push_back({float(i),0,0});}
        members.push_back(count);cloud.positions.push_back({1000,1000,0});
        require(meshPointCloudSurface(region,cloud,members).triangles.empty(),
            "a line plus a remote isolated point has no observed two-dimensional surface");
        PointCloudSelection selection;selection.setCloud(cloud);selection.addSurfaces({{region,members}});
        PointCloudActor actor;actor.setCloud(cloud);actor.setAnnotations(&selection);
        auto* data=vtkPolyDataMapper::SafeDownCast(actor.actor()->GetMapper())->GetInput();
        require(actor.displayedSurfaceRegions().empty() && data->GetNumberOfVerts()==vtkIdType(cloud.positions.size()),
            "insufficient surface evidence stays visible as original points");
    }
    PlyCloud cloud;std::vector<vtkIdType> members;
    for(int x=0;x<=8;++x)for(int y=0;y<=8;++y){members.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({float(x),float(y),0});}
    const auto facePoints=cloud.positions.size();
    for(int x=20;x<=40;++x){members.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({float(x),0,0});}
    members.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({1000,1000,0});
    const auto mesh=meshPointCloudSurface(region,cloud,members);
    require(std::abs(meshArea(mesh)-64)<1e-8,"a distant line cannot enlarge the finite domain of a complete face");
    for(const auto& t:mesh.triangles)for(auto id:t)
        require(mesh.positions[size_t(id)][0]<=8 && mesh.positions[size_t(id)][1]<=8,"no triangle reaches unsupported remote samples");
    PointCloudSelection selection;selection.setCloud(cloud);selection.addSurfaces({{region,members}});
    PointCloudActor actor;actor.setCloud(cloud);actor.setAnnotations(&selection);
    auto* data=vtkPolyDataMapper::SafeDownCast(actor.actor()->GetMapper())->GetInput();
    require(data->GetNumberOfVerts()==vtkIdType(cloud.positions.size()-facePoints),
        "unsupported points in a partly rendered collection remain visible");
}

void narrowAndUnevenPlanes()
{
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    PlyCloud strip;std::vector<vtkIdType> ids;
    for(int x=0;x<=30;++x)for(float y:{0.F,.15F}){ids.push_back(vtkIdType(strip.positions.size()));strip.positions.push_back({float(x),y,0});}
    require(std::abs(meshArea(meshPointCloudSurface(region,strip,ids))-4.5)<1e-5,
        "a narrow face with two observed sample rows is retained");
    PlyCloud uneven;ids.clear();
    for(int x=0;x<=20;++x)for(int y=0;y<=20;++y){ids.push_back(vtkIdType(uneven.positions.size()));uneven.positions.push_back({x*.5F,y*.5F,0});}
    for(int x=11;x<=20;++x)for(int y=0;y<=10;++y){ids.push_back(vtkIdType(uneven.positions.size()));uneven.positions.push_back({float(x),float(y),0});}
    require(meshArea(meshPointCloudSurface(region,uneven,ids))>=199.9,
        "a continuous plane with different sampling densities has no artificial internal gap");
}

void finitePlaneBoundaries()
{
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    for(bool openNotch:{false,true}) {
        PlyCloud cloud;std::vector<vtkIdType> ids;
        for(int x=0;x<=12;++x)for(int y=0;y<=12;++y) {
            if(openNotch ? (x==6 && y>=4) : (x==6 && y==6))continue;
            ids.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({float(x),float(y),0});
        }
        const auto mesh=meshPointCloudSurface(region,cloud,ids);
        require(!mesh.triangles.empty(),"finite face boundary is recovered");
        if(!openNotch)require(std::abs(meshArea(mesh)-144)<1e-8,
            "one missing interior sample is repaired without punching a display hole");
        else for(const auto& t:mesh.triangles) {
            double x=0,y=0;
            for(auto id:t){x+=mesh.positions[size_t(id)][0]/3;y+=mesh.positions[size_t(id)][1]/3;}
            require(!(x>5 && x<7 && y>6),"an exterior-connected narrow notch is never repaired as an interior sampling cavity");
        }
    }
}

void distantCoplanarIslands()
{
    PointCloudRegion region;region.geometry="plane";
    region.parameters={{"normal",QJsonArray{0,0,1}},{"offset",0}};
    PlyCloud cloud;std::vector<vtkIdType> ids;
    for(float origin:{0.F,1000000.F})for(int x=0;x<=8;++x)for(int y=0;y<=8;++y) {
        ids.push_back(vtkIdType(cloud.positions.size()));cloud.positions.push_back({origin+x,float(y),0});
    }
    const auto mesh=meshPointCloudSurface(region,cloud,ids);
    require(std::abs(meshArea(mesh)-128)<1e-8,"large separation never changes the sampling scale or erases either finite patch");
    for(const auto& t:mesh.triangles) {
        double low=INFINITY,high=-INFINITY;
        for(auto id:t){low=std::min(low,mesh.positions[size_t(id)][0]);high=std::max(high,mesh.positions[size_t(id)][0]);}
        require(high-low<=8,"distant coplanar islands remain separately bounded domains");
    }
}

void traceSurface(const QString& excerptPath,const QString& reportPath)
{
    QFile input(excerptPath);require(input.open(QIODevice::ReadOnly),"open extracted surface evidence");
    const auto root=QJsonDocument::fromJson(input.readAll()).object();
    const auto inputPoints=root["points"].toArray();
    const int targetId=root["target"].toObject()["id"].toInt();
    PlyCloud cloud;std::vector<vtkIdType> targetMembers;
    std::map<int,std::vector<vtkIdType>> members;
    std::vector<qint64> sourceRows;
    for(const auto& value:inputPoints) {
        const auto point=value.toObject();const auto p=point["position"].toArray();
        const auto id=vtkIdType(cloud.positions.size());
        cloud.positions.push_back({float(p[0].toDouble()),float(p[1].toDouble()),float(p[2].toDouble())});
        sourceRows.push_back(point["source_index"].toInteger());
        const int owner=point["owner"].toInt();members[owner].push_back(id);
        if(owner==targetId)targetMembers.push_back(id);
    }
    PointCloudRegion target;
    std::vector<DetectedPointCloudSurface> family;
    for(const auto& value:root["family"].toArray()) {
        const auto json=value.toObject();PointCloudRegion region;
        region.geometry=json["geometry"].toString();region.parameters=json["parameters"].toObject();
        region.name=json["name"].toString();region.residual=json["residual"].toDouble();
        region.confidence=json["confidence"].toDouble();region.fitStale=json["fitStale"].toBool();
        const int owner=json["id"].toInt();
        if(owner==targetId)target=region;
        family.push_back({region,members.at(owner)});
    }
    const auto describe=[&](const PointCloudSurfaceMesh& mesh) {
        double maximumArea=0,maximumEdge=0,foreignArea=0;int foreignTriangles=0;
        QJsonArray triangles;
        for(const auto& triangle:mesh.triangles) {
            PointCloudSurfaceMesh single;single.positions=mesh.positions;single.triangles.push_back(triangle);
            const double area=meshArea(single);maximumArea=std::max(maximumArea,area);
            bool foreign=false;QJsonArray rows;
            for(size_t i=0;i<3;++i) {
                const auto vertex=triangle[i];const auto next=triangle[(i+1)%3];
                const auto source=mesh.sourcePointIds[size_t(vertex)];
                foreign=foreign || std::find(targetMembers.begin(),targetMembers.end(),source)==targetMembers.end();
                rows.append(sourceRows[size_t(source)]);
                double squared=0;for(size_t axis=0;axis<3;++axis)
                    squared+=std::pow(mesh.positions[size_t(vertex)][axis]-mesh.positions[size_t(next)][axis],2);
                maximumEdge=std::max(maximumEdge,std::sqrt(squared));
            }
            if(foreign){++foreignTriangles;foreignArea+=area;}
            triangles.append(QJsonObject{{"source_rows",rows},{"area",area},{"uses_other_collection",foreign}});
        }
        QJsonArray supportChecks;
        for(const auto& check:mesh.domainSupportChecks) supportChecks.append(QJsonObject{
            {"observed_points",qint64(check.observedPoints)},{"outer_boundary_points",qint64(check.outerBoundaryPoints)},
            {"hole_boundary_points",qint64(check.holeBoundaryPoints)},{"interior_points",qint64(check.interiorPoints)},
            {"interior_cells",qint64(check.occupiedInteriorCells)},
            {"outer_boundary_fraction",double(check.outerBoundaryPoints)/check.observedPoints},
            {"boundary_band_width",check.boundaryBandWidth},{"candidate_area",check.area},
            {"interior_area_measured",check.interiorAreaMeasured},{"interior_area",check.interiorArea},
            {"expected_interior_points",check.expectedInteriorPoints},{"rejected",check.rejected}});
        return QJsonObject{{"triangles",triangles},{"triangle_count",int(mesh.triangles.size())},
            {"domain_support_checks",supportChecks},
            {"area",meshArea(mesh)},{"maximum_triangle_area",maximumArea},{"maximum_edge",maximumEdge},
            {"other_collection_triangles",foreignTriangles},{"other_collection_triangle_area",foreignArea},
            {"covered_points",int(mesh.coveredPointIds.size())},{"components",int(surfaceMeshComponentCount(mesh))}};
    };
    const auto targetOnly=meshPointCloudSurface(target,cloud,targetMembers);
    PointCloudSelection selection;selection.setCloud(cloud);selection.addSurfaces(family);
    const auto display=meshPointCloudSurfaces(selection);
    QJsonObject combined;
    for(const auto& entry:display.meshes)combined[QString::number(entry.first)]=describe(entry.second);
    const QJsonObject report{{"source_excerpt",excerptPath},{"target_point_count",int(targetMembers.size())},
        {"family_point_count",int(cloud.positions.size())},{"target_only",describe(targetOnly)},{"display_collections",combined}};
    QFile output(reportPath);require(output.open(QIODevice::WriteOnly),"write surface trace report");
    output.write(QJsonDocument(report).toJson());
    std::cout<<QJsonDocument(report).toJson(QJsonDocument::Compact).constData()<<'\n';
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        if(argc==4 && QString::fromLocal8Bit(argv[1])=="--trace-surface") {
            traceSurface(QString::fromLocal8Bit(argv[2]),QString::fromLocal8Bit(argv[3]));return 0;
        }
        unsupportedPlaneSamples();surfaceDisplay();vertexTouchingBoundaries();finalSurfaceMinimum();wideConeSupport();curvedSurfaces();validatedSphereDisplay();irregularSampling();fragmentedPlanes();narrowAndUnevenPlanes();finitePlaneBoundaries();distantCoplanarIslands();relativeDensitySurfaces();boundaryAndInteriorSupport();
        PlyCloud cloud;cloud.positions={{0,0,0},{1,0,0},{2,0,0}};
        cloud.colors={{12,34,56},{78,90,12},{34,56,78}};
        PointCloudSelection selection;selection.setCloud(cloud);
        PointCloudActor actor;require(actor.setCloud(cloud),"set cloud");actor.setAnnotations(&selection);
        auto* base=vtkPolyDataMapper::SafeDownCast(actor.actor()->GetMapper())->GetInput();
        auto* overlay=vtkPolyDataMapper::SafeDownCast(actor.selectionActor()->GetMapper())->GetInput();
        auto* colors=base->GetPointData()->GetScalars();const auto initial=colors->GetMTime();
        selection.select({1});actor.setAnnotations(&selection);
        require(colors->GetMTime()==initial,"selecting a point must not rewrite all base colors");
        require(overlay->GetNumberOfPoints()==1 && actor.selectionActor()->GetVisibility(),"selection overlay must show one point");
        double point[3];overlay->GetPoint(0,point);require(point[0]==1,"overlay must use the selected source position");
        require(base->GetNumberOfPoints()==3 && !actor.selectionActor()->GetPickable(),"source picking IDs must stay stable");
        selection.select({2});actor.setAnnotations(&selection);overlay->GetPoint(0,point);
        require(point[0]==2 && colors->GetMTime()==initial,"moving selection updates only the overlay");
        require(selection.assign("plane","wall")>0,"assign region");actor.setAnnotations(&selection);
        require(colors->GetMTime()>initial,"membership edit refreshes base colors");
        actor.setAnnotations(nullptr);require(!actor.selectionActor()->GetVisibility(),"capture hides selection overlay");
        require(colors->GetComponent(2,0)==34 && colors->GetComponent(2,1)==56,"capture restores source colors");
        actor.setAnnotations(&selection);actor.setVisible(false);
        require(!actor.selectionActor()->GetVisibility(),"hidden cloud hides selection");
        actor.setVisible(true);require(actor.selectionActor()->GetVisibility(),"selection visibility restored");
        actor.clear();actor.setVisible(true);require(!actor.selectionActor()->GetVisibility(),"clear removes stale selection");
        require(actor.setCloud(cloud),"reload cloud");actor.setVisible(true);
        require(!actor.selectionActor()->GetVisibility(),"loading a new cloud never shows old selected points");
        std::cout<<"Point cloud rendering cache test passed\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
