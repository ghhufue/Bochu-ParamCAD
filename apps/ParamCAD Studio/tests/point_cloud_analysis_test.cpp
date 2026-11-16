#include "io/PointCloudAnalysis.h"
#include "io/PlyReader.h"

#include <QCoreApplication>
#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>
#include <limits>
#include <reconstruction/settings/ModuleConfig.h>

namespace {
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
}
int main(int argc, char** argv)
{
    QCoreApplication application(argc,argv);
    try {
        PlyCloud largePlane;
        std::vector<vtkIdType> planeIds;
        for(int x=0;x<120;++x) for(int y=0;y<60;++y) {
            largePlane.positions.push_back({float(x*10),float(y*10),0});
            largePlane.normals.push_back(x>40 && x<80?std::array<float,3>{0,1,0}:
                (x>90?std::array<float,3>{0,0,0}:std::array<float,3>{0,0,1}));
            const auto row=largePlane.positions.size()*3+20;
            largePlane.sourceRows.push_back(row);planeIds.push_back(vtkIdType(row));
        }
        const auto wholePlane=analysePointCloud(largePlane,planeIds,.05,"plane",reconstruction::LengthUnit::Millimeter);
        require(wholePlane.error.isEmpty() && wholePlane.surfaces.size()==1
            && wholePlane.assignedPoints==largePlane.positions.size(),
            "grown plane ownership is not clipped by the old display halo or re-filtered by inconsistent source normals");
        require(wholePlane.surfaces[0].points.front()==planeIds.front() && wholePlane.surfaces[0].points.back()==planeIds.back(),
            "large-plane expansion retains sparse original document row IDs");
        PlyCloud cloud;
        for (int axis=0; axis<3; ++axis) for (float sign : {-1.0F,1.0F}) {
            for (int i=0;i<20;++i) for (int j=0;j<20;++j) {
                std::array<float,3> p{}, n{};
                p[static_cast<size_t>(axis)]=sign*3; n[static_cast<size_t>(axis)]=sign;
                p[static_cast<size_t>((axis+1)%3)]=-3+(i+0.5F)*0.3F;
                p[static_cast<size_t>((axis+2)%3)]=-3+(j+0.5F)*0.3F;
                cloud.positions.push_back(p); cloud.normals.push_back(n);
            }
        }
        std::vector<vtkIdType> ids;
        for (size_t i=0;i<cloud.positions.size();++i) ids.push_back(static_cast<vtkIdType>(i*2+7));
        auto result=analysePointCloud(cloud,ids,0.01);
        require(result.error.isEmpty() && result.surfaces.size()==6, "analytic box exposes six fitted faces");
        require(result.assignedPoints>=cloud.positions.size()*0.99, "face support maps to original observations");
        std::set<vtkIdType> valid(ids.begin(),ids.end()), assigned;
        for(const auto& surface:result.surfaces) for(auto id:surface.points) {
            require(valid.count(id) && assigned.insert(id).second, "stable source IDs are mapped once");
        }
        PlyCloud cone;std::vector<vtkIdType> coneIds;const double coneAngle=.4;
        for(int i=0;i<48;++i)for(int j=0;j<20;++j) {
            const double theta=2*std::acos(-1.)*i/48,z=5+j*.25,r=z*std::tan(coneAngle);
            coneIds.push_back(vtkIdType(cone.positions.size()));
            cone.positions.push_back({float(r*std::cos(theta)),float(r*std::sin(theta)),float(z)});
            cone.normals.push_back({float(std::cos(coneAngle)*std::cos(theta)),
                float(std::cos(coneAngle)*std::sin(theta)),float(-std::sin(coneAngle))});
        }
        const auto coneResult=analysePointCloud(cone,coneIds,.01,"cone");
        require(coneResult.error.isEmpty() && !coneResult.surfaces.empty(),"normal cone is detected with automatic support settings");
        reconstruction::settings::addOverride("detection.cone_minimum_support_points=2000");
        const auto restrictedCone=analysePointCloud(cone,coneIds,.01,"cone");
        require(restrictedCone.surfaces.empty(),"explicit cone minimum must survive client analysis option construction");
        reconstruction::settings::clearOverrides();
        auto invalid=cloud;
        invalid.positions.insert(invalid.positions.begin(),{std::numeric_limits<float>::quiet_NaN(),0,0});
        invalid.normals.insert(invalid.normals.begin(),{0,0,1});
        std::vector<vtkIdType> original;
        for (std::size_t i = 0; i < invalid.positions.size(); ++i) {
            const auto sourceRow = 2 * i + 500;
            invalid.sourceRows.push_back(sourceRow);
            original.push_back(static_cast<vtkIdType>(sourceRow));
        }
        const auto finiteResult=analysePointCloud(invalid,original,0.01);
        require(finiteResult.error.isEmpty() && finiteResult.assignedPoints==cloud.positions.size(),
                "invalid source rows do not shift support IDs");
        for (const auto& surface : finiteResult.surfaces)
            for (const auto id : surface.points)
                require(id > 500 && id % 2 == 0, "memberships use surviving original rows");
        for (const auto& patch : finiteResult.analysis.patches)
            for (const auto id : patch.supportPointIds)
                require(id > 501 && id % 2 == 1, "evidence keeps sparse one-based source IDs");
        PlyCloud dense;
        for(int repeat=0;repeat<24;++repeat) {
            dense.positions.insert(dense.positions.end(),cloud.positions.begin(),cloud.positions.end());
            dense.normals.insert(dense.normals.end(),cloud.normals.begin(),cloud.normals.end());
        }
        ids.clear(); for(size_t i=0;i<dense.positions.size();++i) ids.push_back(static_cast<vtkIdType>(i+100));
        result=analysePointCloud(dense,ids,0.01);
        require(result.error.isEmpty() && result.processedPoints<dense.positions.size()
            && result.assignedPoints>=dense.positions.size()*0.99, "voxel downsampling does not lose original point memberships");
        PlyCloud mixedScale;
        for(int x=0;x<250;++x)for(int y=0;y<220;++y) {
            mixedScale.positions.push_back({float(x*20),float(y*20),0});mixedScale.normals.push_back({0,0,1});
        }
        const auto smallStart=mixedScale.positions.size();
        for(int ring=0;ring<6;++ring)for(int angle=0;angle<96;++angle) {
            const double t=2*std::acos(-1.0)*angle/96;
            mixedScale.positions.push_back({float(100+2*std::cos(t)),float(100+2*std::sin(t)),float(20+ring)});
            mixedScale.normals.push_back({float(std::cos(t)),float(std::sin(t)),0});
        }
        ids.resize(mixedScale.positions.size());for(size_t i=0;i<ids.size();++i)ids[i]=vtkIdType(i);
        result=analysePointCloud(mixedScale,ids,0.02);
        require(result.error.isEmpty(),"mixed-scale scene analysis succeeds");
        size_t smallCylinder=0;
        for(const auto& surface:result.surfaces)if(surface.surface.geometry=="cylinder")
            for(auto id:surface.points)smallCylinder+=size_t(id)>=smallStart;
        require(smallCylinder>=(mixedScale.positions.size()-smallStart)*0.98,
            "large scene diagonal must not erase or exclude a radius-2 cylinder");
        if(argc>1) {
            PlyCloud input; QString error;
            require(PlyReader::read(QString::fromLocal8Bit(argv[1]),input,error), "load sample cloud");
            ids.clear(); for(size_t i=0;i<input.positions.size();++i) ids.push_back(static_cast<vtkIdType>(i));
            result=analysePointCloud(input,ids,0.08);
            require(result.error.isEmpty() && !result.surfaces.empty(), "sample cloud surfaces detected");
            std::cout << "Sample: " << result.surfaces.size() << " surfaces, " << result.assignedPoints << "/" << input.positions.size() << " points.\n";
        }
        reconstruction::TemplateFit fit;
        reconstruction::PlaneEvidence plane;
        plane.plane.normal = {0, 0, 1}; plane.plane.offset = -3000;
        plane.meanAbsoluteResidual = 2;
        fit.surface = plane;
        auto annotation = describeTemplateFit(fit, 1000);
        reconstruction::SurfaceAnalysisResult retained;
        plane.id=1;retained.surfaces.push_back(plane);
        reconstruction::SurfacePatch patch;patch.id=1;patch.surfaceId=1;patch.supportPointIds={1,3,999};
        retained.patches.push_back(patch);
        const auto displayed=describeAnalysisPatches(retained,10,1000);
        require(displayed.empty(),"failed CAD analysis must not publish a subminimum two-point surface");
        retained.patches[0].supportPointIds={1,2,3,4,5,6,7,8,9,10,11,12,999};
        const auto supported=describeAnalysisPatches(retained,20,1000);
        require(supported.size()==1 && supported[0].points.front()==0 && supported[0].points.back()==11
            && supported[0].surface.geometry=="plane" && supported[0].surface.parameters["offset"].toDouble()==-3
            && !supported[0].surface.templateLocked,"supported CAD analysis preserves stable rows and source units");
        reconstruction::settings::addOverride("analysis.minimum_surface_points=13");
        require(describeAnalysisPatches(retained,20,1000).empty(),"final CAD patch publication honours the configured minimum");
        reconstruction::settings::clearOverrides();
        require(annotation.parameters["offset"].toDouble() == -3 && annotation.residual == 0.002,
                "template plane equations and residuals return to source meter units");
        fit.surface.reset(); fit.box = reconstruction::BoxCandidate{};
        fit.box->primitive.pose.origin = {1000, 2000, 3000};
        fit.box->primitive.sizeX = 4000; fit.box->primitive.sizeY = 5000; fit.box->primitive.sizeZ = 6000;
        annotation = describeTemplateFit(fit, 1000);
        require(annotation.geometry == "cuboid" && annotation.parameters["size_x"].toDouble() == 4
                && annotation.parameters["size_z"].toDouble() == 6,
                "cuboid template dimensions return to source meter units");
        std::cout<<"Point cloud analysis tests passed.\n"; return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
