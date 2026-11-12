#include "capture/QualityEvaluation.h"
#include <QElapsedTimer>
#include <vtkCellArray.h>
#include <vtkGenericCell.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStaticCellLocator.h>
#include <vtkStaticPointLocator.h>
#include <algorithm>
#include <cmath>
#include <random>

namespace capture {
namespace {
Matrix4 readMatrix(const QJsonArray& rows) {
    Matrix4 m{};for(int i=0;i<4;++i)for(int j=0;j<4;++j)m[i*4+j]=rows[i].toArray()[j].toDouble();return m;
}
struct View {Device device;Matrix4 world;};
View view(const QJsonObject& definition,const Matrix4& rig) {
    View v;v.device.width=definition["width"].toInt();v.device.height=definition["height"].toInt();
    const auto k=definition["K"].toArray();
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)v.device.K[i*3+j]=k[i].toArray()[j].toDouble();
    v.world=multiply(rig,readMatrix(definition["device_to_rig"].toArray()));return v;
}
bool visible(const std::array<double,3>& p,const View& view,vtkStaticCellLocator* locator,double epsilon) {
    const auto& m=view.world;double local[3]{};
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)local[i]+=m[j*4+i]*(p[j]-m[j*4+3]);
    if(local[2]<=0)return false;
    const double u=view.device.K[0]*local[0]/local[2]+view.device.K[2];
    const double v=view.device.K[4]*local[1]/local[2]+view.device.K[5];
    if(u<0 || u>=view.device.width || v<0 || v>=view.device.height)return false;
    const double start[3]{m[3],m[7],m[11]};double end[3];
    for(int i=0;i<3;++i)end[i]=p[i]+(p[i]-start[i])*1e-5;
    double t,x[3],pc[3];int sub=0;vtkIdType cell;
    if(!locator->IntersectWithLine(start,end,epsilon,t,x,pc,sub,cell))return false;
    return std::hypot(x[0]-p[0],x[1]-p[1],x[2]-p[2])<epsilon*10;
}
}

QJsonObject evaluateSurface(const Geometry& geometry,const std::vector<std::array<double,3>>& points,
    const QJsonObject& acquisition,const QJsonObject& calibration,double tolerance,int samples) {
    QElapsedTimer timer;timer.start();
    if(points.empty() || geometry.triangles.empty() || tolerance<=0 || samples<=0)return {{"status","invalid_input"}};
    vtkNew<vtkPoints> vertices;vertices->SetDataTypeToDouble();
    for(const auto& p:geometry.positions)vertices->InsertNextPoint(p.data());
    vtkNew<vtkCellArray> triangles;std::vector<double> area;
    for(const auto& face:geometry.triangles) {
        const vtkIdType ids[3]{face[0],face[1],face[2]};triangles->InsertNextCell(3,ids);
        const auto& a=geometry.positions[face[0]];const auto& b=geometry.positions[face[1]];const auto& c=geometry.positions[face[2]];
        const double u[3]{b[0]-a[0],b[1]-a[1],b[2]-a[2]},v[3]{c[0]-a[0],c[1]-a[1],c[2]-a[2]};
        area.push_back(.5*std::hypot(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]));
    }
    vtkNew<vtkPolyData> surface;surface->SetPoints(vertices);surface->SetPolys(triangles);
    vtkNew<vtkStaticCellLocator> surfaceLocator;surfaceLocator->SetDataSet(surface);surfaceLocator->BuildLocator();
    vtkNew<vtkGenericCell> genericCell;
    std::vector<double> errors;errors.reserve(points.size());int outliers=0;
    vtkNew<vtkPoints> observations;observations->SetDataTypeToDouble();
    for(const auto& p:points) {
        double closest[3],distance2;int sub;vtkIdType cell;
        surfaceLocator->FindClosestPoint(p.data(),closest,genericCell,cell,sub,distance2);
        const double distance=std::sqrt(distance2);errors.push_back(distance);if(distance>tolerance)++outliers;
        observations->InsertNextPoint(p.data());
    }
    vtkNew<vtkPolyData> cloud;cloud->SetPoints(observations);
    vtkNew<vtkStaticPointLocator> pointLocator;pointLocator->SetDataSet(cloud);pointLocator->BuildLocator();
    std::vector<std::pair<View,View>> views;
    const auto devices=calibration["devices"].toObject();
    for(const auto& value:acquisition["stations"].toArray()) {
        const auto station=value.toObject();const auto rig=readMatrix(station["rig_to_world"].toArray());
        views.emplace_back(view(devices[station["camera_id"].toString()].toObject(),rig),
                           view(devices[station["projector_id"].toString()].toObject(),rig));
    }
    double bounds[6];surface->GetBounds(bounds);
    const double epsilon=std::max(1e-6,std::hypot(bounds[1]-bounds[0],bounds[3]-bounds[2],bounds[5]-bounds[4])*1e-7);
    std::mt19937 random(617);std::discrete_distribution<std::size_t> choose(area.begin(),area.end());
    std::uniform_real_distribution<double> unit(0,1);
    int cameraVisible=0,jointVisible=0,covered=0,jointCovered=0;
    for(int i=0;i<samples;++i) {
        const auto& face=geometry.triangles[choose(random)];
        const double u=std::sqrt(unit(random)),v=unit(random);std::array<double,3> p;
        for(int j=0;j<3;++j)p[j]=(1-u)*geometry.positions[face[0]][j]+u*(1-v)*geometry.positions[face[1]][j]+u*v*geometry.positions[face[2]][j];
        bool camera=false,joint=false;
        for(const auto& pair:views) {
            const bool seen=visible(p,pair.first,surfaceLocator,epsilon);camera|=seen;
            if(seen && visible(p,pair.second,surfaceLocator,epsilon)){joint=true;break;}
        }
        if(camera)++cameraVisible;if(joint)++jointVisible;
        const vtkIdType nearest=pointLocator->FindClosestPoint(p.data());
        const auto& q=points[static_cast<std::size_t>(nearest)];
        const bool present=std::hypot(p[0]-q[0],p[1]-q[1],p[2]-q[2])<=tolerance;
        if(present)++covered;if(present && joint)++jointCovered;
    }
    std::sort(errors.begin(),errors.end());
    return {{"schema","paramcad.structured-light.evaluation"},{"version",1},{"status","complete"},
        {"unit","mm"},{"tolerance_mm",tolerance},{"reference_samples",samples},{"sampling_seed",617},
        {"sampling","triangle-area-weighted; reference is rendered tessellation, not exact B-Rep"},
        {"point_count",static_cast<qint64>(points.size())},{"point_error_p50_mm",errors[errors.size()/2]},
        {"point_error_p95_mm",errors[static_cast<std::size_t>(.95*(errors.size()-1))]},
        {"outlier_fraction",static_cast<double>(outliers)/points.size()},
        {"full_surface_coverage",static_cast<double>(covered)/samples},
        {"camera_visible_fraction",static_cast<double>(cameraVisible)/samples},
        {"joint_geometric_visible_fraction",static_cast<double>(jointVisible)/samples},
        {"joint_visible_coverage",jointVisible?QJsonValue(static_cast<double>(jointCovered)/jointVisible):QJsonValue()},
        {"elapsed_ms",timer.elapsed()},{"reference_used_for_reconstruction",false}};
}
} // namespace capture
