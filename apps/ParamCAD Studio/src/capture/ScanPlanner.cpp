#include "capture/ScanPlanner.h"
#include <QFileInfo>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace capture {
namespace {
using V=std::array<double,3>;
V normalized(V a) {const double n=std::hypot(a[0],a[1],a[2]);if(n<1e-12)throw std::invalid_argument("Invalid view direction");for(auto& v:a)v/=n;return a;}
V cross(V a,V b) {return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
}
Matrix4 lookAt(const V& position,const V& target,const V& up) {
    const auto f=normalized({target[0]-position[0],target[1]-position[1],target[2]-position[2]});
    const auto r=normalized(cross(f,up));const auto d=cross(f,r);
    return {r[0],d[0],f[0],position[0],r[1],d[1],f[1],position[1],
            r[2],d[2],f[2],position[2],0,0,0,1};
}

RenderJob planScan(const Geometry& geometry,const ScanOptions& options,
                   const QString& directory,const Matrix4* singleCameraPose) {
    if(geometry.positions.empty() || options.stationCount<1 || options.stationCount>256
        || !std::isfinite(options.lightIntensity) || options.lightIntensity<=0
        || options.lightIntensity>1000
        || !std::isfinite(options.fieldOfViewDegrees) || options.fieldOfViewDegrees<10 || options.fieldOfViewDegrees>100)
        throw std::invalid_argument("Invalid geometry, station count or field of view");
    V low=geometry.positions.front(),high=low;
    for(const auto& p:geometry.positions)for(int i=0;i<3;++i){low[i]=std::min(low[i],p[i]);high[i]=std::max(high[i],p[i]);}
    const V center{(low[0]+high[0])/2,(low[1]+high[1])/2,(low[2]+high[2])/2};
    const double diagonal=std::hypot(high[0]-low[0],high[1]-low[1],high[2]-low[2]);
    if(diagonal<1e-8)throw std::invalid_argument("Scene is degenerate");
    const double radians=options.fieldOfViewDegrees*3.14159265358979323846/180;
    const double distance=options.distance>0?options.distance:diagonal/(1.4*std::tan(radians/2));
    const double baseline=options.baseline>0?options.baseline:.1*diagonal;
    RenderJob job;job.geometry=geometry;job.material=options.material;
    job.directory=directory;job.id=QFileInfo(directory).fileName();
    job.camera.width=options.cameraWidth;job.camera.height=options.cameraHeight;
    const double cf=options.cameraHeight/(2*std::tan(radians/2));
    job.camera.K={cf,0,options.cameraWidth/2.0,0,cf,options.cameraHeight/2.0,0,0,1};
    job.projector.width=options.projectorWidth;job.projector.height=options.projectorHeight;
    const double pf=options.projectorHeight/(2*std::tan(radians*.65));
    job.projector.K={pf,0,options.projectorWidth/2.0,0,pf,options.projectorHeight/2.0,0,0,1};
    job.projector.deviceToRig=lookAt({baseline,0,0},{0,0,distance},{0,-1,0});
    job.phaseSteps=options.phaseShift?4:0;job.patterns=grayPatterns(options.projectorWidth,options.projectorHeight,job.phaseSteps);
    if(options.multiExposure)job.exposures={.25,1.0,4.0};
    job.referenceDistance=distance;job.nearPlane=std::max(.01,distance*.05);
    job.farPlane=distance+diagonal*3;job.lightIntensity=options.lightIntensity;
    const int count=singleCameraPose?1:options.stationCount;
    for(int i=0;i<count;++i) {
        Matrix4 pose;
        if(singleCameraPose)pose=*singleCameraPose;
        else {
            const double z=1-2*(i+.5)/count,phi=i*2.399963229728653;
            const double radius=std::sqrt(1-z*z);
            const V position{center[0]+distance*radius*std::cos(phi),
                             center[1]+distance*radius*std::sin(phi),center[2]+distance*z};
            pose=lookAt(position,center,{0,0,1});
        }
        job.stations.push_back({QString("s%1").arg(i,3,10,QLatin1Char('0')),pose});
    }
    const QString error=job.validate();
    if(!error.isEmpty())throw std::invalid_argument(error.toStdString());
    return job;
}
} // namespace capture
