#include "reconstruction/recognition/RecognitionAnalysisOptions.h"
#include "SurfaceCoverageValidation.h"
#include "io/PlyReader.h"
#include "io/PointCloudAdapter.h"
#include "io/PointCloudAnalysis.h"
#include <reconstruction/surface/SurfaceAnalysis.h>
#include <reconstruction/preprocessing/PointCloudPreprocessor.h>
#include <QElapsedTimer>
#include <QJsonArray>
#include <unordered_set>
#include <algorithm>
#include <numeric>
#include <stdexcept>

void validatePlaneGrowth(const QStringList& args,QJsonObject& report) {
    if(args.size()!=4) throw std::runtime_error("Usage: analyze-plane-growth cloud.ply report.json");
    PlyCloud cloud;QString failure;
    if(!PlyReader::read(args[2],cloud,failure)) throw std::runtime_error(failure.toStdString());
    const auto unit=PointCloudAdapter::lengthUnitFor(args[2]);
    if(unit!=reconstruction::LengthUnit::Millimeter) throw std::runtime_error("Plane growth validation requires millimetre metadata");
    const auto source=PointCloudAdapter::toPointStore(cloud,unit);
    reconstruction::PointStore points({},unit);reconstruction::PointCloudPreprocessingReport pre;std::string error;
    if(!reconstruction::preprocessPointCloud(source,PointCloudAdapter::preprocessingOptionsFor(cloud),points,pre,error))
        throw std::runtime_error(error);
    const auto planes=PointCloudAdapter::planeOptionsFor(points);
    auto options=reconstruction::recognitionAnalysisOptions(points,planes,PointCloudAdapter::cylinderOptionsFor(points));
    options.detection.detectCylinders=options.detection.detectSpheres=options.detection.detectCones=options.detection.detectTori=false;
    report["input_points"]=qint64(cloud.positions.size());report["working_points"]=qint64(points.size());
    report["tolerance_mm"]=planes.distanceThreshold;
    const auto analyze=[&](bool expand) {
        options.expandPlaneSupport=expand;
        reconstruction::SurfaceAnalysisResult analysis;QElapsedTimer timer;timer.start();
        if(!reconstruction::analyzeSurfaces(points,options,analysis,error)) throw std::runtime_error(error);
        QJsonArray surfaces;std::unordered_set<reconstruction::PointId> covered;
        std::vector<double> sizes;double maximumResidual=0;
        for(const auto& e:analysis.surfaces) if(const auto* p=std::get_if<reconstruction::PlaneEvidence>(&e)) {
            covered.insert(p->supportPointIds.begin(),p->supportPointIds.end());sizes.push_back(double(p->supportPointIds.size()));
            maximumResidual=std::max(maximumResidual,p->maxAbsoluteResidual);
            surfaces.append(QJsonObject{{"points",qint64(p->supportPointIds.size())},{"mean_residual_mm",p->meanAbsoluteResidual},
                {"max_residual_mm",p->maxAbsoluteResidual},{"normal",QJsonArray{p->plane.normal.x,p->plane.normal.y,p->plane.normal.z}},
                {"offset_mm",p->plane.offset}});
        }
        std::sort(sizes.begin(),sizes.end());
        return QJsonObject{{"planes",surfaces},{"plane_count",qint64(sizes.size())},{"connected_patches",qint64(analysis.patches.size())},
            {"covered_points",qint64(covered.size())},{"coverage",double(covered.size())/points.size()},
            {"largest_plane_points",sizes.empty()?0:sizes.back()},{"median_plane_points",sizes.empty()?0:sizes[sizes.size()/2]},
            {"max_residual_mm",maximumResidual},{"elapsed_ms",timer.elapsed()}};
    };
    report["baseline"]=analyze(false);report["expanded"]=analyze(true);
    std::vector<vtkIdType> ids(cloud.positions.size());std::iota(ids.begin(),ids.end(),0);
    const auto displayed=analysePointCloud(cloud,ids,planes.distanceThreshold,{},unit);
    if(!displayed.error.isEmpty()) throw std::runtime_error(displayed.error.toStdString());
    qint64 planePoints=0,planeRegions=0;
    for(const auto& s:displayed.surfaces) if(s.surface.geometry=="plane") {planePoints+=qint64(s.points.size());++planeRegions;}
    report["client"]=QJsonObject{{"plane_regions",planeRegions},{"plane_points",planePoints},
        {"plane_fraction",double(planePoints)/cloud.positions.size()},{"assigned_points",qint64(displayed.assignedPoints)}};
    report["status"]="analyzed";
}
