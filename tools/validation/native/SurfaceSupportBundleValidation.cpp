#include <reconstruction/analysis/SurfaceSupportValidation.h>
#include <reconstruction/settings/ModuleConfig.h>
#include "io/PointCloudAnalysis.h"
#include "io/AnalysisBundleExporter.h"
#include "rendering/PointCloudSurfaceMesh.h"
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDataStream>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <iostream>
#include <stdexcept>
#include <numeric>

using namespace reconstruction;
namespace {
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
QByteArray read(const QDir& root,const QString& name) {
    const auto path=QFileInfo(root.filePath(name)).canonicalFilePath();
    check(!path.isEmpty() && path.startsWith(root.canonicalPath()+"/",Qt::CaseInsensitive),"Invalid bundle asset path");
    QFile f(path);check(f.open(QIODevice::ReadOnly),"Cannot open bundle asset");return f.readAll();
}
QJsonObject object(const QByteArray& bytes){QJsonParseError error;auto d=QJsonDocument::fromJson(bytes,&error);check(error.error==QJsonParseError::NoError && d.isObject(),"Invalid bundle metadata");return d.object();}
modeling::Vec3 vector(const QJsonObject& p,const char* key){auto a=p[key].toArray();check(a.size()==3,"Invalid vector");return {a[0].toDouble(),a[1].toDouble(),a[2].toDouble()};}
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        check(argc>=3,"Usage: surface_support_bundle_validation BUNDLE REPORT.json [tolerance] [--detect] [--export-bundle DIRECTORY]");
        bool detect=false;QString exportDirectory;double explicitTolerance=0;
        for(int i=3;i<argc;++i){const auto arg=QString::fromLocal8Bit(argv[i]);if(arg=="--detect")detect=true;
            else if(arg=="--export-bundle"){check(i+1<argc,"Missing export directory");exportDirectory=QString::fromLocal8Bit(argv[++i]);detect=true;}
            else{bool ok=false;explicitTolerance=arg.toDouble(&ok);check(ok && explicitTolerance>0,"Invalid tolerance/argument");}}
        QDir root(QString::fromLocal8Bit(argv[1]));const auto manifest=object(read(root,"manifest.json"));
        check(manifest["schema"]=="paramcad.analysis" && manifest["version"].toInt()==1,"Unsupported bundle");
        auto bytes=read(root,"points.bin");check(bytes.size()>=48,"Truncated observations");
        const auto count=manifest["source"].toObject()["point_count"].toInteger();check(bytes.size()==48+count*32,"Invalid point stride/count");
        QDataStream stream(bytes);stream.setByteOrder(QDataStream::LittleEndian);stream.setFloatingPointPrecision(QDataStream::SinglePrecision);stream.skipRawData(48);
        std::vector<PointSample> samples;samples.reserve(std::size_t(count));
        for(qint64 i=0;i<count;++i) {float x,y,z,nx,ny,nz;quint32 color,flags;stream>>x>>y>>z>>nx>>ny>>nz>>color>>flags;check(flags==0,"Invalid point record");
            PointSample p;p.id=std::uint64_t(i+1);p.position={x,y,z};if(std::hypot(nx,ny,nz)>1e-12)p.normal=modeling::Vec3{nx,ny,nz};samples.push_back(p);}
        check(stream.status()==QDataStream::Ok,"Truncated observations");
        const auto mapping=read(root,"point-map.bin");
        std::vector<PrimitiveEvidence> proposals;QJsonArray sourceMetadata;double tolerance=0;
        for(const auto& line:read(root,"surfaces.jsonl").split('\n'))if(!line.trimmed().isEmpty()) {
            const auto row=object(line);if(row["geometry"]=="unassigned")continue;
            const auto raw=read(root,row["metadata"].toString());check(QCryptographicHash::hash(raw,QCryptographicHash::Sha256).toHex()==row["sha256"].toString().toLatin1(),"Surface metadata checksum mismatch");
            const auto meta=object(raw),p=meta["parameters"].toObject();const auto kind=meta["geometry"].toString();PrimitiveEvidence e;
            if(kind=="plane"){PlaneEvidence v;v.plane={vector(p,"normal"),p["offset"].toDouble()};e=v;
                if(meta["confidence"].toDouble()<.999 && meta["fit_residual"].toDouble()>0)tolerance=meta["fit_residual"].toDouble()/(1-meta["confidence"].toDouble());}
            else if(kind=="sphere"){SphereEvidence v;v.sphere={vector(p,"center"),p["radius"].toDouble()};e=v;}
            else if(kind=="cylinder"){CylinderEvidence v;v.cylinder={vector(p,"center"),vector(p,"axis"),p["radius"].toDouble()};v.axialMinimum=p["axial_min"].toDouble();v.axialMaximum=p["axial_max"].toDouble();e=v;}
            else if(kind=="cone"){ConeEvidence v;v.cone={vector(p,"center"),vector(p,"axis"),p["angle"].toDouble()};e=v;}
            else if(kind=="ring"){TorusEvidence v;v.torus={vector(p,"center"),vector(p,"axis"),p["major_radius"].toDouble(),p["minor_radius"].toDouble()};e=v;}
            else continue;
            const auto ref=meta["point_map"].toObject();const auto offset=ref["offset_bytes"].toInteger(),n=ref["count"].toInteger();check(offset>=48 && n>=0 && offset+n*8<=mapping.size(),"Invalid point mapping");
            QDataStream map(mapping);map.setByteOrder(QDataStream::LittleEndian);map.skipRawData(int(offset));
            std::visit([&](auto& v){v.id=proposals.size()+1;v.confidence=meta["confidence"].toDouble();v.meanAbsoluteResidual=meta["fit_residual"].toDouble();for(qint64 i=0;i<n;++i){quint64 id;map>>id;check(id<std::uint64_t(count),"Mapped point out of range");v.supportPointIds.push_back(id+1);}},e);
            proposals.push_back(std::move(e));sourceMetadata.append(QJsonObject{{"id",meta["id"]},{"name",meta["name"]}});
        }
        if(explicitTolerance>0)tolerance=explicitTolerance;
        check(tolerance>0 && std::isfinite(tolerance),"Cannot derive positive fit tolerance");
        const auto settings=loadAnalysisSettings();SurfaceAnalysisOptions options;options.supportValidation=settings.supportValidation;options.supportMeshing=settings.meshing;
        options.detection.plane.distanceThreshold=options.detection.sphere.distanceThreshold=options.detection.cylinder.distanceThreshold=tolerance;options.relations.distanceTolerance=tolerance*4;
        PointStore points(std::move(samples),LengthUnit::Arbitrary);std::vector<PrimitiveEvidence> accepted;SurfaceSupportValidationReport report;std::string error;QElapsedTimer timer;timer.start();
        std::optional<PointCloudAnalysisResult> detected;PlyCloud cloud;
        if(detect)check(manifest["source"].toObject()["active_point_count"].toInteger()==count,"Production replay requires an all-active source snapshot");
        if(detect) {
            for(const auto& p:points.points()){cloud.positions.push_back({float(p.position.x),float(p.position.y),float(p.position.z)});const auto n=p.normal.value_or(modeling::Vec3{});cloud.normals.push_back({float(n.x),float(n.y),float(n.z)});}
            std::vector<vtkIdType> ids(cloud.positions.size());std::iota(ids.begin(),ids.end(),0);
            detected=analysePointCloud(cloud,ids,tolerance);check(detected->error.isEmpty(),qPrintable(detected->error));
            report=detected->sourceSupportValidation;
        } else check(validateSurfaceSupport(points,proposals,options,accepted,report,error),error.c_str());
        auto result=object(QByteArray::fromStdString(serializeSurfaceSupportReport(report)));result["bundle_id"]=manifest["bundle_id"];result["elapsed_ms"]=timer.elapsed();result["tolerance"]=tolerance;result["source_metadata"]=sourceMetadata;result["accepted_equations"]=qint64(accepted.size());
        if(!detect){QJsonArray named;for(auto value:result["decisions"].toArray()){auto d=value.toObject();const auto id=d["proposal_id"].toInt()-1;if(id>=0 && id<sourceMetadata.size())d["source"]=sourceMetadata[id];named.append(d);}result["decisions"]=named;}
        else {
            result["run_mode"]="production_detection";
            result["final_validation"]=object(QByteArray::fromStdString(serializeSurfaceSupportReport(detected->finalSupportValidation)));
            result["processed_points"]=qint64(detected->processedPoints);result["assigned_points"]=qint64(detected->assignedPoints);
            result["preferred_planes"]=qint64(detected->preferredPlaneCount);
            QJsonArray regions;
            AnalysisBundleSnapshot snapshot;snapshot.cloud=cloud;snapshot.deleted.resize(cloud.positions.size());snapshot.memberships.resize(cloud.positions.size());
            snapshot.sourceName=manifest["source"].toObject()["name"].toString();snapshot.sourceUnit=manifest["source"].toObject()["unit"].toString();snapshot.positionsFingerprint=manifest["source"].toObject()["positions_sha256"].toString();
            snapshot.view=manifest["view"].toObject();
            snapshot.supportValidation={{"version",1},{"scope","full_source"},{"before_ownership",object(QByteArray::fromStdString(serializeSurfaceSupportReport(report)))},{"after_ownership",result["final_validation"]}};
            for(const auto& item:detected->surfaces) {
                auto region=item.surface;region.id=int(snapshot.regions.size()+1);snapshot.regions.push_back(region);
                for(auto id:item.points){check(id>=0 && std::size_t(id)<snapshot.memberships.size(),"Invalid published membership");check(snapshot.memberships[std::size_t(id)]==0,"Duplicate published membership");snapshot.memberships[std::size_t(id)]=region.id;}
                regions.append(QJsonObject{{"id",region.id},{"name",region.name},{"geometry",region.geometry},{"point_count",qint64(item.points.size())},{"parameters",region.parameters},{"support_validation",region.supportValidation}});
                if(!exportDirectory.isEmpty()) {
                    auto mesh=meshPointCloudSurface(region,cloud,item.points,settings.meshing);
                    snapshot.surfaces.colorRegions[region.id]=region.id;
                    snapshot.surfaces.planeSupportChecks[region.id]=mesh.domainSupportChecks;
                    if(!mesh.triangles.empty()){snapshot.surfaces.meshes.emplace(region.id,std::move(mesh));snapshot.displayedSurfaces.insert(region.id);}
                }
            }
            result["regions"]=regions;
            if(!exportDirectory.isEmpty()){QString failure;check(exportAnalysisBundle(snapshot,exportDirectory,failure),qPrintable(failure));result["export_bundle"]=exportDirectory;}
            result["elapsed_ms"]=timer.elapsed();
        }
        QFile out(QString::fromLocal8Bit(argv[2]));check(out.open(QIODevice::WriteOnly|QIODevice::NewOnly),"Report already exists or cannot be written");check(out.write(QJsonDocument(result).toJson())>0,"Cannot write report");
        std::cout<<"Validated "<<proposals.size()<<" original proposals, "<<(detected?detected->surfaces.size():accepted.size())<<" output surfaces in "<<timer.elapsed()<<" ms.\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
