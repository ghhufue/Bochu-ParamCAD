#include "io/AnalysisBundleExporter.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QUuid>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

static void initializeAnalysisExportResource() {
    static const bool ready=[] {Q_INIT_RESOURCE(analysis_export);return true;}();
    Q_UNUSED(ready);
}

namespace {
void check(bool condition,const QString& message) {
    if(!condition)throw std::runtime_error(message.toUtf8().constData());
}
void cancelled(const std::function<bool()>& stop) {
    check(!stop || !stop(),QStringLiteral("Analysis export cancelled"));
}
QString hash(const QString& path,const std::function<bool()>& stop) {
    QFile file(path);check(file.open(QIODevice::ReadOnly),file.errorString());
    QCryptographicHash digest(QCryptographicHash::Sha256);
    while(!file.atEnd()) {
        cancelled(stop);const auto data=file.read(1024*1024);
        check(!data.isEmpty() || file.atEnd(),file.errorString());digest.addData(data);
    }
    return QString::fromLatin1(digest.result().toHex());
}
void json(const QString& path,const QJsonObject& object) {
    QFile file(path);check(file.open(QIODevice::WriteOnly),file.errorString());
    const auto bytes=QJsonDocument(object).toJson(QJsonDocument::Indented);
    check(file.write(bytes)==bytes.size(),file.errorString());
    check(file.flush(),file.errorString());
}
void line(QFile& file,const QJsonObject& object) {
    const auto bytes=QJsonDocument(object).toJson(QJsonDocument::Compact)+'\n';
    check(file.write(bytes)==bytes.size(),file.errorString());
}
void header(QDataStream& stream,const char* magic,quint32 stride,quint64 count,const QByteArray& identity) {
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    stream.writeRawData(magic,8);stream<<quint32(1)<<stride<<count;
    stream.writeRawData(identity.constData(),16);stream<<quint64(0);
}
QJsonObject range(qint64 offset,size_t count) {
    return {{"offset_bytes",offset},{"count",qint64(count)}};
}
QJsonArray vector(const std::array<double,3>& values) {return {values[0],values[1],values[2]};}
QJsonArray supportChecks(const std::vector<PlanarDomainSupportCheck>& checks) {
    QJsonArray result;
    for(const auto& c:checks)result.append(QJsonObject{
        {"observed_points",qint64(c.observedPoints)},{"outer_boundary_points",qint64(c.outerBoundaryPoints)},
        {"hole_boundary_points",qint64(c.holeBoundaryPoints)},{"interior_points",qint64(c.interiorPoints)},
        {"occupied_interior_cells",qint64(c.occupiedInteriorCells)},{"boundary_band_width",c.boundaryBandWidth},
        {"outer_boundary_fraction",c.observedPoints?double(c.outerBoundaryPoints)/c.observedPoints:0.},
        {"candidate_area",c.area},{"interior_area_measured",c.interiorAreaMeasured},
        {"interior_area",c.interiorArea},{"expected_interior_points",c.expectedInteriorPoints},{"rejected",c.rejected}});
    return result;
}
struct Summary {
    size_t count=0,normalCount=0;
    std::array<double,3> low{INFINITY,INFINITY,INFINITY},high{-INFINITY,-INFINITY,-INFINITY},sum{};
    void add(const PlyCloud& cloud,size_t i) {
        ++count;
        for(size_t axis=0;axis<3;++axis) {
            const double p=cloud.positions[i][axis];check(std::isfinite(p),"Active observation has non-finite coordinates");
            low[axis]=std::min(low[axis],p);high[axis]=std::max(high[axis],p);sum[axis]+=p;
        }
        if(cloud.normals.size()==cloud.positions.size()) {
            const auto& n=cloud.normals[i];
            normalCount+=std::isfinite(n[0]) && std::isfinite(n[1]) && std::isfinite(n[2])
                && std::hypot(double(n[0]),double(n[1]),double(n[2]))>1e-12;
        }
    }
    QJsonObject data() const {
        if(!count)return {{"bounds",QJsonValue::Null},{"centroid",QJsonValue::Null},{"nonzero_normals",0}};
        auto center=sum;for(auto& p:center)p/=count;
        return {{"bounds",QJsonArray{vector(low),vector(high)}},{"centroid",vector(center)},
                {"nonzero_normals",qint64(normalCount)}};
    }
};
}

bool exportAnalysisBundle(const AnalysisBundleSnapshot& snapshot,const QString& directory,
                          QString& error,const std::function<bool()>& stop)
{
    error.clear();
    try {
        initializeAnalysisExportResource();cancelled(stop);
        const QFileInfo destination(QDir::cleanPath(QFileInfo(directory).absoluteFilePath()));
        check(!destination.exists(),QStringLiteral("Export directory already exists; choose a new directory"));
        check(QDir().mkpath(destination.absolutePath()),QStringLiteral("Cannot create export parent directory"));
        QTemporaryDir stage(destination.absolutePath()+"/.pcad-analysis-XXXXXX");
        check(stage.isValid(),QStringLiteral("Cannot create analysis export staging directory"));
        QDir root(stage.path());check(root.mkdir("surfaces") && root.mkdir("plane-groups"),"Cannot create metadata directories");
        const QUuid uuid=QUuid::createUuid();const auto identity=uuid.toRfc4122();
        const QString bundleId=uuid.toString(QUuid::WithoutBraces);
        const size_t count=snapshot.cloud.positions.size();
        check(count>0 && snapshot.deleted.size()==count && snapshot.memberships.size()==count,"Invalid point-cloud snapshot dimensions");
        std::map<int,PointCloudRegion> regions;
        PointCloudRegion unassigned;unassigned.name=QStringLiteral("未归属");unassigned.geometry="unassigned";
        regions.emplace(0,unassigned);
        for(const auto& region:snapshot.regions)
            check(region.id>0 && regions.emplace(region.id,region).second,"Invalid or duplicate surface id");
        std::map<int,std::vector<quint64>> members;
        std::map<int,Summary> summaries;
        for(const auto& region:regions)members.emplace(region.first,std::vector<quint64>{});
        size_t active=0;
        for(size_t i=0;i<count;++i) {
            if((i&4095)==0)cancelled(stop);
            if(snapshot.deleted[i])continue;
            const int region=snapshot.memberships[i];check(regions.count(region)!=0,"Membership references a missing surface");
            members.at(region).push_back(quint64(i));summaries[region].add(snapshot.cloud,i);++active;
        }
        for(auto id:snapshot.selected)check(id>=0 && size_t(id)<count && !snapshot.deleted[size_t(id)],"Selection references an inactive observation");

        const bool normals=snapshot.cloud.normals.size()==count,colors=snapshot.cloud.colors.size()==count;
        QFile points(root.filePath("points.bin"));check(points.open(QIODevice::WriteOnly),points.errorString());
        QDataStream pointStream(&points);header(pointStream,"PCADPTS1",32,count,identity);
        for(size_t i=0;i<count;++i) {
            if((i&4095)==0)cancelled(stop);
            for(float p:snapshot.cloud.positions[i])pointStream<<p;
            for(size_t axis=0;axis<3;++axis)pointStream<<(normals?snapshot.cloud.normals[i][axis]:0.F);
            for(size_t axis=0;axis<3;++axis)pointStream<<quint8(colors?snapshot.cloud.colors[i][axis]:0);
            pointStream<<quint8(snapshot.deleted[i]!=0)<<quint32(0);
        }
        check(pointStream.status()==QDataStream::Ok && points.flush(),points.errorString());points.close();
        QFile mapping(root.filePath("point-map.bin"));check(mapping.open(QIODevice::WriteOnly),mapping.errorString());
        QDataStream mapStream(&mapping);header(mapStream,"PCADMAP1",8,active+snapshot.selected.size(),identity);
        std::map<int,QJsonObject> ranges;
        for(const auto& entry:members) {
            ranges[entry.first]=range(mapping.pos(),entry.second.size());
            for(auto id:entry.second)mapStream<<id;
        }
        const auto selectedRange=range(mapping.pos(),snapshot.selected.size());
        for(auto id:snapshot.selected)mapStream<<quint64(id);
        check(mapStream.status()==QDataStream::Ok && mapping.flush(),mapping.errorString());mapping.close();

        std::map<int,std::vector<int>> planeGroups;
        for(const auto& entry:snapshot.surfaces.colorRegions)if(regions.count(entry.first) && regions.count(entry.second))
            planeGroups[entry.second].push_back(entry.first);
        for(const auto& entry:snapshot.surfaces.planeSupportChecks)if(regions.count(entry.first) && !planeGroups.count(entry.first))
            planeGroups[entry.first].push_back(entry.first);
        std::map<int,QJsonObject> groupReferences;
        for(const auto& group:planeGroups) {
            QJsonArray entries;
            for(int id:group.second)entries.append(QJsonObject{{"id",id},{"name",regions.at(id).name},{"point_count",qint64(members.at(id).size())}});
            const auto name=QStringLiteral("plane-groups/%1.json").arg(group.first,8,10,QChar('0'));
            const auto found=snapshot.surfaces.planeSupportChecks.find(group.first);
            json(root.filePath(name),{{"bundle_id",bundleId},{"id",group.first},{"members",entries},
                {"parameters",regions.at(group.first).parameters},
                {"domain_support_checks",found==snapshot.surfaces.planeSupportChecks.end()?QJsonArray{}:supportChecks(found->second)}});
            const auto digest=hash(root.filePath(name),stop);
            for(int id:group.second)groupReferences[id]={{"id",group.first},{"metadata",name},{"sha256",digest}};
        }
        QFile meshes(root.filePath("meshes.bin"));check(meshes.open(QIODevice::WriteOnly),meshes.errorString());
        QDataStream meshStream(&meshes);header(meshStream,"PCADMSH1",0,0,identity);
        QFile index(root.filePath("surfaces.jsonl"));check(index.open(QIODevice::WriteOnly),index.errorString());
        for(const auto& entry:regions) {
            cancelled(stop);const int id=entry.first;const auto& region=entry.second;
            QJsonObject record{{"bundle_id",bundleId},{"id",id},{"name",region.name},{"geometry",region.geometry},
                {"point_count",qint64(members.at(id).size())},{"point_map",ranges.at(id)},
                {"parameters",region.parameters},{"source",id?region.source:QStringLiteral("unassigned")},
                {"fit_stale",region.fitStale},{"template_locked",region.templateLocked},
                {"fit_residual",region.residual},{"confidence",region.confidence},
                {"summary",summaries[id].data()},{"displayed_as_surface",snapshot.displayedSurfaces.count(id)!=0}};
            if(groupReferences.count(id))record["shared_plane"]=groupReferences.at(id);
            const auto meshIt=snapshot.surfaces.meshes.find(id);
            if(meshIt!=snapshot.surfaces.meshes.end()) {
                const auto& mesh=meshIt->second;
                check(mesh.positions.size()==mesh.sourcePointIds.size(),"Display mesh lacks observation provenance");
                const auto verticesRange=range(meshes.pos(),mesh.positions.size());
                size_t foreign=0;meshStream.setFloatingPointPrecision(QDataStream::DoublePrecision);
                for(size_t i=0;i<mesh.positions.size();++i) {
                    if((i&4095)==0)cancelled(stop);
                    const auto source=mesh.sourcePointIds[i];check(source>=0 && size_t(source)<count,"Mesh references an invalid source row");
                    for(double p:mesh.positions[i])meshStream<<p;
                    meshStream<<quint64(source);foreign+=snapshot.memberships[size_t(source)]!=id;
                }
                const auto trianglesRange=range(meshes.pos(),mesh.triangles.size());
                for(const auto& triangle:mesh.triangles)for(auto vertex:triangle) {
                    check(vertex>=0 && size_t(vertex)<mesh.positions.size(),"Invalid display triangle");meshStream<<quint64(vertex);
                }
                record["render_mesh"]=QJsonObject{{"vertices",verticesRange},{"triangles",trianglesRange},
                    {"foreign_source_vertices",qint64(foreign)},{"covered_observations",qint64(mesh.coveredPointIds.size())},
                    {"provenance",QStringLiteral("cached_display_mesh")}};
            }
            const auto name=QStringLiteral("surfaces/%1.json").arg(id,8,10,QChar('0'));
            json(root.filePath(name),record);
            line(index,{{"id",id},{"name",region.name},{"geometry",region.geometry},
                {"point_count",qint64(members.at(id).size())},{"metadata",name},{"sha256",hash(root.filePath(name),stop)}});
        }
        check(meshStream.status()==QDataStream::Ok && meshes.flush(),meshes.errorString());meshes.close();
        check(index.flush(),index.errorString());index.close();
        QFile relations(root.filePath("relations.jsonl"));check(relations.open(QIODevice::WriteOnly),relations.errorString());
        for(const auto& relation:snapshot.relations)line(relations,{{"id",relation.id},{"first",relation.first},
            {"second",relation.second},{"type",relation.type},{"automatic",relation.automatic},{"confidence",relation.confidence}});
        check(relations.flush(),relations.errorString());relations.close();
        check(QFile::copy(":/analysis-export/inspect_bundle.py",root.filePath("inspect_bundle.py")),"Cannot write bundled inspector");
        check(QFile::copy(":/analysis-export/README.md",root.filePath("README.md")),"Cannot write analysis guide");
        QJsonArray assets;
        for(const auto& name:{"points.bin","point-map.bin","meshes.bin","surfaces.jsonl","relations.jsonl","inspect_bundle.py","README.md"}) {
            const auto path=root.filePath(QString::fromLatin1(name));
            assets.append(QJsonObject{{"path",QString::fromLatin1(name)},{"bytes",QFileInfo(path).size()},{"sha256",hash(path,stop)}});
        }
        json(root.filePath("manifest.json"),{{"schema","paramcad.analysis"},{"version",1},{"bundle_id",bundleId},
            {"created_utc",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
            {"document_revision",QString::number(snapshot.documentRevision)},
            {"source",QJsonObject{{"name",snapshot.sourceName},{"unit",snapshot.sourceUnit},
                {"point_count",qint64(count)},{"active_point_count",qint64(active)},
                {"has_normals",normals},{"has_colors",colors},{"positions_sha256",snapshot.positionsFingerprint}}},
            {"surface_count",qint64(snapshot.regions.size())},{"relation_count",qint64(snapshot.relations.size())},
            {"selected_points",selectedRange},{"view",snapshot.view},{"assets",assets},
            {"binary_format",QJsonObject{{"endianness","little"},{"header_bytes",48},{"point_record_bytes",32},
                {"map_record_bytes",8},{"mesh_vertex_bytes",32},{"mesh_triangle_bytes",24},
                {"point_identity","zero_based_original_source_row"}}}});
        cancelled(stop);
        check(QDir().rename(stage.path(),destination.absoluteFilePath()),"Cannot publish analysis export directory");
        stage.setAutoRemove(false);
        return true;
    } catch(const std::exception& failure) {
        error=QString::fromUtf8(failure.what());return false;
    }
}
