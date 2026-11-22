#include "io/AnalysisBundleReader.h"
#include <QCryptographicHash>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QUuid>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace {
void check(bool ok,const QString& why){if(!ok)throw std::runtime_error(why.toUtf8().constData());}
QByteArray digest(const QByteArray& bytes){return QCryptographicHash::hash(bytes,QCryptographicHash::Sha256).toHex();}
QJsonObject object(const QByteArray& bytes){QJsonParseError error;auto doc=QJsonDocument::fromJson(bytes,&error);
    check(error.error==QJsonParseError::NoError&&doc.isObject(),"Invalid bundle JSON");return doc.object();}
qint64 integer(const QJsonValue& v){check(v.isDouble()&&std::isfinite(v.toDouble())&&v.toDouble()>=0
    &&v.toDouble()<=9007199254740991.&&std::floor(v.toDouble())==v.toDouble(),"Invalid integer in bundle metadata");return v.toInteger();}
struct Range{qint64 offset,count;};
class Bundle {
public:
    QDir root;QJsonObject manifest;QJsonArray index;QUuid uuid;QJsonArray relationIndex;
    explicit Bundle(const QString& directory):root(QFileInfo(directory).canonicalFilePath()){
        check(QFileInfo(directory).isDir(),"Bundle directory does not exist");
        manifest=json("manifest.json");check(manifest["schema"]=="paramcad.analysis"&&manifest["version"].toInt()==1,"Unsupported bundle schema/version");
        uuid=QUuid(manifest["bundle_id"].toString());check(!uuid.isNull(),"Invalid bundle UUID");
        std::set<qint64> ids;
        for(const auto& line:read("surfaces.jsonl").split('\n'))if(!line.trimmed().isEmpty()){
            const auto row=object(line);check(ids.insert(integer(row["id"])).second,"Duplicate surface id");index.append(row);}
    }
    QString path(const QString& name)const{
        check(!QDir::isAbsolutePath(name),"Absolute paths are not allowed in a bundle");
        const auto candidate=QFileInfo(root.filePath(name)).canonicalFilePath();
        const auto prefix=root.canonicalPath()+"/";
        check(!candidate.isEmpty()&&candidate.startsWith(prefix,Qt::CaseInsensitive),"Missing asset or bundle path escapes directory");return candidate;}
    QByteArray read(const QString& name)const{QFile file(path(name));check(file.open(QIODevice::ReadOnly),file.errorString());return file.readAll();}
    QJsonObject json(const QString& name)const{return object(read(name));}
    const QJsonArray& relations(){
        if(relationIndex.isEmpty()&&manifest["relation_count"].toInt()>0)
            for(const auto& line:read("relations.jsonl").split('\n'))if(!line.trimmed().isEmpty())relationIndex.append(object(line));
        return relationIndex;
    }
    QJsonValue supportValidation(const QJsonObject* surface=nullptr)const {
        const auto ref=manifest["support_validation"].toObject();if(ref.isEmpty())return QJsonValue::Null;
        const auto raw=read(ref["metadata"].toString());check(digest(raw)==ref["sha256"].toString().toLatin1(),"Support validation checksum mismatch");
        const auto report=object(raw);check(report["bundle_id"]==manifest["bundle_id"] && report["version"].toInt()==1,"Support validation belongs to a different snapshot/version");
        if(!surface)return report;
        const auto stage=report.contains("after_ownership")?report["after_ownership"].toObject():report;
        const auto selected=surface->value("support_validation").toObject();QJsonArray matches;
        if(selected.contains("proposal_id"))for(auto value:stage["decisions"].toArray()) {
            const auto d=value.toObject();
            if(d["proposal_id"]==selected["proposal_id"]&&
               (!selected.contains("patch_id")||d["patch_id"]==selected["patch_id"]))matches.append(d);
        }
        return QJsonObject{{"version",report["version"]},{"scope",report["scope"]},{"mode",stage["mode"]},
            {"reference",stage["reference"]},{"options",stage["options"]},{"selected_decisions",matches}};
    }
    QJsonObject surface(const QJsonObject& row)const{
        const auto raw=read(row["metadata"].toString());check(digest(raw)==row["sha256"].toString().toLatin1(),"Surface metadata checksum mismatch");
        auto result=object(raw);check(result["bundle_id"]==manifest["bundle_id"]&&result["id"]==row["id"],"Surface belongs to a different snapshot");return result;}
    QJsonValue group(const QJsonObject& s)const{
        if(!s.contains("shared_plane"))return QJsonValue::Null;
        const auto ref=s["shared_plane"].toObject();const auto raw=read(ref["metadata"].toString());
        check(digest(raw)==ref["sha256"].toString().toLatin1(),"Plane group checksum mismatch");auto result=object(raw);
        check(result["bundle_id"]==manifest["bundle_id"]&&result["id"]==ref["id"],"Plane group belongs to a different snapshot");return result;}
    quint64 binary(QFile& file,const QString& name,const QByteArray& magic,quint32 stride)const{
        file.setFileName(path(name));check(file.open(QIODevice::ReadOnly),file.errorString());
        QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);
        char actual[8],identity[16];quint32 version=0,actualStride=0;quint64 count=0,reserved=0;
        stream.readRawData(actual,8);stream>>version>>actualStride>>count;stream.readRawData(identity,16);stream>>reserved;
        check(stream.status()==QDataStream::Ok&&QByteArray(actual,8)==magic&&version==1&&actualStride==stride
            &&QByteArray(identity,16)==uuid.toRfc4122()&&reserved==0,"Invalid binary header or snapshot UUID");
        if(stride)check(count<=quint64((std::numeric_limits<qint64>::max()-48)/stride)&&file.size()==48+qint64(count)*stride,"Truncated or extended binary records");
        return count;}
    Range range(const QJsonObject& value,qint64 stride,qint64 size)const{
        const auto offset=integer(value["offset_bytes"]),count=integer(value["count"]);
        check(offset>=48&&offset<=size&&count<=(size-offset)/stride,"Invalid binary range");return{offset,count};}
    std::vector<quint64> indices(const QJsonObject& s)const{
        QFile file;binary(file,"point-map.bin","PCADMAP1",8);const auto r=range(s["point_map"].toObject(),8,file.size());
        check((r.offset-48)%8==0&&r.count==integer(s["point_count"]),"Mapping alignment/count mismatch");
        file.seek(r.offset);QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);
        std::vector<quint64> result;result.reserve(size_t(r.count));const auto count=integer(manifest["source"].toObject()["point_count"]);
        for(qint64 i=0;i<r.count;++i){quint64 id=0;stream>>id;check(stream.status()==QDataStream::Ok&&id<quint64(count),"Invalid source index");result.push_back(id);}return result;}
    struct Point{quint64 id;std::array<double,3> p,n;std::array<int,3> color;};
    template<class F>void points(const QJsonObject& s,F f,qint64 limit=std::numeric_limits<qint64>::max())const{
        QFile file;auto count=binary(file,"points.bin","PCADPTS1",32);
        check(count==quint64(integer(manifest["source"].toObject()["point_count"])),"Point count differs from manifest");
        QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        qint64 readCount=0;for(auto id:indices(s)){if(readCount++>=limit)break;file.seek(48+qint64(id)*32);Point p{};p.id=id;
            for(auto& x:p.p){float v;stream>>v;x=v;}for(auto& x:p.n){float v;stream>>v;x=v;}
            for(auto& x:p.color){quint8 v;stream>>v;x=v;}quint8 deleted;quint32 reserved;stream>>deleted>>reserved;
            check(stream.status()==QDataStream::Ok&&!deleted&&!reserved,"Invalid/deleted point record");
            for(auto x:p.p)check(std::isfinite(x),"Non-finite active observation");f(p);}}
    static QJsonArray vector(const std::array<double,3>& p){return{p[0],p[1],p[2]};}
    static double distance(const std::array<double,3>& a,const std::array<double,3>& b){return std::hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]);}
    struct MeshVertex { std::array<double,3> p; quint64 id; };
    struct Mesh {
        std::vector<MeshVertex> vertices;
        std::vector<reconstruction::PlanarSurfaceTriangle> triangles;
    };
    Mesh readMesh(const QJsonObject& s)const{
        if(!s.contains("render_mesh"))return {};
        QFile file;binary(file,"meshes.bin","PCADMSH1",0);const auto m=s["render_mesh"].toObject();
        const auto vr=range(m["vertices"].toObject(),32,file.size()),tr=range(m["triangles"].toObject(),24,file.size());
        QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
        Mesh mesh;
        mesh.vertices.reserve(size_t(vr.count));
        mesh.triangles.reserve(size_t(tr.count));
        const auto count = quint64(integer(manifest["source"].toObject()["point_count"]));
        file.seek(vr.offset);for(qint64 i=0;i<vr.count;++i){MeshVertex v{};for(auto& x:v.p){stream>>x;check(std::isfinite(x),"Non-finite mesh vertex");}stream>>v.id;
            check(v.id<count,"Invalid mesh source index");mesh.vertices.push_back(v);}
        file.seek(tr.offset);for(qint64 i=0;i<tr.count;++i){quint64 a=0,b=0,c=0;stream>>a>>b>>c;
            check(a<mesh.vertices.size()&&b<mesh.vertices.size()&&c<mesh.vertices.size(),"Invalid triangle vertex index");
            mesh.triangles.push_back({size_t(a),size_t(b),size_t(c)});}
        check(stream.status()==QDataStream::Ok,"Truncated mesh");
        return mesh;
    }
    QJsonValue mesh(const QJsonObject& s)const{
        if(!s.contains("render_mesh"))return QJsonValue::Null;
        const auto data=readMesh(s);
        const auto& vertices=data.vertices;
        const auto ids=indices(s);std::set<quint64> members(ids.begin(),ids.end());qint64 foreignVertices=0,foreignTriangles=0;
        for(const auto& vertex:vertices)foreignVertices+=!members.count(vertex.id);
        double area=0,foreignArea=0,longest=0;std::vector<QJsonObject> largest;
        for(const auto& [a,b,c]:data.triangles){
            const auto& A=vertices[a];const auto& B=vertices[b];const auto& C=vertices[c];
            std::array<double,3> u{},v{},cross{};for(int j=0;j<3;++j){u[j]=B.p[j]-A.p[j];v[j]=C.p[j]-A.p[j];}
            cross={u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
            const auto ar=std::hypot(cross[0],cross[1],cross[2])*.5;area+=ar;
            const bool foreign=!members.count(A.id)||!members.count(B.id)||!members.count(C.id);
            if(foreign){++foreignTriangles;foreignArea+=ar;}longest=std::max({longest,distance(A.p,B.p),distance(B.p,C.p),distance(C.p,A.p)});
            largest.push_back({{"area",ar},{"source_indices",QJsonArray{qint64(A.id),qint64(B.id),qint64(C.id)}},{"uses_other_collection",foreign}});
            std::stable_sort(largest.begin(),largest.end(),[](auto a,auto b){return a["area"].toDouble()>b["area"].toDouble();});if(largest.size()>3)largest.pop_back();}
        QJsonArray top;for(const auto& value:largest)top.append(value);
        return QJsonObject{{"area",area},{"triangle_count",qint64(data.triangles.size())},{"maximum_edge",longest},{"foreign_source_vertices",foreignVertices},
            {"foreign_triangles",foreignTriangles},{"foreign_triangle_area",foreignArea},{"largest_triangles",top}};}
    QJsonObject analyze(const QJsonObject& s)const{
        qint64 count=0,normals=0;std::array<double,3> low{INFINITY,INFINITY,INFINITY},high{-INFINITY,-INFINITY,-INFINITY},sum{};
        const auto plane=s["geometry"]=="plane"?s["parameters"].toObject():QJsonObject{};const auto normal=plane["normal"].toArray();
        const double length=normal.size()==3?std::hypot(normal[0].toDouble(),normal[1].toDouble(),normal[2].toDouble()):0;
        double residualSum=0,squared=0,maximum=0;
        points(s,[&](const Point& p){++count;normals+=std::hypot(p.n[0],p.n[1],p.n[2])>1e-12;
            for(int i=0;i<3;++i){low[i]=std::min(low[i],p.p[i]);high[i]=std::max(high[i],p.p[i]);sum[i]+=p.p[i];}
            if(length>0){double residual=plane["offset"].toDouble();for(int i=0;i<3;++i)residual+=normal[i].toDouble()*p.p[i];residual=std::abs(residual)/length;
                residualSum+=residual;squared+=residual*residual;maximum=std::max(maximum,residual);}});
        for(auto& x:sum)if(count)x/=count;
        QJsonObject result{{"id",s["id"]},{"name",s["name"]},{"observations_read",count},{"nonzero_normals",normals},
            {"bounds",count?QJsonValue(QJsonArray{vector(low),vector(high)}):QJsonValue(QJsonValue::Null)},
            {"centroid",count?QJsonValue(vector(sum)):QJsonValue(QJsonValue::Null)},{"mesh",mesh(s)}};
        if(length>0&&count)result["measured_plane_residual"]=QJsonObject{{"mean",residualSum/count},{"max",maximum},{"rms",std::sqrt(squared/count)}};
        if(s.contains("support_validation"))result["support_validation"]=s["support_validation"];
        return result;}
    QJsonObject selectedValidation(const QJsonObject& s)const {
        const auto reference=manifest["support_validation"].toObject();
        if(reference.isEmpty())return {};
        const auto report=supportValidation();
        const auto selected=s["support_validation"].toObject();
        std::set<int> beforeOwnershipProposalIds;QJsonArray selectedRecoveryLinks;
        const int recoveryProposal=selected["source_recovery_proposal_id"].toInt(-1);
        for(const auto& value:report["before_ownership"].toObject()["source_recovery_proposal_links"].toArray()) {
            const auto link=value.toObject();
            if(link["source_recovery_proposal_id"].toInt(-2)==recoveryProposal) {
                beforeOwnershipProposalIds.insert(link["proposal_id"].toInt(-1));
                selectedRecoveryLinks.append(link);
            }
        }
        QJsonObject output{{"version",report["version"]},{"scope",report["scope"]},
            {"proposal_id",selected["proposal_id"]},{"patch_id",selected["patch_id"]}};
        for(const auto* stageName:{"before_ownership","after_ownership"}) {
            const auto stage=report[stageName].toObject();
            QJsonArray decisions;
            const bool beforeOwnership=QString::fromUtf8(stageName)=="before_ownership";
            for(const auto& value:stage["decisions"].toArray()) {
                const auto decision=value.toObject();
                const bool linked=beforeOwnership
                    ? beforeOwnershipProposalIds.count(decision["proposal_id"].toInt(-2))!=0
                    : decision["proposal_id"]==selected["proposal_id"] && decision["patch_id"]==selected["patch_id"];
                if(linked)decisions.append(decision);
            }
            if(!decisions.isEmpty()) {
                QJsonObject selectedStage{{"mode",stage["mode"]},{"reference",stage["reference"]},{"decisions",decisions}};
                if(beforeOwnership)selectedStage["source_recovery_proposal_links"]=selectedRecoveryLinks;
                output[stageName]=selectedStage;
            }
        }
        const auto trace=report["plane_merge_trace"].toObject();
        if(!trace.isEmpty() && !selected["source_recovery_proposal_id"].isUndefined()) {
            QJsonArray attempts;
            const auto proposal=selected["source_recovery_proposal_id"].toInt(-1);
            for(const auto& value:trace["attempts"].toArray()) {
                const auto attempt=value.toObject();
                const auto firstResults=attempt["first_result_proposal_ids"].toArray(),secondResults=attempt["second_result_proposal_ids"].toArray();
            const auto matches=[&](const QJsonArray& values) {
                return std::any_of(values.begin(),values.end(),[&](const auto& value){return value.toInt()==proposal;});
            };
                if(matches(firstResults)||matches(secondResults))attempts.append(attempt);
            }
            output["plane_merge_trace"]=QJsonObject{{"version",trace["version"]},{"source",trace["source"]},
                {"options",trace["options"]},{"truncated",trace["truncated"]},
                {"event_limit",trace["event_limit"]},{"attempts",attempts}};
        }
        return output;
    }
    QJsonObject diagnose(const QJsonObject& s) {
        const auto support=s["support_validation"].toObject();
        const auto validation=selectedValidation(s);
        const bool mergeTraceAvailable=validation.contains("plane_merge_trace");
        const bool sourceCandidateLinked=!support["source_recovery_proposal_id"].isUndefined();
        const auto trace=validation["plane_merge_trace"].toObject();
        const bool tracePresent=supportValidation().toObject().contains("plane_merge_trace");
        const auto mesh=s["render_mesh"].toObject();
        QJsonArray incident;int incidentCount=0;
        for(const auto& value:relations()) {
            const auto relation=value.toObject();
            if(relation["first"]==s["id"]||relation["second"]==s["id"]) {
                ++incidentCount;
                if(incident.size()<100)incident.append(relation);
            }
        }
        QJsonObject target{{"id",s["id"]},{"name",s["name"]},{"geometry",s["geometry"]},
            {"source",s["source"]},{"point_count",s["point_count"]},{"parameters",s["parameters"]},
            {"fit_residual",s["fit_residual"]},{"confidence",s["confidence"]},
            {"bounds",s["summary"].toObject()["bounds"]},{"centroid",s["summary"].toObject()["centroid"]},
            {"fit_stale",s["fit_stale"]}};
        QJsonObject output{{"schema","paramcad.face_diagnostics"},{"version",1},
            {"bundle_id",manifest["bundle_id"]},{"source",manifest["source"]},{"target",target},
            {"lineage",QJsonObject{{"status",sourceCandidateLinked?"source_recovery_candidate_link":(support.contains("proposal_id")?"final_validation_link":"surface_only")},
                {"proposal_id",support["proposal_id"]},{"patch_id",support["patch_id"]},
                {"source_recovery_proposal_id",support["source_recovery_proposal_id"]},
                {"trace_available",tracePresent},
                {"candidate_linked",sourceCandidateLinked},
                {"matching_attempt_found",mergeTraceAvailable&&!trace["attempts"].toArray().isEmpty()},
                {"trace_truncated",trace["truncated"]}}},
            {"support_validation",validation},{"relations",incident},
            {"relation_count",incidentCount},{"relations_truncated",incidentCount>incident.size()},
            {"display_family",group(s)},
            {"display_mesh",mesh.isEmpty()?QJsonValue(QJsonValue::Null):QJsonValue(QJsonObject{
                {"vertices",mesh["vertices"].toObject()["count"]},{"triangles",mesh["triangles"].toObject()["count"]},
                {"foreign_source_vertices",mesh["foreign_source_vertices"]},{"covered_observations",mesh["covered_observations"]}})},
            {"measured_geometry",analyze(s)}};
        return output;
    }
    QJsonObject comparePlanes(const QJsonObject& first,const QJsonObject& second) {
        check(first["geometry"]=="plane"&&second["geometry"]=="plane","compare currently requires two plane surfaces");
        auto equation=[](const QJsonObject& s){
            const auto p=s["parameters"].toObject();const auto a=p["normal"].toArray();
            check(a.size()==3,"Invalid plane normal");
            std::array<double,4> e{a[0].toDouble(),a[1].toDouble(),a[2].toDouble(),p["offset"].toDouble()};
            const auto length=std::hypot(e[0],e[1],e[2]);check(std::isfinite(length)&&length>1e-12,"Invalid plane equation");
            for(auto& v:e)v/=length;
            const auto major=std::max_element(e.begin(),e.begin()+3,[](double a,double b){return std::abs(a)<std::abs(b);});
            if(*major<0)for(auto& v:e)v=-v;
            return e;
        };
        const auto a=equation(first),b=equation(second);
        const double alignment=std::clamp(std::abs(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]),0.,1.);
        const double angle=std::acos(alignment)*180/3.14159265358979323846;
        const double offset=std::abs(a[3]-b[3]);
        std::set<quint64> firstIds,secondIds;for(auto id:indices(first))firstIds.insert(id);for(auto id:indices(second))secondIds.insert(id);
        qint64 overlap=0;for(auto id:firstIds)overlap+=secondIds.count(id)!=0;
        const auto bounds=[](const QJsonObject& s){QJsonArray out;for(const auto& v:s["summary"].toObject()["bounds"].toArray())out.append(v.toArray());return out;};
        const auto ba=bounds(first),bb=bounds(second);double gap2=0;
        if(ba.size()==2&&bb.size()==2)for(int axis=0;axis<3;++axis){
            const double dx=std::max({0.,ba[0].toArray()[axis].toDouble()-bb[1].toArray()[axis].toDouble(),bb[0].toArray()[axis].toDouble()-ba[1].toArray()[axis].toDouble()});gap2+=dx*dx;}
        std::array<double,4> common{};for(int i=0;i<4;++i)common[i]=(a[i]+b[i])*.5;
        const auto commonLength=std::hypot(common[0],common[1],common[2]);check(commonLength>1e-12,"Cannot form a diagnostic midpoint plane");
        for(auto& value:common)value/=commonLength;
        auto residuals=[&](const QJsonObject& s){std::vector<double> values;points(s,[&](const Point& p){values.push_back(std::abs(common[0]*p.p[0]+common[1]*p.p[1]+common[2]*p.p[2]+common[3]));});
            std::sort(values.begin(),values.end());if(values.empty())return QJsonObject{};
            const auto percentile=[&](double q){return values[std::min(values.size()-1,size_t(std::ceil(q*values.size())-1))];};
            return QJsonObject{{"count",qint64(values.size())},{"p50",percentile(.5)},{"p95",percentile(.95)},
                {"p99",percentile(.99)},{"max",values.back()}};};
        QJsonArray relationRows;
        for(const auto& value:relations()){const auto r=value.toObject();const auto u=r["first"].toInt(),v=r["second"].toInt();
            if((u==first["id"].toInt()&&v==second["id"].toInt())||(v==first["id"].toInt()&&u==second["id"].toInt()))relationRows.append(r);}
        const auto groupA=first["shared_plane"].toObject(),groupB=second["shared_plane"].toObject();
        const bool sameDisplayGroup=!groupA.isEmpty()&&groupA["id"]==groupB["id"];
        const auto supportReport=supportValidation().toObject();
        const auto trace=supportReport["plane_merge_trace"].toObject();QJsonArray mergeAttempts;
        const int firstProposal=first["support_validation"].toObject()["source_recovery_proposal_id"].toInt(-1);
        const int secondProposal=second["support_validation"].toObject()["source_recovery_proposal_id"].toInt(-1);
        for(const auto& value:trace["attempts"].toArray()) {
            const auto attempt=value.toObject();
            const auto firstResults=attempt["first_result_proposal_ids"].toArray(),secondResults=attempt["second_result_proposal_ids"].toArray();
            const auto contains=[&](const QJsonArray& values,int value) {
                return std::any_of(values.begin(),values.end(),[&](const auto& item){return item.toInt()==value;});
            };
            if((contains(firstResults,firstProposal)&&contains(secondResults,secondProposal))||
               (contains(firstResults,secondProposal)&&contains(secondResults,firstProposal)))mergeAttempts.append(attempt);
        }
        const bool candidateLinksAvailable=firstProposal>=0&&secondProposal>=0;
        const bool sameRecoveryProposal=candidateLinksAvailable&&firstProposal==secondProposal;
        return {{"schema","paramcad.face_diagnostics"},{"version",1},{"bundle_id",manifest["bundle_id"]},
            {"comparison",QJsonObject{{"first",first["id"]},{"second",second["id"]},
                {"angle_degrees",angle},{"offset_difference",offset},{"same_equation_within_numeric_tolerance",angle<1e-5&&offset<1e-5},
                {"bounds_distance_lower_bound",std::sqrt(gap2)},{"support_overlap_points",overlap},
                {"support_overlap_fraction_of_smaller",double(overlap)/std::max<qint64>(1,std::min<qint64>(firstIds.size(),secondIds.size()))},
                {"diagnostic_midpoint_plane_residuals",QJsonObject{{"first",residuals(first)},{"second",residuals(second)}}}}},
            {"recorded_relations",relationRows},{"same_display_group",sameDisplayGroup},
            {"merge_trace_available",!trace.isEmpty()},
            {"candidate_links_available",candidateLinksAvailable},
            {"same_recovery_proposal",sameRecoveryProposal},
            {"matching_attempt_found",!mergeAttempts.isEmpty()},
            {"trace_truncated",trace["truncated"]},
            {"merge_attempts",mergeAttempts},
            {"interpretation",!trace.isEmpty()&&firstProposal>=0&&secondProposal>=0?
                "Geometric comparisons are measurements on the exported supports. Midpoint-plane residuals are diagnostic probes, not the detector's merge fit. Merge attempts are linked through each surface's source proposal ID.":
                "Geometric comparisons are measurements on the exported supports. Midpoint-plane residuals are diagnostic probes, not the detector's merge fit. This bundle has no merge trace linked to both selected source proposals."}};
    }
};
}

std::vector<AnalysisBundlePlaneEntry> listAnalysisBundlePlanes(const QString& directory)
{
    const Bundle bundle(directory);
    std::vector<AnalysisBundlePlaneEntry> result;
    for (const auto& entry : bundle.index) {
        const auto row = entry.toObject();
        if (row["geometry"] == "plane") result.push_back({row["id"].toInt(), row["name"].toString()});
    }
    return result;
}

std::vector<AnalysisBundlePlane> loadAnalysisBundlePlanes(const QString& directory, const std::vector<int>& ids)
{
    const Bundle bundle(directory);
    std::vector<AnalysisBundlePlane> result;
    result.reserve(ids.size());
    for (const auto id : ids) {
        const auto it = std::find_if(bundle.index.begin(), bundle.index.end(), [id](const auto& value) {
            return value.toObject()["id"].toInt() == id;
        });
        check(it != bundle.index.end(), "Requested surface ID is missing from the bundle");
        const auto surface = bundle.surface((*it).toObject());
        check(surface["geometry"] == "plane" && !surface["fit_stale"].toBool(), "Surface has no current fitted plane");
        const auto params = surface["parameters"].toObject();
        const auto normal = params["normal"].toArray();
        check(normal.size() == 3 && normal[0].isDouble() && normal[1].isDouble() && normal[2].isDouble()
            && params["offset"].isDouble(), "Invalid plane equation in bundle");
        auto& plane = result.emplace_back();
        plane.surface.id = id;
        plane.surface.plane = {{normal[0].toDouble(), normal[1].toDouble(), normal[2].toDouble()}, params["offset"].toDouble()};
        plane.surface.residual = surface["fit_residual"].toDouble();
        auto mesh = bundle.readMesh(surface);
        for (const auto& vertex : mesh.vertices) plane.surface.vertices.push_back({vertex.p[0], vertex.p[1], vertex.p[2]});
        plane.surface.triangles = std::move(mesh.triangles);
        if (plane.surface.triangles.empty()) bundle.points(surface, [&](const Bundle::Point& point) {
            plane.observations.push_back({point.p[0], point.p[1], point.p[2]});
        });
    }
    return result;
}

QJsonObject inspectAnalysisBundle(const QStringList& arguments){
    QMap<QString,QString> options;QString command;bool group=false,full=false;
    const QStringList values{"--bundle","--id","--name","--ids","--filter","--kind","--limit","--offset"};
    for(qsizetype i=0;i<arguments.size();++i){const auto a=arguments[i];
        if(a=="--group"){check(!group,"Duplicate --group");group=true;}else if(a=="--full"){check(!full,"Duplicate --full");full=true;}
        else if(values.contains(a)){check(i+1<arguments.size()&&!options.contains(a),"Missing or duplicate bundle query option");options[a]=arguments[++i];}
        else {check(!a.startsWith('-')&&command.isEmpty(),"Unexpected bundle query argument: "+a);command=a;}}
    check(!options.value("--bundle").isEmpty(),"pcad inspect requires --bundle DIRECTORY");
    check(QStringList{"list","show","analyze","explain","compare","points","verify","manifest"}.contains(command),"Expected inspect list|show|analyze|explain|compare|points|verify|manifest");
    for(auto it=options.begin();it!=options.end();++it){const auto key=it.key();if(key=="--bundle")continue;
        const bool selected=command=="show"||command=="analyze"||command=="explain"||command=="points";
        check(((key=="--id"||key=="--name")&&selected)||((key=="--filter"||key=="--kind"||key=="--offset")&&command=="list")
            ||(key=="--limit"&&(command=="list"||command=="points"))||(key=="--ids"&&command=="compare"),"Option not supported by inspect "+command+": "+key);}
    check(!group||command=="show","--group only applies to show");check(!full||command=="verify","--full only applies to verify");
    Bundle b(options["--bundle"]);if(command=="manifest")return b.manifest;
    const auto number=[&](const QString& key,int fallback,int low,int high){bool ok=true;const int n=options.contains(key)?options[key].toInt(&ok):fallback;
        check(ok&&n>=low&&n<=high,"Invalid query limit, offset or id");return n;};
    if(command=="compare") {
        check(!group&&!full&&!options.contains("--id")&&!options.contains("--name")&&options.contains("--ids"),
            "inspect compare requires --ids ID,ID and accepts no other query options");
        const auto parts=options["--ids"].split(',',Qt::KeepEmptyParts);check(parts.size()==2,"inspect compare requires exactly two IDs");
        bool firstOk=false,secondOk=false;const int firstId=parts[0].trimmed().toInt(&firstOk),secondId=parts[1].trimmed().toInt(&secondOk);
        check(firstOk&&secondOk&&firstId>=0&&secondId>=0&&firstId!=secondId,"Invalid IDs for inspect compare");
        auto findSurface=[&](int wanted){for(auto row:b.index)if(row.toObject()["id"].toInt()==wanted)return b.surface(row.toObject());
            throw std::runtime_error("Surface not found: "+std::to_string(wanted));};
        return b.comparePlanes(findSurface(firstId),findSurface(secondId));
    }
    if(command=="list"){check(!group&&!full,"Unsupported list flags");const int limit=number("--limit",20,1,100),offset=number("--offset",0,0,INT_MAX);QJsonArray rows;
        for(auto row:b.index){auto r=row.toObject();if(r["name"].toString().contains(options["--filter"],Qt::CaseInsensitive)
            &&(!options.contains("--kind")||r["geometry"].toString()==options["--kind"]))rows.append(r);}
        QJsonArray page;for(qsizetype i=offset;i<rows.size()&&page.size()<limit;++i)page.append(rows[i]);return{{"total_matches",rows.size()},{"surfaces",page}};}
    if(command=="verify"){
        b.supportValidation();
        for(auto row:b.index)b.group(b.surface(row.toObject()));
        if(full)for(auto asset:b.manifest["assets"].toArray()){auto a=asset.toObject();QFile file(b.path(a["path"].toString()));check(file.open(QIODevice::ReadOnly),file.errorString());
            check(file.size()==integer(a["bytes"]),"Asset size mismatch");QCryptographicHash hash(QCryptographicHash::Sha256);
            check(hash.addData(&file)&&hash.result().toHex()==a["sha256"].toString().toLatin1(),"Asset checksum mismatch: "+a["path"].toString());}
        return{{"bundle_id",b.manifest["bundle_id"]},{"surface_records",b.index.size()},{"verification",full?"all_assets":"metadata_only"}};}
    check(options.contains("--id")!=options.contains("--name"),"Specify exactly one --id or --name");QJsonArray matches;
    const int id=options.contains("--id")?number("--id",0,0,INT_MAX):-1;
    for(auto row:b.index){auto r=row.toObject();if(id>=0?r["id"].toInt()==id:r["name"].toString()==options["--name"])matches.append(r);}
    check(matches.size()==1,matches.isEmpty()?"Surface not found":"Ambiguous name; use --id");auto s=b.surface(matches[0].toObject());
    if(command=="show")return group?QJsonObject{{"surface",s},{"shared_plane",b.group(s)},{"support_validation_report",b.supportValidation(&s)}}:s;
    if(command=="analyze")return b.analyze(s);
    if(command=="explain")return b.diagnose(s);
    const int limit=number("--limit",10,1,100);QJsonArray points;
    b.points(s,[&](const Bundle::Point& p){points.append(QJsonObject{{"source_index",qint64(p.id)},{"position",Bundle::vector(p.p)},
        {"normal",Bundle::vector(p.n)},{"color",QJsonArray{p.color[0],p.color[1],p.color[2]}}});},limit);
    return{{"id",s["id"]},{"raw_points",points},{"total_points",s["point_count"]}};
}
