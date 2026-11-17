#include "AnalysisBundleReader.h"
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
    QDir root;QJsonObject manifest;QJsonArray index;QUuid uuid;
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
    QJsonValue mesh(const QJsonObject& s)const{
        if(!s.contains("render_mesh"))return QJsonValue::Null;
        QFile file;binary(file,"meshes.bin","PCADMSH1",0);const auto m=s["render_mesh"].toObject();
        const auto vr=range(m["vertices"].toObject(),32,file.size()),tr=range(m["triangles"].toObject(),24,file.size());
        QDataStream stream(&file);stream.setByteOrder(QDataStream::LittleEndian);stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
        struct Vertex{std::array<double,3> p;quint64 id;};std::vector<Vertex> vertices;vertices.reserve(size_t(vr.count));
        const auto ids=indices(s);std::set<quint64> members(ids.begin(),ids.end());qint64 foreignVertices=0,foreignTriangles=0;
        file.seek(vr.offset);for(qint64 i=0;i<vr.count;++i){Vertex v{};for(auto& x:v.p){stream>>x;check(std::isfinite(x),"Non-finite mesh vertex");}stream>>v.id;
            check(v.id<quint64(integer(manifest["source"].toObject()["point_count"])),"Invalid mesh source index");foreignVertices+=!members.count(v.id);vertices.push_back(v);}
        double area=0,foreignArea=0,longest=0;std::vector<QJsonObject> largest;
        file.seek(tr.offset);for(qint64 i=0;i<tr.count;++i){quint64 a,b,c;stream>>a>>b>>c;
            check(a<vertices.size()&&b<vertices.size()&&c<vertices.size(),"Invalid triangle vertex index");
            const auto& A=vertices[size_t(a)];const auto& B=vertices[size_t(b)];const auto& C=vertices[size_t(c)];
            std::array<double,3> u{},v{},cross{};for(int j=0;j<3;++j){u[j]=B.p[j]-A.p[j];v[j]=C.p[j]-A.p[j];}
            cross={u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
            const auto ar=std::hypot(cross[0],cross[1],cross[2])*.5;area+=ar;
            const bool foreign=!members.count(A.id)||!members.count(B.id)||!members.count(C.id);
            if(foreign){++foreignTriangles;foreignArea+=ar;}longest=std::max({longest,distance(A.p,B.p),distance(B.p,C.p),distance(C.p,A.p)});
            largest.push_back({{"area",ar},{"source_indices",QJsonArray{qint64(A.id),qint64(B.id),qint64(C.id)}},{"uses_other_collection",foreign}});
            std::stable_sort(largest.begin(),largest.end(),[](auto a,auto b){return a["area"].toDouble()>b["area"].toDouble();});if(largest.size()>3)largest.pop_back();}
        check(stream.status()==QDataStream::Ok,"Truncated mesh");QJsonArray top;for(const auto& value:largest)top.append(value);
        return QJsonObject{{"area",area},{"triangle_count",tr.count},{"maximum_edge",longest},{"foreign_source_vertices",foreignVertices},
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
        return result;}
};
}
QJsonObject inspectAnalysisBundle(const QStringList& arguments){
    QMap<QString,QString> options;QString command;bool group=false,full=false;
    const QStringList values{"--bundle","--id","--name","--filter","--kind","--limit","--offset"};
    for(qsizetype i=0;i<arguments.size();++i){const auto a=arguments[i];
        if(a=="--group"){check(!group,"Duplicate --group");group=true;}else if(a=="--full"){check(!full,"Duplicate --full");full=true;}
        else if(values.contains(a)){check(i+1<arguments.size()&&!options.contains(a),"Missing or duplicate bundle query option");options[a]=arguments[++i];}
        else {check(!a.startsWith('-')&&command.isEmpty(),"Unexpected bundle query argument: "+a);command=a;}}
    check(!options.value("--bundle").isEmpty(),"pcad inspect requires --bundle DIRECTORY");
    check(QStringList{"list","show","analyze","points","verify","manifest"}.contains(command),"Expected inspect list|show|analyze|points|verify|manifest");
    for(auto it=options.begin();it!=options.end();++it){const auto key=it.key();if(key=="--bundle")continue;
        const bool selected=command=="show"||command=="analyze"||command=="points";
        check(((key=="--id"||key=="--name")&&selected)||((key=="--filter"||key=="--kind"||key=="--offset")&&command=="list")
            ||(key=="--limit"&&(command=="list"||command=="points")),"Option not supported by inspect "+command+": "+key);}
    check(!group||command=="show","--group only applies to show");check(!full||command=="verify","--full only applies to verify");
    Bundle b(options["--bundle"]);if(command=="manifest")return b.manifest;
    const auto number=[&](const QString& key,int fallback,int low,int high){bool ok=true;const int n=options.contains(key)?options[key].toInt(&ok):fallback;
        check(ok&&n>=low&&n<=high,"Invalid query limit, offset or id");return n;};
    if(command=="list"){check(!group&&!full,"Unsupported list flags");const int limit=number("--limit",20,1,100),offset=number("--offset",0,0,INT_MAX);QJsonArray rows;
        for(auto row:b.index){auto r=row.toObject();if(r["name"].toString().contains(options["--filter"],Qt::CaseInsensitive)
            &&(!options.contains("--kind")||r["geometry"].toString()==options["--kind"]))rows.append(r);}
        QJsonArray page;for(qsizetype i=offset;i<rows.size()&&page.size()<limit;++i)page.append(rows[i]);return{{"total_matches",rows.size()},{"surfaces",page}};}
    if(command=="verify"){
        for(auto row:b.index)b.group(b.surface(row.toObject()));
        if(full)for(auto asset:b.manifest["assets"].toArray()){auto a=asset.toObject();QFile file(b.path(a["path"].toString()));check(file.open(QIODevice::ReadOnly),file.errorString());
            check(file.size()==integer(a["bytes"]),"Asset size mismatch");QCryptographicHash hash(QCryptographicHash::Sha256);
            check(hash.addData(&file)&&hash.result().toHex()==a["sha256"].toString().toLatin1(),"Asset checksum mismatch: "+a["path"].toString());}
        return{{"bundle_id",b.manifest["bundle_id"]},{"surface_records",b.index.size()},{"verification",full?"all_assets":"metadata_only"}};}
    check(options.contains("--id")!=options.contains("--name"),"Specify exactly one --id or --name");QJsonArray matches;
    const int id=options.contains("--id")?number("--id",0,0,INT_MAX):-1;
    for(auto row:b.index){auto r=row.toObject();if(id>=0?r["id"].toInt()==id:r["name"].toString()==options["--name"])matches.append(r);}
    check(matches.size()==1,matches.isEmpty()?"Surface not found":"Ambiguous name; use --id");auto s=b.surface(matches[0].toObject());
    if(command=="show")return group?QJsonObject{{"surface",s},{"shared_plane",b.group(s)}}:s;
    if(command=="analyze")return b.analyze(s);
    const int limit=number("--limit",10,1,100);QJsonArray points;
    b.points(s,[&](const Bundle::Point& p){points.append(QJsonObject{{"source_index",qint64(p.id)},{"position",Bundle::vector(p.p)},
        {"normal",Bundle::vector(p.n)},{"color",QJsonArray{p.color[0],p.color[1],p.color[2]}}});},limit);
    return{{"id",s["id"]},{"raw_points",points},{"total_points",s["point_count"]}};
}
