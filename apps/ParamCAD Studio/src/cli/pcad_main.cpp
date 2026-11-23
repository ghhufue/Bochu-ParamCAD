#include "CommandProtocol.h"
#include "ParamCADBuildInfo.h"
#include "BackendLogScope.h"
#include "CommandArguments.h"
#include "io/AnalysisBundleReader.h"
#include "OfflineCadCommands.h"
#include "modules/ModuleCli.h"
#include "app/ProjectPaths.h"
#include "io/PointCloudMetadata.h"
#include <reconstruction/settings/ModuleSettings.h>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLocalSocket>
#include <QProcess>
#include <QSaveFile>
#include <QTimer>
#include <QUuid>
#include <iostream>
#include <map>
#include <optional>

int run_preprocess(const QStringList&);
int run_detect(const QStringList&);
int run_analyze(const QStringList&);
int run_recognize(const QStringList&);
int run_assemble(const QStringList&);
int run_project(const QStringList&);
namespace {
using pcad::Options;
void output(const QJsonObject& result,const QString& path={}) {
    const auto bytes=QJsonDocument(result).toJson(QJsonDocument::Indented);
    if(path.isEmpty())std::cout<<bytes.constData();
    else {QSaveFile file(path);if(!file.open(QIODevice::WriteOnly)||file.write(bytes)!=bytes.size()||!file.commit())Options::fail("Cannot write report: "+path);}
}
QJsonObject exchange(const QString& endpoint,const QJsonObject& request,int timeout,bool noWait=false) {
    QLocalSocket socket;socket.connectToServer(endpoint);
    if(!socket.waitForConnected(std::min(timeout,1000)))Options::fail("Studio connection failed (no offline fallback): "+socket.errorString());
    const auto bytes=QJsonDocument(request).toJson(QJsonDocument::Compact)+'\n';
    if(socket.write(bytes)!=bytes.size()||!socket.waitForBytesWritten(1000))Options::fail("Cannot send command to Studio; execution is not retried");
    QElapsedTimer timer;timer.start();QByteArray buffer;
    while(timer.elapsed()<timeout){buffer+=socket.readAll();
        if(buffer.size()>pcad::maximumMessageBytes)Options::fail("Studio response exceeds protocol limit");
        qsizetype end;while((end=buffer.indexOf('\n'))>=0){QJsonParseError error;auto doc=QJsonDocument::fromJson(buffer.left(end),&error);buffer.remove(0,end+1);
            const auto result=doc.object();if(error.error!=QJsonParseError::NoError||result["id"]!=request["id"]||result["version"].toInt()!=pcad::protocolVersion)
                Options::fail("Invalid Studio response");
            if(noWait||result["status"]!="accepted")return result;
        }
        if(socket.state()==QLocalSocket::UnconnectedState)Options::fail("Studio disconnected before completion; command is not retried");
        socket.waitForReadyRead(int(std::min<qint64>(1000,timeout-timer.elapsed())));
    }
    Options::fail("Timed out waiting for Studio. The operation may still be running; use pcad status/cancel. Request: "+request["id"].toString());return{};
}
QJsonObject request(const QString& command,const QStringList& args={}) {
    return{{"schema","paramcad.cli.request"},{"version",pcad::protocolVersion},
        {"id",QUuid::createUuid().toString(QUuid::WithoutBraces)},{"command",command},{"arguments",QJsonArray::fromStringList(args)}};
}
QJsonArray sessions() {
    QJsonArray result;QDir dir(pcad::sessionDirectory());
    for(const auto& name:dir.entryList({"*.json"},QDir::Files,QDir::Name)){
        QFile file(dir.filePath(name));if(!file.open(QIODevice::ReadOnly)||file.size()>16384)continue;
        auto value=QJsonDocument::fromJson(file.readAll()).object();
        if(value["version"].toInt()!=pcad::protocolVersion||value["endpoint"].toString().isEmpty())continue;
        try{auto ping=exchange(value["endpoint"].toString(),request("ping"),250);if(ping["session"]==value["session"]){value["responsive"]=true;result.append(value);}}
        catch(const std::exception&){
            // A stalled GUI is still an online session. Creation time plus
            // executable identity prevents stale registration/PID reuse from
            // routing requests to a different process.
            const auto token=value["process_token"].toString();
            if(!token.isEmpty()&&token==pcad::processToken(value["pid"].toInteger())){value["responsive"]=false;result.append(value);}
        }
    }return result;
}
void help(const QString& command={}) {
    std::cout<<"pcad <command> [options] [--mode auto|online|offline | --online | --offline]\n"
        "Global: --session ID --timeout SECONDS --no-wait --report FILE --config-dir DIR\n"
        "Auto uses a running Studio for dual-mode commands; otherwise runs offline.\n"
        "Online errors never fall back offline. Offline never starts Studio.\n\n";
    for(const auto& c:pcad::capabilities())if(command.isEmpty()||c.command==command)
        std::cout<<c.command.toStdString()<<" ["<<(c.online?"online":"")<<(c.online&&c.offline?"/":"")<<(c.offline?"offline":"")<<"]  "<<c.description.toStdString()<<'\n';
    if(command=="reconstruct cad")std::cout<<"Offline: --in PLY --output NEW_DIR [--unit mm|m] [--no-projection-fallback] [--config FILE] [--set key=value] [--path MODE]\nOnline: [--in PLY] [--no-projection-fallback]; uses the current document, templates and UI settings.\n";
    if(command=="reconstruct cloud")std::cout<<"--images DIR --output ROOT [--engine auto|colmap|structured_light] [--dense] [--matcher exhaustive|sequential] [--voxel-size-mm 0.2] [--min-contrast 8] [--no-camera-info]\n";
    if(command=="inspect")std::cout<<"--bundle DIR list [--filter TEXT] [--kind TYPE] [--limit 20] [--offset 0]\n--bundle DIR show (--id ID|--name NAME) [--group]\n--bundle DIR analyze (--id ID|--name NAME)\n--bundle DIR explain (--id ID|--name NAME)\n--bundle DIR compare --ids ID,ID\n--bundle DIR points (--id ID|--name NAME) [--limit 10]\n--bundle DIR verify [--full]\n--bundle DIR manifest\n";
    if(command=="studio")std::cout<<"[--exe FILE] [--dry-run]\n";
    if(command=="capture views")std::cout<<"--in MODEL.step --output NEW_DIR [--texture IMAGE]\n";
    if(command=="capture frame")std::cout<<"--output IMAGE.png [--target cloud|surfaces|model|cad] [--azimuth DEG] [--elevation DEG] [--distance VALUE] [--width PX] [--height PX]\n";
    if(command=="capture structured-light")std::cout<<"--in MODEL.step --output NEW_DIR [--scan-stations 16] [--scan-size 1280x960] [--projector-size 512x512] [--scan-reconstruct] [--scan-renderer vtk|mitsuba] [--scan-no-phase-shift] [--scan-multi-exposure] [--scan-samples 32] [--scan-max-depth 4] [--scan-metallic 0.9] [--scan-roughness 0.4] [--scan-light-intensity 2] [--scan-baseline 0]\n";
    if(command=="cloud open"||command=="project open")std::cout<<"--in FILE\n";
    if(command=="project save"||command.startsWith("export "))std::cout<<"--out PATH\n";
    if(command=="cloud info")std::cout<<"Offline: --in PLY [--unit mm|m|arbitrary]\nOnline: current cloud and bounded collection metadata [--limit 20] [--offset 0]\n";
}
QJsonObject cloudInfo(const QStringList& arguments){
    Options o(arguments,{"--in","--unit"});PlyCloud cloud;QString error;const auto path=o.require("--in");
    if(!PlyReader::read(path,cloud,error))Options::fail(error);
    auto unit=o.get("--unit",pointCloudUnitName(path));if(!QStringList{"mm","m","arbitrary"}.contains(unit))Options::fail("Invalid unit");
    qint64 finite=0;std::array<double,3> low{INFINITY,INFINITY,INFINITY},high{-INFINITY,-INFINITY,-INFINITY};
    for(auto p:cloud.positions){if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2]))continue;++finite;
        for(int a=0;a<3;++a){low[a]=std::min(low[a],double(p[a]));high[a]=std::max(high[a],double(p[a]));}}
    return{{"schema","paramcad.cloud.info"},{"version",1},{"mode","offline"},{"source",QFileInfo(path).absoluteFilePath()},
        {"unit",unit},{"point_count",qint64(cloud.positions.size())},{"finite_points",finite},
        {"has_normals",cloud.normals.size()==cloud.positions.size()},{"has_colors",cloud.colors.size()==cloud.positions.size()},
        {"bounds",finite?QJsonValue(QJsonArray{QJsonArray{low[0],low[1],low[2]},QJsonArray{high[0],high[1],high[2]}}):QJsonValue(QJsonValue::Null)}};
}
int offlineCad(const QStringList& arguments,const QString& reportPath){
    Options o(arguments,{"--in","--output","--unit","--config","--set","--path"},{"--no-projection-fallback","--no-specialized-recovery"});
    const auto source=QFileInfo(o.require("--in")).absoluteFilePath();const auto out=QFileInfo(o.require("--output")).absoluteFilePath();
    if(QFileInfo::exists(out))Options::fail("Output directory already exists; choose a new directory");
    const auto unit=o.get("--unit",pointCloudUnitName(source));if(unit!="mm"&&unit!="m")Options::fail("CAD reconstruction requires known mm/m units");
    if(!QFileInfo(source).isFile())Options::fail("Input PLY does not exist");
    if(!QDir().mkpath(out+"/input"))Options::fail("Cannot create output directory");
    if(!QFile::copy(source,out+"/input/cloud.ply"))Options::fail("Cannot snapshot input PLY");
    output({{"unit",unit}},out+"/input/cloud.json");
    const QString internalReport=out+"/report.json";
    QStringList native{"pcad","reconstruct-cloud",out+"/input/cloud.ply",internalReport,out+"/model.step"};
    for(const auto& key:{QString("--config"),QString("--set"),QString("--path")})for(const auto& v:o.values.value(key))native<<key<<v;
    for(const auto& flag:o.flags)native<<flag;
    QJsonObject report{{"schema","paramcad.cli.result"},{"version",1},{"mode","offline"},{"cad_constructed",false}};
    try{pcad::BackendLogScope logs;reconstructOfflineCad(native,report);}catch(const std::exception& e){report["status"]="failed";report["error"]=QString::fromUtf8(e.what());output(report,internalReport);throw;}
    output(report,internalReport);output(report,reportPath);return report["cad_constructed"].toBool()?0:1;
}
int offlineCloud(const QStringList& arguments,const QString& reportPath){
    const auto r=pcad::reconstructionRequest(arguments);ReconstructionController controller;QEventLoop loop;QJsonObject result;
    QObject::connect(&controller,&ReconstructionController::finished,&loop,[&](const QString& path,int count,const QString& camera,bool dense){
        result={{"schema","paramcad.cli.result"},{"version",1},{"mode","offline"},{"status","complete"},{"point_cloud",path},{"point_count",count},{"camera_mode",camera},{"dense",dense}};loop.quit();});
    QObject::connect(&controller,&ReconstructionController::failed,&loop,[&](const QString& error){result=pcad::response("failed",error,2);result["mode"]="offline";loop.quit();});
    QObject::connect(&controller,&ReconstructionController::logMessage,&loop,[](const QString& line){std::cerr<<line.toUtf8().constData()<<'\n';});
    if(!controller.start(r))Options::fail(controller.lastError());if(result.isEmpty())loop.exec();output(result,reportPath);return result["status"]=="complete"?0:2;
}
}
int main(int argc,char** argv){
    QCoreApplication app(argc,argv);QCoreApplication::setApplicationName("ParamCAD Studio");QCoreApplication::setOrganizationName("ECE4500J");
    QString report;
    try {
        QStringList args=QCoreApplication::arguments().mid(1);QString mode="auto",session,configDir;bool modeSet=false,noWait=false,timeoutSet=false;int timeout=300000;
        for(qsizetype i=0;i<args.size();){const auto key=args[i];
            if(key=="--online"||key=="--offline"||key=="--mode"){
                if(modeSet)Options::fail("Specify online/offline mode only once");modeSet=true;
                if(key=="--mode"){if(i+1>=args.size())Options::fail("Missing --mode value");mode=args.takeAt(i+1);}else mode=key.mid(2);args.removeAt(i);
            }else if(key=="--session"||key=="--timeout"||key=="--report"||key=="--config-dir"){
                if(i+1>=args.size()||args[i+1].startsWith("--"))Options::fail("Missing value for "+key);const auto v=args.takeAt(i+1);args.removeAt(i);
                if(key=="--session"){if(!session.isEmpty())Options::fail("Duplicate session");session=v;}
                if(key=="--report"){if(!report.isEmpty())Options::fail("Duplicate report");report=QFileInfo(v).absoluteFilePath();}
                if(key=="--config-dir")configDir=QFileInfo(v).absoluteFilePath();
                if(key=="--timeout"){timeoutSet=true;bool ok=false;double seconds=v.toDouble(&ok);if(!ok||!std::isfinite(seconds)||seconds<=0||seconds>86400)Options::fail("Timeout must be in (0,86400] seconds");timeout=int(seconds*1000);}
            }else if(key=="--no-wait"){noWait=true;args.removeAt(i);}else ++i;
        }
        if(!QStringList{"auto","online","offline"}.contains(mode))Options::fail("Mode must be auto, online or offline");
        if(args.isEmpty()||args==QStringList{"--help"}||args==QStringList{"-h"}){help();return 0;}
        if (args == QStringList{"--version"}) {
            using namespace pcad::build;
            output({{"schema", "paramcad.cli.version"}, {"version", 1},
                    {"application_version", QString::fromLatin1(version)},
                    {"build_configuration", QString::fromLatin1(configuration)},
                    {"build_utc", QString::fromLatin1(builtAtUtc)},
                    {"git_commit", QString::fromLatin1(gitCommit)},
                    {"source_state", QString::fromLatin1(sourceState)},
                    {"worktree_fingerprint", QString::fromLatin1(worktreeFingerprint)}}, report);
            return 0;
        }
        const auto first=args.takeFirst();QString command=first;
        if(QStringList{"reconstruct","cloud","project","export","capture"}.contains(first)&&!args.isEmpty()&&!args.first().startsWith('-'))command+=" "+args.takeFirst();
        // 'project' alone remains the projection module command.
        const std::map<QString,int(*)(const QStringList&)> modules{{"preprocess",run_preprocess},{"detect",run_detect},{"analyze",run_analyze},{"recognize",run_recognize},{"assemble",run_assemble},{"project",run_project}};
        if(args.contains("--help")||args.contains("-h")){
            help(command);if(modules.count(command))return modules.at(command)({"pcad "+command,"--help"});return 0;}
        if(command=="capabilities"){if(!args.isEmpty())Options::fail("capabilities takes no options");output(pcad::capabilityDocument(),report);return 0;}
        std::optional<pcad::Capability> capability;for(const auto& c:pcad::capabilities())if(c.command==command)capability=c;
        if(!capability)Options::fail("Unknown command: "+command+"; run pcad --help");
        if(command=="studio"){
            if(mode=="online")Options::fail("pcad studio launches a process; --online is not applicable");
            Options o(args,{"--exe"},{"--dry-run"});const auto exe=o.get("--exe",QCoreApplication::applicationDirPath()+"/ParamCAD Studio.exe");
            if(!QFileInfo(exe).isFile())Options::fail("Studio executable not found: "+exe);
            qint64 pid=0;if(!o.flags.contains("--dry-run")&&!QProcess::startDetached(exe,{},QFileInfo(exe).absolutePath(),&pid))Options::fail("Cannot start Studio");
            output({{"status",o.flags.contains("--dry-run")?"dry_run":"started"},{"executable",exe},{"pid",pid}},report);return 0;
        }
        if(command=="sessions"){if(!args.isEmpty())Options::fail("sessions takes no options");output({{"sessions",sessions()}},report);return 0;}
        QJsonArray live;const bool configOnly=args.contains("--print-config");
        if(mode!="offline"&&capability->online&&!configOnly)live=sessions();
        QJsonObject selected;
        if(!session.isEmpty()) {if(mode=="offline"||!capability->online)Options::fail("--session requires an online command");
            for(auto item:live)if(item.toObject()["session"].toString()==session)selected=item.toObject();
            if(selected.isEmpty())Options::fail("Requested Studio session is not responding");}
        else if(live.size()==1)selected=live[0].toObject();
        else if(live.size()>1)Options::fail("Multiple Studios are running; choose --session from pcad sessions");
        const bool online=mode=="online"||(mode=="auto"&&!selected.isEmpty());
        if(online){
            if(!capability->online||configOnly)Options::fail("This operation supports offline mode only");
            if(selected.isEmpty())Options::fail("No responding Studio session; run pcad studio explicitly");
            if(!configDir.isEmpty())Options::fail("Online commands use Studio settings; --config-dir is offline only");
            for(qsizetype i=0;i+1<args.size();++i)if(QStringList{"--in","--out","--images","--output","--texture"}.contains(args[i])){args[i+1]=QFileInfo(args[i+1]).absoluteFilePath();++i;}
            auto result=exchange(selected["endpoint"].toString(),request(command,args),timeout,noWait);output(result,report);return result["exit_code"].toInt();
        }
        if(!capability->offline)Options::fail("This command requires a running Studio (--online); Studio was not started");
        if(noWait||timeoutSet)Options::fail("--no-wait and --timeout apply to online commands only");
        if(!configDir.isEmpty())reconstruction::settings::setConfigDirectory(std::filesystem::path(configDir.toStdU16String()));
        if(modules.count(command)){QStringList moduleArgs{"pcad "+command};moduleArgs+=args;if(!report.isEmpty())moduleArgs<<"--report"<<report;return modules.at(command)(moduleArgs);}
        if(command=="reconstruct cad")return offlineCad(args,report);
        if(command=="reconstruct cloud")return offlineCloud(args,report);
        if(command=="cloud info"){output(cloudInfo(args),report);return 0;}
        if(command=="inspect"){output(inspectAnalysisBundle(args),report);return 0;}
        Options::fail("Offline command has no handler");
    }catch(const std::exception& e){auto result=pcad::response("failed",QString::fromUtf8(e.what()),2);try{output(result,report);}catch(...){std::cerr<<e.what()<<'\n';}return 2;}
    return 2;
}
