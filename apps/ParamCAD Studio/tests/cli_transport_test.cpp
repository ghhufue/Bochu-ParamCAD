#include "cli/StudioCommandServer.h"
#include <QCoreApplication>
#include <QEventLoop>
#include <QJsonDocument>
#include <QProcess>
#include <QTemporaryDir>
#include <QTimer>
#include <QFile>
#include <QDir>
#include <iostream>
#include <stdexcept>

static void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
struct Run{int code;QJsonObject result;QByteArray output;};
static Run run(const QString& exe,const QStringList& args){
    QProcess process;QEventLoop loop;QTimer deadline;deadline.setSingleShot(true);
    QObject::connect(&process,&QProcess::finished,&loop,&QEventLoop::quit);
    QObject::connect(&deadline,&QTimer::timeout,&loop,[&]{process.kill();loop.quit();});
    process.start(exe,args);require(process.waitForStarted(3000),"CLI starts");deadline.start(20000);loop.exec();
    require(deadline.isActive(),"CLI finishes before test deadline");const auto bytes=process.readAllStandardOutput();
    return{process.exitCode(),QJsonDocument::fromJson(bytes).object(),bytes};
}
int main(int argc,char** argv){
    QCoreApplication app(argc,argv);
    try{
        require(argc==2,"CLI executable argument");const auto exe=QString::fromLocal8Bit(argv[1]);QTemporaryDir temp;
        require(temp.isValid(),"temporary sessions");qputenv("PCAD_SESSION_DIR",temp.path().toUtf8());
        auto noStudio=run(exe,{"status"});require(noStudio.code==2,"online-only command never launches Studio");
        int dispatched=0;bool reject=false,delayed=false;
        StudioCommandServer server([&](const QJsonObject& request,pcad::Reply reply){
            ++dispatched;
            if(reject){reply(pcad::response("failed","synthetic online failure",1));return;}
            if(delayed){reply(pcad::response("accepted"));return;}
            auto value=pcad::response("complete");value["command"]=request["command"];reply(value);
        });QString error;require(server.start(error),qPrintable(error));
        auto online=run(exe,{"detect"});require(online.code==0&&online.result["mode"]=="online"&&dispatched==1,"auto dispatches to Studio");
        auto offline=run(exe,{"detect","--offline","--synthetic","box"});
        require(offline.code==0&&offline.result["mode"]=="offline"&&dispatched==1,"explicit offline never dispatches online");
        auto inspect=run(exe,{"inspect","--bundle",temp.path(),"manifest"});
        require(inspect.code==2&&dispatched==1,"offline-only inspection never dispatches");
        reject=true;auto failed=run(exe,{"detect","--synthetic","box"});
        require(failed.code==1&&failed.result["error"]=="synthetic online failure"&&dispatched==2,"online failure never retries offline");reject=false;
        delayed=true;auto accepted=run(exe,{"detect","--no-wait"});require(accepted.code==0&&accepted.result["status"]=="accepted","no-wait returns acceptance");
        auto timed=run(exe,{"detect","--timeout","0.1"});require(timed.code==2&&timed.result["error"].toString().contains("may still be running"),"timeout states uncertainty without replay");delayed=false;
        StudioCommandServer second([](const QJsonObject&,pcad::Reply reply){reply(pcad::response("complete"));});require(second.start(error),"second session");
        auto ambiguous=run(exe,{"detect"});require(ambiguous.code==2&&ambiguous.result["error"].toString().contains("Multiple"),"multiple sessions require explicit selection");
        auto selected=run(exe,{"detect","--session",server.sessionId()});require(selected.code==0,"explicit session routes");
        auto conflict=run(exe,{"detect","--online","--offline"});require(conflict.code==2,"mode conflicts rejected");
        auto unsupported=run(exe,{"inspect","--online"});require(unsupported.code==2,"unsupported online mode rejected");
#ifdef _WIN32
        const auto stalledDir=temp.path()+"/stalled";QDir().mkpath(stalledDir);qputenv("PCAD_SESSION_DIR",stalledDir.toUtf8());
        QLocalServer stalled;require(stalled.listen("pcad-stalled-test-"+server.sessionId()),"stalled server starts");
        QFile registration(stalledDir+"/stalled.json");require(registration.open(QIODevice::WriteOnly),"stalled registration");
        registration.write(QJsonDocument(QJsonObject{{"version",1},{"session","stalled"},{"endpoint",stalled.serverName()},
            {"pid",QCoreApplication::applicationPid()},{"process_token",pcad::processToken(QCoreApplication::applicationPid())}}).toJson());registration.close();
        auto stalledResult=run(exe,{"detect","--synthetic","box","--timeout","0.1"});
        require(stalledResult.code==2&&stalledResult.result["error"].toString().contains("may still be running"),"living unresponsive Studio never falls back offline");
#endif
        std::cout<<"CLI transport tests passed\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
