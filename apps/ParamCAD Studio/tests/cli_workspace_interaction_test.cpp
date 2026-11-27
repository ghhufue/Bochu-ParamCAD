#include "app/MainWindow.h"
#include "cli/StudioCommandServer.h"
#include "app/ModelingController.h"
#include "core/PointCloudDocument.h"
#include "ui/viewport/VTKViewer.h"
#include <QApplication>
#include <QEventLoop>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonDocument>
#include <QProcess>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>
#include <iostream>
#include <stdexcept>

static void require(bool value,const char* why){if(!value)throw std::runtime_error(why);}
static QJsonObject command(const QString& exe,const QStringList& args,int expected=0){
    QProcess process;QEventLoop loop;QTimer timer;timer.setSingleShot(true);
    QObject::connect(&process,&QProcess::finished,&loop,&QEventLoop::quit);
    QObject::connect(&timer,&QTimer::timeout,&loop,[&]{process.kill();loop.quit();});
    process.start(exe,args);require(process.waitForStarted(5000),"CLI starts");timer.start(60000);loop.exec();
    const auto out=process.readAllStandardOutput(),err=process.readAllStandardError();
    if(process.exitCode()!=expected){std::cerr<<out.constData()<<err.constData();throw std::runtime_error("Unexpected CLI exit code");}
    require(timer.isActive(),"CLI completed");return QJsonDocument::fromJson(out).object();
}
int main(int argc,char** argv){
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());QApplication app(argc,argv);
    try{
        require(argc==2,"CLI executable argument");const auto exe=QString::fromLocal8Bit(argv[1]);QTemporaryDir temp;
        qputenv("PCAD_SESSION_DIR",(temp.path()+"/sessions").toUtf8());
        const auto cloud=temp.filePath("box.ply");
        command(exe,{"preprocess","--synthetic","box","--out",cloud});
        MainWindow window;window.show();QApplication::processEvents();
        StudioCommandServer server([&](const QJsonObject& request,pcad::Reply reply){window.executeCliCommand(request,std::move(reply));});
        QString error;require(server.start(error),qPrintable(error));
        command(exe,{"cloud","open","--in",cloud});
        auto* document=window.findChild<PointCloudDocument*>();auto* model=window.findChild<ModelingController*>();
        require(document&&document->pointCount()>0,"CLI open updated actual document");
        auto info=command(exe,{"cloud","info"});require(info["unit"]=="mm","unit sidecar survives preprocessing");
        command(exe,{"detect"});require(!document->selection().regions().empty(),"online detection published GUI collections");
        const auto detectedBundle=temp.filePath("detected.pcad-analysis");
        command(exe,{"export","analysis","--out",detectedBundle});
        require(QFile::exists(detectedBundle+"/support-validation.json"),
            "automatic relation inference preserves the freshly detected support report");
        auto* viewport = window.findChild<VTKViewer*>();
        auto cad=command(exe,{"reconstruct","cad"});
        const auto quality = cad["result"].toObject()["brep_quality"].toObject();
        require(cad["status"]=="complete" && viewport && viewport->hasBody() && model->features().empty() &&
            quality["valid_topology"].toBool() && quality["free_edge_count"].toInt(-1)==0,
            "online CAD committed a closed B-Rep without features");
        // The ordinary CAD workflow may schedule a follow-up surface analysis.
        QElapsedTimer elapsed;elapsed.start();while(elapsed.elapsed()<15000){auto status=command(exe,{"status"});if(!status["busy"].toBool())break;QApplication::processEvents();}
        const auto project=temp.filePath("model.pcad"),step=temp.filePath("model.step"),bundle=temp.filePath("snapshot.pcad-analysis");
        command(exe,{"project","save","--out",project});command(exe,{"export","step","--out",step});
        command(exe,{"export","analysis","--out",bundle});
        require(QFile::exists(project)&&QFile::exists(step)&&QFile::exists(bundle+"/manifest.json"),"online exports actual UI state");
        auto check=command(exe,{"inspect","--bundle",bundle,"verify","--full"});require(check["verification"]=="all_assets","native inspector reads actual C++ export");
        auto offline=command(exe,{"reconstruct","cad","--offline","--in",cloud,"--output",temp.filePath("offline")});
        require(offline["cad_constructed"].toBool() && offline["history_free"].toBool() &&
            offline["project_round_trip_verified"].toBool() && offline["brep_round_trip_verified"].toBool() &&
            QFile::exists(offline["brep"].toString()), "offline exports and verifies history-free CAD");
        command(exe,{"project","open","--in",project});
        require(viewport->hasBody() && model->features().empty(), "online project reload restores the history-free solid");
        command(exe,{"capture","structured-light","--in",step,"--output",temp.filePath("scan"),
            "--scan-stations","1","--scan-size","32x32","--projector-size","32x32","--scan-no-phase-shift"});
        require(QFile::exists(temp.filePath("scan/capture/capture.json")),"online structured scan publishes capture manifest");
        QFile captureConfig(temp.filePath("capture.conf"));require(captureConfig.open(QIODevice::WriteOnly),"capture configuration");
        captureConfig.write("image_width=64\nimage_height=64\nring1_shots=2\nring2_shots=0\nring3_shots=0\nring4_shots=0\nring5_shots=0\nring6_shots=0\nring7_shots=0\n");captureConfig.close();
        qputenv("PARAMCAD_CAPTURE_CONFIG",captureConfig.fileName().toUtf8());
        command(exe,{"capture","views","--in",step,"--output",temp.filePath("views")});
        require(QFile::exists(temp.filePath("views/capture_report.json")),"online view capture uses actual capture workflow");
        std::cout<<"CLI workspace integration passed\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
