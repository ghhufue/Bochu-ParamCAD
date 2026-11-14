#include "app/MainWindow.h"
#include "app/CaptureController.h"
#include "app/CaptureWorkspaceController.h"
#include "app/ReconstructionController.h"
#include "capture/CaptureSessionController.h"
#include "core/PointCloudDocument.h"
#include "ui/panels/CapturePanel.h"
#include "ui/panels/ReconstructPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDirIterator>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonDocument>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>

#include <functional>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void waitFor(const std::function<bool()>& predicate,int limit=120000) {
    QElapsedTimer timer;timer.start();
    while(!predicate() && timer.elapsed()<limit) {QApplication::processEvents(QEventLoop::AllEvents,10);QThread::msleep(1);}
    require(predicate(),"client workflow timed out");
}
QJsonObject readJson(const QString& path) {
    QFile file(path);require(file.open(QIODevice::ReadOnly),qPrintable(path));
    return QJsonDocument::fromJson(file.readAll()).object();
}
}

int main(int argc,char** argv) {
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc,argv);app.setOrganizationName("ParamCADStructuredLightTests");app.setApplicationName("client-contracts");
    QTemporaryDir temporary;
    const QString root=argc>1?QFileInfo(QString::fromLocal8Bit(argv[1])).absoluteFilePath():temporary.path();
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat,QSettings::UserScope,temporary.path());
    if(argc>2)QSettings().setValue("reconstruction/pythonInterpreter",QString::fromLocal8Bit(argv[2]));
    try {
        MainWindow window;window.setAttribute(Qt::WA_DontShowOnScreen);window.show();app.processEvents();
        auto* viewer=window.findChild<VTKViewer*>();auto* panel=window.findChild<CapturePanel*>();
        auto* reconstructionPanel=window.findChild<ReconstructPanel*>();
        auto* capture=window.findChild<CaptureController*>();
        auto* session=window.findChild<capture::CaptureSessionController*>();
        auto* reconstruction=window.findChild<ReconstructionController*>();
        auto* document=window.findChild<PointCloudDocument*>();
        require(viewer && panel && reconstructionPanel && session && reconstruction && document,"missing production components");
        capture->setBaseDirectory(root+"/captures");
        viewer->setBodyShape(BRepPrimAPI_MakeBox(40,30,20).Shape());viewer->resetCamera();
        const auto original=viewer->cameraParameters();
        panel->setDefaultDistance(std::hypot(original.position[0]-original.target[0],
            original.position[1]-original.target[1],original.position[2]-original.target[2]));
        panel->findChild<QSpinBox*>("scanCameraWidth")->setValue(128);
        panel->findChild<QSpinBox*>("scanCameraHeight")->setValue(96);
        panel->findChild<QSpinBox*>("scanProjectorWidth")->setValue(128);
        panel->findChild<QSpinBox*>("scanProjectorHeight")->setValue(128);
        panel->findChild<QSpinBox*>("scanStationCount")->setValue(6);
        panel->findChild<QCheckBox*>("scanPhaseShift")->setChecked(true);
        panel->findChild<QCheckBox*>("scanMultiExposure")->setChecked(true);
        QString captured,error;
        QObject::connect(session,&capture::CaptureSessionController::completed,&window,[&](const QString& path){captured=path;});
        QObject::connect(session,&capture::CaptureSessionController::failed,&window,[&](const QString& message){error=message;});
        panel->findChild<QPushButton*>("scanMultipleButton")->click();
        require(session->isRunning(),qPrintable(session->lastError()));
        waitFor([&]{return !session->isRunning();});require(error.isEmpty(),qPrintable(error));require(!captured.isEmpty(),"capture not completed");
        const auto manifest=readJson(captured+"/capture.json");
        require(manifest["status"].toString()=="complete" && manifest["stations"].toArray().size()==6,"wrong station count");
        require(reconstructionPanel->imageDirectory()==captured,"capture did not feed the production reconstruction panel");
        const auto after=viewer->cameraParameters();
        for(int i=0;i<3;++i)require(std::abs(original.position[i]-after.position[i])<1e-8,"capture changed viewport camera");
        reconstructionPanel->setOutputRoot(root+"/reconstructions");
        require(!reconstructionPanel->findChild<QCheckBox*>("reconstructDense")->isEnabled(),"COLMAP controls remained enabled for structured light");
        // Fusion spacing now belongs to the point-cloud page, so the client
        // contract fixture pins it dense enough for its point-count assertion.
        auto* downsampleSpacing=window.findChild<QComboBox*>("downsampleSpacing");
        require(downsampleSpacing,"point-cloud page exposes the fusion spacing");
        downsampleSpacing->setCurrentText(QStringLiteral("0.2"));
        QString cloud;
        QObject::connect(reconstruction,&ReconstructionController::finished,&window,
            [&](const QString& path,int count,const QString&,bool){cloud=path;require(count>0,"empty reconstruction result");});
        QObject::connect(reconstruction,&ReconstructionController::failed,&window,[&](const QString& message){error=message;});
        // Watchdog also escapes unexpected modal error paths in the real window.
        QTimer watchdog;watchdog.setSingleShot(true);
        QObject::connect(&watchdog,&QTimer::timeout,&app,[]{std::cerr<<"client workflow watchdog expired"<<std::endl;std::exit(3);});
        watchdog.start(120000);
        reconstructionPanel->findChild<QPushButton*>("reconstructRunButton")->click();
        waitFor([&]{return !reconstruction->isRunning();});
        require(error.isEmpty(),qPrintable(error));require(!cloud.isEmpty(),"no cloud produced");
        require(document->sourcePath()==cloud && document->pointCount()>1000,"production document did not load the point cloud");
        require(document->sourceUnit()=="mm","structured-light output lost its physical unit");
        const auto result=readJson(reconstruction->runDirectory()+"/result.json");
        require(result["status"].toString()=="complete" && result["valid_stations"].toInt()==6,"multi-station reconstruction incomplete");
        std::cout<<"client reconstructed and loaded "<<document->pointCount()<<" points: "<<cloud.toStdString()<<std::endl;
        const auto oldPath=document->sourcePath();const int oldCount=document->pointCount();
        error.clear();reconstructionPanel->findChild<QPushButton*>("reconstructRunButton")->click();
        require(reconstruction->isRunning(),"repeat reconstruction did not start");
        reconstructionPanel->findChild<QPushButton*>("reconstructCancelButton")->click();
        waitFor([&]{return !reconstruction->isRunning();});
        require(reconstruction->wasCancelled(),"reconstruction cancel was not recorded");
        require(document->sourcePath()==oldPath && document->pointCount()==oldCount,"cancel replaced the previous cloud");
        require(readJson(reconstruction->runDirectory()+"/result.json")["status"].toString()=="cancelled","cancelled run is not terminal");

        // A second successful run reuses the raw acquisition without sharing outputs.
        const QString cancelledRun=reconstruction->runDirectory();
        error.clear();cloud.clear();reconstructionPanel->findChild<QPushButton*>("reconstructRunButton")->click();
        waitFor([&]{return !reconstruction->isRunning();});
        require(error.isEmpty() && !cloud.isEmpty(),"repeated reconstruction failed");
        require(reconstruction->runDirectory()!=cancelledRun && document->sourcePath()!=oldPath,
                "repeated reconstruction reused a previous run directory");
        const auto retainedCloud=document->sourcePath();const auto retainedCount=document->pointCount();

        // Exercise the production error path on a separate incomplete capture.
        const QString broken=root+"/broken_capture";
        QDirIterator assets(captured,QDir::Files,QDirIterator::Subdirectories);
        while(assets.hasNext()) {
            const QString source=assets.next();const QString destination=broken+"/"+QDir(captured).relativeFilePath(source);
            require(QDir().mkpath(QFileInfo(destination).absolutePath()),"cannot create bad-input fixture directory");
            require(QFile::copy(source,destination),"cannot copy bad-input fixture");
        }
        auto bad=readJson(broken+"/capture.json");auto badStations=bad["stations"].toArray();
        auto badStation=badStations[0].toObject();auto badFrames=badStation["frames"].toArray();badFrames.removeLast();
        badStation["frames"]=badFrames;badStations[0]=badStation;bad["stations"]=badStations;
        QString writeError;require(capture::saveJson(broken+"/capture.json",bad,writeError),qPrintable(writeError));
        window.setProperty("paramcadBatchCapture",true);error.clear();
        ReconstructRequest badRequest;badRequest.imageDirectory=broken;badRequest.outputRoot=root+"/reconstructions";
        require(reconstruction->start(badRequest),qPrintable(reconstruction->lastError()));
        waitFor([&]{return !reconstruction->isRunning();});window.setProperty("paramcadBatchCapture",false);
        require(!error.isEmpty() && readJson(reconstruction->runDirectory()+"/result.json")["status"]=="failed",
                "missing frame did not fail through the client");
        require(document->sourcePath()==retainedCloud && document->pointCount()==retainedCount,
                "failed reconstruction replaced a valid point cloud");

        // The single-view button must produce and reconstruct exactly one station.
        error.clear();captured.clear();cloud.clear();
        panel->findChild<QPushButton*>("scanSingleButton")->click();
        require(session->isRunning(),"single station did not start");waitFor([&]{return !session->isRunning();});
        require(error.isEmpty() && readJson(captured+"/capture.json")["stations"].toArray().size()==1,
                "single-view scan did not produce exactly one station");
        reconstructionPanel->findChild<QPushButton*>("reconstructRunButton")->click();waitFor([&]{return !reconstruction->isRunning();});
        require(error.isEmpty() && !cloud.isEmpty(),"single station reconstruction failed");

        error.clear();captured.clear();
        panel->findChild<QPushButton*>("scanMultipleButton")->click();
        require(session->isRunning(),"second capture did not start");
        panel->findChild<QPushButton*>("scanCancelButton")->click();waitFor([&]{return !session->isRunning();});
        require(readJson(session->directory()+"/capture.json")["status"].toString()=="cancelled","capture cancellation not recorded");
        require(captured.isEmpty(),"cancelled capture was submitted as complete");
        window.grab().save(root+"/client.png");
        watchdog.stop();
        std::cout<<"structured-light client workflow and cancellation contracts passed"<<std::endl;
        return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
