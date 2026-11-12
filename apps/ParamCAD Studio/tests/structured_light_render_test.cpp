#include "capture/CaptureSessionController.h"
#include "capture/VtkProjectionBackend.h"

#include <QApplication>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QImage>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QTimer>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
capture::RenderJob plane(const QString& directory,bool occluder=false) {
    capture::RenderJob job;
    job.id=QFileInfo(directory).fileName();job.directory=directory;
    job.camera.width=64;job.camera.height=48;
    job.camera.K={80,0,32,0,80,24,0,0,1};
    job.projector.width=128;job.projector.height=96;
    job.projector.K={120,0,64,0,120,48,0,0,1};job.projector.deviceToRig[3]=15;
    job.patterns=capture::grayPatterns(128,96,4);job.phaseSteps=4;
    job.nearPlane=5;job.farPlane=500;job.material.baseColor={.8,.8,.8};
    job.material.metallic=0;job.material.roughness=.7;job.stations.push_back({"s0",capture::identity()});
    job.geometry.positions={{{-40,-30,100}},{{40,-30,100}},{{40,30,100}},{{-40,30,100}}};
    job.geometry.normals={{{0,0,-1}},{{0,0,-1}},{{0,0,-1}},{{0,0,-1}}};
    job.geometry.uv={{{0,0}},{{1,0}},{{1,1}},{{0,1}}};
    job.geometry.triangles={{{0,2,1}},{{0,3,2}}};
    if(occluder) {
        job.geometry.positions.insert(job.geometry.positions.end(),{{{2,-5,80}},{{5,-5,80}},{{5,5,80}},{{2,5,80}}});
        job.geometry.normals.insert(job.geometry.normals.end(),4,{0,0,-1});
        job.geometry.uv.insert(job.geometry.uv.end(),{{{0,0}},{{1,0}},{{1,1}},{{0,1}}});
        job.geometry.triangles.insert(job.geometry.triangles.end(),{{{4,6,5}},{{4,7,6}}});
    }
    return job;
}
void captureJob(const capture::RenderJob& job,bool cancel=false) {
    capture::CaptureSessionController controller(new capture::VtkProjectionBackend);
    QEventLoop loop;
    QString error;bool complete=false,cancelled=false;
    QObject::connect(&controller,&capture::CaptureSessionController::completed,&loop,[&](const QString&){complete=true;loop.quit();});
    QObject::connect(&controller,&capture::CaptureSessionController::failed,&loop,[&](const QString& message){error=message;loop.quit();});
    QObject::connect(&controller,&capture::CaptureSessionController::cancelled,&loop,[&](const QString&){cancelled=true;loop.quit();});
    QTimer timeout;timeout.setSingleShot(true);
    QObject::connect(&timeout,&QTimer::timeout,&loop,[&]{error="rendering timed out";controller.cancel();loop.quit();});
    if(cancel)QObject::connect(&controller,&capture::CaptureSessionController::progress,&controller,
        [&](int current,int){if(current==2)controller.cancel();});
    require(controller.start(job),qPrintable(controller.lastError()));
    timeout.start(60000);loop.exec();
    require(error.isEmpty(),qPrintable(error));
    require(cancel ? cancelled && !complete : complete,"wrong acquisition terminal state");
    QFile file(job.directory+"/capture.json");require(file.open(QIODevice::ReadOnly),"missing acquisition manifest");
    const auto manifest=QJsonDocument::fromJson(file.readAll()).object();
    require(manifest["status"].toString()==(cancel?"cancelled":"complete"),"wrong manifest terminal state");
}
}

int main(int argc,char** argv) {
    QApplication app(argc,argv);
    QTemporaryDir temporary;
    const QString root=argc>1?QFileInfo(QString::fromLocal8Bit(argv[1])).absoluteFilePath():temporary.path();
    try {
        const auto plain=plane(root+"/plane");captureJob(plain);
        const QImage white(plain.directory+"/stations/s0/e0/white.png");
        const QImage black(plain.directory+"/stations/s0/e0/black.png");
        require(!white.isNull() && white.size()==QSize(64,48),"wrong render dimensions");
        const int signal=qGray(white.pixel(32,24))-qGray(black.pixel(32,24));
        std::cout<<"direct projected signal at center: "<<signal<<std::endl;
        require(signal>30 && signal<245,"projection failed or saturated");
        auto shadow=plane(root+"/shadow",true);captureJob(shadow);
        const QImage shadowWhite(shadow.directory+"/stations/s0/e0/white.png");
        const QImage shadowBlack(shadow.directory+"/stations/s0/e0/black.png");
        const int blocked=qGray(shadowWhite.pixel(32,24))-qGray(shadowBlack.pixel(32,24));
        std::cout<<"occluded projected signal at center: "<<blocked<<std::endl;
        require(blocked<signal/3,"projector light leaks through an occluder");
        auto invalid=plain;invalid.projector.deviceToRig=invalid.camera.deviceToRig;
        require(!invalid.validate().isEmpty(),"zero baseline was accepted");
        invalid=plain;invalid.stations.push_back(invalid.stations.front());
        require(!invalid.validate().isEmpty(),"duplicate station identity was accepted");

        // The same surface map must survive black/white frames and projector motion.
        QImage material(8,8,QImage::Format_RGB888);
        for(int y=0;y<8;++y)for(int x=0;x<8;++x)material.setPixelColor(x,y,x<4?QColor(32,32,32):QColor(224,224,224));
        const QString materialPath=root+"/material.png";
        require(material.save(materialPath),"cannot create material fixture");
        const auto originalHash=capture::fileHash(materialPath);
        auto textured=plane(root+"/textured");textured.material.baseColorMap=materialPath;textured.ambient=.2;
        captureJob(textured);
        auto moved=textured;moved.id="moved";moved.directory=root+"/moved";moved.projector.deviceToRig[3]=-5;
        captureJob(moved);
        const QImage textureBlack(textured.directory+"/stations/s0/e0/black.png");
        const QImage movedBlack(moved.directory+"/stations/s0/e0/black.png");
        require(textureBlack==movedBlack,"projector motion moved or changed the surface material");
        require(qGray(textureBlack.pixel(48,24))>qGray(textureBlack.pixel(16,24))+15,
                "material details disappeared when projected light was black");
        require(capture::fileHash(materialPath)==originalHash,"capture modified the original material asset");
        const QImage firstStripe(textured.directory+"/stations/s0/e0/x_4_0.png");
        const QImage movedStripe(moved.directory+"/stations/s0/e0/x_4_0.png");
        int changed=0;
        for(int y=5;y<43;++y)for(int x=5;x<59;++x)
            if(std::abs(qGray(firstStripe.pixel(x,y))-qGray(movedStripe.pixel(x,y)))>20)++changed;
        require(changed>100,"moving only the projector did not move the projected stripes");

        auto backside=plane(root+"/backside");
        for(auto& normal:backside.geometry.normals)normal={0,0,1};
        captureJob(backside);
        const QImage backWhite(backside.directory+"/stations/s0/e0/white.png");
        const QImage backBlack(backside.directory+"/stations/s0/e0/black.png");
        require(backWhite==backBlack,"back-facing surface received projected illumination");

        auto steel=plane(root+"/steel");steel.material.metallic=1;steel.material.roughness=.2;
        steel.lightIntensity=.1;captureJob(steel);
        auto otherView=steel;otherView.id="other_view";otherView.directory=root+"/steel_other_view";
        otherView.camera.deviceToRig[3]=-20;captureJob(otherView);
        const QImage steelWhite(steel.directory+"/stations/s0/e0/white.png");
        const QImage otherWhite(otherView.directory+"/stations/s0/e0/white.png");
        const int firstHighlight=qGray(steelWhite.pixel(32,24));
        const int otherHighlight=qGray(otherWhite.pixel(48,24)); // same world point, different observer
        std::cout<<"steel response at the same surface point: "<<firstHighlight<<" / "<<otherHighlight<<std::endl;
        require(std::abs(firstHighlight-otherHighlight)>10,"metal reflection did not respond to camera direction");
        auto repeat=plain;repeat.id="repeat";repeat.directory=root+"/repeat";captureJob(repeat);
        require(QImage(repeat.directory+"/stations/s0/e0/x_4_0.png")==QImage(plain.directory+"/stations/s0/e0/x_4_0.png"),
                "deterministic direct-light rendering is not repeatable");
        captureJob(plane(root+"/cancelled"),true);
        std::cout<<"structured-light render contracts passed; captures: "<<root.toStdString()<<std::endl;
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
