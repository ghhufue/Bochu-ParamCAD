// Protocol contract for the Mitsuba process adapter. A stub worker stands in
// for the real renderer so the test never needs Mitsuba itself: it verifies
// argument passing, frame validation, EXR bookkeeping and cancellation.
#include "capture/CaptureSessionController.h"
#include "capture/MitsubaBackend.h"

#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QTimer>

#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}

QByteArray workerSource() {
    return R"PY(import json, os, sys, time
def main():
    args = sys.argv[1:]
    root = args[args.index('--capture') + 1]
    with open(os.path.join(root, 'capture.json'), encoding='utf-8') as handle:
        capture = json.load(handle)
    frames = []
    for station_index, station in enumerate(capture['stations']):
        for exposure_index in range(len(capture['exposures'])):
            for pattern_index, pattern in enumerate(capture['pattern_set']['patterns']):
                frames.append((station_index, exposure_index, pattern_index, pattern['id'], station['id']))
    cancel = os.path.join(root, '.render-cancel')
    for completed, (station, exposure, pattern, pattern_id, station_id) in enumerate(frames, 1):
        if os.path.exists(cancel):
            print(json.dumps({'type': 'cancelled'}), flush=True)
            return 3
        relative = 'stations/%s/e%d/%s' % (station_id, exposure, pattern_id)
        directory = os.path.join(root, os.path.dirname(relative))
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(root, relative + '.png'), 'wb') as handle:
            handle.write(b'stub-png')
        with open(os.path.join(root, relative + '.exr'), 'wb') as handle:
            handle.write(b'stub-exr')
        print(json.dumps({'type': 'frame', 'station': station, 'exposure': exposure,
                          'pattern': pattern, 'image': relative + '.png', 'completed': completed}), flush=True)
        time.sleep(0.005)
    print(json.dumps({'type': 'complete', 'frames': len(frames)}), flush=True)
    return 0
sys.exit(main())
)PY";
}

capture::RenderJob plane(const QString& directory) {
    capture::RenderJob job;
    job.id=QFileInfo(directory).fileName();job.directory=directory;
    job.camera.width=64;job.camera.height=48;
    job.camera.K={80,0,32,0,80,24,0,0,1};
    job.projector.width=128;job.projector.height=96;
    job.projector.K={120,0,64,0,120,48,0,0,1};job.projector.deviceToRig[3]=15;
    job.patterns=capture::grayPatterns(128,96);job.stations.push_back({"s0",capture::identity()});
    job.geometry.positions={{{-40,-30,100}},{{40,-30,100}},{{40,30,100}},{{-40,30,100}}};
    job.geometry.normals={{{0,0,-1}},{{0,0,-1}},{{0,0,-1}},{{0,0,-1}}};
    job.geometry.uv={{{0,0}},{{1,0}},{{1,1}},{{0,1}}};
    job.geometry.triangles={{{0,2,1}},{{0,3,2}}};
    return job;
}

struct Outcome {QString error;bool complete=false,cancelled=false;};

Outcome run(capture::CaptureSessionController& controller,const capture::RenderJob& job,bool cancel) {
    Outcome outcome;QEventLoop loop;QTimer timeout;timeout.setSingleShot(true);
    QObject::connect(&controller,&capture::CaptureSessionController::completed,&loop,[&](const QString&){outcome.complete=true;loop.quit();});
    QObject::connect(&controller,&capture::CaptureSessionController::failed,&loop,[&](const QString& message){outcome.error=message;loop.quit();});
    QObject::connect(&controller,&capture::CaptureSessionController::cancelled,&loop,[&](const QString&){outcome.cancelled=true;loop.quit();});
    QObject::connect(&timeout,&QTimer::timeout,&loop,[&]{outcome.error="adapter timed out";controller.cancel();loop.quit();});
    if(cancel)QObject::connect(&controller,&capture::CaptureSessionController::progress,&controller,
        [&](int current,int){if(current==2)controller.cancel();});
    require(controller.start(job),qPrintable(controller.lastError()));
    timeout.start(60000);loop.exec();
    return outcome;
}

QJsonObject manifest(const QString& directory) {
    QFile file(directory+"/capture.json");
    require(file.open(QIODevice::ReadOnly),"missing capture manifest");
    return QJsonDocument::fromJson(file.readAll()).object();
}
}

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    QTemporaryDir temporary;
    require(temporary.isValid(),"cannot create temporary directory");
    QStringList prefix;
    QString python=QStandardPaths::findExecutable("python");
    if(python.isEmpty())python=QStandardPaths::findExecutable("python3");
    if(python.isEmpty()) {
        std::cout<<"skipped: no Python interpreter for the stub worker"<<std::endl;
        return 0;
    }
    const QString worker=temporary.path()+"/stub_renderer.py";
    QFile script(worker);
    require(script.open(QIODevice::WriteOnly),"cannot write stub worker");
    script.write(workerSource());script.close();

    try {
        const capture::RenderJob base=plane(temporary.path()+"/success");
        capture::CaptureSessionController success(new capture::MitsubaBackend(python,prefix,worker,4,4,0));
        const auto outcome=run(success,base,false);
        require(outcome.error.isEmpty(),qPrintable(outcome.error));
        require(outcome.complete && !outcome.cancelled,"successful adapter run did not complete");
        const auto capture=manifest(base.directory);
        require(capture["status"].toString()=="complete","wrong manifest terminal state");
        const int frames=static_cast<int>(base.frames().size());
        require(capture["captured_frames"].toInt()==frames,"wrong captured frame count");
        const auto station=capture["stations"].toArray()[0].toObject();
        const auto last=station["frames"].toArray().last().toObject();
        require(last["image"].toString().endsWith(".png") && last["linear_image"].toString().endsWith(".exr"),
                "linear float measurement was not recorded");

        const capture::RenderJob cancelled=plane(temporary.path()+"/cancelled");
        capture::CaptureSessionController stopping(new capture::MitsubaBackend(python,prefix,worker,4,4,0));
        const auto stop=run(stopping,cancelled,true);
        require(stop.error.isEmpty(),qPrintable(stop.error));
        require(stop.cancelled && !stop.complete,"cancellation did not reach the cancelled terminal state");
        require(manifest(cancelled.directory)["status"].toString()=="cancelled","wrong cancelled manifest state");
        require(!QFile::exists(cancelled.directory+"/.render-cancel"),"cancel marker was not removed");

        const auto capabilities=capture::MitsubaBackend(python,prefix,worker,4,4,0).capabilities();
        require(capabilities["backend"].toString()=="mitsuba","wrong backend identity");
        require(capabilities["linear_float_images"].toBool(),"missing linear float capability");
        require(capabilities["indirect_reflection"].toBool(),"max depth 4 must declare indirect reflection");
        require(!capture::MitsubaBackend(python,prefix,worker,4,2,0).capabilities()["indirect_reflection"].toBool(),
                "max depth 2 must not declare indirect reflection");

        capture::MitsubaBackend missing(python,prefix,temporary.path()+"/absent.py",4,4,0);
        require(!missing.validate(base).isEmpty(),"missing worker was accepted");
        capture::MitsubaBackend invalid(python,prefix,worker,0,4,0);
        require(!invalid.validate(base).isEmpty(),"zero samples were accepted");
        capture::MitsubaBackend deep(python,prefix,worker,4,17,0);
        require(!deep.validate(base).isEmpty(),"invalid path depth was accepted");
        capture::RenderJob offCenter=base;offCenter.projector.K[2]+=1;
        require(!capture::MitsubaBackend(python,prefix,worker,4,4,0).validate(offCenter).isEmpty(),
                "off-center principal point was accepted");
        capture::RenderJob aligned=base;aligned.projector.deviceToRig=aligned.camera.deviceToRig;
        require(!capture::MitsubaBackend(python,prefix,worker,4,4,0).validate(aligned).isEmpty(),
                "zero baseline was accepted");
        std::cout<<"mitsuba adapter protocol contract passed"<<std::endl;
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"failure: "<<error.what()<<std::endl;
        return 1;
    }
}
