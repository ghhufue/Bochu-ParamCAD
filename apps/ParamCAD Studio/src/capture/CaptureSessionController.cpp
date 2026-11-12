#include "capture/CaptureSessionController.h"

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QSaveFile>

namespace capture {
bool saveJson(const QString& path,const QJsonObject& value,QString& error) {
    QSaveFile file(path);
    if(!file.open(QIODevice::WriteOnly) || file.write(QJsonDocument(value).toJson())<0 || !file.commit()) {
        error="Cannot write JSON: "+path;return false;
    }
    return true;
}
QString fileHash(const QString& path) {
    QFile file(path);if(!file.open(QIODevice::ReadOnly))return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if(!hash.addData(&file))return {};
    return QString::fromLatin1(hash.result().toHex());
}

CaptureSessionController::CaptureSessionController(RenderBackend* backend,QObject* parent)
    :QObject(parent),m_backend(nullptr) {
    setBackend(backend);
}

bool CaptureSessionController::setBackend(RenderBackend* backend) {
    if(m_running || !backend)return false;
    Q_ASSERT(backend);
    if(m_backend==backend)return true;
    delete m_backend;m_backend=backend;
    backend->setParent(this);
    connect(backend,&RenderBackend::frameCompleted,this,&CaptureSessionController::onFrame);
    connect(backend,&RenderBackend::finished,this,&CaptureSessionController::onFinished);
    connect(backend,&RenderBackend::progress,this,[this](int completed,int total){
        event("progress",{{"completed",completed},{"total",total}});emit progress(completed,total);
    });
    return true;
}

bool CaptureSessionController::start(RenderJob job) {
    if(m_running) {m_error="An acquisition is already running";return false;}
    m_error=m_backend->validate(job);
    if(!m_error.isEmpty())return false;
    job.directory=QFileInfo(job.directory).absoluteFilePath();
    const QDir directory(job.directory);
    if(directory.exists() && !directory.entryList(QDir::AllEntries|QDir::NoDotAndDotDot|QDir::Hidden).isEmpty()) {
        m_error="Acquisition directory must be empty";return false;
    }
    if(!QDir().mkpath(job.directory+"/patterns") || !QDir().mkpath(job.directory+"/assets")) {
        m_error="Cannot create acquisition directory";return false;
    }
    int mapIndex=0;
    for(QString* path:{&job.material.baseColorMap,&job.material.metallicMap,&job.material.roughnessMap}) {
        if(path->isEmpty())continue;
        const QString destination=job.directory+QString("/assets/map_%1.%2").arg(mapIndex++).arg(QFileInfo(*path).suffix());
        if(!QFile::copy(*path,destination)) {m_error="Cannot snapshot material: "+*path;return false;}
        *path=destination;
    }
    m_job=std::move(job);m_frames=m_job.frames();m_count=0;m_elapsed.start();
    auto scene=m_job.sceneJson();
    auto material=scene["material"].toObject();
    for(const QString& key:{QString("base_color_map"),QString("metallic_map"),QString("roughness_map")}) {
        const QString path=material[key].toString();
        if(!path.isEmpty()) {material[key]=directory.relativeFilePath(path);material[key+"_sha256"]=fileHash(path);}
    }
    scene["material"]=material;
    if(!saveJson(m_job.directory+"/scene.json",scene,m_error)
        || !saveJson(m_job.directory+"/calibration.json",m_job.calibrationJson(),m_error))return false;
    QJsonArray patterns;
    for(const auto& p:m_job.patterns) {
        auto record=p.json();const QString relative="patterns/"+p.id+".png";
        const QString path=m_job.directory+"/"+relative;
        if(!patternImage(p,m_job.projector.width,m_job.projector.height,m_job.phasePeriod).save(path,"PNG")) {
            m_error="Cannot save projection pattern";return false;
        }
        record["image"]=relative;record["sha256"]=fileHash(path);patterns.append(record);
    }
    QJsonArray stations,exposures;
    for(const auto& s:m_job.stations)stations.append(QJsonObject{{"id",s.id},{"rig_to_world",matrixJson(s.rigToWorld)},
        {"camera_id","camera"},{"projector_id","projector"},{"frames",QJsonArray{}}});
    for(double gain:m_job.exposures)exposures.append(gain);
    m_manifest={{"schema","paramcad.structured-light.capture"},{"version",1},{"status","capturing"},
        {"job_id",m_job.id},{"unit","mm"},{"pixel_convention","top-left; centers=(i+0.5,j+0.5)"},
        {"intensity_encoding","linear_uint8"},{"calibration","calibration.json"},
        {"calibration_sha256",fileHash(m_job.directory+"/calibration.json")},{"scene","scene.json"},
        {"scene_sha256",fileHash(m_job.directory+"/scene.json")},{"rendering",m_backend->capabilities()},
        {"exposures",exposures},{"stations",stations},
        {"pattern_set",QJsonObject{{"width",m_job.projector.width},{"height",m_job.projector.height},
            {"phase_steps",m_job.phaseSteps},{"period",m_job.phasePeriod},{"patterns",patterns}}}};
    if(!saveManifest())return false;
    m_running=true;event("started",{{"total",static_cast<int>(m_frames.size())}});
    m_backend->start(m_job);
    return true;
}

void CaptureSessionController::cancel() {if(m_running)m_backend->cancel();}

void CaptureSessionController::onFrame(int station,int exposure,int pattern,const QString& relativeImage) {
    if(!m_running)return;
    if(m_count>=static_cast<int>(m_frames.size()) || station!=m_frames[m_count].station
        || exposure!=m_frames[m_count].exposure || pattern!=m_frames[m_count].pattern || relativeImage!=m_frames[m_count].image) {
        m_error="Rendering backend returned an unexpected frame";m_backend->cancel();return;
    }
    const QString hash=fileHash(m_job.directory+"/"+relativeImage);
    if(hash.size()!=64) {m_error="Captured frame is missing or unreadable";m_backend->cancel();return;}
    auto stations=m_manifest["stations"].toArray();auto entry=stations[station].toObject();
    auto frames=entry["frames"].toArray();
    frames.append(QJsonObject{{"pattern_id",m_job.patterns[pattern].id},{"exposure_id",QString::number(exposure)},
        {"exposure",m_job.exposures[exposure]},{"image",relativeImage},{"sha256",hash},{"status","complete"}});
    if(m_backend->capabilities()["linear_float_images"].toBool()) {
        auto record=frames.last().toObject();QString linearImage=relativeImage;linearImage.chop(4);linearImage+=".exr";
        const auto floatHash=fileHash(m_job.directory+"/"+linearImage);
        if(floatHash.size()!=64) {m_error="Missing linear float rendering measurement";m_backend->cancel();return;}
        record["linear_image"]=linearImage;record["linear_sha256"]=floatHash;frames[frames.size()-1]=record;
    }
    entry["frames"]=frames;stations[station]=entry;m_manifest["stations"]=stations;++m_count;
    // Commit at each reference boundary, not just at the end of a long sweep.
    if(pattern==static_cast<int>(m_job.patterns.size())-1 && !saveManifest())m_backend->cancel();
}

void CaptureSessionController::onFinished(bool cancelled,const QString& error) {
    if(!m_running)return;
    m_running=false;
    if(!error.isEmpty())m_error=error;
    const bool complete=!cancelled && m_error.isEmpty() && m_count==static_cast<int>(m_frames.size());
    if(!cancelled && !complete && m_error.isEmpty())m_error="Acquisition did not produce the complete frame sequence";
    const QString status=complete?"complete":(cancelled && m_error.isEmpty()?"cancelled":"failed");
    m_manifest["status"]=status;m_manifest["elapsed_ms"]=m_elapsed.elapsed();m_manifest["captured_frames"]=m_count;
    if(!m_error.isEmpty())m_manifest["error"]=m_error;
    const bool saved=saveManifest();event(status,{{"captured_frames",m_count},{"error",m_error}});
    if(complete && saved)emit completed(m_job.directory);
    else if(status=="cancelled" && saved)emit this->cancelled(m_job.directory);
    else emit failed(m_error);
}

bool CaptureSessionController::saveManifest() {return saveJson(m_job.directory+"/capture.json",m_manifest,m_error);}
void CaptureSessionController::event(const QString& type,const QJsonObject& fields) {
    auto message=fields;message["schema"]="paramcad.capture.event";message["version"]=1;
    message["job_id"]=m_job.id;message["type"]=type;message["elapsed_ms"]=m_elapsed.elapsed();
    QFile file(m_job.directory+"/events.jsonl");
    if(file.open(QIODevice::Append))file.write(QJsonDocument(message).toJson(QJsonDocument::Compact)+"\n");
}
} // namespace capture
