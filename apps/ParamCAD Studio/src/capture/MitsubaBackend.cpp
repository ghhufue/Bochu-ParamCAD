#include "capture/MitsubaBackend.h"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QProcessEnvironment>
#include <QTimer>
#include <cmath>

namespace capture {
MitsubaBackend::MitsubaBackend(QString python,QStringList prefixArguments,QString worker,
    int samples,int maxDepth,int seed,QObject* parent)
    :RenderBackend(parent),m_python(std::move(python)),m_worker(std::move(worker)),
     m_prefix(std::move(prefixArguments)),m_samples(samples),m_depth(maxDepth),m_seed(seed) {
    auto environment=QProcessEnvironment::systemEnvironment();
    environment.insert("PYTHONIOENCODING","utf-8");environment.insert("PYTHONUNBUFFERED","1");
    environment.insert("PYTHONDONTWRITEBYTECODE","1");m_process.setProcessEnvironment(environment);
    connect(&m_process,&QProcess::readyReadStandardOutput,this,&MitsubaBackend::readOutput);
    connect(&m_process,&QProcess::readyReadStandardError,this,[this] {
        m_stderr+=m_process.readAllStandardError();m_stderr=m_stderr.right(16384);
    });
    connect(&m_process,&QProcess::finished,this,&MitsubaBackend::finish);
    connect(&m_process,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error) {
        if(error==QProcess::FailedToStart && m_running) {
            m_running=false;emit finished(false,"Cannot start bundled renderer: "+m_process.errorString());
        }
    });
}
MitsubaBackend::~MitsubaBackend() {
    if(m_process.state()!=QProcess::NotRunning) {m_process.kill();m_process.waitForFinished(3000);}
}
QJsonObject MitsubaBackend::capabilities() const {
    return {{"backend","mitsuba"},{"version","3.9.1"},{"variant","scalar_rgb"},
        {"illumination",m_depth>2?"path_traced":"direct"},{"indirect_reflection",m_depth>2},
        {"material_model","Mitsuba Principled metallic-roughness"},{"linear_float_images",true},
        {"intensity_encoding","linear_uint8"},{"float_image_encoding","linear_RGB_float32_EXR"},
        {"quantization","round(clamp(linear_radiance * exposure,0,1)*255); no auto normalization"},
        {"samples_per_pixel",m_samples},{"max_path_depth",m_depth},{"random_seed",m_seed},
        {"seed_schedule","seed + station/exposure index; shared by all pattern frames; correlated noise"},{"filter","box pixel; nearest Gray; bilinear phase"},
        {"ambient_model","constant environment radiance"},{"device_model","centered square-pixel PINHOLE"}};
}
QString MitsubaBackend::validate(const RenderJob& job) const {
    const auto error=job.validate();if(!error.isEmpty())return error;
    if(m_python.isEmpty() || !QFileInfo(m_python).isFile() || !QFileInfo(m_worker).isFile())
        return "Mitsuba rendering needs the bundled interpreter and worker; use a complete offline package";
    if(m_samples<1 || m_samples>4096 || m_depth<2 || m_depth>16 || m_seed<0)return "Invalid path tracing settings";
    for(const auto* device:{&job.camera,&job.projector})
        if(std::abs(device->K[0]-device->K[4])>1e-7 || std::abs(device->K[2]-device->width*.5)>1e-7
            || std::abs(device->K[5]-device->height*.5)>1e-7)
            return "Mitsuba adapter requires centered square-pixel pinhole devices";
    return {};
}
void MitsubaBackend::start(const RenderJob& job) {
    if(m_running) {emit finished(false,"A rendering job is already active");return;}
    const auto error=validate(job);if(!error.isEmpty()) {emit finished(false,error);return;}
    m_directory=job.directory;m_total=static_cast<int>(job.frames().size());m_completed=0;
    m_cancelled=false;m_completeEvent=false;m_error.clear();m_buffer.clear();m_stderr.clear();m_running=true;
    m_process.setProgram(m_python);
    m_process.setArguments(m_prefix+QStringList{"-B",m_worker,"--capture",job.directory,
        "--samples",QString::number(m_samples),"--max-depth",QString::number(m_depth),"--seed",QString::number(m_seed)});
    m_process.setWorkingDirectory(QFileInfo(m_worker).absolutePath());m_process.start();
}
void MitsubaBackend::cancel() {
    if(!m_running)return;
    m_cancelled=true;QFile marker(m_directory+"/.render-cancel");
    if(marker.open(QIODevice::WriteOnly))marker.write("cancel\n");
    const auto pid=m_process.processId();
    QTimer::singleShot(3000,this,[this,pid] {
        if(m_running && m_cancelled && m_process.processId()==pid)m_process.kill();
    });
}
void MitsubaBackend::readOutput() {
    m_buffer+=m_process.readAllStandardOutput();
    while(m_buffer.contains('\n')) {
        const auto newline=m_buffer.indexOf('\n');const auto line=m_buffer.left(newline).trimmed();
        m_buffer.remove(0,newline+1);if(line.isEmpty())continue;
        QJsonParseError parseError;const auto record=QJsonDocument::fromJson(line,&parseError).object();
        if(parseError.error!=QJsonParseError::NoError) {m_error="Malformed renderer event";cancel();continue;}
        const auto type=record["type"].toString();
        if(type=="frame") {
            ++m_completed;
            emit frameCompleted(record["station"].toInt(-1),record["exposure"].toInt(-1),
                record["pattern"].toInt(-1),record["image"].toString());
            emit progress(m_completed,m_total);
        } else if(type=="error")m_error=record["message"].toString();
        else if(type=="complete")m_completeEvent=record["frames"].toInt()==m_total;
    }
}
void MitsubaBackend::finish(int code,QProcess::ExitStatus status) {
    if(!m_running)return;readOutput();m_running=false;
    if(!m_cancelled && m_error.isEmpty() && (code!=0 || status!=QProcess::NormalExit || !m_completeEvent || m_completed!=m_total))
        m_error="Renderer did not complete: "+QString::fromUtf8(m_stderr);
    QFile::remove(m_directory+"/.render-cancel");
    emit finished(m_cancelled && m_error.isEmpty(),m_error);
}
}
