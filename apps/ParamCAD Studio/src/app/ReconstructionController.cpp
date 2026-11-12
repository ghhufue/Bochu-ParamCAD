#include "app/ReconstructionController.h"

#include "app/ProjectPaths.h"
#include "capture/CaptureSessionController.h"
#include "io/CameraInfoBuilder.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QSettings>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStandardPaths>
#include <QTimer>

namespace {

// colmap.log records every invocation as "$ colmap <command> <args>", which is
// enough to tell the user which stage the pipeline is in.
struct StageInfo
{
    int stage = 0;
    const char* label = "";
};

StageInfo stageFor(const QString& command, bool dense)
{
    if (command == QLatin1String("feature_extractor")) {
        return {1, "Feature extraction"};
    }
    if (command == QLatin1String("exhaustive_matcher") || command == QLatin1String("sequential_matcher")) {
        return {2, "Feature matching"};
    }
    if (command == QLatin1String("mapper") || command == QLatin1String("point_triangulator")) {
        return {3, "Sparse reconstruction"};
    }
    if (command == QLatin1String("image_undistorter")) {
        return {4, "Image undistortion"};
    }
    if (command == QLatin1String("patch_match_stereo")) {
        return {5, "Dense depth maps"};
    }
    if (command == QLatin1String("stereo_fusion")) {
        return {6, "Dense cloud fusion"};
    }
    if (command == QLatin1String("model_converter")) {
        // The sparse pipeline writes the cloud directly; the dense one is done
        // by stereo_fusion afterwards.
        return {dense ? 6 : 4, "Export PLY"};
    }
    return {0, ""};
}

const int kSparseStages = 4;
const int kDenseStages = 6;
const int kPollIntervalMs = 400;

} // namespace

ReconstructionController::ReconstructionController(QObject* parent)
    : QObject(parent)
{
    m_process = new QProcess(this);
    m_process->setProcessChannelMode(QProcess::MergedChannels);

    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    // Unbuffered so progress arrives while COLMAP runs, and UTF-8 so non-ASCII
    // paths survive the round trip.
    environment.insert(QStringLiteral("PYTHONUNBUFFERED"), QStringLiteral("1"));
    environment.insert(QStringLiteral("PYTHONIOENCODING"), QStringLiteral("utf-8"));
    m_process->setProcessEnvironment(environment);

    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(kPollIntervalMs);

    connect(m_pollTimer, &QTimer::timeout, this, &ReconstructionController::pollColmapLog);
    connect(m_process, &QProcess::readyReadStandardOutput, this, &ReconstructionController::parseStdout);
    connect(m_process, &QProcess::finished, this, &ReconstructionController::handleFinished);
}

bool ReconstructionController::isRunning() const noexcept
{
    return m_running;
}

bool ReconstructionController::wasCancelled() const noexcept
{
    return m_cancelled;
}

const QString& ReconstructionController::lastError() const noexcept
{
    return m_lastError;
}

QString ReconstructionController::runDirectory() const
{
    return m_runDirectory;
}

QString ReconstructionController::resolveRepoRoot() const
{
    return ProjectPaths::projectRoot();
}

QString ReconstructionController::resolvePython(QStringList& prefixArgs) const
{
    return ProjectPaths::pythonInterpreter(prefixArgs);
}

QString ReconstructionController::createRunDirectory(const QString& outputRoot) const
{
    if (!QDir().mkpath(outputRoot)) {
        return QString();
    }
    const QDir root(outputRoot);
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));

    // Two runs within the same second must not collide, and the pipeline
    // refuses to overwrite an existing output or a non-empty workspace.
    QString name = stamp;
    for (int suffix = 2; root.exists(name); ++suffix) {
        name = stamp + QStringLiteral("-") + QString::number(suffix);
    }
    if (!root.mkpath(name)) {
        return QString();
    }
    return root.filePath(name);
}

bool ReconstructionController::buildArguments(const ReconstructRequest& request,
                                              QStringList& arguments, QString& cameraModeHint)
{
    const QString imageDirectory = QDir::cleanPath(request.imageDirectory);
    m_outputPath = m_runDirectory + QStringLiteral("/cloud.ply");

    if(m_engine=="structured_light") {
        cameraModeHint="provided";
        arguments << "--capture" << imageDirectory << "--output" << m_runDirectory
                  << "--cancel-file" << (m_runDirectory+".cancel")
                  << "--voxel-size-mm" << QString::number(request.voxelSizeMm,'g',12)
                  << "--min-contrast" << QString::number(request.minContrast,'g',12);
        return true;
    }

    arguments << QStringLiteral("--images") << imageDirectory;
    arguments << QStringLiteral("--output") << m_outputPath;
    arguments << QStringLiteral("--workspace") << (m_runDirectory + QStringLiteral("/workspace"));
    if (request.matcher == QLatin1String("sequential")) {
        arguments << QStringLiteral("--matcher") << QStringLiteral("sequential");
    }
    if (request.dense) {
        arguments << QStringLiteral("--dense") << QStringLiteral("--gpu");
    }

    cameraModeHint = QStringLiteral("estimated");
    if (!request.useCameraInfo) {
        return true;
    }

    const QString cameraInfoPath = m_runDirectory + QStringLiteral("/camera_info.json");
    QString error;
    const CameraInfoBuildResult cameraInfo =
        CameraInfoBuilder::build(imageDirectory, cameraInfoPath, error);
    if (!error.isEmpty()) {
        m_lastError = error;
        return false;
    }
    cameraModeHint = cameraInfo.cameraMode;
    if (cameraInfo.built) {
        arguments << QStringLiteral("--camera-info") << cameraInfoPath
                  << QStringLiteral("--input-unit") << QStringLiteral("mm");
        return true;
    }

    // No usable capture metadata: leave --camera-info off entirely and let
    // COLMAP estimate the cameras, which is a normal outcome.
    m_cameraInfoNote = tr("未使用相机元数据（%1）；将由 COLMAP 估计相机")
                           .arg(cameraInfo.skipReason);
    return true;
}

bool ReconstructionController::start(const ReconstructRequest& request)
{
    if (m_running) {
        m_lastError = tr("已有重建任务正在运行");
        return false;
    }
    m_lastError.clear();
    m_cameraInfoNote.clear();
    m_lastOutputLine.clear();
    m_stdoutBuffer.clear();
    m_pointCloudPath.clear();
    m_cameraMode.clear();
    m_pointCount = 0;
    m_lastStage = 0;
    m_logOffset = 0;
    m_cancelled = false;
    m_dense = request.dense;
    m_engine=request.engine;
    const bool structuredCapture=QFileInfo::exists(QDir(request.imageDirectory).filePath("capture.json"));
    if(m_engine=="auto")m_engine=structuredCapture?"structured_light":"colmap";
    if(m_engine!="colmap" && m_engine!="structured_light") {m_lastError=tr("未知重建后端");return false;}
    if(m_engine=="colmap" && structuredCapture) {
        m_lastError=tr("此目录是编码结构光采集，请使用结构光后端；编码序列不能作为普通多视角照片匹配。");return false;
    }
    if(m_engine=="structured_light")m_dense=true;

    if (!QFileInfo(request.imageDirectory).isDir()) {
        m_lastError = tr("图像目录不存在：%1").arg(request.imageDirectory);
        return false;
    }
    if (m_engine=="colmap" && CameraInfoBuilder::findImages(request.imageDirectory).size() < 2) {
        m_lastError = tr("%1 中至少需要两张图像").arg(request.imageDirectory);
        return false;
    }
    if (request.outputRoot.isEmpty()) {
        m_lastError = tr("未指定输出目录");
        return false;
    }

    m_repoRoot = resolveRepoRoot();
    if (m_repoRoot.isEmpty()) {
        m_lastError = tr("无法在 %1 之上找到 core/reconstruction/reconstruct.py")
                          .arg(QCoreApplication::applicationDirPath());
        return false;
    }

    QStringList prefixArgs;
    const QString python = resolvePython(prefixArgs);
    if (python.isEmpty()) {
        m_lastError = tr("未找到随包 Python 运行时，请检查完整离线包；开发环境可设置 reconstruction/pythonInterpreter。");
        return false;
    }

    m_runDirectory = createRunDirectory(request.outputRoot);
    if (m_runDirectory.isEmpty()) {
        m_lastError = tr("无法在 %1 下创建运行目录").arg(request.outputRoot);
        return false;
    }

    QString cameraModeHint;
    QStringList arguments;
    if (!buildArguments(request, arguments, cameraModeHint)) {
        return false;
    }

    arguments.prepend(m_repoRoot + (m_engine=="structured_light"
        ? QStringLiteral("/core/reconstruction/reconstruct_structured_light.py")
        : QStringLiteral("/core/reconstruction/reconstruct.py")));
    arguments = prefixArgs + arguments;

    // Set program and arguments separately so paths with spaces or non-ASCII
    // characters never go through a shell.
    m_process->setProgram(python);
    m_process->setArguments(arguments);
    m_process->setWorkingDirectory(m_repoRoot);

    m_running = true;
    m_process->start();
    if (!m_process->waitForStarted(5000)) {
        const QString message = tr("无法启动 %1：%2").arg(python, m_process->errorString());
        m_running = false;
        reportFailure(message);
        return false;
    }

    if(m_engine=="colmap")m_pollTimer->start();
    emit started(m_runDirectory, cameraModeHint);
    emit progressChanged(0, m_engine=="structured_light"?5:(m_dense ? kDenseStages : kSparseStages), tr("正在启动"));
    if (!m_cameraInfoNote.isEmpty()) {
        emit logMessage(m_cameraInfoNote);
    }
    return true;
}

void ReconstructionController::cancel()
{
    if (!m_running || m_process == nullptr) {
        return;
    }
    m_cancelled = true;
    if(m_engine=="structured_light") {
        QFile marker(m_runDirectory+".cancel");
        if(marker.open(QIODevice::WriteOnly))marker.write("cancel\n");
        const auto pid=m_process->processId();
        QTimer::singleShot(3000,this,[this,pid] {
            if(m_running && m_cancelled && m_process->processId()==pid)m_process->kill();
        });
        return;
    }

    // Killing the Python process does not stop the colmap.exe it spawned, so the
    // whole tree has to go. taskkill walks the parent/child chain, which only
    // exists while Python is still alive: killing Python first would orphan
    // colmap.exe and leave it running, so taskkill has to finish first.
#if defined(Q_OS_WIN)
    const qint64 pid = m_process->processId();
    if (pid > 0) {
        QProcess::execute(QStringLiteral("taskkill"),
                          {QStringLiteral("/F"), QStringLiteral("/T"),
                           QStringLiteral("/PID"), QString::number(pid)});
    }
#endif
    m_process->kill();
}

void ReconstructionController::pollColmapLog()
{
    if (m_runDirectory.isEmpty() || m_engine!="colmap") {
        return;
    }
    QFile log(m_runDirectory + QStringLiteral("/workspace/colmap.log"));
    if (!log.open(QIODevice::ReadOnly)) {
        return;
    }
    const qint64 size = log.size();
    if (size <= m_logOffset || !log.seek(m_logOffset)) {
        return;
    }
    const QByteArray data = log.read(size - m_logOffset);
    const QString text = QString::fromUtf8(data);
    if (!text.endsWith(QLatin1Char('\n'))) {
        // A partially written line is picked up on the next poll.
        const int lastBreak = text.lastIndexOf(QLatin1Char('\n'));
        if (lastBreak < 0) {
            return;
        }
        m_logOffset += static_cast<qint64>(text.left(lastBreak + 1).toUtf8().size());
        return;
    }
    m_logOffset = size;

    static const QRegularExpression commandPattern(QStringLiteral("^\\s*\\$\\s*colmap\\s+(\\S+)"));
    for (const QString& rawLine : text.split(QLatin1Char('\n'))) {
        const QString line = rawLine.trimmed();
        if (line.isEmpty()) {
            continue;
        }
        emit logMessage(line);
        const QRegularExpressionMatch match = commandPattern.match(line);
        if (!match.hasMatch()) {
            continue;
        }
        const StageInfo info = stageFor(match.captured(1), m_dense);
        if (info.stage == 0 || info.stage == m_lastStage) {
            continue;
        }
        m_lastStage = info.stage;
        emit progressChanged(info.stage, m_dense ? kDenseStages : kSparseStages,
                             QString::fromUtf8(info.label));
    }
}

void ReconstructionController::parseStdout()
{
    if (m_process == nullptr) {
        return;
    }
    m_stdoutBuffer += QString::fromUtf8(m_process->readAllStandardOutput());
    int newline = m_stdoutBuffer.indexOf(QLatin1Char('\n'));
    while (newline >= 0) {
        const QString line = m_stdoutBuffer.left(newline);
        m_stdoutBuffer.remove(0, newline + 1);
        handleStdoutLine(line.trimmed());
        newline = m_stdoutBuffer.indexOf(QLatin1Char('\n'));
    }
}

void ReconstructionController::handleStdoutLine(const QString& line)
{
    if (line.isEmpty()) {
        return;
    }
    m_lastOutputLine = line;
    if(m_engine=="structured_light") {
        QJsonParseError error;
        const auto message=QJsonDocument::fromJson(line.toUtf8(),&error).object();
        if(error.error==QJsonParseError::NoError && !message.isEmpty()) {
            const QString type=message["type"].toString();
            if(type=="progress") {
                const QString stage=message["stage"].toString();
                const QStringList stages{"validate","decode","triangulate","fuse","export"};
                const QStringList labels{tr("校验采集数据"),tr("解码投影图案"),tr("三角化点云"),tr("融合扫描站"),tr("导出点云")};
                const int index=stages.indexOf(stage);
                if(index>=0)emit progressChanged(index+1,5,QString("%1 · %2/%3")
                    .arg(labels[index]).arg(message["completed"].toInt()).arg(message["total"].toInt()));
            } else if(type=="result") {
                m_pointCloudPath=message["point_cloud"].toString();m_pointCount=message["point_count"].toInt();
                m_cameraMode="provided";
            } else if(type=="log" || type=="error") {
                m_lastOutputLine=message["message"].toString();emit logMessage(m_lastOutputLine);
            }
            return;
        }
    }

    const QStringList markers{QStringLiteral("point cloud: "), QStringLiteral("points: "),
                              QStringLiteral("camera mode: "), QStringLiteral("manifest: ")};
    for (const QString& marker : markers) {
        if (!line.startsWith(marker)) {
            continue;
        }
        const QString value = line.mid(marker.size()).trimmed();
        if (marker.startsWith(QStringLiteral("point cloud"))) {
            m_pointCloudPath = value;
        } else if (marker.startsWith(QStringLiteral("points"))) {
            m_pointCount = value.toInt();
        } else if (marker.startsWith(QStringLiteral("camera mode"))) {
            m_cameraMode = value;
        }
        return;
    }
    emit logMessage(line);
}

void ReconstructionController::handleFinished(int exitCode, QProcess::ExitStatus status)
{
    if (!m_running) {
        return;
    }
    m_pollTimer->stop();

    parseStdout();
    if (!m_stdoutBuffer.trimmed().isEmpty()) {
        handleStdoutLine(m_stdoutBuffer.trimmed());
        m_stdoutBuffer.clear();
    }
    // The final stage may have been written just before the process exited.
    pollColmapLog();

    m_running = false;
    if (m_cancelled) {
        if(m_engine=="structured_light") {
            QFile file(m_runDirectory+"/result.json");QJsonObject result;
            if(file.open(QIODevice::ReadOnly))result=QJsonDocument::fromJson(file.readAll()).object();
            file.close();result["status"]="cancelled";
            result["schema"]="paramcad.structured-light.result";result["version"]=1;result["unit"]="mm";
            QString error;capture::saveJson(m_runDirectory+"/result.json",result,error);
        }
        reportFailure(tr("重建已取消"));
        QFile::remove(m_runDirectory+".cancel");
        return;
    }
    if (exitCode != 0 || status == QProcess::CrashExit) {
        reportFailure(m_lastOutputLine.isEmpty()
                          ? tr("重建失败，退出码 %1").arg(exitCode)
                          : m_lastOutputLine);
        return;
    }

    if(m_engine=="structured_light") {
        QFile file(m_runDirectory+"/result.json");
        if(!file.open(QIODevice::ReadOnly)) {reportFailure(tr("缺少结构光重建结果清单"));return;}
        const auto result=QJsonDocument::fromJson(file.readAll()).object();
        if(result["schema"].toString()!="paramcad.structured-light.result"
            || result["status"].toString()!="complete" || result["point_count"].toInt()<=0
            || !QFileInfo(m_outputPath).isFile() || QFileInfo(m_outputPath).size()<=100) {
            reportFailure(tr("结构光重建未生成有效点云"));return;
        }
        m_pointCloudPath=m_outputPath;m_pointCount=result["point_count"].toInt();m_cameraMode="provided";
    }

    emit finished(m_pointCloudPath.isEmpty() ? m_outputPath : m_pointCloudPath,
                  m_pointCount, m_cameraMode, m_dense);
}

void ReconstructionController::reportFailure(const QString& message)
{
    m_lastError = message;
    emit failed(message);
}
