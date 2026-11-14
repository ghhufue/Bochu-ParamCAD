#include "io/PointCloudDownsampleStore.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSaveFile>

#include <algorithm>

namespace pointcloud {
namespace {

const QRegularExpression& scaleSuffix()
{
    static const QRegularExpression pattern(
        QStringLiteral("^(.*)_([0-9]+(?:\\.[0-9]+)?)mm$"));
    return pattern;
}

bool parseCachedScale(const QFileInfo& info, double& scale)
{
    if (info.dir().dirName() != QStringLiteral("downsampled")) return false;
    const auto match = scaleSuffix().match(info.completeBaseName());
    if (!match.hasMatch()) return false;
    bool ok = false;
    const double value = match.captured(2).toDouble(&ok);
    if (!ok || !(value > 0.0)) return false;
    scale = value;
    return true;
}

} // namespace

QString formatVoxelSize(double voxelSize)
{
    return QString::number(voxelSize, 'g', 12);
}

QString downsampleDirectory(const QString& sourcePath)
{
    return QDir(QFileInfo(sourcePath).absolutePath()).filePath(QStringLiteral("downsampled"));
}

QString downsamplePath(const QString& sourcePath, double voxelSize)
{
    const QFileInfo source(sourcePath);
    return QDir(downsampleDirectory(sourcePath))
        .filePath(QStringLiteral("%1_%2mm.ply")
                      .arg(source.completeBaseName(), formatVoxelSize(voxelSize)));
}

bool isDownsampledCloud(const QString& path, double* voxelSize)
{
    double scale = 0.0;
    if (!parseCachedScale(QFileInfo(path), scale)) return false;
    if (voxelSize) *voxelSize = scale;
    return true;
}

QString rootCloudPath(const QString& path)
{
    double scale = 0.0;
    const QFileInfo cached(path);
    if (!parseCachedScale(cached, scale)) return path;
    const auto match = scaleSuffix().match(cached.completeBaseName());
    const QString base = match.captured(1);
    QDir sourceDirectory(cached.dir().absolutePath());
    sourceDirectory.cdUp();
    const QString candidate = sourceDirectory.filePath(base + QStringLiteral(".") + cached.suffix());
    if (QFileInfo::exists(candidate)) return candidate;
    for (const auto& suffix : {QStringLiteral("ply"), QStringLiteral("PLY")}) {
        const QString alternative = sourceDirectory.filePath(base + QStringLiteral(".") + suffix);
        if (QFileInfo::exists(alternative)) return alternative;
    }
    return candidate;
}

QVector<double> existingScales(const QString& sourcePath)
{
    QVector<double> scales;
    const QDir directory(downsampleDirectory(sourcePath));
    const QString prefix = QFileInfo(sourcePath).completeBaseName() + QStringLiteral("_");
    for (const auto& entry : directory.entryInfoList({QStringLiteral("*.ply")}, QDir::Files)) {
        double scale = 0.0;
        if (!parseCachedScale(entry, scale)) continue;
        if (!entry.completeBaseName().startsWith(prefix)) continue;
        if (!scales.contains(scale)) scales.push_back(scale);
    }
    std::sort(scales.begin(), scales.end());
    return scales;
}

bool writeUnitSidecar(const QString& cloudPath, const QString& unit, double voxelSize,
                      QString& error)
{
    const QFileInfo cloud(cloudPath);
    QJsonObject metadata;
    metadata.insert(QStringLiteral("unit"), unit.isEmpty() ? QStringLiteral("arbitrary") : unit);
    metadata.insert(QStringLiteral("voxel_size_mm"), voxelSize);
    metadata.insert(QStringLiteral("source"), QFileInfo(rootCloudPath(cloudPath)).fileName());
    const QString path =
        cloud.dir().filePath(cloud.completeBaseName() + QStringLiteral(".json"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        error = file.errorString();
        return false;
    }
    const QByteArray payload = QJsonDocument(metadata).toJson(QJsonDocument::Indented);
    if (file.write(payload) != payload.size()) {
        error = file.errorString();
        return false;
    }
    if (!file.commit()) {
        error = file.errorString();
        return false;
    }
    return true;
}

} // namespace pointcloud
