#pragma once
#include <QString>
#include <QVector>

namespace pointcloud {

// Downsampled clouds live next to their source:
//   <source directory>/downsampled/<source base>_<spacing>mm.ply
// The spacing is part of the name so every scale is cached independently and a
// repeated request can be served without recomputing anything.
QString downsampleDirectory(const QString& sourcePath);
QString downsamplePath(const QString& sourcePath, double voxelSize);
QString formatVoxelSize(double voxelSize);

// True when the path is one of those cached files; the spacing is parsed back
// from the file name.
bool isDownsampledCloud(const QString& path, double* voxelSize = nullptr);

// Returns the original cloud a cached file was derived from, or the path itself
// when it is not a cached file. The candidate is returned even if it is missing
// so callers can decide whether to fall back.
QString rootCloudPath(const QString& path);

// Existing cached spacings for this source, ascending and de-duplicated.
QVector<double> existingScales(const QString& sourcePath);

// Unit metadata is stored beside the cloud because the document derives the
// unit from sidecars, not from PLY comments.
bool writeUnitSidecar(const QString& cloudPath, const QString& unit, double voxelSize,
                      QString& error);

} // namespace pointcloud
