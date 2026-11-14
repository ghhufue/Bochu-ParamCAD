#pragma once
#include "io/PlyReader.h"

#include <QString>

namespace pointcloud {

// Merge every observation that falls into the same voxel into one point. The
// position is the arithmetic mean of the voxel's members, normals are averaged
// and re-normalised, and colours are averaged per channel. Vertex rows are not
// carried over because they refer to the source observation order.
bool voxelDownsample(const PlyCloud& source, double voxelSize, PlyCloud& result,
                     QString& error);

} // namespace pointcloud
