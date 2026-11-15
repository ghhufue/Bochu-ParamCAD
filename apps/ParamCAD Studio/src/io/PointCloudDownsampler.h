#pragma once
#include "io/PlyReader.h"

#include <QString>

namespace pointcloud {

// Merge compatible observations inside each voxel. Signed normals and local
// plane offsets keep distinct walls/edges separate. With no normals, merge by
// voxel alone. Positions are arithmetic means, normals are averaged
// and re-normalised, and colours are averaged per channel. Vertex rows are not
// carried over because they refer to the source observation order.
bool voxelDownsample(const PlyCloud& source, double voxelSize, PlyCloud& result,
                     QString& error);

} // namespace pointcloud
