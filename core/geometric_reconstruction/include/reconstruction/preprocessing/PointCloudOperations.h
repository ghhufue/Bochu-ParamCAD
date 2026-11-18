#pragma once
#include "reconstruction/preprocessing/PointCloudPreprocessor.h"
#include <unordered_set>

namespace reconstruction {
// Focused operations for part/projection fitting. They do not rescale units,
// load settings, remove statistical outliers, or renumber source observations.
std::vector<PointSample> samplePointCloudVoxels(const std::vector<PointSample>& points,
    double voxelSize, const std::unordered_set<PointId>& protectedIds = {});
void estimatePointCloudNormals(std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options, PointCloudPreprocessingReport& report);
} // namespace reconstruction
