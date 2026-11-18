#include "reconstruction/preprocessing/PointCloudOperations.h"
#include "detail/PreprocessingInternal.h"
#include <stdexcept>

namespace reconstruction {
std::vector<PointSample> samplePointCloudVoxels(const std::vector<PointSample>& points,
    double voxelSize,const std::unordered_set<PointId>& protectedIds) {
    if(!std::isfinite(voxelSize) || voxelSize<0)throw std::invalid_argument("Invalid voxel size");
    return detail::preprocessing::voxelDownsample(points,voxelSize,protectedIds);
}
void estimatePointCloudNormals(std::vector<PointSample>& points,
    const PointCloudPreprocessingOptions& options,PointCloudPreprocessingReport& report) {
    std::string error;
    if(!detail::preprocessing::validOptions(options,error))throw std::invalid_argument(error);
    detail::preprocessing::estimateNormalsAndCurvature(points,options,report);
}
} // namespace reconstruction
