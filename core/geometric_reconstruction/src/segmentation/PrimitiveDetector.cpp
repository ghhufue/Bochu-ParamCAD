#include "reconstruction/segmentation/PrimitiveDetector.h"

#include "reconstruction/segmentation/CgalEfficientRansacDetector.h"

namespace reconstruction {

bool detectPrimitiveEvidence(
    const PointStore& points,
    const PrimitiveDetectionOptions& options,
    PrimitiveDetectionResult& output,
    std::string& error) {
    return detectWithCgalEfficientRansac(points, options, output, error);
}

}  // namespace reconstruction
