#include "reconstruction/pipeline/ParametricReconstruction.h"
#include "reconstruction/recognition/RecognitionAnalysisOptions.h"

namespace reconstruction {
bool reconstructParametricSolid(const PointStore &points, const PlaneDetectionOptions &planes,
                                const CylinderDetectionOptions &cylinders,
                                CompoundCandidate &output, std::string &error,
                                const std::vector<PrimitiveEvidence>& seeds) {
    output={};
    SurfaceAnalysisResult analysis;
    if (!analyzeSurfaces(points, recognitionAnalysisOptions(points, planes, cylinders), analysis, error, seeds)) return false;
    return recognizeAndRepairParametricSolid(points, analysis, output, error);
}

} // namespace reconstruction
