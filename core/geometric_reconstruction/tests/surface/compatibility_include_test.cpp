// Existing consumers can keep the historical includes while migrating gradually.
#include "reconstruction/analysis/SurfaceAnalysis.h"
#include "reconstruction/analysis/SurfaceSupportValidation.h"
#include "reconstruction/analysis/SurfaceFootprint.h"
#include "reconstruction/detection/PrimitiveDetector.h"
#include "reconstruction/detection/PlaneSupportRecovery.h"
#include "reconstruction/detection/MultiScaleDetector.h"
#include "reconstruction/recognition/AssemblyReconstruction.h"
#include "reconstruction/recognition/ProjectionExtrusion.h"
#include "reconstruction/recognition/ProjectionSettings.h"
#include "reconstruction/recognition/GuidedSolidRecognizer.h"
#include "reconstruction/data/ReconstructionCandidate.h"
#include "reconstruction/surface/SurfaceAnalysis.h"
#include "reconstruction/recognition/RecognitionAnalysisOptions.h"

int main() {
    const auto options=reconstruction::recognitionAnalysisOptions(
        reconstruction::PointStore({}),reconstruction::PlaneDetectionOptions{},
        reconstruction::CylinderDetectionOptions{});
    return options.detection.plane.maximumPlanes>0 ? 0 : 1;
}
