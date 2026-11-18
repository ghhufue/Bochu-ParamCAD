#include "reconstruction/recognition/RecognitionAnalysisOptions.h"
#include "reconstruction/settings/ModuleConfig.h"
#include <algorithm>

namespace reconstruction {
SurfaceAnalysisOptions recognitionAnalysisOptions(const PointStore&, const PlaneDetectionOptions& planes,
                                                 const CylinderDetectionOptions& cylinders,
                                                 const RecognitionAnalysisSettings& settings) {
    SurfaceAnalysisOptions o;
    const auto validationSettings=loadAnalysisSettings();o.supportValidation=validationSettings.supportValidation;o.supportMeshing=validationSettings.meshing;
    auto& d = o.detection;
    d.randomSeed = planes.randomSeed; d.plane = planes; d.cylinder = cylinders;
    d.plane.maximumPlanes = settings.maximumPlanes; d.cylinder.maximumCylinders = settings.maximumCylinders;
    // A small wall/cap is not less real because a much larger host is present.
    // Candidate validation, rather than scene-size quotas, rejects accidental fits.
    d.plane.minimumSupportPoints = std::min<std::size_t>(planes.minimumSupportPoints,settings.planeSupportCap);
    d.cylinder.minimumSupportPoints = std::min<std::size_t>(cylinders.minimumSupportPoints,settings.cylinderSupportCap);
    const auto minimum = std::max<std::size_t>(settings.curvedSupportFloor,
        std::min<std::size_t>(settings.curvedSupportCeiling,planes.minimumSupportPoints));
    d.sphere.minimumSupportPoints = d.cone.minimumSupportPoints = d.torus.minimumSupportPoints = minimum;
    d.sphere.distanceThreshold = d.cone.distanceThreshold = d.torus.distanceThreshold = planes.distanceThreshold;
    d.sphere.probability = d.cone.probability = d.torus.probability = settings.curvedProbability;
    d.sphere.maximumSpheres = d.cone.maximumCones = d.torus.maximumTori = settings.curvedMaximumCandidates;
    o.relations.distanceTolerance = planes.distanceThreshold*settings.relationDistanceToleranceMultiplier;
    return o;
}
} // namespace reconstruction
