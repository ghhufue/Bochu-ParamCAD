#pragma once
#include "reconstruction/data/PrimitiveEvidence.h"
#include "reconstruction/settings/SurfaceMeshingSettings.h"
#include <string>
#include <memory>

namespace reconstruction {
struct SurfaceFootprint;
enum class SupportValidationMode { Off=0, Report=1, Enforce=2 };
enum class SupportVerdict { Accepted, Rejected, NeedsMoreSupport };
struct SurfaceSupportValidationOptions {
    SupportValidationMode mode=SupportValidationMode::Enforce;
    double planeDensityRatioMax=4;
    double sphereMinSolidAngle=0.02;
    double sphereMinMinorSpanRadians=0.17453292519943295;
    double cylinderMinAngularSpanRadians=0.3490658503988659;
    double minimumSagittaNoiseRatio=3;
    double referenceMinConfidence=0.7;
    std::size_t referenceMinPoints=200, referenceMinPlanes=3;
    double referenceAreaQuantile=0.75, referenceMaxLogSpread=0.5;
    double minimumExpectedPoints=12, gridSpacingMultiplier=2, minimumOccupiedCellRatio=0.5;
    double noiseFloorToleranceFraction=0.1, modelComplexityWeight=1, minimumModelGain=0.05;
    double planePreferenceMinReferenceAreaFraction=0.1;
};
struct SurfaceSamplingReference {
    bool available=false;
    double density=0, spacing=0, logSpread=0, noise=0;
    std::string reason="MissingDensityReference";
    std::vector<std::uint64_t> patchIds;
};
struct SurfaceSupportDecision {
    std::uint64_t proposalId=0, patchId=0;
    PrimitiveKind kind=PrimitiveKind::Plane;
    SupportVerdict verdict=SupportVerdict::Accepted;
    bool densityEvaluated=false, occupancyEvaluated=false, protectedManual=false;
    std::size_t pointCount=0, coveredPoints=0, effectivePoints=0, finalCoveredPoints=0;
    double area=0, finalArea=0, density=0, densityRatio=0, occupiedCellRatio=0;
    double solidAngle=0, minorSpanRadians=0, angularSpanRadians=0;
    double sagitta=0, noise=0, planeCost=0, curvedCost=0;
    std::string noiseSource;
    std::string parametersJson="{}";
    std::shared_ptr<const SurfaceFootprint> footprint;
    std::vector<std::string> reasons;
    // Original observations, retained for source-resolution revalidation.
    std::vector<PointId> supportPointIds;
};
struct SurfaceSupportValidationReport {
    static constexpr unsigned version=1;
    std::uint64_t pointFingerprint=0;
    LengthUnit unit=LengthUnit::Arbitrary;
    SupportValidationMode mode=SupportValidationMode::Off;
    SurfaceSupportValidationOptions options;
    std::vector<double> settingsSignature;
    SurfaceSamplingReference reference;
    std::vector<SurfaceSupportDecision> decisions;
};
const char* supportVerdictName(SupportVerdict verdict) noexcept;
bool validSurfaceSupportOptions(const SurfaceSupportValidationOptions& options) noexcept;
std::string serializeSurfaceSupportReport(const SurfaceSupportValidationReport& report);
} // namespace reconstruction
