#pragma once

#include "core/PointCloudSelection.h"
#include <reconstruction/recognition/GuidedSolidRecognizer.h>

struct PointCloudAnalysisResult
{
    std::vector<DetectedPointCloudSurface> surfaces;
    reconstruction::SurfaceAnalysisResult analysis;
    reconstruction::SurfaceSupportValidationReport sourceSupportValidation, finalSupportValidation;
    QString error;
    size_t processedPoints = 0;
    size_t assignedPoints = 0;
    size_t preferredPlaneCount = 0;
};

// Runs off the UI thread. Detector point IDs refer to the input observations;
// projection to all original observations recovers downsampled support without
// confusing representative indices with source indices.
PointCloudAnalysisResult analysePointCloud(const PlyCloud& cloud,
    const std::vector<vtkIdType>& originalIds, double distanceTolerance,
    const QString& geometry = QString(),
    reconstruction::LengthUnit unit = reconstruction::LengthUnit::Arbitrary);
PointCloudRegion describeTemplateFit(const reconstruction::TemplateFit& fit, double scaleToMillimeters);
// Display the retained analysis without rerunning detection after CAD failure.
std::vector<DetectedPointCloudSurface> describeAnalysisPatches(
    const reconstruction::SurfaceAnalysisResult& analysis, std::size_t sourcePointCount,
    double scaleToMillimeters);
