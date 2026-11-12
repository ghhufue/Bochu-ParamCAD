#pragma once
#include "capture/CaptureJob.h"

namespace capture {
// Post-reconstruction evaluation only. This reference geometry is never passed
// to the image decoder or used to supply/fill reconstructed points.
QJsonObject evaluateSurface(const Geometry& reference,const std::vector<std::array<double,3>>& points,
    const QJsonObject& acquisition,const QJsonObject& calibration,double toleranceMm=.5,int samples=20000);
}
