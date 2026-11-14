#pragma once
#include "capture/CaptureJob.h"

namespace capture {
struct ScanOptions {
    int cameraWidth=1280,cameraHeight=960,projectorWidth=512,projectorHeight=512;
    int stationCount=16;
    double baseline=0; // zero selects 20% of scene diagonal
    double distance=0; // zero selects a scene-fitting distance
    double lightIntensity=2.0; // projector radiance scale at the reference distance
    double fieldOfViewDegrees=35;
    bool phaseShift=true;
    bool multiExposure=false;
    QString renderBackend="vtk";
    int samplesPerPixel=32,maxPathDepth=4,randomSeed=0;
    Material material;
};

Matrix4 lookAt(const std::array<double,3>& position,const std::array<double,3>& target,
               const std::array<double,3>& up);
RenderJob planScan(const Geometry& geometry,const ScanOptions& options,
                   const QString& directory,const Matrix4* singleCameraPose=nullptr);
} // namespace capture
