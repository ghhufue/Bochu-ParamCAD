#pragma once

#include <QImage>
#include <QJsonArray>
#include <QJsonObject>
#include <QString>

#include <array>
#include <cstdint>
#include <vector>

namespace capture {

using Matrix4 = std::array<double, 16>; // row-major device-to-world/rig; +X right, +Y down, +Z forward
Matrix4 identity();
Matrix4 multiply(const Matrix4& a, const Matrix4& b);

struct Geometry {
    std::vector<std::array<double, 3>> positions;
    std::vector<std::array<double, 3>> normals;
    std::vector<std::array<double, 2>> uv;
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

struct Material {
    std::array<double, 3> baseColor{0.45, 0.45, 0.45}; // linear RGB
    double metallic = 0.9;
    double roughness = 0.4;
    QString baseColorMap; // sRGB; data maps below are linear
    QString metallicMap;
    QString roughnessMap;
};

struct Device {
    int width = 256;
    int height = 256;
    std::array<double, 9> K{256, 0, 128, 0, 256, 128, 0, 0, 1};
    Matrix4 deviceToRig = identity();
};

struct Pattern {
    QString id;
    QString kind;
    QString axis;
    int bit = -1;
    bool inverted = false;
    double phase = 0;
    QJsonObject json() const;
};

struct Station {
    QString id;
    Matrix4 rigToWorld = identity();
};

struct Frame {
    int station = 0;
    int exposure = 0;
    int pattern = 0;
    QString image;
};

// Immutable, renderer-neutral input. No vtkActor, GLSL or engine scene objects.
struct RenderJob {
    QString id;
    QString directory;
    Geometry geometry;
    Material material;
    Device camera;
    Device projector;
    std::vector<Station> stations;
    std::vector<Pattern> patterns;
    std::vector<double> exposures{1.0};
    int phaseSteps = 0;
    double phasePeriod = 16;
    double lightIntensity = 2.0; // irradiance scale at referenceDistance
    double referenceDistance = 100;
    double ambient = 0.015;
    double nearPlane = 0.01;
    double farPlane = 10000;

    std::vector<Frame> frames() const;
    QString validate() const;
    QJsonObject calibrationJson() const;
    QJsonObject sceneJson() const;
};

std::vector<Pattern> grayPatterns(int width, int height, int phaseSteps = 0);
QImage patternImage(const Pattern& pattern, int width, int height, double period);
QJsonArray matrixJson(const Matrix4& matrix);
QJsonObject deviceJson(const Device& device);

} // namespace capture
