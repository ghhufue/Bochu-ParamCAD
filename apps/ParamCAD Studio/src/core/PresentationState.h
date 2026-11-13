#pragma once
#include <QColor>
#include <QString>
#include <array>

struct ObjectPresentation
{
    std::array<double, 3> position{0, 0, 0}, rotation{0, 0, 0}, scale{1, 1, 1};
    bool visible = true;
    int representation = 0;
    double opacity = 1;
    QColor color{71, 145, 214};
    // Physical surface used both by the viewport preview and by the
    // structured-light capture, so a material is set once and then seen.
    double metallic = 0.9;
    double roughness = 0.4;
    bool materialPreview = true;
};
struct RenderPresentation
{
    QColor background{26, 28, 33};
    QColor background2{43, 46, 54};
    bool gradientBackground = true;
    bool lighting = true, textureEnabled = false;
    int width = 1920, height = 1080, textureProjection = 0;
    QString texturePath;
};
