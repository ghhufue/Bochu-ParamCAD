#pragma once
#include <QJsonObject>
#include <QStringList>
#include <reconstruction/parts/PlanarSurfaceJunctions.h>

struct AnalysisBundlePlaneEntry {
    int id;
    QString name;
};

struct AnalysisBundlePlane {
    reconstruction::PlanarSurfaceInput surface;
    // Only loaded if the bundle has no saved mesh for this surface.
    std::vector<modeling::Vec3> observations;
};

std::vector<AnalysisBundlePlaneEntry> listAnalysisBundlePlanes(const QString& directory);
std::vector<AnalysisBundlePlane> loadAnalysisBundlePlanes(const QString& directory,
    const std::vector<int>& ids);
QJsonObject inspectAnalysisBundle(const QStringList& arguments);
