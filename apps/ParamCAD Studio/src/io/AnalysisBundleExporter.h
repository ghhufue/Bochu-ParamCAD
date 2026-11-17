#pragma once

#include "rendering/PointCloudSurfaceMesh.h"
#include <QJsonObject>
#include <QString>
#include <functional>
#include <set>

// A value snapshot: a background export cannot observe later document edits.
struct AnalysisBundleSnapshot
{
    PlyCloud cloud;
    std::vector<unsigned char> deleted;
    std::vector<int> memberships;
    std::vector<PointCloudRegion> regions;
    std::vector<PointCloudRelation> relations;
    std::vector<vtkIdType> selected;
    PointCloudSurfaceDisplay surfaces;
    std::set<int> displayedSurfaces;
    QString sourceName;
    QString sourceUnit;
    QString positionsFingerprint;
    quint64 documentRevision = 0;
    QJsonObject view;
    QJsonObject supportValidation;
};

// The destination must not already exist. A staged directory is published only
// after all files are complete; cancellation leaves no partial result there.
bool exportAnalysisBundle(const AnalysisBundleSnapshot& snapshot,const QString& directory,
                          QString& error,const std::function<bool()>& cancelled = {});

