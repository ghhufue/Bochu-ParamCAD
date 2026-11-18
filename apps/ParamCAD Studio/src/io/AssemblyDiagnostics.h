#pragma once
#include "io/PlyReader.h"
#include <reconstruction/parts/AssemblyReconstruction.h>
#include <QJsonObject>
QJsonObject writeAssemblyDiagnostics(const QString& reportPath,const PlyCloud& observations,
    const reconstruction::AssemblyReconstructionResult& result,const QString& unit);
QJsonObject projectionAxisDiagnostics(const reconstruction::ProjectionExtrusionResult& fit);
