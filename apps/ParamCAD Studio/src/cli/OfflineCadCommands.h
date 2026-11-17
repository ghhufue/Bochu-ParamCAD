#pragma once
#include <QJsonObject>
#include <QStringList>
// Reuses CadReconstructionTask and the core modeling/export path, without widgets.
void reconstructOfflineCad(const QStringList& arguments,QJsonObject& report);
void reconstructOfflineProjection(const QStringList& arguments,QJsonObject& report);
