#pragma once
#include "io/AssemblyDiagnostics.h"
#include <QStringList>
// Offline parameter probes; uses the same observation-only assembly implementation.
void probeAssemblyBoundaries(const QStringList& args,QJsonObject& report);
