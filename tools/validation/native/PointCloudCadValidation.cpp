#include "PointCloudCadValidation.h"
#include "cli/OfflineCadCommands.h"
void validatePointCloudCad(const QStringList& args, QJsonObject& report) {
    reconstructOfflineCad(args, report, false);
}
void validateProjectionCad(const QStringList& args,QJsonObject& report){reconstructOfflineProjection(args,report);}
