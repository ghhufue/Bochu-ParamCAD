#pragma once
#include <QJsonObject>
#include <QStringList>
#include <functional>

namespace pcad {
constexpr int protocolVersion = 1;
constexpr qint64 maximumMessageBytes = 1024 * 1024;
using Reply = std::function<void(QJsonObject)>;
struct Capability { QString command; bool online; bool offline; QString description; };
QList<Capability> capabilities();
QJsonObject capabilityDocument();
QString sessionDirectory();
QString processToken(qint64 processId);
QJsonObject response(const QString& status, const QString& error = {}, int exitCode = 0);
}
