#include <QJsonObject>
#include <QJsonDocument>
#include <QCryptographicHash>
#include <QFile>
#include <QSaveFile>
namespace capture {
bool saveJson(const QString& path,const QJsonObject& value,QString& error) {
    QSaveFile file(path);
    if(!file.open(QIODevice::WriteOnly) || file.write(QJsonDocument(value).toJson())<0 || !file.commit()) {
        error="Cannot write JSON: "+path;return false;
    }
    return true;
}
QString fileHash(const QString& path) {
    QFile file(path);if(!file.open(QIODevice::ReadOnly))return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if(!hash.addData(&file))return {};
    return QString::fromLatin1(hash.result().toHex());
}

}
