#include "StudioCommandServer.h"
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QLocalSocket>
#include <QPointer>
#include <QSaveFile>
#include <QTimer>
#include <QUuid>
#include <memory>

StudioCommandServer::StudioCommandServer(Handler handler,QObject* parent)
    :QObject(parent),m_handler(std::move(handler)),m_id(QUuid::createUuid().toString(QUuid::WithoutBraces)) {
    m_server.setSocketOptions(QLocalServer::UserAccessOption);
    connect(&m_server,&QLocalServer::newConnection,this,[this] {
        while(auto* socket=m_server.nextPendingConnection()) {
            auto buffer=std::make_shared<QByteArray>();auto consumed=std::make_shared<bool>(false);
            connect(socket,&QLocalSocket::disconnected,socket,&QObject::deleteLater);
            auto* deadline=new QTimer(socket);deadline->setSingleShot(true);deadline->start(5000);
            connect(deadline,&QTimer::timeout,socket,[socket,consumed] {if(!*consumed)socket->disconnectFromServer();});
            connect(socket,&QLocalSocket::readyRead,this,[this,socket,buffer,consumed,deadline] {
                if(*consumed)return;
                buffer->append(socket->readAll());
                if(buffer->size()>pcad::maximumMessageBytes){*consumed=true;socket->disconnectFromServer();return;}
                const auto end=buffer->indexOf('\n');if(end<0)return;
                *consumed=true;deadline->stop();
                QJsonParseError parse;const auto doc=QJsonDocument::fromJson(buffer->left(end),&parse);
                const auto request=doc.object();const QString id=request["id"].toString();
                QPointer<QLocalSocket> guard(socket);
                const auto reply=[guard,id](QJsonObject value) {
                    if(!guard || guard->state()!=QLocalSocket::ConnectedState)return;
                    value["id"]=id;value["mode"]="online";
                    guard->write(QJsonDocument(value).toJson(QJsonDocument::Compact)+'\n');guard->flush();
                    if(value["status"]!="accepted")guard->disconnectFromServer();
                };
                if(parse.error!=QJsonParseError::NoError || !doc.isObject() || id.isEmpty()
                    || request["schema"]!="paramcad.cli.request" || request["version"].toInt()!=pcad::protocolVersion) {
                    reply(pcad::response("failed","Invalid CLI protocol request",2));return;
                }
                if(request["command"]=="ping") {
                    auto result=pcad::response("complete");result["session"]=m_id;
                    result["executable"]=QCoreApplication::applicationFilePath();reply(result);return;
                }
                try {m_handler(request,reply);}
                catch(const std::exception& error){reply(pcad::response("failed",QString::fromUtf8(error.what()),2));}
            });
        }
    });
}
bool StudioCommandServer::start(QString& error) {
    const QString endpoint="paramcad-studio-"+m_id;
    if(!m_server.listen(endpoint)){error=m_server.errorString();return false;}
    QDir directory(pcad::sessionDirectory());
    if(!QDir().mkpath(directory.absolutePath())){error="Cannot create CLI session directory";m_server.close();return false;}
    m_registration=directory.filePath(m_id+".json");
    QSaveFile file(m_registration);
    const auto bytes=QJsonDocument(QJsonObject{{"version",pcad::protocolVersion},{"session",m_id},
        {"endpoint",endpoint},{"pid",QCoreApplication::applicationPid()},
        {"process_token",pcad::processToken(QCoreApplication::applicationPid())},
        {"executable",QCoreApplication::applicationFilePath()},
        {"started_utc",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)}}).toJson();
    if(!file.open(QIODevice::WriteOnly)||file.write(bytes)!=bytes.size()||!file.commit()) {
        error="Cannot register CLI session";m_server.close();return false;
    }
    return true;
}
StudioCommandServer::~StudioCommandServer(){m_server.close();if(!m_registration.isEmpty())QFile::remove(m_registration);}
