#pragma once
#include "CommandProtocol.h"
#include <QLocalServer>
#include <QObject>

// Local, user-only command transport. The handler always runs on the GUI thread.
// A request can receive accepted followed by a terminal reply after UI commit.
class StudioCommandServer final : public QObject {
public:
    using Handler = std::function<void(const QJsonObject&, pcad::Reply)>;
    explicit StudioCommandServer(Handler handler,QObject* parent=nullptr);
    ~StudioCommandServer() override;
    bool start(QString& error);
    QString sessionId() const {return m_id;}
private:
    QLocalServer m_server;
    Handler m_handler;
    QString m_id,m_registration;
};
