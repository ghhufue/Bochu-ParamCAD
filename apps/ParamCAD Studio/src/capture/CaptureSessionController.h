#pragma once
#include "capture/RenderBackend.h"
#include <QElapsedTimer>

namespace capture {

// Owns acquisition identity and commit semantics; the backend only renders.
class CaptureSessionController final : public QObject {
    Q_OBJECT
public:
    explicit CaptureSessionController(RenderBackend* backend,QObject* parent=nullptr);
    bool start(RenderJob job);
    bool setBackend(RenderBackend* backend); // Takes ownership on success, only while idle.
    void cancel();
    bool isRunning() const {return m_running;}
    QString lastError() const {return m_error;}
    QString directory() const {return m_job.directory;}
signals:
    void progress(int completed,int total);
    void completed(const QString& directory);
    void failed(const QString& error);
    void cancelled(const QString& directory);
private:
    void onFrame(int station,int exposure,int pattern,const QString& relativeImage);
    void onFinished(bool cancelled,const QString& error);
    bool saveManifest();
    void event(const QString& type,const QJsonObject& fields={});
    RenderBackend* m_backend;
    RenderJob m_job;
    std::vector<Frame> m_frames;
    QJsonObject m_manifest;
    QElapsedTimer m_elapsed;
    QString m_error;
    bool m_running=false;
    int m_count=0;
};

bool saveJson(const QString& path,const QJsonObject& value,QString& error);
QString fileHash(const QString& path);
} // namespace capture
