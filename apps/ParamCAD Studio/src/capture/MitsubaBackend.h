#pragma once
#include "capture/RenderBackend.h"
#include <QProcess>

namespace capture {
// Process transport only; the same client coordinator owns plans and manifests.
class MitsubaBackend final : public RenderBackend {
    Q_OBJECT
public:
    MitsubaBackend(QString python, QStringList prefixArguments, QString worker,
                   int samples=32, int maxDepth=4, int seed=0, QObject* parent=nullptr);
    ~MitsubaBackend() override;
    QJsonObject capabilities() const override;
    QString validate(const RenderJob& job) const override;
    void start(const RenderJob& job) override;
    void cancel() override;
private:
    void readOutput();
    void finish(int exitCode, QProcess::ExitStatus status);
    QProcess m_process;
    QString m_python, m_worker, m_directory, m_error;
    QStringList m_prefix;
    QByteArray m_buffer, m_stderr;
    int m_samples, m_depth, m_seed, m_total=0, m_completed=0;
    bool m_running=false, m_cancelled=false, m_completeEvent=false;
};
}
