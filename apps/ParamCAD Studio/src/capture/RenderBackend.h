#pragma once

#include "capture/CaptureJob.h"
#include <QObject>

namespace capture {

// A logical asynchronous interface: VTK renders on its owning GUI thread,
// while a future Mitsuba adapter can use QProcess without changing callers.
class RenderBackend : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    virtual QJsonObject capabilities() const = 0;
    virtual QString validate(const RenderJob& job) const = 0;
    virtual void start(const RenderJob& job) = 0;
    virtual void cancel() = 0;
signals:
    void frameCompleted(int station, int exposure, int pattern, const QString& relativeImage);
    void progress(int completed, int total);
    void finished(bool cancelled, const QString& error);
};

} // namespace capture
