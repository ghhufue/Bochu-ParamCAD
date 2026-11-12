#pragma once
#include "capture/RenderBackend.h"
#include <memory>

namespace capture {

class VtkProjectionBackend final : public RenderBackend {
    Q_OBJECT
public:
    explicit VtkProjectionBackend(QObject* parent = nullptr);
    ~VtkProjectionBackend() override;
    QJsonObject capabilities() const override;
    QString validate(const RenderJob& job) const override;
    void start(const RenderJob& job) override;
    void cancel() override;
private:
    struct State;
    std::unique_ptr<State> m_state;
    void nextFrame();
};
} // namespace capture
