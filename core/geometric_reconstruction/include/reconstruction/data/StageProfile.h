#pragma once

#include <chrono>
#include <cstddef>
#include <utility>

namespace reconstruction::diagnostics {

class StageProfile {
public:
    explicit StageProfile(const char* stage, std::size_t inputCount = 0) noexcept;
    ~StageProfile() noexcept;
    StageProfile(const StageProfile&) = delete;
    StageProfile& operator=(const StageProfile&) = delete;

private:
    const char* stage_ = nullptr;
    std::size_t inputCount_ = 0;
    std::chrono::steady_clock::time_point started_;
    double processCpu_ = 0;
    double threadCpu_ = 0;
};

template<class Work>
decltype(auto) profileStage(const char* stage, Work&& work) {
    const StageProfile profile(stage);
    return std::forward<Work>(work)();
}

} // namespace reconstruction::diagnostics
