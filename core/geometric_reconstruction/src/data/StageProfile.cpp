#include "reconstruction/data/StageProfile.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>

#ifdef _WIN32
#include <windows.h>
#endif

namespace reconstruction::diagnostics {
namespace {
using Clock = std::chrono::steady_clock;

thread_local unsigned depth = 0;

#ifdef _WIN32
double seconds(FILETIME time) noexcept {
    ULARGE_INTEGER ticks;
    ticks.LowPart = time.dwLowDateTime;
    ticks.HighPart = time.dwHighDateTime;
    return double(ticks.QuadPart) / 10000000.0;
}
#endif

double processCpuSeconds() noexcept {
#ifdef _WIN32
    FILETIME created{}, exited{}, kernel{}, user{};
    if (GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user))
        return seconds(kernel) + seconds(user);
#else
    const auto ticks = std::clock();
    if (ticks != std::clock_t(-1)) return double(ticks) / CLOCKS_PER_SEC;
#endif
    return -1;
}

double threadCpuSeconds() noexcept {
#ifdef _WIN32
    FILETIME created{}, exited{}, kernel{}, user{};
    if (GetThreadTimes(GetCurrentThread(), &created, &exited, &kernel, &user))
        return seconds(kernel) + seconds(user);
#endif
    return -1;
}

double elapsedCpu(double start, double end) noexcept {
    return start >= 0 && end >= start ? end - start : -1;
}

const Clock::time_point& epoch() noexcept {
    static const auto start = Clock::now();
    return start;
}
}

StageProfile::StageProfile(const char* stage, std::size_t inputCount) noexcept {
    if (!stage) return;
    static const bool enabled = [] {
#ifdef _WIN32
        char value[2]{};
        return GetEnvironmentVariableA("PCAD_CAD_PROFILE", value, sizeof(value)) == 1 && value[0] == '1';
#else
        const char* value = std::getenv("PCAD_CAD_PROFILE");
        return value && value[0] == '1' && value[1] == '\0';
#endif
    }();
    if (!enabled) return;
    stage_ = stage;
    inputCount_ = inputCount;
    const auto start = epoch();
    started_ = Clock::now();
    processCpu_ = processCpuSeconds();
    threadCpu_ = threadCpuSeconds();
    std::fprintf(stderr, "PCAD_PROFILE begin stage=%s depth=%u points=%zu t=%.6f\n",
        stage_, depth++, inputCount_, std::chrono::duration<double>(started_ - start).count());
    std::fflush(stderr);
}

StageProfile::~StageProfile() noexcept {
    if (!stage_) return;
    const auto end = Clock::now();
    const double cpu = elapsedCpu(processCpu_, processCpuSeconds());
    const double threadCpu = elapsedCpu(threadCpu_, threadCpuSeconds());
    std::fprintf(stderr,
        "PCAD_PROFILE end stage=%s depth=%u points=%zu t=%.6f wall=%.6f cpu=%.6f thread_cpu=%.6f\n",
        stage_, --depth, inputCount_, std::chrono::duration<double>(end - epoch()).count(),
        std::chrono::duration<double>(end - started_).count(), cpu, threadCpu);
    std::fflush(stderr);
}

} // namespace reconstruction::diagnostics
