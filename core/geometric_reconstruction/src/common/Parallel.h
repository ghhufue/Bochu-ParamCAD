#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <thread>
#include <vector>

namespace reconstruction::detail {

inline std::size_t automaticWorkerCount() noexcept {
    return std::clamp<std::size_t>(std::thread::hardware_concurrency(), 1, 8);
}

template<class Function>
void runWorkers(std::size_t workers, Function function) {
    if (workers == 1) { function(0); return; }
    std::vector<std::exception_ptr> failures(workers);
    const auto run = [&](std::size_t worker) {
        try { function(worker); }
        catch (...) { failures[worker] = std::current_exception(); }
    };
    std::vector<std::thread> threads;
    threads.reserve(workers - 1);
    try {
        for (std::size_t worker = 1; worker < workers; ++worker) threads.emplace_back(run, worker);
    } catch (...) {
        for (auto& thread : threads) thread.join();
        throw;
    }
    run(0);
    for (auto& thread : threads) thread.join();
    for (const auto& failure : failures) if (failure) std::rethrow_exception(failure);
}

// Independent index ranges only. Callers keep reductions and dependency-driven
// algorithms ordered. Small inputs stay serial; auto mode caps memory/CPU use.
template<class Function>
void parallelRanges(std::size_t count, std::size_t requestedWorkers, Function function) {
    const auto desired = requestedWorkers ? std::min<std::size_t>(requestedWorkers, 32)
                                          : automaticWorkerCount();
    const auto workers = std::max<std::size_t>(1, std::min(desired, count / 4096));
    runWorkers(workers, [&](std::size_t worker) {
        function(count * worker / workers, count * (worker + 1) / workers);
    });
}

template<class Function>
void parallelTasks(std::size_t count, std::size_t requestedWorkers, Function function) {
    const auto desired = requestedWorkers ? std::min<std::size_t>(requestedWorkers, 32)
                                          : automaticWorkerCount();
    const auto workers = std::min(count, desired);
    if (workers < 2) {
        for (std::size_t index = 0; index < count; ++index) function(index);
        return;
    }
    std::atomic_size_t next{0};
    runWorkers(workers, [&](std::size_t) {
        while (true) {
            const auto index = next.fetch_add(1, std::memory_order_relaxed);
            if (index >= count) break;
            function(index);
        }
    });
}

} // namespace reconstruction::detail
