#pragma once

#include <algorithm>
#include <cstddef>
#include <exception>
#include <thread>
#include <vector>

namespace reconstruction::detail {

// Independent index ranges only. Callers keep reductions and dependency-driven
// algorithms ordered. Small inputs stay serial; auto mode caps memory/CPU use.
template<class Function>
void parallelRanges(std::size_t count, std::size_t requestedWorkers, Function function) {
    const auto available = std::max(1U, std::thread::hardware_concurrency());
    const auto desired = requestedWorkers ? std::min<std::size_t>(requestedWorkers, 32)
                                         : std::min<std::size_t>(available, 8);
    const auto workers = std::max<std::size_t>(1, std::min(desired, count / 4096));
    if (workers == 1) { function(0, count); return; }
    std::vector<std::exception_ptr> failures(workers);
    const auto run = [&](std::size_t worker) {
        try { function(count * worker / workers, count * (worker + 1) / workers); }
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

} // namespace reconstruction::detail
