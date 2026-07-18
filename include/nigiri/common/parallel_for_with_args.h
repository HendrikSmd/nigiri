#pragma once

#include <algorithm>
#include <thread>

#include "utl/parallel_for.h"

namespace nigiri {

template <typename ThreadLocal, typename Fun,
          typename ProgressUpdateFn = utl::noop_progress_update,
          typename... Args>
inline utl::errors_t parallel_for_run_threadlocal(
    size_t const job_count, size_t const num_threads, Fun func,
    ProgressUpdateFn&& progress_update = ProgressUpdateFn{},
    utl::parallel_error_strategy const err_strat =
        utl::parallel_error_strategy::QUIT_EXEC,
    Args&&... args) {
  utl::errors_t errors;
  std::mutex errors_mutex;
  std::atomic<size_t> counter(0);
  std::atomic<bool> quit{false};
  std::vector<std::thread> threads;
  for (auto i = 0u; i < num_threads; ++i) {
    threads.emplace_back([&, ...thread_local_args = std::forward<Args>(args)]() {
      auto threadlocal = ThreadLocal{thread_local_args...};

      while (!quit) {
        auto const idx = counter.fetch_add(1);
        if (idx >= job_count) {
          break;
        }

        try {
          func(threadlocal, idx);
          progress_update(idx);
        } catch (...) {
          std::lock_guard<std::mutex> lock{errors_mutex};
          errors.emplace_back(std::pair{i, std::current_exception()});
          if (err_strat == utl::parallel_error_strategy::QUIT_EXEC) {
            quit = true;
            break;
          }
        }
      }
    });
  }

  std::for_each(begin(threads), end(threads), [](auto& t) { t.join(); });

  if (err_strat == utl::parallel_error_strategy::QUIT_EXEC && !errors.empty()) {
    std::rethrow_exception(errors.front().second);
  }

  return errors;
}

template <typename ThreadLocal, typename Fun,
          typename ProgressUpdateFn = utl::noop_progress_update,
          typename... Args>
inline utl::errors_t parallel_for_run_threadlocal(
    size_t const job_count, Fun func,
    ProgressUpdateFn&& progress_update = ProgressUpdateFn{},
    utl::parallel_error_strategy const err_strat =
        utl::parallel_error_strategy::QUIT_EXEC,
    Args&&... args) {
  return parallel_for_run_threadlocal<ThreadLocal>(
      job_count, std::max(1U, std::thread::hardware_concurrency()), std::move(func),
      std::forward<ProgressUpdateFn>(progress_update), err_strat,
      std::forward<Args>(args)...);
}

}

