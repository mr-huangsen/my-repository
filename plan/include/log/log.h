#ifndef COMMONLOG_MACROS_H
#define COMMONLOG_MACROS_H

#include "commonlog.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <type_traits>

// Initialize commonlog before using the logging macros.
// The format string and arguments follow fmt syntax, for example:
//   LOG_ONCE(spdlog::level::info, "logger started: {}", mode);
//   LOG_FIRST_N(spdlog::level::warn, 3, "retry: {}", retry);
//   LOG_EVERY_N(spdlog::level::debug, 100, "speed: {:.2f}", speed);
//   LOG_IF(spdlog::level::err, fault, "fault: {}", fault_code);
//   LOG_ON_CHANGE(spdlog::level::info, mode, "mode: {}", mode);
//
// State is independent at each macro expansion and shared by its threads.
// Only calls with an initialized logger and an enabled level advance state.
// Suppressed logs do not evaluate format arguments; gate arguments are
// evaluated once per enabled call. Logging keeps the caller's source location.
// ONCE emits once; FIRST_N emits at most N times (a lifetime quota).
// EVERY_N emits on calls 1, N+1, 2*N+1, ...; changing N restarts the period.
// N must be an integer other than bool. N <= 0 suppresses without advancing.
// ON_CHANGE emits initially and whenever the observed value differs from the
// previous enabled observation. It stores a copy and compares with operator==.
// Use std::string for string contents; raw pointers compare their addresses.
// Value types must be copy constructible and support equality comparison.
// Protect externally shared mutable values yourself when passing them in.
// The gates select logging attempts: an output failure still consumes a quota.

namespace commonlog_macro_detail {

template <typename Integer>
inline std::uint64_t positive_count(Integer count) {
  static_assert(std::is_integral<Integer>::value &&
                    !std::is_same<Integer, bool>::value,
                "Log count must be an integer other than bool");
  static_assert(sizeof(Integer) <= sizeof(std::uint64_t),
                "Log count must fit in uint64_t");
  if (count <= 0) {
    return 0;
  }
  return static_cast<std::uint64_t>(count);
}

class FirstNState {
public:
  bool take(std::uint64_t limit) {
    auto count = count_.load(std::memory_order_relaxed);
    while (count < limit) {
      if (count_.compare_exchange_weak(count, count + 1,
                                       std::memory_order_relaxed,
                                       std::memory_order_relaxed)) {
        return true;
      }
    }
    return false;
  }

private:
  // Saturates at the quota; incrementing cannot overflow even at UINT64_MAX.
  std::atomic<std::uint64_t> count_{0};
};

class EveryNState {
public:
  bool take(std::uint64_t period) {
    if (period == 0) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (period_ != period) {
      period_ = period;
      position_ = 0;
    }
    const bool emit = position_ == 0;
    position_ = position_ == period - 1 ? 0 : position_ + 1;
    return emit;
  }

private:
  std::mutex mutex_;
  std::uint64_t period_{0};
  std::uint64_t position_{0};
};

template <typename Value>
class ChangeState {
public:
  bool take(const Value &value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (previous_ && *previous_ == value) {
      return false;
    }
    // emplace requires copy construction, without requiring copy assignment.
    previous_.emplace(value);
    return true;
  }

private:
  std::mutex mutex_;
  std::optional<Value> previous_;
};

} // namespace commonlog_macro_detail

// Keep logger lookup, filtering and source location in the caller's scope.
// The predicate's lambda gives each expansion its own static state, including
// multiple expansions on the same source line. Do not move logging into it.
#define COMMONLOG_DETAIL_EMIT(level, predicate, ...)                           \
  do {                                                                         \
    auto commonlog_logger_ = ::spdlog::get("commonlog");                       \
    const auto commonlog_level_ = (level);                                     \
    if (commonlog_logger_ &&                                                   \
        commonlog_logger_->should_log(commonlog_level_) && (predicate)) {      \
      SPDLOG_LOGGER_CALL(commonlog_logger_, commonlog_level_,                  \
                         __VA_ARGS__);                                         \
    }                                                                          \
  } while (0)

#define LOG_DEBUG(...)                                                         \
  COMMONLOG_DETAIL_EMIT(::spdlog::level::debug, true, __VA_ARGS__)
#define LOG_INFO(...)                                                          \
  COMMONLOG_DETAIL_EMIT(::spdlog::level::info, true, __VA_ARGS__)
#define LOG_WARN(...)                                                          \
  COMMONLOG_DETAIL_EMIT(::spdlog::level::warn, true, __VA_ARGS__)
#define LOG_ERROR(...)                                                         \
  COMMONLOG_DETAIL_EMIT(::spdlog::level::err, true, __VA_ARGS__)
#define LOG_CRITICAL(...)                                                      \
  COMMONLOG_DETAIL_EMIT(::spdlog::level::critical, true, __VA_ARGS__)

#define LOG_ONCE(level, ...)                                                   \
  COMMONLOG_DETAIL_EMIT(                                                       \
      (level),                                                                 \
      ([&]() {                                                                 \
        static ::commonlog_macro_detail::FirstNState commonlog_state_;         \
        return commonlog_state_.take(1);                                       \
      }()),                                                                    \
      __VA_ARGS__)

#define LOG_FIRST_N(level, n, ...)                                             \
  COMMONLOG_DETAIL_EMIT(                                                       \
      (level),                                                                 \
      ([&]() {                                                                 \
        static ::commonlog_macro_detail::FirstNState commonlog_state_;         \
        return commonlog_state_.take(                                          \
            ::commonlog_macro_detail::positive_count((n)));                    \
      }()),                                                                    \
      __VA_ARGS__)

#define LOG_EVERY_N(level, n, ...)                                             \
  COMMONLOG_DETAIL_EMIT(                                                       \
      (level),                                                                 \
      ([&]() {                                                                 \
        static ::commonlog_macro_detail::EveryNState commonlog_state_;         \
        return commonlog_state_.take(                                          \
            ::commonlog_macro_detail::positive_count((n)));                    \
      }()),                                                                    \
      __VA_ARGS__)

#define LOG_IF(level, condition, ...)                                          \
  COMMONLOG_DETAIL_EMIT((level), static_cast<bool>(condition), __VA_ARGS__)

#define LOG_ON_CHANGE(level, value, ...)                                       \
  COMMONLOG_DETAIL_EMIT(                                                       \
      (level),                                                                 \
      ([&]() {                                                                 \
        using LogValue = ::std::decay_t<decltype((value))>;                    \
        static ::commonlog_macro_detail::ChangeState<LogValue>                 \
            commonlog_state_;                                                  \
        return commonlog_state_.take((value));                                 \
      }()),                                                                    \
      __VA_ARGS__)

#endif // COMMONLOG_MACROS_H
