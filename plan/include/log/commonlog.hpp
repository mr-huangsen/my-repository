#pragma once

#include "commonlog_detail.hpp"

#include <condition_variable>
#include <thread>
#include <vector>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

class commonlog {
public:
  struct Options {
    std::chrono::milliseconds rotation_interval{std::chrono::minutes(5)};
    std::chrono::milliseconds flush_interval{std::chrono::seconds(1)};
    bool with_console{true};
  };

  struct Status {
    std::uint64_t io_errors{0};
    std::uint64_t seal_errors{0};
    std::string last_error;
  };

  static commonlog &get() {
    static commonlog instance;
    return instance;
  }

  bool init(const std::string &directory, const std::string &filename) {
    return init(directory, filename, Options{});
  }

  bool init(const std::string &directory, const std::string &filename,
            const Options &options) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      commonlog_detail::require(options.rotation_interval.count() > 0 &&
                                    options.flush_interval.count() > 0,
                                "log intervals must be positive");
      const auto path = commonlog_detail::normalize_directory(directory);
      const auto registered = spdlog::get(logger_name_);
      if (session_) {
        commonlog_detail::require(
            session_->matches(path, filename, options),
            "logger already initialized with different settings");
        if (registered != session_->logger)
          spdlog::register_logger(session_->logger);
        return true;
      }
      commonlog_detail::require(!registered,
                                "logger name commonlog is already in use");
      auto session = std::make_unique<Session>(path, filename, options, errors_);
      session->start();
      spdlog::register_logger(session->logger);
      session_ = std::move(session);
      return true;
    } catch (const std::exception &error) {
      errors_->record(error.what());
      return false;
    }
  }

  void shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_ && spdlog::get(logger_name_) == session_->logger)
      spdlog::drop(logger_name_);
    session_.reset();
  }

  Status status() const {
    std::lock_guard<std::mutex> lock(errors_->mutex);
    return errors_->status;
  }

private:
  static constexpr auto logger_name_ = "commonlog";

  struct Errors {
    void record(const std::string &message, bool sealing = false) {
      std::lock_guard<std::mutex> lock(mutex);
      ++status.io_errors;
      if (sealing)
        ++status.seal_errors;
      status.last_error = message;
    }

    std::mutex mutex;
    Status status;
  };

  struct Session {
    Session(const std::filesystem::path &directory, const std::string &filename,
            const Options &options, std::shared_ptr<Errors> errors)
        : directory(directory), filename(filename), options(options),
          errors(std::move(errors)), file(directory, filename, logger_name_) {
      std::vector<spdlog::sink_ptr> sinks{file.sink()};
      sinks.front()->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
      if (options.with_console) {
        auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v");
        sinks.push_back(std::move(console));
      }
      logger = std::make_shared<spdlog::logger>(logger_name_, sinks.begin(),
                                                sinks.end());
      logger->set_level(spdlog::level::info);
      logger->flush_on(spdlog::level::err);
      logger->set_error_handler(
          [errors = this->errors](const std::string &message) {
            errors->record(message);
          });
    }

    ~Session() {
      if (worker.joinable()) {
        {
          std::lock_guard<std::mutex> lock(wait_mutex);
          stopping = true;
        }
        wake.notify_one();
        worker.join();
      }
      try {
        file.close();
      } catch (const std::exception &error) {
        errors->record(error.what(), true);
      }
    }

    bool matches(const std::filesystem::path &path, const std::string &name,
                 const Options &other) const {
      return directory == path && filename == name &&
             options.rotation_interval == other.rotation_interval &&
             options.flush_interval == other.flush_interval &&
             options.with_console == other.with_console;
    }

    void start() {
      file.rotate();
      worker = std::thread([this] { run(); });
    }

    void run() {
      using Clock = std::chrono::steady_clock;
      const auto started = Clock::now();
      auto rotation = started + options.rotation_interval;
      auto flush = started + options.flush_interval;
      std::unique_lock<std::mutex> lock(wait_mutex);
      while (!wake.wait_until(lock, std::min(rotation, flush),
                              [this] { return stopping; })) {
        lock.unlock();
        const bool rotating = Clock::now() >= rotation;
        try {
          if (rotating)
            file.rotate();
          else
            file.sink()->flush();
        } catch (const std::exception &error) {
          errors->record(error.what(), rotating);
        }
        const auto now = Clock::now();
        if (rotating)
          rotation = now + options.rotation_interval;
        flush = now + options.flush_interval;
        lock.lock();
      }
    }

    std::filesystem::path directory;
    std::string filename;
    Options options;
    std::shared_ptr<Errors> errors;
    commonlog_detail::TimedFile file;
    std::shared_ptr<spdlog::logger> logger;
    std::mutex wait_mutex;
    std::condition_variable wake;
    std::thread worker;
    bool stopping{false};
  };

  commonlog() {
    // Initialize the registry first so it outlives this controller at exit.
    (void)spdlog::get(logger_name_);
  }
  ~commonlog() { shutdown(); }

  mutable std::mutex mutex_;
  std::shared_ptr<Errors> errors_{std::make_shared<Errors>()};
  std::unique_ptr<Session> session_;
};
