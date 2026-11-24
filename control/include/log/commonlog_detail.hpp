#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include <fcntl.h>
#include <linux/fs.h>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <spdlog/sinks/basic_file_sink.h>

namespace commonlog_detail {

inline void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

inline void checked(long result, const char *operation) {
  if (result < 0)
    throw std::system_error(errno, std::generic_category(), operation);
}

class Descriptor {
public:
  Descriptor() = default;
  ~Descriptor() { reset(); }
  Descriptor(const Descriptor &) = delete;
  Descriptor &operator=(const Descriptor &) = delete;

  void reset() noexcept {
    if (value >= 0)
      ::close(std::exchange(value, -1));
  }

  int value{-1};
};

inline bool safe_name(const std::string &value) {
  return !value.empty() && value.size() <= 240 && value != "." &&
         value != ".." &&
         std::all_of(value.begin(), value.end(), [](unsigned char byte) {
           return (byte >= 'a' && byte <= 'z') ||
                  (byte >= 'A' && byte <= 'Z') ||
                  (byte >= '0' && byte <= '9') || byte == '_' || byte == '-' ||
                  byte == '.';
         });
}

inline std::filesystem::path normalize_directory(const std::string &input) {
  namespace fs = std::filesystem;
  require(!input.empty(), "empty log directory");
  auto path = fs::absolute(input).lexically_normal();
  while (path != path.root_path() && path.filename().empty())
    path = path.parent_path();
  require(safe_name(path.filename().string()), "invalid log directory name");
  fs::path ancestor;
  for (const auto &part : path) {
    ancestor /= part;
    require(!fs::is_symlink(fs::symlink_status(ancestor)),
            "symlink log directory refused");
  }
  return path;
}

inline std::int64_t epoch_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

inline std::string random_writer_id() {
  std::array<unsigned char, 16> bytes{};
  std::size_t done = 0;
  while (done < bytes.size()) {
    const auto count = ::getrandom(bytes.data() + done, bytes.size() - done, 0);
    if (count < 0 && errno == EINTR)
      continue;
    checked(count, "get random writer id");
    require(count > 0, "empty random source");
    done += static_cast<std::size_t>(count);
  }

  // Unpadded URL-safe Base64 retains all 128 random bits in 22 characters.
  constexpr char alphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  std::string result;
  result.reserve(22);
  std::uint32_t buffer = 0;
  unsigned bits = 0;
  for (const auto byte : bytes) {
    buffer = (buffer << 8) | byte;
    bits += 8;
    while (bits >= 6) {
      bits -= 6;
      result += alphabet[(buffer >> bits) & 63];
    }
  }
  if (bits)
    result += alphabet[(buffer << (6 - bits)) & 63];
  return result;
}

inline void rename_unique(const std::filesystem::path &from,
                          const std::filesystem::path &to) {
  checked(::syscall(SYS_renameat2, AT_FDCWD, from.c_str(), AT_FDCWD, to.c_str(),
                    RENAME_NOREPLACE),
          "rename log file without overwrite");
}

inline void write_state(int descriptor, const std::string &contents) {
  std::size_t done = 0;
  while (done < contents.size()) {
    const auto count =
        ::write(descriptor, contents.data() + done, contents.size() - done);
    if (count < 0 && errno == EINTR)
      continue;
    checked(count, "write log state");
    require(count > 0, "zero state write");
    done += static_cast<std::size_t>(count);
  }
}

inline struct stat stat_owned(const std::filesystem::path &path, int fd) {
  struct stat opened{}, named{};
  checked(::fstat(fd, &opened), "stat open log file");
  checked(::lstat(path.c_str(), &named), "stat log pathname");
  require(S_ISREG(named.st_mode) && named.st_nlink == 1 &&
              named.st_dev == opened.st_dev && named.st_ino == opened.st_ino,
          "log pathname changed");
  return opened;
}

inline void remove_owned(const std::filesystem::path &path, int fd) {
  stat_owned(path, fd);
  checked(::unlink(path.c_str()), "remove owned log file");
}

inline void write_metadata(const std::filesystem::path &path,
                           const std::string &contents) {
  auto temporary = path;
  temporary += ".tmp";
  Descriptor file;
  file.value = ::open(temporary.c_str(),
                      O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
                      0640);
  checked(file.value, "create metadata temporary file");
  try {
    write_state(file.value, contents);
    checked(::close(std::exchange(file.value, -1)), "close metadata");
    rename_unique(temporary, path);
  } catch (...) {
    ::unlink(temporary.c_str());
    throw;
  }
}

struct Segment {
  Descriptor lease;
  Descriptor owner;
  std::unique_ptr<spdlog::sinks::basic_file_sink_st> sink;
  std::filesystem::path path, owner_path;
  std::string id, basename;
  std::uint64_t sequence{0};
  std::int64_t opened_ms{0};
  int offset_minutes{0};
  bool failed{false};
};

// Keep one sink attached to the logger. Switch its file under the same mutex
// used by writes, then close the detached file outside that mutex.
class SegmentSink : public spdlog::sinks::base_sink<std::mutex> {
public:
  std::unique_ptr<Segment> replace(std::unique_ptr<Segment> &next) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (next)
      next->sink->set_formatter(formatter_->clone());
    return std::exchange(current_, std::move(next));
  }

protected:
  void sink_it_(const spdlog::details::log_msg &message) override {
    if (!current_)
      return;
    try {
      current_->sink->log(message);
    } catch (...) {
      current_->failed = true;
      throw;
    }
  }

  void flush_() override {
    if (!current_)
      return;
    try {
      current_->sink->flush();
    } catch (...) {
      current_->failed = true;
      throw;
    }
  }

  void set_formatter_(std::unique_ptr<spdlog::formatter> formatter) override {
    if (current_)
      current_->sink->set_formatter(formatter->clone());
    formatter_ = std::move(formatter);
  }

private:
  std::unique_ptr<Segment> current_;
};

class TimedFile {
public:
  TimedFile(std::filesystem::path directory, const std::string &filename,
            const std::string &logger_name)
      : directory_(std::move(directory)), state_(directory_ / ".state"),
        prefix_(std::filesystem::path(filename).stem().string()),
        logger_name_(logger_name), writer_id_(random_writer_id()) {
    namespace fs = std::filesystem;
    require(safe_name(filename) && safe_name(prefix_) && prefix_.size() <= 64,
            "invalid log filename");
    fs::create_directories(directory_);
    require(!fs::is_symlink(fs::symlink_status(state_)),
            "symlink state directory refused");
    fs::create_directory(state_);
    const auto lock_path = state_ / (writer_id_ + ".lock");
    writer_lock_.value = ::open(lock_path.c_str(),
                                O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC,
                                0640);
    checked(writer_lock_.value, "create writer lock");
    checked(::flock(writer_lock_.value, LOCK_EX | LOCK_NB), "lock writer");
  }

  std::shared_ptr<spdlog::sinks::sink> sink() const { return sink_; }

  void rotate() {
    auto next = std::make_unique<Segment>();
    std::unique_ptr<Segment> previous;
    try {
      next->opened_ms = epoch_ms();
      const auto seconds = static_cast<std::time_t>(next->opened_ms / 1000);
      std::tm local{};
      require(::localtime_r(&seconds, &local) != nullptr, "invalid local time");
      next->offset_minutes = static_cast<int>(local.tm_gmtoff / 60);
      std::ostringstream stamp;
      stamp.imbue(std::locale::classic());
      stamp << std::put_time(&local, "%Y%m%d_%H%M%S");
      reserve(*next, stamp.str());
      next->sink = std::make_unique<spdlog::sinks::basic_file_sink_st>(
          next->path.string(), false);
      previous = sink_->replace(next);
    } catch (...) {
      // An unattached segment has received no writes.
      discard(*next);
      throw;
    }
    if (previous)
      seal(*previous);
  }

  void close() {
    std::unique_ptr<Segment> empty;
    auto previous = sink_->replace(empty);
    if (previous)
      seal(*previous);
  }

private:
  static void discard(Segment &segment) noexcept {
    segment.sink.reset();
    try {
      if (segment.lease.value >= 0)
        remove_owned(segment.path, segment.lease.value);
      if (segment.owner.value >= 0)
        remove_owned(segment.owner_path, segment.owner.value);
    } catch (...) {
      // Preserve any state that could not be verified and removed safely.
    }
  }

  void reserve(Segment &segment, const std::string &stamp) {
    namespace fs = std::filesystem;
    for (;;) {
      require(sequence_ != std::numeric_limits<std::uint64_t>::max(),
              "log sequence exhausted");
      segment.sequence = sequence_++;
      const auto suffix = std::to_string(segment.sequence);
      segment.id = writer_id_ + '_' + suffix;
      segment.basename = prefix_ + '_' + stamp + '_' + suffix;
      segment.path = directory_ / (segment.basename + ".active");
      segment.owner_path = state_ / (segment.basename + ".owner");
      segment.owner.value =
          ::open(segment.owner_path.c_str(),
                 O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0640);
      if (segment.owner.value < 0 && errno == EEXIST)
        continue;
      checked(segment.owner.value, "reserve log fragment name");
      checked(::flock(segment.owner.value, LOCK_EX | LOCK_NB), "lock log owner");
      const auto occupied = [](const fs::path &path) {
        return fs::symlink_status(path).type() != fs::file_type::not_found;
      };
      const bool sealed = occupied(directory_ / (segment.basename + ".log")) ||
                          occupied(state_ / (segment.basename + ".meta"));
      if (!sealed)
        segment.lease.value =
            ::open(segment.path.c_str(),
                   O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0640);
      if (sealed || (segment.lease.value < 0 && errno == EEXIST)) {
        remove_owned(segment.owner_path, segment.owner.value);
        segment.owner.reset();
        continue;
      }
      checked(segment.lease.value, "reserve active log file");
      checked(::flock(segment.lease.value, LOCK_EX | LOCK_NB), "lock log file");

      std::ostringstream owner;
      owner.imbue(std::locale::classic());
      owner << "format_version=1\nkind=segment_owner\nwriter_id=" << writer_id_
            << "\nsegment_id=" << segment.id << "\ndata_file=" << segment.basename
            << ".log\nactive_file=" << segment.basename
            << ".active\nopened_epoch_ms=" << segment.opened_ms << '\n';
      write_state(segment.owner.value, owner.str());
      return;
    }
  }

  void seal(Segment &segment) {
    if (!segment.failed)
      segment.sink->flush();
    segment.sink.reset();
    if (segment.failed)
      return;

    const auto info = stat_owned(segment.path, segment.lease.value);
    if (info.st_size == 0) {
      checked(::unlink(segment.path.c_str()), "remove empty log file");
      remove_owned(segment.owner_path, segment.owner.value);
      return;
    }
    const auto basename = segment.basename + ".log";
    rename_unique(segment.path, directory_ / basename);
    const auto retained =
        info.st_mtim.tv_sec * 1000 + info.st_mtim.tv_nsec / 1000000;
    std::ostringstream metadata;
    metadata.imbue(std::locale::classic());
    metadata << "format_version=2\nkind=log\nmodule="
             << directory_.filename().string() << "\nlogger=" << logger_name_
             << "\nwriter_id=" << writer_id_ << "\nsegment_id=" << segment.id
             << "\npid=" << ::getpid() << "\ndata_file=" << basename
             << "\nsize_bytes=" << info.st_size
             << "\nopened_epoch_ms=" << segment.opened_ms
             << "\nsealed_epoch_ms=" << epoch_ms()
             << "\nretention_epoch_ms=" << retained
             << "\nutc_offset_minutes=" << segment.offset_minutes
             << "\nrecovered=0\nrotation_mode=time_only\ndurability=flush_only"
             << "\nfilename_format=local_time_v1\nwriter_id_encoding=base64url"
             << "\nsequence=" << segment.sequence << '\n';
    write_metadata(state_ / (segment.basename + ".meta"), metadata.str());
  }

  std::filesystem::path directory_, state_;
  std::string prefix_, logger_name_, writer_id_;
  Descriptor writer_lock_;
  std::shared_ptr<SegmentSink> sink_{std::make_shared<SegmentSink>()};
  std::uint64_t sequence_{0};
};

} // namespace commonlog_detail
