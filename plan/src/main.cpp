#include <rclcpp/rclcpp.hpp>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <sys/stat.h>

#include "plan.h"
#include "log/log.h"

namespace fs = std::filesystem;

namespace {

void remove_expired_logs(const fs::path& directory,
                         std::chrono::system_clock::time_point now)
{
    constexpr auto retention = std::chrono::hours(73);
    try {
        for (const auto& entry : fs::directory_iterator(directory)) {
            // 只清理已封存的日志，保留正在写入的 .active 文件和 .state 目录。
            if (entry.path().extension() != ".log") {
                continue;
            }

            struct statx info {};
            if (::statx(AT_FDCWD, entry.path().c_str(), AT_SYMLINK_NOFOLLOW,
                        STATX_TYPE | STATX_BTIME, &info) != 0) {
                const std::error_code error(errno, std::generic_category());
                LOG_WARN("读取日志创建时间失败，保留文件 {}: {}",
                         entry.path().string(), error.message());
                continue;
            }
            if (!(info.stx_mask & STATX_TYPE) || !S_ISREG(info.stx_mode)) {
                continue;
            }
            // ctime 是属性变更时间，mtime 是修改时间，均不能代替创建时间。
            if (!(info.stx_mask & STATX_BTIME)) {
                LOG_WARN("文件系统不支持读取创建时间，保留日志: {}",
                         entry.path().string());
                continue;
            }

            const auto created = std::chrono::seconds(info.stx_btime.tv_sec)
                               + std::chrono::nanoseconds(info.stx_btime.tv_nsec);
            if (now.time_since_epoch() - created <= retention) {
                continue;
            }

            std::error_code error;
            if (fs::remove(entry.path(), error)) {
                LOG_INFO("已删除创建时间超过 73 小时的日志: {}", entry.path().string());
            } else if (error) {
                LOG_WARN("删除过期日志失败 {}: {}", entry.path().string(), error.message());
            }
        }
    } catch (const fs::filesystem_error& error) {
        LOG_WARN("遍历日志目录失败: {}", error.what());
    }
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("plan");

    // 编译时记录功能包路径，避免日志位置受启动时工作目录影响。
    const fs::path log_path = fs::path(PLAN_PACKAGE_SOURCE_DIR) / "logs";
    try {
        fs::create_directories(log_path);
    } catch (const fs::filesystem_error& error) {
        std::cerr << "创建日志目录失败: " << error.what() << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    auto& logger = commonlog::get();
    if (!logger.init(log_path.string(), "plan.log")) {
        std::cerr << "日志初始化失败: " << logger.status().last_error << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
    LOG_INFO("日志目录: {}", log_path.string());
    remove_expired_logs(log_path, std::chrono::system_clock::now());
    LOG_ONCE(spdlog::level::info, "plan node started");

    DataPool::Initialize(nh);
    Plan plan(nh);
    rclcpp::spin(nh);

    logger.shutdown();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
