# 在 ROS 2 功能包中使用 commonlog

把 `commonlog.hpp` 和 `commonlog_detail.hpp` 一起复制到功能包的
`include/my_pkg/log/`，其中 `my_pkg` 换成自己的包名。两个头文件需要放在同一目录。
在 Linux 上使用 C++17 或更新标准，构建环境需提供可由 CMake 找到的 spdlog。

## CMakeLists.txt

在功能包的 `CMakeLists.txt` 中加入以下配置。把 `my_node` 换成已有的节点 target，
并把 target 相关的三行放在 `add_executable()` 或 `add_library()` 之后。

```cmake
find_package(spdlog CONFIG REQUIRED)
find_package(Threads REQUIRED)

target_compile_features(my_node PRIVATE cxx_std_17)
target_include_directories(my_node PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(my_node spdlog::spdlog Threads::Threads)
```

同一个 target 的 `target_link_libraries` 调用要统一写法。如果已有调用使用
`PRIVATE`、`PUBLIC` 或 `INTERFACE`，在上面的链接行也使用 `PRIVATE`。
spdlog 及其 fmt 配置由构建环境提供。如果有多套 spdlog，可以通过 CMake 的
`spdlog_DIR` 选择。

## 节点入口

```cpp
#include "my_pkg/log/commonlog.hpp"
#include <iostream>

int main() {
  auto &file_log = commonlog::get();
  if (!file_log.init("/tmp/my_node", "my_node.log")) {
    std::cerr << file_log.status().last_error << '\n';
    return 1;
  }

  auto logger = spdlog::get("commonlog");
  logger->info("节点启动，周期 {} ms", 50);

  // 在此运行节点。退出时先停止 executor 和其他写日志的线程。
  file_log.shutdown();
  if (file_log.status().io_errors != 0) {
    std::cerr << file_log.status().last_error << '\n';
    return 1;
  }
}
```

初始化成功后，同一个可执行程序的各个 `.cpp` 都能通过
`spdlog::get("commonlog")` 取得这个 logger。`shutdown()` 会注销它并封存当前片段，
之后可以用新配置再初始化。ROS 2 的 `RCLCPP_*` 系列仍走 ROS 2 自己的日志系统。

默认日志等级为 `INFO`，同时输出到控制台，每 5 分钟切换片段，每秒刷新一次，
`ERROR` 及以上立即刷新。用原生 `logger->set_level(...)`、`logger->flush_on(...)`、
`logger->set_pattern(...)` 和 `logger->flush()` 来调整级别、刷新条件、格式或手动刷新。
第三个参数 `commonlog::Options` 用来设置轮转周期、文件刷新周期和是否打开控制台。
周期刷新作用于本库的文件日志。spdlog 的 `flush_every()` 会刷新进程中所有已注册的 logger。

日志目录必须可写。以 `my_node.log` 初始化时，活动文件名为
`my_node_日期_时间_序号.active`。轮转或正常关闭时，非空片段改名为 `.log`，
并在 `.state/` 下写入对应的 `.meta`。完成标记提交后，片段才算封存完成。
