cmake_minimum_required(VERSION 3.16)
project(control VERSION 1.2.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(igv_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Threads REQUIRED)
# Existing solver uses the OSQP 0.6 C API.
find_path(OSQP_INCLUDE_DIR osqp/osqp.h REQUIRED)
find_library(OSQP_LIBRARY NAMES osqp REQUIRED)
add_library(control_lib STATIC
  src/latcontrol.cpp src/loncontrol.cpp src/datapool.cpp src/workflow.cpp
  common/differential_tracker.cpp common/qp_solve.cpp)
target_include_directories(control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OSQP_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
ament_target_dependencies(control_lib rclcpp std_msgs geometry_msgs nav_msgs igv_msgs tf2 tf2_geometry_msgs)
target_link_libraries(control_lib Eigen3::Eigen spdlog::spdlog Threads::Threads TBB::tbb ${OSQP_LIBRARY})
add_executable(control_node src/main.cpp)
target_compile_definitions(control_node PRIVATE
  CONTROL_PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(control_node control_lib)
add_executable(test_node src/test.cpp)
add_executable(stop_test src/stop_test.cpp)
ament_target_dependencies(test_node rclcpp std_msgs nav_msgs igv_msgs)
ament_target_dependencies(stop_test rclcpp std_msgs nav_msgs igv_msgs )
install(TARGETS control_lib EXPORT controlTargets ARCHIVE DESTINATION lib)
install(TARGETS control_node test_node stop_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(controlTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs igv_msgs tf2 tf2_geometry_msgs Eigen3 spdlog Threads TBB)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME ros2_transport_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/ros2_smoke.py
      --binary $<TARGET_FILE:control_node>
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/control.yaml)
  set_tests_properties(ros2_transport_smoke PROPERTIES TIMEOUT 120)
  add_test(NAME ros2_launch_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/launch_smoke.py
      --binary $<TARGET_FILE:control_node>
      --package-source ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(ros2_launch_smoke PROPERTIES TIMEOUT 120)
endif()
ament_package()
