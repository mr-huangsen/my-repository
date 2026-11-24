cmake_minimum_required(VERSION 3.8)
project(plan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(igv_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

set(OSQP_LIBRARIES "/usr/local/lib/libosqp.so")

include_directories(include)

set(PLAN_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  igv_msgs
  ament_index_cpp
)

add_library(common STATIC
  common/AABB.cpp
  common/frenet_change.cpp
  common/niit_geometry.cpp
  common/OBB.cpp
  common/path_smoother.cpp
  common/qp_solve.cpp
  common/sample_curves.cpp
)
target_include_directories(common PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(common ${PLAN_DEPENDENCIES})
target_link_libraries(common spdlog::spdlog Threads::Threads ${OSQP_LIBRARIES})

add_library(lattic_plan STATIC
  plan_type_src/lattic_plan/Frenet_path.cpp
)
target_include_directories(lattic_plan PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(lattic_plan ${PLAN_DEPENDENCIES})
target_link_libraries(lattic_plan common)

add_library(ilqr_plan STATIC
  plan_type_src/ilqr/utils.cpp
  plan_type_src/ilqr/cilqr_solver.cpp
)
target_include_directories(ilqr_plan PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ilqr_plan ${PLAN_DEPENDENCIES})
target_link_libraries(ilqr_plan common spdlog::spdlog)

add_library(plan_lib STATIC
  src/plan.cpp
  src/datapool.cpp
)
target_include_directories(plan_lib PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(plan_lib ${PLAN_DEPENDENCIES})
target_link_libraries(plan_lib lattic_plan ilqr_plan tf2::tf2 spdlog::spdlog)

add_executable(plan_node src/main.cpp)
target_compile_definitions(plan_node PRIVATE
  PLAN_PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
ament_target_dependencies(plan_node ${PLAN_DEPENDENCIES})
target_link_libraries(plan_node plan_lib spdlog::spdlog)

add_executable(plan_test_node src/test.cpp)
target_compile_definitions(plan_test_node PRIVATE
  PLAN_PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
ament_target_dependencies(plan_test_node ${PLAN_DEPENDENCIES})
target_link_libraries(plan_test_node spdlog::spdlog Threads::Threads)

install(DIRECTORY include config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  common
  lattic_plan
  ilqr_plan
  plan_lib
  plan_node
  plan_test_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
