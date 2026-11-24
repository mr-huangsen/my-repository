cmake_minimum_required(VERSION 3.8)
project(igv_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  "msg/ControlInformation.msg"
  "msg/LocalTrajectory.msg"
  "msg/Location.msg"
  "msg/ControlStatus.msg"    
  "msg/ICCPStatus.msg"
  "msg/RelayControl.msg"
  "srv/LocalPath.srv"  
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
