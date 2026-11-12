cmake_minimum_required(VERSION 3.8)
project(unitree_api)

# Upstream message schemas are unchanged. Use standard Jazzy ROSIDL bindings.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Request.msg"
  "msg/RequestHeader.msg"
  "msg/RequestIdentity.msg"
  "msg/RequestLease.msg"
  "msg/RequestPolicy.msg"
  "msg/Response.msg"
  "msg/ResponseHeader.msg"
  "msg/ResponseStatus.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
