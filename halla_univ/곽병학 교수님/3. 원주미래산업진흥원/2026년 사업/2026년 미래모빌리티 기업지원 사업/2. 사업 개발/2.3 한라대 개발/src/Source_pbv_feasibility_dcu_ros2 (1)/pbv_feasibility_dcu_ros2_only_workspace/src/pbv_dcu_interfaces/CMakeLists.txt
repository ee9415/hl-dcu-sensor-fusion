cmake_minimum_required(VERSION 3.8)
project(pbv_dcu_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerceptionSnapshot.msg"
  "msg/FaultStatus.msg"
  "msg/StateStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/ControlCommand.msg"
  "msg/CanTx.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
