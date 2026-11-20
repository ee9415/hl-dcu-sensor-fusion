cmake_minimum_required(VERSION 3.14)
project(pbv_dcu_core_ros2 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pbv_dcu_interfaces REQUIRED)

include_directories(include)

set(PBV_CORE_SOURCES
  src/common/config.c
  src/common/data_types.c
  src/common/log.c
  src/common/time_utils.c
  src/io/socketcan_io.c
  src/fault_management/fault_management.c
  src/state_manager/state_manager.c
  src/safety_supervisor/safety_supervisor.c
  src/decision_control/decision_control.c
  src/can_interface/can_interface.c
)

add_library(pbv_dcu_core STATIC ${PBV_CORE_SOURCES})
target_include_directories(pbv_dcu_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pbv_dcu_core m)

add_executable(pbv_dcu_pipeline_node src/ros2/pbv_dcu_pipeline_node.cpp)
target_link_libraries(pbv_dcu_pipeline_node pbv_dcu_core m)
ament_target_dependencies(pbv_dcu_pipeline_node rclcpp pbv_dcu_interfaces)

install(TARGETS pbv_dcu_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS pbv_dcu_pipeline_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(pbv_dcu_core)
ament_export_dependencies(rclcpp pbv_dcu_interfaces)
ament_package()
