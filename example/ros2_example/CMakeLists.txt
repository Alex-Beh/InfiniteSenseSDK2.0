cmake_minimum_required(VERSION 3.16)

project(ros2_example VERSION 1.0)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# === InfiniteSense SDK ===
set(INFINITE_SENSE_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/infinite_sense_core/include")
set(INFINITE_SENSE_LIB_NAME infinite_sense_core)

# === Component (shared library) ===
add_library(synchronizer_component SHARED
  synchronizer_component.cpp
)

target_include_directories(synchronizer_component 
  PUBLIC
    $<BUILD_INTERFACE:${INFINITE_SENSE_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(synchronizer_component
  ${INFINITE_SENSE_LIB_NAME}
)

ament_target_dependencies(synchronizer_component
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(synchronizer_component SynchronizerComponent)

## === ROS2 GPRMC Publisher Node ===
add_executable(gprmc_publisher_node gprmc_publisher_node.cpp)
ament_target_dependencies(gprmc_publisher_node rclcpp std_msgs sensor_msgs)

# === Standalone ROS 2 Node (executable) ===
add_executable(synchronizer_node
  synchronizer_node.cpp
)
target_include_directories(synchronizer_node 
  PUBLIC
    ${INFINITE_SENSE_INCLUDE_DIR}
)
target_link_libraries(synchronizer_node
  ${INFINITE_SENSE_LIB_NAME}
)
ament_target_dependencies(synchronizer_node
  rclcpp
  std_msgs
)

# === Installation ===
install(TARGETS
    synchronizer_component
    synchronizer_node
    gprmc_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# === Export ===
ament_export_targets(synchronizer_component HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components std_msgs)
