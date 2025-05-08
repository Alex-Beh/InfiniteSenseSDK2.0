find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(synchronizer_component SHARED
  synchronizer_component.cpp
)

target_link_libraries(synchronizer_component
    infinite_sense_core  # Assuming this is defined in your top-level CMake
)
ament_target_dependencies(synchronizer_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)

# Include directories (not targets)
target_include_directories(synchronizer_component 
  PRIVATE
    ${CMAKE_SOURCE_DIR}/infinite_sense_core/include 
)

# Register the component
rclcpp_components_register_nodes(synchronizer_component "SynchronizerComponent")

# Install targets
install(TARGETS synchronizer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)