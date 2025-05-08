cmake_minimum_required(VERSION 3.16)

project(infinite_sense VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")
endif ()

add_subdirectory(infinite_sense_core)

add_subdirectory(tools)
add_subdirectory(example)

find_package(ZeroMQ QUIET)
if (ZeroMQ_FOUND)
  message(STATUS "ZMQ found, Compiling ZMQ nodes")
  find_package(ZeroMQ REQUIRED)
  if (NOT ZeroMQ_LIBRARIES)
    message(WARNING "ZeroMQ_LIBRARIES is empty. Using manual linking fallback.")
    set(ZeroMQ_LIBRARIES zmq)
  endif()

  find_package(Threads REQUIRED)

  message(STATUS "ZeroMQ_LIBRARIES: ${ZeroMQ_LIBRARIES}")
  message(STATUS "ZeroMQ_INCLUDE_DIRS: ${ZeroMQ_INCLUDE_DIRS}")

  add_executable(${PROJECT_NAME}_zmq_node
          example/ZMQ/zmq_main.cpp
          infinite_sense_core/src/messenger.cpp
  )
  target_include_directories(${PROJECT_NAME}_zmq_node PRIVATE
      ${ZeroMQ_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}_zmq_node PRIVATE
      Threads::Threads
      ${ZeroMQ_LIBRARIES}
      infinite_sense_core
  )
endif ()

# Conditionally add ROS 2 components
find_package(rclcpp QUIET)
find_package(rclcpp_components QUIET)
if(rclcpp_FOUND AND rclcpp_components_FOUND)
  message(STATUS "ROS 2 detected, adding example/ros2_components components...")
  add_subdirectory(example/ros2_components)
else()
  message(STATUS "ROS 2 not found. Skipping ROS 2 components.")
endif()
