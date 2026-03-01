cmake_minimum_required(VERSION 3.16)
project(depth_stream_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(CUDA REQUIRED)

add_subdirectory(../common common)

add_executable(depth_stream_cpp
  main.cpp
)

target_include_directories(depth_stream_cpp PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(depth_stream_cpp
  xlernav_common
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(depth_stream_cpp
  sensor_msgs
  std_msgs
  builtin_interfaces
  rosidl_runtime_cpp
)

set_target_properties(depth_stream_cpp PROPERTIES
  BUILD_RPATH "/opt/ros/jazzy/lib;${DEPTH_ANYTHING_DIR}/lib"
)
