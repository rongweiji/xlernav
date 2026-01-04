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

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr /usr/include /usr/include/x86_64-linux-gnu
  PATH_SUFFIXES include)

set(DEPTH_ANYTHING_VENDOR_DIR "${CMAKE_SOURCE_DIR}/vendor/depth_anything_v3")
set(DEPTH_ANYTHING_INCLUDE_DIR "${DEPTH_ANYTHING_VENDOR_DIR}/include")
set(DEPTH_ANYTHING_LIB "${DEPTH_ANYTHING_VENDOR_DIR}/lib/libtensorrt_depth_anything.so")

add_executable(depth_stream_cpp
  main.cpp
  camera_info.cpp
)

target_include_directories(depth_stream_cpp PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${DEPTH_ANYTHING_INCLUDE_DIR}
)

target_compile_definitions(depth_stream_cpp PRIVATE DEPTH_ANYTHING_STANDALONE)

target_link_libraries(depth_stream_cpp
  ${OpenCV_LIBRARIES}
  ${DEPTH_ANYTHING_LIB}
  ${CUDA_LIBRARIES}
)

ament_target_dependencies(depth_stream_cpp
  sensor_msgs
  std_msgs
  builtin_interfaces
  rosidl_runtime_cpp
)

set_target_properties(depth_stream_cpp PROPERTIES
  BUILD_RPATH "/opt/ros/jazzy/lib;${DEPTH_ANYTHING_VENDOR_DIR}/lib"
)
