cmake_minimum_required(VERSION 3.16)
project(orbslam3_stream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(CUDA REQUIRED)

set(ORB_SLAM3_DIR "${CMAKE_SOURCE_DIR}/vendor/ORB_SLAM3")
set(ORB_SLAM3_LIB "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so")

set(DEPTH_ANYTHING_DIR "${CMAKE_SOURCE_DIR}/../depth_stream_cpp/vendor/depth_anything_v3")
set(DEPTH_ANYTHING_LIB "${DEPTH_ANYTHING_DIR}/lib/libtensorrt_depth_anything.so")

add_executable(orbslam3_stream
  main.cpp
  calibration.cpp
)

target_include_directories(orbslam3_stream PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${DEPTH_ANYTHING_DIR}/include
)

target_compile_definitions(orbslam3_stream PRIVATE DEPTH_ANYTHING_STANDALONE HAVE_EPOXY HAVE_EIGEN)

target_link_libraries(orbslam3_stream
  ${OpenCV_LIBRARIES}
  ${ORB_SLAM3_LIB}
  ${Pangolin_LIBRARIES}
  ${DEPTH_ANYTHING_LIB}
  ${CUDA_LIBRARIES}
)

ament_target_dependencies(orbslam3_stream
  sensor_msgs
  std_msgs
  builtin_interfaces
  rosidl_runtime_cpp
)

set_target_properties(orbslam3_stream PROPERTIES
  BUILD_RPATH "/opt/ros/jazzy/lib;${DEPTH_ANYTHING_DIR}/lib;${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
)
