cmake_minimum_required(VERSION 3.16)
project(voxel_stream)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Qt6 COMPONENTS Widgets OpenGL OpenGLWidgets QUIET)
if (Qt6_FOUND)
  set(QT_LIBS Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets)
else()
  find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
  set(QT_LIBS Qt5::Widgets Qt5::OpenGL)
endif()
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(CUDA REQUIRED)

add_subdirectory(../common common)

set(ORB_SLAM3_DIR "${CMAKE_SOURCE_DIR}/../orbslam3_stream/vendor/ORB_SLAM3")
set(ORB_SLAM3_LIB "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so")

add_executable(voxel_stream
  main.cpp
  voxel_map.cpp
)

target_include_directories(voxel_stream PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
)

target_compile_definitions(voxel_stream PRIVATE HAVE_EPOXY HAVE_EIGEN)

target_link_libraries(voxel_stream
  xlernav_common
  ${OpenCV_LIBRARIES}
  ${ORB_SLAM3_LIB}
  ${Pangolin_LIBRARIES}
  ${QT_LIBS}
)

ament_target_dependencies(voxel_stream
  sensor_msgs
  std_msgs
  builtin_interfaces
  rosidl_runtime_cpp
)

set_target_properties(voxel_stream PROPERTIES
  BUILD_RPATH "/opt/ros/jazzy/lib;${DEPTH_ANYTHING_DIR}/lib;${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
)
