cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_ROOT "" CACHE PATH "Path to ORB_SLAM3 repository")
if(NOT ORB_SLAM3_ROOT)
  set(ORB_SLAM3_ROOT "/mnt/g/LocalGitProject/ORB_SLAM3")
endif()

if(NOT EXISTS "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
  message(FATAL_ERROR "ORB_SLAM3 library not found at ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
endif()

include_directories(
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(orb_slam3_rgbd_node
  src/orb_slam3_rgbd_node.cpp
)

ament_target_dependencies(orb_slam3_rgbd_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  message_filters
  image_transport
)

target_link_libraries(orb_slam3_rgbd_node
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
  boost_serialization
  crypto
)

install(TARGETS orb_slam3_rgbd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
