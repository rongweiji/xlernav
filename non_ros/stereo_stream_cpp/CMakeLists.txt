cmake_minimum_required(VERSION 3.16)
project(stereo_stream_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

find_package(Qt6 COMPONENTS Widgets QUIET)
if (Qt6_FOUND)
  set(QT_LIBS Qt6::Widgets)
else()
  find_package(Qt5 COMPONENTS Widgets REQUIRED)
  set(QT_LIBS Qt5::Widgets)
endif()

add_executable(stereo_stream_cpp
  main.cpp
)

target_include_directories(stereo_stream_cpp PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/../common
)

target_compile_options(stereo_stream_cpp PRIVATE
  ${GST_CFLAGS_OTHER}
)

target_link_libraries(stereo_stream_cpp
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${QT_LIBS}
  Threads::Threads
)
