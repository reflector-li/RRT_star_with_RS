cmake_minimum_required(VERSION 3.0.2)
project(rrt_star_rs)

# set(OpenCV_DIR /usr/local/share/OpenCV)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /usr/local/share/cmake/yaml-cpp)
set(CMAKE_BUILD_TYPE "Debug")

# glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
# find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tpcap
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)


# hybrid a star
add_library(
  RRTStarRS_LIB SHARED
  src/rs_path.cpp
  src/rrt_star.cpp
  src/rrt_star_rs_flow.cpp
  src/file_tools.cpp
)

target_link_libraries(
    RRTStarRS_LIB
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
    Python3::Python Python3::Module Python3::NumPy
)

add_executable(rrt_search app/search.cpp)
target_link_libraries(rrt_search RRTStarRS_LIB yaml-cpp yaml-cpp)
