cmake_minimum_required(VERSION 3.5)
project(inesctec_mrdt_slam_2d_datasets)



## GLOBAL MACROS

set(${PROJECT_NAME}_COMPILE_OPT "-Wall;-Wextra;-Wpedantic;-std=c++17")



## SYTEM DEPENDENCIES

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)



## ROBOT OPERATING SYSTEM (ROS) 1 - NOETIC

if("$ENV{ROS_VERSION}" STREQUAL "1")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  rosbag
  sensor_msgs
  tf
  tf2_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    nav_msgs
    rosbag
    sensor_msgs
    tf
    tf2_msgs
  DEPENDS
    Boost
    yaml-cpp
)

link_directories(${catkin_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)



add_executable(carmen_to_rosbag
  src/inesctec_mrdt_slam_2d_datasets/carmen_to_rosbag_parser.cpp
  src/carmen_to_rosbag.cpp
)
add_dependencies(carmen_to_rosbag
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(carmen_to_rosbag
  ${catkin_LIBRARIES}
  Boost::program_options
  yaml-cpp
)
target_compile_options(carmen_to_rosbag PRIVATE
  ${${PROJECT_NAME}_COMPILE_OPT}
)

endif()
