cmake_minimum_required(VERSION 3.5)
project(inesctec_mrdt_slam_distmap_2d_ros)



set(${PROJECT_NAME}_LIB_COMPILE_OPT "-Wall;-Wextra;-Wpedantic;-std=c++17")

###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 2 <<< #####
###################################################

if("$ENV{ROS_VERSION}" STREQUAL "2")

message(STATUS "Robot Operating System (ROS) detected: ROS $ENV{ROS_VERSION} $ENV{ROS_DISTRO}")

find_package(ament_cmake REQUIRED)
find_package(srrg_cmake_modules REQUIRED)

# Compile options + specific Find<library>.cmake helpers
include(${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt)
message("...loading ${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt...")

# ROS 2 Package Dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(inesctec_mrdt_slam_distmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# System dependencies
find_package(Boost REQUIRED COMPONENTS program_options)

# ROS 2 Interfaces Generation (.msg, .srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFilename.srv"
)

# Source code
add_subdirectory(src)

# Installation
install(
  TARGETS
    tracker
    tracker_offline
    slam_online
    slam_offline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.launch.xml"
)

install(
  FILES
    LICENSE
    README.md
  DESTINATION share/${PROJECT_NAME}
)


# Create ROS 2 ament package
ament_export_dependencies(
  rclcpp
  rclcpp_components
  inesctec_mrdt_slam_distmap_2d
  geometry_msgs
  message_filters
  nav_msgs
  rosidl_default_generators
  rosidl_default_runtime
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  sensor_msgs
  srrg_cmake_modules
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  tf2
)

ament_package()

###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

message(STATUS "Robot Operating System (ROS) detected: ROS $ENV{ROS_VERSION} $ENV{ROS_DISTRO}")

# Find ROS 1 packages dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  inesctec_mrdt_slam_distmap_2d
  geometry_msgs
  message_generation
  message_filters
  message_runtime
  nav_msgs
  rosbag
  sensor_msgs
  srrg_cmake_modules
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  tf2
)

# Compile options + specific Find<library>.cmake helpers for SRRG Software
include(${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt)
message("...loading ${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt...")

# System dependencies
find_package(Boost REQUIRED COMPONENTS program_options)

# Custom ROS message and service files definition
add_service_files(FILES SetFilename.srv)

generate_messages(DEPENDENCIES std_msgs std_srvs)

# Create ROS 1 catkin package
catkin_package(
  INCLUDE_DIRS
    include
  # LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    roscpp
    inesctec_mrdt_slam_distmap_2d
    geometry_msgs
    message_generation
    message_filters
    message_runtime
    nav_msgs
    rosbag
    sensor_msgs
    srrg_cmake_modules
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
    tf2
  DEPENDS
    Boost
  # CFG_EXTRAS
  #   ${PROJECT_NAME}-extras.cmake
)

# Source code
add_subdirectory(src)

# Installation
install(
  TARGETS
    tracker
    tracker_offline
    slam_online
    slam_offline
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.launch"
)

install(
  DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.launch"
)

install(
  FILES
    LICENSE
    README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

endif()
