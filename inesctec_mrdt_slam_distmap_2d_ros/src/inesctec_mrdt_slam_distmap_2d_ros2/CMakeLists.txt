if(NOT COMMAND rosidl_get_typesupport_target)

  function(rosidl_get_typesupport_target var generate_interfaces_target typesupport_name)

    if(NOT TARGET ${generate_interfaces_target})
      message(FATAL_ERROR "${generate_interfaces_target} is not a CMake target. Maybe rosidl_generate_interfaces was given a different target name?")
    endif()

    set(output_target "${generate_interfaces_target}__${typesupport_name}")

    if(NOT TARGET ${output_target})
      # CMake if() evaluates strings ending in `-NOTFOUND` as false
      set(output_target "${output_target}-NOTFOUND")
    endif()

    set("${var}" "${output_target}" PARENT_SCOPE)

  endfunction()

endif()

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(tracker
  tracker_ros2_api.cpp
  tracker_ros2.cpp
  tracker_node_ros2.cpp
)
target_include_directories(tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tracker PUBLIC
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  tf2::tf2
)
target_compile_options(tracker PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
  -DROS_DISTRO_$ENV{ROS_DISTRO}
)

add_executable(tracker_offline
  tracker_ros2_api.cpp
  tracker_ros2_offline.cpp
  tracker_node_ros2_offline.cpp
)
target_include_directories(tracker_offline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tracker_offline PUBLIC
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  ${nav_msgs_TARGETS}
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  tf2::tf2

  Boost::program_options
)
target_compile_options(tracker_offline PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
  -DROS_DISTRO_$ENV{ROS_DISTRO}
)

add_executable(slam_online
  slam_ros2_api.cpp
  slam_ros2_online.cpp
  slam_node_ros2_online.cpp
)
target_include_directories(slam_online PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slam_online PUBLIC
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  tf2::tf2
)
target_compile_options(slam_online PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
  -DROS_DISTRO_$ENV{ROS_DISTRO}
)

add_executable(slam_offline
  slam_ros2_api.cpp
  slam_ros2_offline.cpp
  slam_node_ros2_offline.cpp
)
target_include_directories(slam_offline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slam_offline PUBLIC
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  ${nav_msgs_TARGETS}
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  tf2::tf2

  Boost::program_options
)
target_compile_options(slam_offline PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
  -DROS_DISTRO_$ENV{ROS_DISTRO}
)
