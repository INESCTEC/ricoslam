if("$ENV{ROS_VERSION}" STREQUAL "2")

add_subdirectory(inesctec_mrdt_slam_distmap_2d_ros2)

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

add_subdirectory(inesctec_mrdt_slam_distmap_2d_ros1)

endif()
