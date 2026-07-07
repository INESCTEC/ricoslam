add_executable(tracker
  tracker_ros1_api.cpp
  tracker_ros1.cpp
  tracker_node_ros1.cpp
)
target_include_directories(tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(tracker PUBLIC
  ${catkin_LIBRARIES}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
)
target_compile_options(tracker PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
)

add_executable(tracker_offline
  tracker_ros1_api.cpp
  tracker_ros1_offline.cpp
  tracker_node_ros1_offline.cpp
)
target_include_directories(tracker_offline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(tracker_offline PUBLIC
  ${catkin_LIBRARIES}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
  Boost::program_options
)
target_compile_options(tracker_offline PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
)

add_executable(slam_online
  slam_ros1_api.cpp
  slam_ros1_online.cpp
  slam_node_ros1_online.cpp
)
target_include_directories(slam_online PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(slam_online PUBLIC
  ${catkin_LIBRARIES}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
)
target_compile_options(slam_online PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
)

add_executable(slam_offline
  slam_ros1_api.cpp
  slam_ros1_offline.cpp
  slam_node_ros1_offline.cpp
)
target_include_directories(slam_offline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(slam_offline PUBLIC
  ${catkin_LIBRARIES}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
  Boost::program_options
)
target_compile_options(slam_offline PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
)

add_executable(offline_global_bundle_adjustment
  offline_global_bundle_adjustment.cpp
  ogba_app_ros1.cpp
)
target_include_directories(offline_global_bundle_adjustment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(offline_global_bundle_adjustment PUBLIC
  ${catkin_LIBRARIES}
  inesctec_mrdt_slam_distmap_2d::inesctec_mrdt_slam_distmap_2d
  Boost::program_options
)
target_compile_options(offline_global_bundle_adjustment PRIVATE
  ${${PROJECT_NAME}_LIB_COMPILE_OPT}
)
