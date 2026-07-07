# Unit tests
set(${PROJECT_NAME}_UNIT_TESTS
  test_data_structures
  test_geometry
  test_pcl
  test_system_utils
  test_test
  test_srrg2_core
  test_srrg2_solver
  test_slam_grid_mapper
  test_slam
  test_slam_msg
)

set(${PROJECT_NAME}_test_data_structures_SOURCES
  "testing/data_structures/correspondence_test.cpp"
  "testing/data_structures/derived_types_pack_executor_test.cpp"
  "testing/data_structures/field_pack_test.cpp"
  "testing/data_structures/field_pack_derived_test.cpp"
  "testing/data_structures/matrix_test.cpp"
  "testing/data_structures/matrix_sparse_test.cpp"
  "testing/data_structures/matrix_sparse_map_test.cpp"
  "testing/data_structures/distance_map_cell_test.cpp"
  "testing/data_structures/distance_map_static_test.cpp"
  "testing/data_structures/distance_map_sparse_static_test.cpp"
  "testing/data_structures/distance_map_nano_static_test.cpp"
  "testing/data_structures/traverse_line_test.cpp"
  "testing/data_structures/grid_map_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_geometry_SOURCES
  "testing/geometry/defs_test.cpp"
  "testing/geometry/geometry_2d_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_pcl_SOURCES
  "testing/pcl/point_field_coordinates_test.cpp"
  "testing/pcl/point_field_direction_test.cpp"
  "testing/pcl/point_base_test.cpp"
  "testing/pcl/point_derived_test.cpp"
  "testing/pcl/point_test.cpp"
  "testing/pcl/point_normal_test.cpp"
  "testing/pcl/point_cloud_test.cpp"
  "testing/pcl/point_utils_test.cpp"
  "testing/pcl/normal_computator_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_system_utils_SOURCES
  "testing/system_utils/filesystem_test.cpp"
  "testing/system_utils/shell_colors_test.cpp"
  "testing/system_utils/ros_log_offline_test.cpp"
  "testing/system_utils/ros_yaml_offline_test.cpp"
  "testing/system_utils/time_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_test_SOURCES
  "testing/test/point_cloud_examples_test.cpp"
  "testing/test/laser_scan_examples_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_srrg2_core_SOURCES
  "testing/srrg2_core/serializer_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_srrg2_solver_SOURCES
  "testing/srrg2_solver/factor_correspondence_free_driven_test.cpp"
  "testing/srrg2_solver/factor_correspondence_free_driven_dynamic_test.cpp"
  "testing/srrg2_solver/measurement_owner_distance_map_point_cloud_test.cpp"
  "testing/srrg2_solver/robustifier_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_slam_grid_mapper_SOURCES
  "testing/slam/grid_mapper_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_slam_SOURCES
  "testing/slam/triggering_criteria_test.cpp"
  "testing/slam/merging_criteria_test.cpp"
  "testing/slam/splitting_criteria_test.cpp"
  "testing/slam/aligner_base_test.cpp"
  "testing/slam/aligner_only_prior_test.cpp"
  "testing/slam/aligner_distance_map_point_point_test.cpp"
  "testing/slam/aligner_distance_map_point_plane_test.cpp"
  "testing/slam/merger_test.cpp"
  "testing/slam/tracker_base_test.cpp"
  "testing/slam/loop_finder_entry_test.cpp"
  "testing/slam/loop_finder_entry_container_test.cpp"
  "testing/slam/loop_finder_visitor_test.cpp"
  "testing/slam/loop_finder_test.cpp"
  "main.cpp"
)
set(${PROJECT_NAME}_test_slam_msg_SOURCES
  "testing/slam/msg/header_test.cpp"
  "testing/slam/msg/laser_scan_test.cpp"
  "main.cpp"
)

# Interactive test examples
set(${PROJECT_NAME}_TEST_EXAMPLES
  test_example_srrg2_core_vector_data_boss
  test_example_srrg2_solver_variable_se2_distance_map
  test_example_srrg2_solver_se2_distance_map_pose_pose_geodesic_error_factor
  test_example_srrg2_solver_se2_distance_map_point_point_error_factor
  test_example_srrg2_solver_se2_distance_map_point_point_left_error_factor
  test_example_srrg2_solver_se2_distance_map_icp_point_point_error_factor
  test_example_srrg2_solver_se2_distance_map_icp_point_point_left_error_factor
  test_example_srrg2_solver_se2_distance_map_point_plane_error_factor
  test_example_srrg2_solver_se2_distance_map_point_plane_left_error_factor
  test_example_srrg2_solver_se2_distance_map_icp_point_plane_error_factor
  test_example_srrg2_solver_se2_distance_map_icp_point_plane_left_error_factor
  test_example_srrg2_solver_se2_distance_map_pose_pose_bundle_error_factor
  test_example_srrg2_solver_se2_distance_map_pose_pose_bundle_plane_error_factor
  test_example_srrg2_solver_se2_distance_map_prior_error_factor
  test_example_slam_distmap_2d_slam_aligner_distance_map_point_point_opengl
  test_example_slam_distmap_2d_slam_aligner_distance_map_point_plane_opengl
  test_phd_thesis_aligner_synthetic_benchmark_point_cloud_examples
  test_phd_thesis_aligner_synthetic_benchmark_laser_cloud_examples
  test_phd_thesis_gba_synthetic_benchmark_laser_cloud_examples
)

set(${PROJECT_NAME}_test_example_srrg2_core_vector_data_boss_SOURCES
  "testing/srrg2_core/vector_data_boss_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_variable_se2_distance_map_SOURCES
  "testing/srrg2_solver/variable_se2_distance_map_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_pose_pose_geodesic_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_pose_pose_geodesic_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_point_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_point_point_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_point_left_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_point_point_left_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_point_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_icp_point_point_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_point_left_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_icp_point_point_left_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_plane_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_point_plane_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_plane_left_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_point_plane_left_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_plane_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_icp_point_plane_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_plane_left_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_icp_point_plane_left_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_pose_pose_bundle_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_pose_pose_bundle_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_pose_pose_bundle_plane_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_pose_pose_bundle_plane_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_prior_error_factor_SOURCES
  "testing/srrg2_solver/se2_distance_map_prior_error_factor_test.cpp"
)
set(${PROJECT_NAME}_test_example_slam_distmap_2d_slam_aligner_distance_map_point_point_opengl_SOURCES
  "testing/slam/aligner_distance_map_point_point_opengl_test.cpp"
)
set(${PROJECT_NAME}_test_example_slam_distmap_2d_slam_aligner_distance_map_point_plane_opengl_SOURCES
  "testing/slam/aligner_distance_map_point_plane_opengl_test.cpp"
)

set(${PROJECT_NAME}_test_phd_thesis_aligner_synthetic_benchmark_point_cloud_examples_SOURCES
  "testing/phd_thesis/aligner_synthetic_benchmark_point_cloud_examples.cpp"
)

set(${PROJECT_NAME}_test_phd_thesis_aligner_synthetic_benchmark_laser_cloud_examples_SOURCES
  "testing/phd_thesis/aligner_synthetic_benchmark_laser_scan_examples.cpp"
)

set(${PROJECT_NAME}_test_phd_thesis_gba_synthetic_benchmark_laser_cloud_examples_SOURCES
  "testing/phd_thesis/gba_synthetic_benchmark_laser_scan_examples.cpp"
)

###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 2 <<< #####
###################################################

if("$ENV{ROS_VERSION}" STREQUAL "2")

if( (BUILD_TESTING) AND
    (NOT "$ENV{ROS_DISTRO}" STREQUAL "foxy"))  # foxy gtest version broken API

  find_package(ament_cmake_gtest REQUIRED)

  foreach(TEST_ITEM IN LISTS ${PROJECT_NAME}_UNIT_TESTS)

    ament_add_gtest(${PROJECT_NAME}_${TEST_ITEM} ${${PROJECT_NAME}_${TEST_ITEM}_SOURCES})

    target_include_directories(${PROJECT_NAME}_${TEST_ITEM} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
      $<INSTALL_INTERFACE:include>
    )

    target_link_libraries(${PROJECT_NAME}_${TEST_ITEM} ${PROJECT_NAME})

  endforeach()

endif()

###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

foreach(TEST_ITEM IN LISTS ${PROJECT_NAME}_UNIT_TESTS)

  catkin_add_gtest(${PROJECT_NAME}_${TEST_ITEM} ${${PROJECT_NAME}_${TEST_ITEM}_SOURCES})

  if(TARGET ${PROJECT_NAME}_${TEST_ITEM})

    target_include_directories(${PROJECT_NAME}_${TEST_ITEM} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
      $<INSTALL_INTERFACE:include>
    )

    target_link_libraries(${PROJECT_NAME}_${TEST_ITEM} ${PROJECT_NAME})

  endif()

endforeach()

##########################
#####  !!! ERROR !!! #####
##########################

else()

message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()

##################################
#####  >>> TEST EXAMPLES <<< #####
##################################

foreach(TEST_EXAMPLE_ITEM IN LISTS ${PROJECT_NAME}_TEST_EXAMPLES)

  add_executable(${PROJECT_NAME}_${TEST_EXAMPLE_ITEM} ${${PROJECT_NAME}_${TEST_EXAMPLE_ITEM}_SOURCES})
  target_include_directories(${PROJECT_NAME}_${TEST_EXAMPLE_ITEM} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_${TEST_EXAMPLE_ITEM} PUBLIC ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_${TEST_EXAMPLE_ITEM} PRIVATE ${${PROJECT_NAME}_COMPILE_OPT})

endforeach()
