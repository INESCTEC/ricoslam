add_library(${PROJECT_NAME} SHARED
  "data_structures/instances.cpp"
  "system_utils/filesystem.cpp"
  "test/laser_scan_examples.cpp"
  "srrg2_solver/instances.cpp"
  "srrg2_solver/solver_actions.cpp"
  "srrg2_qgl_viewport/viewer_core_shared_qgl_custom.cpp"
  "slam/msg/instances.cpp"
  "slam/msg/header.cpp"
  "slam/msg/laser_scan.cpp"
  "slam/loop_finder_visitor.cpp"
  "slam/grid_mapper.cpp"
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${OpenCV_LIBS}
  yaml-cpp

  srrg2_core::srrg2_geometry_library
  srrg2_core::srrg2_boss_library
  srrg2_core::srrg2_point_cloud_library
  srrg2_core::srrg2_data_structures_library
  srrg2_core::srrg2_viewer_library

  srrg2_qgl_viewport::srrg2_qgl_viewport_library

  srrg2_solver::srrg2_solver_core_library
  srrg2_solver::srrg2_solver_types_common_library
  srrg2_solver::srrg2_solver_types2d_library
)
target_compile_options(${PROJECT_NAME} PRIVATE
  ${${PROJECT_NAME}_COMPILE_OPT}
)

add_subdirectory(apps)
