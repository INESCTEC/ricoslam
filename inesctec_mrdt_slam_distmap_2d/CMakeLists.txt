cmake_minimum_required(VERSION 3.5)
project(inesctec_mrdt_slam_distmap_2d)



set(${PROJECT_NAME}_LIB_COMPILE_OPT "-Wall;-Wextra;-Wpedantic;-std=c++17")

set(${PROJECT_NAME}_EXECUTABLE_TARGETS
  ${PROJECT_NAME}_test_example_srrg2_core_vector_data_boss
  ${PROJECT_NAME}_test_example_srrg2_solver_variable_se2_distance_map
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_pose_pose_geodesic_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_point_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_point_left_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_point_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_point_left_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_plane_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_point_plane_left_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_plane_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_icp_point_plane_left_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_pose_pose_bundle_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_pose_pose_bundle_plane_error_factor
  ${PROJECT_NAME}_test_example_srrg2_solver_se2_distance_map_prior_error_factor
  ${PROJECT_NAME}_test_example_slam_distmap_2d_slam_aligner_distance_map_point_point_opengl
  ${PROJECT_NAME}_test_example_slam_distmap_2d_slam_aligner_distance_map_point_plane_opengl
)

set(${PROJECT_NAME}_APPS
  generate_occgrid2d_from_srrg
)



###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 2 <<< #####
###################################################

if("$ENV{ROS_VERSION}" STREQUAL "2")

# Find ROS 2 packages dependencies
find_package(ament_cmake REQUIRED)
find_package(srrg_cmake_modules REQUIRED)

# Compile options + specific Find<library>.cmake helpers
include(${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt)
message("...loading ${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt...")

# Find SRRG dependencies
find_package(srrg2_core REQUIRED)
find_package(srrg2_qgl_viewport REQUIRED)
find_package(srrg2_solver REQUIRED)

# System dependencies
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Source code
add_subdirectory(src)
add_subdirectory(test)

# Installation
install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  TARGETS ${${PROJECT_NAME}_EXECUTABLE_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  TARGETS ${${PROJECT_NAME}_APPS}
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
  srrg_cmake_modules
  srrg2_core
  srrg2_qgl_viewport
  srrg2_solver
  Eigen3
  OpenCV
  yaml-cpp
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

###################################################
#####  >>> ROBOT OPERATING SYSTEM (ROS) 1 <<< #####
###################################################

elseif("$ENV{ROS_VERSION}" STREQUAL "1")

# Find ROS 1 packages dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  srrg_cmake_modules
  srrg2_core
  srrg2_qgl_viewport
  srrg2_solver
)

# Compile options + specific Find<library>.cmake helpers for SRRG Software
include(${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt)
message("...loading ${srrg_cmake_modules_DIR}/CMakeCompileOptions.txt...")

# System dependencies
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create ROS 1 catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_srvs
    srrg_cmake_modules
    srrg2_core
    srrg2_qgl_viewport
    srrg2_solver
  DEPENDS
    Eigen3
    OpenCV
    yaml-cpp
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

# Source code
add_subdirectory(src)
add_subdirectory(test)

# Installation
install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(EXPORT export_${PROJECT_NAME}
  FILE export_${PROJECT_NAME}.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)

install(
  TARGETS ${${PROJECT_NAME}_EXECUTABLE_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  INCLUDES DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS ${${PROJECT_NAME}_APPS}
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

# Export targets
export(EXPORT export_${PROJECT_NAME}
  FILE "${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake/export_${PROJECT_NAME}.cmake"
  NAMESPACE ${PROJECT_NAME}::
)

##########################
#####  !!! ERROR !!! #####
##########################

else()

message(FATAL_ERROR "No valid version of ROS is installed or environment variables not set")

endif()
