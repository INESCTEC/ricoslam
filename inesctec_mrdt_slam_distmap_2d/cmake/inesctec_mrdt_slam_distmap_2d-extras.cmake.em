# inesctec_mrdt_slam_distmap_2d-extras.cmake.em
#
# This file imports the exported targets from the inesctec_mrdt_slam_distmap_2d library.
# This CMake file is required only in ROS 1, given that it is not possible to
# declare the exported targets when the find_package(catkin COMPONENTS ...)
# load the <package name>Config.cmake file.
# Indeed, when using find_package(catkin COMPONENTS ...), all the targets of a
# specific library are scoped to catkin::, making difficult to make it
# compatible with both ROS 1 and ROS 2.
#
# Useful links:
# https://docs.ros.org/en/melodic/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# https://catkin-tools.readthedocs.io/en/stable/migration.html#exporting-build-utilities
#
# WARNING:
# due to the implementation of ROS 1 CMake versions of the forked SRRG software,
# may only be possible to correctly link the packages after
# $ catkin_make install

@[if INSTALLSPACE]@

include(${inesctec_mrdt_slam_distmap_2d_DIR}/export_inesctec_mrdt_slam_distmap_2d.cmake)

# @[else]@

# include(${CATKIN_DEVEL_PREFIX}/share/inesctec_mrdt_slam_distmap_2d/cmake/export_inesctec_mrdt_slam_distmap_2d.cmake)

@[end if]@
