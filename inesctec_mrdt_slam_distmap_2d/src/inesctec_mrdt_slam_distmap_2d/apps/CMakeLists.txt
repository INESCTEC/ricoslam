add_executable(generate_occgrid2d_from_srrg
  generate_occgrid2d_from_srrg.cpp
)
target_include_directories(generate_occgrid2d_from_srrg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(generate_occgrid2d_from_srrg PUBLIC
  ${PROJECT_NAME}
  Boost::program_options
)
target_compile_options(${PROJECT_NAME} PRIVATE
  ${${PROJECT_NAME}_COMPILE_OPT}
)
