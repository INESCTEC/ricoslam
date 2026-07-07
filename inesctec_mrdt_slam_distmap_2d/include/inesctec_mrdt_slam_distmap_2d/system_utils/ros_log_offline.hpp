#pragma once

#include <cstdio>
#include <iostream>

#include "inesctec_mrdt_slam_distmap_2d/system_utils/shell_colors.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace system_utils
{

#define OFFLDEBUG(...)               \
  std::cout << SC_GREEN;             \
  std::fprintf(stdout, __VA_ARGS__); \
  std::cout << SC_RESET << std::endl

#define OFFLINFO(...)                \
  std::cout << SC_BLACK;             \
  std::fprintf(stdout, __VA_ARGS__); \
  std::cout << SC_RESET << std::endl

#define OFFLWARN(...)                \
  std::cerr << SC_YELLOW;            \
  std::fprintf(stderr, __VA_ARGS__); \
  std::cerr << SC_RESET << std::endl

#define OFFLERROR(...)               \
  std::cerr << SC_RED;               \
  std::fprintf(stderr, __VA_ARGS__); \
  std::cerr << SC_RESET << std::endl

#define OFFLFATAL(...)               \
  std::cerr << SC_BRED;              \
  std::fprintf(stderr, __VA_ARGS__); \
  std::cerr << SC_RESET << std::endl

}  // namespace system_utils

}  // namespace inesctec_mrdt_slam_distmap_2d
