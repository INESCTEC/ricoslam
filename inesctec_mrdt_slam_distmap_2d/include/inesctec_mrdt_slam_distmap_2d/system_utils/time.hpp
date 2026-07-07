#pragma once

#include <chrono>
#include <ios>
#include <iostream>
#include <string>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace system_utils
{

#define PRINT_AND_LOG_TIME(description, ...)                                  \
  {                                                                           \
    auto start = std::chrono::high_resolution_clock::now();                   \
    __VA_ARGS__;                                                              \
    auto end = std::chrono::high_resolution_clock::now();                     \
    std::cout << std::fixed << "[" << std::string(description)                \
              << "] Elapsed time (ms): "                                      \
              << std::chrono::duration_cast<std::chrono::microseconds>(end -  \
                                                                       start) \
                         .count() *                                           \
                     1e-3                                                     \
              << std::endl;                                                   \
  }

}  // namespace system_utils

}  // namespace inesctec_mrdt_slam_distmap_2d
