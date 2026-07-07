#pragma once

#include <exception>
#include <filesystem>
#include <iostream>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace system_utils
{

/**
 * @brief validate and create the directories of a file path
 *        (does not create the file itself!)
 * @param[in] file_path filesystem path for the file
 */
void validateAndCreatePath(const std::string& file_path);

}  // namespace system_utils

}  // namespace inesctec_mrdt_slam_distmap_2d
