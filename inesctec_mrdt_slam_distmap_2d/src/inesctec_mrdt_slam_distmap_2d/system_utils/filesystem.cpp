#include "inesctec_mrdt_slam_distmap_2d/system_utils/filesystem.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace system_utils
{

void validateAndCreatePath(const std::string& file_path)
{
  try
  {
    std::filesystem::path path(file_path);
    std::filesystem::path directory = path.parent_path();
    std::filesystem::path curr_directory = std::filesystem::current_path();

    if (directory.empty())
    {
      return;
    }

    if (std::filesystem::exists(directory))
    {
      if (std::filesystem::is_directory(directory))
      {
        return;
      }
      else
      {
        throw std::runtime_error("validateAndCreatePath | Path (" +
                                 directory.string() +
                                 ") exists but is not a directory");
      }
    }
    else
    {
      if (std::filesystem::create_directories(directory))
      {
        return;
      }
      else
      {
        throw std::runtime_error(
            "validateAndCreatePath | Failed to create directory (" +
            directory.string() + ")");
      }
    }
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    throw std::runtime_error("validateAndCreatePath | Error resolving path (" +
                             file_path + "): " + e.what());
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
        "validateAndCreatePath | Error when processing path (" + file_path +
        "): " + e.what());
  }
}

}  // namespace system_utils

}  // namespace inesctec_mrdt_slam_distmap_2d
