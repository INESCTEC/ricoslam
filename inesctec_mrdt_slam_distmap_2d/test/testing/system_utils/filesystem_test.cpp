#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/system_utils/filesystem.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace system_utils;

TEST(system_utils_filesystem, validateAndCreatePath)
{
  std::filesystem::remove_all(
      std::filesystem::path(__FILE__ + std::string(".d/")));
  std::filesystem::remove_all(
      std::filesystem::path(__FILE__ + std::string(".tmp/")));

  EXPECT_NO_THROW(validateAndCreatePath(__FILE__));
  EXPECT_NO_THROW(validateAndCreatePath(__FILE__ + std::string(".json")));
  EXPECT_NO_THROW(validateAndCreatePath(__FILE__ + std::string(".d/test.txt")));

  EXPECT_TRUE(std::filesystem::exists(
      std::filesystem::path(__FILE__ + std::string(".d/"))));
  EXPECT_TRUE(std::filesystem::is_directory(
      std::filesystem::path(__FILE__ + std::string(".d/"))));

  EXPECT_NO_THROW(
      validateAndCreatePath(__FILE__ + std::string(".tmp/data/test.dat")));

  EXPECT_TRUE(std::filesystem::exists(
      std::filesystem::path(__FILE__ + std::string(".tmp/data/"))));
  EXPECT_TRUE(std::filesystem::is_directory(
      std::filesystem::path(__FILE__ + std::string(".tmp/data/"))));

  // Function does not create the file, only creates the required directories!
  EXPECT_FALSE(std::filesystem::exists(
      std::filesystem::path(__FILE__ + std::string(".json"))));
  EXPECT_FALSE(std::filesystem::exists(
      std::filesystem::path(__FILE__ + std::string(".d/test.txt"))));
  EXPECT_FALSE(std::filesystem::exists(
      std::filesystem::path(__FILE__ + std::string(".tmp/data/test.dat"))));
}  // TEST(system_utils_filesystem, validateAndCreatePath)

}  // namespace testing
