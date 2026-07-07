#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/system_utils/ros_yaml_offline.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace system_utils;

TEST(system_utils_ros_yaml_offline, getParamReturn)
{
  YAML::Node config = YAML::Load(
      "normals_type       : nearest_point\n"
      "normals_min_pts    : 5\n"
      "normals_radius     : 0.15\n");

  EXPECT_STRCASEEQ(getParam<std::string>(config, "normals_type").c_str(),
                   "nearest_point");
  EXPECT_EQ(getParam<int>(config, "normals_min_pts"), 5);
  EXPECT_FLOAT_EQ(getParam<float>(config, "normals_radius"), 0.15f);
  EXPECT_DOUBLE_EQ(getParam<double>(config, "normals_radius"), 0.150);

  EXPECT_ANY_THROW(getParam<int>(config, "normals_test"));
}  // TEST(system_utils_ros_yaml_offline, getParam)

TEST(system_utils_ros_yaml_offline, getParamVar)
{
  YAML::Node config = YAML::Load(
      "normals_type       : nearest_point\n"
      "normals_min_pts    : 5\n"
      "normals_radius     : 0.15\n");

  std::string normals_type;
  int normals_min_pts;
  float normals_radius_f;
  double normals_radius_d;

  EXPECT_NO_THROW(getParam(config, "normals_type", normals_type));
  EXPECT_NO_THROW(getParam(config, "normals_min_pts", normals_min_pts));
  EXPECT_NO_THROW(getParam(config, "normals_radius", normals_radius_f));
  EXPECT_NO_THROW(getParam(config, "normals_radius", normals_radius_d));

  EXPECT_ANY_THROW(getParam(config, "normals_test", normals_radius_d));

  EXPECT_STRCASEEQ(normals_type.c_str(), "nearest_point");
  EXPECT_EQ(normals_min_pts, 5);
  EXPECT_FLOAT_EQ(normals_radius_f, 0.15f);
  EXPECT_DOUBLE_EQ(normals_radius_d, 0.150);
}  // TEST(system_utils_ros_yaml_offline, getParamVar)

TEST(system_utils_ros_yaml_offline, getParamVarWithDefault)
{
  YAML::Node config = YAML::Load(
      "normals_type       : nearest_point\n"
      "normals_min_pts    : 5\n"
      "normals_radius     : 0.15\n");

  bool normals_test;

  getParam(config, "normals_test", normals_test, true);
  EXPECT_TRUE(normals_test);

  getParam(config, "normals_test", normals_test, false);
  EXPECT_FALSE(normals_test);
}  // TEST(system_utils_ros_yaml_offline, getParamVarWithDefault)

TEST(system_utils_ros_yaml_offline, hasParam)
{
  YAML::Node config = YAML::Load(
      "normals_type       : nearest_point\n"
      "normals_min_pts    : 5\n"
      "normals_radius     : 0.15\n");

  EXPECT_TRUE(hasParam(config, "normals_type"));
  EXPECT_TRUE(hasParam(config, "normals_min_pts"));
  EXPECT_TRUE(hasParam(config, "normals_radius"));

  EXPECT_FALSE(hasParam(config, "normals_test"));
}  // TEST(system_utils_ros_yaml_offline, hasParam)

}  // namespace testing
