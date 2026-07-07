#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <limits>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;

TEST(geometry_defs, degToRad)
{
  EXPECT_NEAR(degToRad(static_cast<float>(90.0f)), M_PIf32 * 0.5f, 1e-6);
  EXPECT_NEAR(degToRad(static_cast<float>(-90.0f)), -M_PIf32 * 0.5f, 1e-6);
  EXPECT_NEAR(degToRad(static_cast<float>(30.0f)), M_PIf32 / 6.0f, 1e-6);
  EXPECT_NEAR(degToRad(static_cast<float>(-30.0f)), -M_PIf32 / 6.0f, 1e-6);

  EXPECT_NEAR(degToRad(static_cast<double>(90.0)), M_PIf64 * 0.5, 1e-9);
  EXPECT_NEAR(degToRad(static_cast<double>(-90.0)), -M_PIf64 * 0.5, 1e-9);
  EXPECT_NEAR(degToRad(static_cast<double>(30.0)), M_PIf64 / 6.0, 1e-9);
  EXPECT_NEAR(degToRad(static_cast<double>(-30.0)), -M_PIf64 / 6.0, 1e-9);
}  // TEST(geometry_defs, degToRad)

TEST(geometry_defs, radToDeg)
{
  EXPECT_NEAR(radToDeg(static_cast<float>(M_PIf32 * 0.5f)), 90.0f, 1e-6);
  EXPECT_NEAR(radToDeg(static_cast<float>(-M_PIf32 * 0.5f)), -90.0f, 1e-6);
  EXPECT_NEAR(radToDeg(static_cast<float>(M_PIf32 / 6.0f)), 30.0f, 1e-6);
  EXPECT_NEAR(radToDeg(static_cast<float>(-M_PIf32 / 6.0f)), -30.0f, 1e-6);

  EXPECT_NEAR(radToDeg(static_cast<double>(M_PIf64 * 0.5)), 90.0, 1e-6);
  EXPECT_NEAR(radToDeg(static_cast<double>(-M_PIf64 * 0.5)), -90.0, 1e-6);
  EXPECT_NEAR(radToDeg(static_cast<double>(M_PIf64 / 6.0)), 30.0, 1e-6);
  EXPECT_NEAR(radToDeg(static_cast<double>(-M_PIf64 / 6.0)), -30.0, 1e-6);
}  // TEST(geometry_defs, radToDeg)

TEST(geometry_defs, wrapToPi)
{
  EXPECT_NEAR(wrapToPi(static_cast<float>(M_PIf32)), -M_PIf32, 1e-6);
  EXPECT_NEAR(wrapToPi(static_cast<float>(-M_PIf32)), -M_PIf32, 1e-6);
  EXPECT_NEAR(wrapToPi(static_cast<float>(0.0f)), 0.0f, 1e-6);
  EXPECT_NEAR(wrapToPi(static_cast<float>(3.0f * M_PIf32)), -M_PIf32, 1e-6);
  EXPECT_NEAR(wrapToPi(static_cast<float>(4.0f * M_PIf32)), 0.0, 1e-6);

  EXPECT_FLOAT_EQ(wrapToPi(static_cast<double>(M_PIf64)), -M_PIf64);
  EXPECT_FLOAT_EQ(wrapToPi(static_cast<double>(-M_PIf64)), -M_PIf64);
  EXPECT_FLOAT_EQ(wrapToPi(static_cast<double>(0.0)), 0.0f);
  EXPECT_FLOAT_EQ(wrapToPi(static_cast<double>(3.0 * M_PIf64)), -M_PIf64);
  EXPECT_FLOAT_EQ(wrapToPi(static_cast<double>(4.0 * M_PIf64)), 0.0);
}  // TEST(geometry_defs, wrapToPi)

TEST(geometry_defs, wrapToTwoPi)
{
  EXPECT_NEAR(wrapToTwoPi(static_cast<float>(M_PIf32)), M_PIf32, 1e-6);
  EXPECT_NEAR(wrapToTwoPi(static_cast<float>(-M_PIf32)), M_PIf32, 1e-6);
  EXPECT_NEAR(wrapToTwoPi(static_cast<float>(0.0f)), 0.0f, 1e-6);
  EXPECT_NEAR(wrapToTwoPi(static_cast<float>(2.0f * M_PIf32)), 0.0f, 1e-6);
  EXPECT_NEAR(wrapToTwoPi(static_cast<float>(3.0f * M_PIf32)), M_PIf32, 1e-6);
  EXPECT_NEAR(wrapToTwoPi(static_cast<float>(4.0f * M_PIf32)), 0.0, 1e-6);

  EXPECT_FLOAT_EQ(wrapToTwoPi(static_cast<double>(M_PIf64)), M_PIf64);
  EXPECT_FLOAT_EQ(wrapToTwoPi(static_cast<double>(-M_PIf64)), M_PIf64);
  EXPECT_FLOAT_EQ(wrapToTwoPi(static_cast<double>(0.0)), 0.0f);
  EXPECT_FLOAT_EQ(wrapToTwoPi(static_cast<double>(2.0 * M_PIf64)), 0.0f);
  EXPECT_FLOAT_EQ(wrapToTwoPi(static_cast<double>(3.0 * M_PIf64)), M_PIf64);
  EXPECT_FLOAT_EQ(wrapToTwoPi(static_cast<double>(4.0 * M_PIf64)), 0.0);

  ASSERT_TRUE(-std::numeric_limits<float>::epsilon() < 0.f);
  ASSERT_TRUE(-std::numeric_limits<double>::epsilon() < 0.f);

  EXPECT_TRUE(wrapToTwoPi(-std::numeric_limits<float>::epsilon()) > 0.f);
  EXPECT_TRUE(wrapToTwoPi(-std::numeric_limits<double>::epsilon()) > 0.0);
  EXPECT_TRUE(wrapToTwoPi(static_cast<float>(0.f)) >= 0.f);
  EXPECT_TRUE(wrapToTwoPi(static_cast<double>(0.0)) >= 0.0);
}  // TEST(geometry_defs, wrapToTwoPi)

TEST(geometry_defs, wrapTo180)
{
  EXPECT_NEAR(wrapTo180(static_cast<float>(180.0f)), -180.0f, 1e-6);
  EXPECT_NEAR(wrapTo180(static_cast<float>(-180.0f)), -180.0f, 1e-6);
  EXPECT_NEAR(wrapTo180(static_cast<float>(0.0f)), 0.0f, 1e-6);
  EXPECT_NEAR(wrapTo180(static_cast<float>(3 * 180.0f)), -180.0f, 1e-6);
  EXPECT_NEAR(wrapTo180(static_cast<float>(4 * 180.0f)), 0.0f, 1e-6);

  EXPECT_FLOAT_EQ(wrapTo180(static_cast<double>(180.0)), -180.0);
  EXPECT_FLOAT_EQ(wrapTo180(static_cast<double>(-180.0)), -180.0);
  EXPECT_FLOAT_EQ(wrapTo180(static_cast<double>(0.0)), 0.0);
  EXPECT_FLOAT_EQ(wrapTo180(static_cast<double>(3 * 180.0)), -180.0);
  EXPECT_FLOAT_EQ(wrapTo180(static_cast<double>(4 * 180.0)), 0.0);
}  // TEST(geometry_defs, wrapTo180)

TEST(geometry_defs, wrapTo360)
{
  EXPECT_NEAR(wrapTo360(static_cast<float>(180.0f)), 180.0f, 1e-6);
  EXPECT_NEAR(wrapTo360(static_cast<float>(-180.0f)), 180.0f, 1e-6);
  EXPECT_NEAR(wrapTo360(static_cast<float>(0.0f)), 0.0f, 1e-6);
  EXPECT_NEAR(wrapTo360(static_cast<float>(360.0f)), 0.0f, 1e-6);
  EXPECT_NEAR(wrapTo360(static_cast<float>(3 * 180.0f)), 180.0f, 1e-6);
  EXPECT_NEAR(wrapTo360(static_cast<float>(4 * 180.0f)), 0.0f, 1e-6);

  EXPECT_FLOAT_EQ(wrapTo360(static_cast<double>(180.0)), 180.0);
  EXPECT_FLOAT_EQ(wrapTo360(static_cast<double>(-180.0)), 180.0);
  EXPECT_FLOAT_EQ(wrapTo360(static_cast<double>(0.0)), 0.0);
  EXPECT_FLOAT_EQ(wrapTo360(static_cast<double>(360.0)), 0.0);
  EXPECT_FLOAT_EQ(wrapTo360(static_cast<double>(3 * 180.0)), 180.0);
  EXPECT_FLOAT_EQ(wrapTo360(static_cast<double>(4 * 180.0)), 0.0);

  ASSERT_TRUE(-std::numeric_limits<float>::epsilon() < 0.f);
  ASSERT_TRUE(-std::numeric_limits<double>::epsilon() < 0.f);

  EXPECT_TRUE(wrapTo360(-std::numeric_limits<float>::epsilon()) > 0.f);
  EXPECT_TRUE(wrapTo360(-std::numeric_limits<double>::epsilon()) > 0.0);
  EXPECT_TRUE(wrapTo360(static_cast<float>(0.f)) >= 0.f);
  EXPECT_TRUE(wrapTo360(static_cast<double>(0.0)) >= 0.0);
}  // TEST(geometry_defs, wrapTo360)

TEST(geometry_defs, wrapToRangeUpper)
{
  EXPECT_EQ(wrapToRange(0, 360), 0);
  EXPECT_EQ(wrapToRange(10, 360), 10);
  EXPECT_EQ(wrapToRange(100, 360), 100);
  EXPECT_EQ(wrapToRange(300, 360), 300);
  EXPECT_EQ(wrapToRange(359, 360), 359);
  EXPECT_EQ(wrapToRange(360, 360), 0);
  EXPECT_EQ(wrapToRange(720, 360), 0);

  EXPECT_EQ(wrapToRange(400, 360), 40);
  EXPECT_EQ(wrapToRange(-1, 360), 359);
}  // TEST(geometry_defs, wrapToRangeUpper)

TEST(geometry_defs, wrapToRangeSupInf)
{
  EXPECT_EQ(wrapToRange(0, 0, 360), 0);
  EXPECT_EQ(wrapToRange(10, 0, 360), 10);
  EXPECT_EQ(wrapToRange(100, 0, 360), 100);
  EXPECT_EQ(wrapToRange(300, 0, 360), 300);
  EXPECT_EQ(wrapToRange(359, 0, 360), 359);
  EXPECT_EQ(wrapToRange(360, 0, 360), 0);
  EXPECT_EQ(wrapToRange(720, 0, 360), 0);
  EXPECT_EQ(wrapToRange(400, 0, 360), 40);
  EXPECT_EQ(wrapToRange(-1, 0, 360), 359);

  EXPECT_EQ(wrapToRange(-100, -100, 100), -100);
  EXPECT_EQ(wrapToRange(-10, -100, 100), -10);
  EXPECT_EQ(wrapToRange(0, -100, 100), 0);
  EXPECT_EQ(wrapToRange(10, -100, 100), 10);
  EXPECT_EQ(wrapToRange(99, -100, 100), 99);

  EXPECT_EQ(wrapToRange(100, -100, 100), -100);
  EXPECT_EQ(wrapToRange(200, -100, 100), 0);
  EXPECT_EQ(wrapToRange(300, -100, 100), -100);
}  // TEST(geometry_defs, wrapToRangeSupInf)

}  // namespace testing
