#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/pcl/point_normal.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;
using namespace pcl;

TEST(pcl_point_normal, constructor)
{
  constexpr float x = 3.f;
  constexpr float y = 5.f;

  constexpr float nx = 1.f;
  constexpr float ny = 1.f;

  PointNormal2f pt1;
  PointNormal2f pt2(PointNormal2f::BaseType(
      PointNormal2f::PointBaseType(Vector2f(x, y)), Vector2f(nx, ny)));
  PointNormal2f pt3(Vector2f(x, y), Vector2f(nx, ny));

  EXPECT_EQ(pt1.normal().x(), 0);
  EXPECT_EQ(pt1.normal().y(), 0);

  EXPECT_EQ(pt2.coordinates().x(), x);
  EXPECT_EQ(pt2.coordinates().y(), y);
  EXPECT_EQ(pt2.normal().x(), nx);
  EXPECT_EQ(pt2.normal().y(), ny);

  EXPECT_EQ(pt3.coordinates().x(), x);
  EXPECT_EQ(pt3.coordinates().y(), y);
  EXPECT_EQ(pt3.normal().x(), nx);
  EXPECT_EQ(pt3.normal().y(), ny);

  pt3 = PointNormal2f(Vector2f(x + 2, y + 1), Vector2f(nx - 2, ny - 1));

  EXPECT_EQ(pt3.coordinates().x(), x + 2);
  EXPECT_EQ(pt3.coordinates().y(), y + 1);
  EXPECT_EQ(pt3.normal().x(), nx - 2);
  EXPECT_EQ(pt3.normal().y(), ny - 1);
}  // TEST(pcl_point_normal, constructor)

}  // namespace testing
