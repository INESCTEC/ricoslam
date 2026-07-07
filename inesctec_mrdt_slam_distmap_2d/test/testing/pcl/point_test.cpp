#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/pcl/point.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;
using namespace pcl;

TEST(pcl_point, constructor)
{
  constexpr float x = 3.f;
  constexpr float y = 5.f;

  Point2f pt1;
  Point2f pt2(Vector2f(x, y));
  Point2f pt3(Point2f::BaseType(Vector2f(x, y)));

  EXPECT_EQ(pt2.coordinates().x(), x);
  EXPECT_EQ(pt2.coordinates().y(), y);

  EXPECT_EQ(pt3.coordinates().x(), x);
  EXPECT_EQ(pt3.coordinates().y(), y);

  pt3 = Point2f(Vector2f(x + 2, y + 1));

  EXPECT_EQ(pt3.coordinates().x(), x + 2);
  EXPECT_EQ(pt3.coordinates().y(), y + 1);
}  // TEST(pcl_point, constructor)

}  // namespace testing
