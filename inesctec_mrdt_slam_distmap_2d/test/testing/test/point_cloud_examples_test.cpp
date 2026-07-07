#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace pcl;
using namespace test;

TEST(test_point_cloud_examples, getLaser2DPointCloudRandom)
{
  std::random_device rd;
  std::mt19937 rng(rd());

  Point2fVectorCloud pc2f;
  Point2dVectorCloud pc2d;

  float angle_res_f = geometry::degToRad(1.f);
  float fov_f = geometry::degToRad(270.f);
  float max_range_f = 10.f;

  double angle_res_d = geometry::degToRad(1.0);
  double fov_d = geometry::degToRad(270.0);
  double max_range_d = 10.0;

  getLaser2DPointCloudRandom(pc2f, angle_res_f, fov_f, max_range_f, rng);
  getLaser2DPointCloudRandom(pc2d, angle_res_d, fov_d, max_range_d, rng);

  EXPECT_EQ(pc2f.size(), 271);
  EXPECT_EQ(pc2d.size(), 271);

  angle_res_f = geometry::degToRad(1.f);
  fov_f = geometry::degToRad(360.f);
  max_range_f = 10.f;

  angle_res_d = geometry::degToRad(1.0);
  fov_d = geometry::degToRad(360.0);
  max_range_d = 10.0;

  getLaser2DPointCloudRandom(pc2f, angle_res_f, fov_f, max_range_f, rng);
  getLaser2DPointCloudRandom(pc2d, angle_res_d, fov_d, max_range_d, rng);

  EXPECT_EQ(pc2f.size(), 360);
  EXPECT_EQ(pc2d.size(), 360);
}  // TEST(test_point_cloud_examples, getLaser2DPointCloudRandom)

TEST(test_point_cloud_examples, getLaser2DPointCloudCircle)
{
  std::random_device rd;
  std::mt19937 rng(rd());

  Point2fVectorCloud pc2f;
  Point2dVectorCloud pc2d;

  float angle_res_f = geometry::degToRad(1.f);
  float fov_f = geometry::degToRad(270.f);
  float max_range_f = 10.f;
  float radius_f = 5.f;

  double angle_res_d = geometry::degToRad(1.0);
  double fov_d = geometry::degToRad(270.0);
  double max_range_d = 10.0;
  double radius_d = 5.0;

  getLaser2DPointCloudCircle(pc2f, radius_f, angle_res_f, fov_f, max_range_f,
                             rng);
  getLaser2DPointCloudCircle(pc2d, radius_d, angle_res_d, fov_d, max_range_d,
                             rng);

  EXPECT_EQ(pc2f.size(), 271);
  EXPECT_EQ(pc2d.size(), 271);

  angle_res_f = geometry::degToRad(1.f);
  fov_f = geometry::degToRad(360.f);
  max_range_f = 10.f;

  angle_res_d = geometry::degToRad(1.0);
  fov_d = geometry::degToRad(360.0);
  max_range_d = 10.0;

  getLaser2DPointCloudCircle(pc2f, radius_f, angle_res_f, fov_f, max_range_f,
                             rng);
  getLaser2DPointCloudCircle(pc2d, radius_d, angle_res_d, fov_d, max_range_d,
                             rng);

  EXPECT_EQ(pc2f.size(), 360);
  EXPECT_EQ(pc2d.size(), 360);

  max_range_f = 2.f;
  max_range_d = 2.0;

  getLaser2DPointCloudCircle(pc2f, radius_f, angle_res_f, fov_f, max_range_f,
                             rng);
  getLaser2DPointCloudCircle(pc2d, radius_d, angle_res_d, fov_d, max_range_d,
                             rng);

  EXPECT_TRUE(pc2f.empty());
  EXPECT_TRUE(pc2d.empty());
}  // TEST(test_point_cloud_examples, getLaser2DPointCloudCircle)

TEST(test_point_cloud_examples, getPointCloudSquare)
{
  std::random_device rd;
  std::mt19937 rng(rd());

  Point2fVectorCloud pc2f;
  Point2dVectorCloud pc2d;

  constexpr int num_pts_side = 100;

  float side_f = 5.f;
  double side_d = 5.f;

  getPointCloudSquare(pc2f, side_f, num_pts_side, rng);
  getPointCloudSquare(pc2d, side_d, num_pts_side, rng);

  EXPECT_EQ(pc2f.size(), num_pts_side * 4);
  EXPECT_EQ(pc2d.size(), num_pts_side * 4);
}  // TEST(test_point_cloud_examples, getPointCloudSquare)

TEST(test_point_cloud_examples, getPointCloudCorridor)
{
  std::random_device rd;
  std::mt19937 rng(rd());

  Point2fVectorCloud pc2f;
  Point2dVectorCloud pc2d;

  constexpr int num_pts_side = 100;

  float width_f = 2.5f;
  double width_d = 2.5f;

  float length_f = 5.f;
  double length_d = 5.f;

  getPointCloudCorridor(pc2f, width_f, length_f, num_pts_side, rng);
  getPointCloudCorridor(pc2d, width_d, length_d, num_pts_side, rng);

  EXPECT_EQ(pc2f.size(), (num_pts_side + 1) * 2);
  EXPECT_EQ(pc2d.size(), (num_pts_side + 1) * 2);

  ASSERT_EQ(pc2f.size(), pc2d.size());

  for (size_t idx = 0; idx < pc2f.size(); idx++)
  {
    EXPECT_TRUE((pc2f[idx].coordinates().y() == width_f * 0.5f) ||
                (pc2f[idx].coordinates().y() == -width_f * 0.5f));
    EXPECT_TRUE((pc2d[idx].coordinates().y() == width_d * 0.50) ||
                (pc2d[idx].coordinates().y() == -width_d * 0.50));
  }

  getPointCloudCorridor(pc2f, width_f, length_f, num_pts_side, rng, true);
  getPointCloudCorridor(pc2d, width_d, length_d, num_pts_side, rng, true);

  for (size_t idx = 0; idx < pc2f.size(); idx++)
  {
    EXPECT_TRUE((pc2f[idx].coordinates().x() == width_f * 0.5f) ||
                (pc2f[idx].coordinates().x() == -width_f * 0.5f));
    EXPECT_TRUE((pc2d[idx].coordinates().x() == width_d * 0.50) ||
                (pc2d[idx].coordinates().x() == -width_d * 0.50));
  }

  EXPECT_EQ(pc2f.size(), (num_pts_side + 1) * 2);
  EXPECT_EQ(pc2d.size(), (num_pts_side + 1) * 2);
}  // TEST(test_point_cloud_examples, getPointCloudCorridor)

}  // namespace testing
