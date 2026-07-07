#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;
using namespace pcl;

using PointCloudType = PointNormal2fVectorCloud;
using PointType = typename PointCloudType::PointType;

using NormalNearestPoints1D = NormalComputatorNearestPoints1D_<PointCloudType>;
using NormalSlidingWindow1D = NormalComputatorSlidingWindow1D_<PointCloudType>;

TEST(pcl_normal_computator, nearestPoints1DZeroNormal)
{
  NormalNearestPoints1D normal_computator;

  normal_computator.m_min_pts = 5;
  normal_computator.m_radius = 0.025f;

  PointCloudType pc;

  pc.emplace_back(PointType(Vector2f(0.00f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.05f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.10f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.15f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.20f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.25f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.30f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.35f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.40f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.45f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.50f, 1.00f), Vector2f(0.f, 0.f)));

  normal_computator.computeNormals(pc);

  for (const PointType& pt : pc)
  {
    EXPECT_TRUE(pt.normal().isZero());
  }
}  // TEST(pcl_normal_computator, nearestPoints1DZeroNormal)

TEST(pcl_normal_computator, nearestPoints1DLine)
{
  NormalNearestPoints1D normal_computator;

  normal_computator.m_min_pts = 3;
  normal_computator.m_radius = 0.30f;

  PointCloudType pc;

  pc.emplace_back(PointType(Vector2f(0.00f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.05f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.10f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.15f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.20f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.25f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.30f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.35f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.40f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.45f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.50f, 1.00f), Vector2f(0.f, 0.f)));

  normal_computator.computeNormals(pc);

  for (const PointType& pt : pc)
  {
    EXPECT_FLOAT_EQ(pt.normal().x(), 0.f);
    EXPECT_FLOAT_EQ(pt.normal().y(), -1.f);
  }

  pc.clear();
  pc.emplace_back(PointType(Vector2f(1.00f, 0.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.05f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.10f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.15f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.20f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.25f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.30f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.35f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.40f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.45f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.50f), Vector2f(0.f, 0.f)));

  normal_computator.computeNormals(pc);

  for (const PointType& pt : pc)
  {
    EXPECT_FLOAT_EQ(pt.normal().x(), -1.f);
    EXPECT_FLOAT_EQ(pt.normal().y(), 0.f);
  }
}  // TEST(pcl_normal_computator, nearestPoints1DLine)

TEST(pcl_normal_computator, slidingWindow1DLine)
{
  NormalSlidingWindow1D normal_computator;

  normal_computator.m_window_size = 3;
  normal_computator.m_wrap_range = false;

  PointCloudType pc;

  pc.emplace_back(PointType(Vector2f(0.00f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.05f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.10f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.15f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.20f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.25f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.30f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.35f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.40f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.45f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.50f, 1.00f), Vector2f(0.f, 0.f)));

  normal_computator.computeNormals(pc);

  for (const PointType& pt : pc)
  {
    EXPECT_FLOAT_EQ(pt.normal().x(), 0.f);
    EXPECT_FLOAT_EQ(pt.normal().y(), -1.f);
  }

  pc.clear();
  pc.emplace_back(PointType(Vector2f(1.00f, 0.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.05f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.10f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.15f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.20f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.25f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.30f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.35f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.40f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.45f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.50f), Vector2f(0.f, 0.f)));

  normal_computator.computeNormals(pc);

  for (const PointType& pt : pc)
  {
    EXPECT_FLOAT_EQ(pt.normal().x(), -1.f);
    EXPECT_FLOAT_EQ(pt.normal().y(), 0.f);
  }
}  // TEST(pcl_normal_computator, slidingWindow1DLine)

TEST(pcl_normal_computator, slidingWindow1DWrapRange)
{
  NormalSlidingWindow1D normal_computator;

  normal_computator.m_window_size = 1;
  normal_computator.m_wrap_range = true;

  PointCloudType pc;

  pc.emplace_back(PointType(Vector2f(1.00f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.00f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(-1.00f, 1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(-1.00f, 0.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(-1.00f, -1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(0.00f, -1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, -1.00f), Vector2f(0.f, 0.f)));
  pc.emplace_back(PointType(Vector2f(1.00f, 0.00f), Vector2f(0.f, 0.f)));

  normal_computator.computeNormals(pc);

  EXPECT_FLOAT_EQ(pc.at(0).normal().x(), -sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(0).normal().y(), -sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(1).normal().x(), 0.f);
  EXPECT_FLOAT_EQ(pc.at(1).normal().y(), -1.f);
  EXPECT_FLOAT_EQ(pc.at(2).normal().x(), sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(2).normal().y(), -sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(3).normal().x(), 1.f);
  EXPECT_FLOAT_EQ(pc.at(3).normal().y(), 0.f);
  EXPECT_FLOAT_EQ(pc.at(4).normal().x(), sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(4).normal().y(), sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(5).normal().x(), 0.f);
  EXPECT_FLOAT_EQ(pc.at(5).normal().y(), 1.f);
  EXPECT_FLOAT_EQ(pc.at(6).normal().x(), -sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(6).normal().y(), sqrtf32(2.f) * 0.5f);
  EXPECT_FLOAT_EQ(pc.at(7).normal().x(), -1.f);
  EXPECT_FLOAT_EQ(pc.at(7).normal().y(), 0.f);
}  // TEST(pcl_normal_computator, slidingWindow1DWrapRange)

}  // namespace testing
