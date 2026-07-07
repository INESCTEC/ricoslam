#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point_cloud.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point_normal.hpp>
#include <inesctec_mrdt_slam_distmap_2d/system_utils/time.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;
using namespace pcl;
using namespace Eigen;

template <typename PointCloudContainer_>
void createRandomPointCloud(PointCloudContainer_& pc, size_t n = 10)
{
  srand48(time(0));

  pc.resize(n);

  for (typename PointCloudContainer_::PointType& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
    pt.coordinates().x() = 1.0 + drand48();
    pt.coordinates().y() = 1.0 + drand48();
  }
}  // void createRandomPointCloud(PointCloudContainer_&, size_t = 10)

template <typename PointNormalCloudContainer_>
void createRandomPointNormalCloud(PointNormalCloudContainer_& pc, size_t n = 10)
{
  srand48(time(0));

  pc.resize(n);

  for (typename PointNormalCloudContainer_::PointType& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
    pt.coordinates().x() = 1.0 + drand48();
    pt.coordinates().y() = 1.0 + drand48();
    pt.normal().x() = 1.0 + drand48();
    pt.normal().y() = 1.0 + drand48();
  }
}  // void createRandomPointNormalCloud(PointNormalCloudContainer_&, size_t =
   // 10)

TEST(pcl_point_cloud, copyTo)
{
  size_t num_pts = 10;

  Point2dVectorCloud pc1, pc2;
  createRandomPointCloud(pc1, num_pts);

  pc2.resize(num_pts);
  pc1.copyTo(pc2.begin());

  for (size_t idx = 0; idx < num_pts; idx++)
  {
    EXPECT_FLOAT_EQ(pc1[idx].coordinates().x(), pc2[idx].coordinates().x());
    EXPECT_FLOAT_EQ(pc1[idx].coordinates().y(), pc2[idx].coordinates().y());
  }
}  // TEST(pcl_point_cloud, copyTo)

TEST(pcl_point_cloud, setZero)
{
  size_t num_pts = 10;

  Point2dVectorCloud pc;
  pc.resize(num_pts);
  pc.setZero();

  for (const Point2d& pt : pc)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_FLOAT_EQ(pt.coordinates().x(), 0);
    EXPECT_FLOAT_EQ(pt.coordinates().y(), 0);
  }

  createRandomPointCloud(pc, num_pts);
  pc.setZero();

  for (const Point2d& pt : pc)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_FLOAT_EQ(pt.coordinates().x(), 0);
    EXPECT_FLOAT_EQ(pt.coordinates().y(), 0);
  }

  PointNormal2dVectorCloud pc_n;
  pc_n.resize(num_pts);
  pc_n.setZero();

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_FLOAT_EQ(pt.coordinates().x(), 0);
    EXPECT_FLOAT_EQ(pt.coordinates().y(), 0);
    EXPECT_FLOAT_EQ(pt.normal().x(), 0);
    EXPECT_FLOAT_EQ(pt.normal().y(), 0);
  }

  createRandomPointNormalCloud(pc_n, num_pts);
  pc_n.setZero();

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_FLOAT_EQ(pt.coordinates().x(), 0);
    EXPECT_FLOAT_EQ(pt.coordinates().y(), 0);
    EXPECT_FLOAT_EQ(pt.normal().x(), 0);
    EXPECT_FLOAT_EQ(pt.normal().y(), 0);
  }
}  // TEST(pcl_point_cloud, setZero)

TEST(pcl_point_cloud, setZeroIdx)
{
  size_t num_pts = 10;
  PointNormal2dVectorCloud pc_n;

  createRandomPointNormalCloud(pc_n, num_pts);

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_FALSE(pt.coordinates().x() == 0);
    EXPECT_FALSE(pt.coordinates().y() == 0);
    EXPECT_FALSE(pt.normal().x() == 0);
    EXPECT_FALSE(pt.normal().y() == 0);
  }

  pc_n.setZero<0>();

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_TRUE(pt.coordinates().x() == 0);
    EXPECT_TRUE(pt.coordinates().y() == 0);
    EXPECT_FALSE(pt.normal().x() == 0);
    EXPECT_FALSE(pt.normal().y() == 0);
  }

  pc_n.setZero<1>();

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_FLOAT_EQ(pt.coordinates().x(), 0);
    EXPECT_FLOAT_EQ(pt.coordinates().y(), 0);
    EXPECT_FLOAT_EQ(pt.normal().x(), 0);
    EXPECT_FLOAT_EQ(pt.normal().y(), 0);
  }
}  // TEST(pcl_point_cloud, setZeroIdx)

TEST(pcl_point_cloud, normalize)
{
  size_t num_pts = 10;

  Point2dVectorCloud pc;
  createRandomPointCloud(pc, num_pts);

  pc.normalize();

  for (const Point2d& pt : pc)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_NEAR(pt.coordinates().norm(), 1, 1e-6);
  }

  PointNormal2dVectorCloud pc_n;
  createRandomPointNormalCloud(pc_n, num_pts);

  pc_n.normalize();

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_NEAR(pt.coordinates().norm(), 1, 1e-6);
    EXPECT_NEAR(pt.normal().norm(), 1, 1e-6);
  }

}  // TEST(pcl_point_cloud, normalize)

TEST(pcl_point_cloud, normalizeIdx)
{
  size_t num_pts = 10;
  PointNormal2dVectorCloud pc_n;

  createRandomPointNormalCloud(pc_n, num_pts);

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_FALSE((pt.coordinates().norm() < 1 + 1e-6) &&
                 (pt.coordinates().norm() > 1 - 1e-6));
    EXPECT_FALSE((pt.normal().norm() < 1 + 1e-6) &&
                 (pt.normal().norm() > 1 - 1e-6));
  }

  pc_n.normalize<0>();

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_TRUE((pt.coordinates().norm() < 1 + 1e-6) &&
                (pt.coordinates().norm() > 1 - 1e-6));
    EXPECT_FALSE((pt.normal().norm() < 1 + 1e-6) &&
                 (pt.normal().norm() > 1 - 1e-6));
  }

  pc_n.normalize<1>();

  for (const PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE(pt.m_status == PointStatus::kValid);
    EXPECT_TRUE((pt.coordinates().norm() < 1 + 1e-6) &&
                (pt.coordinates().norm() > 1 - 1e-6));
    EXPECT_TRUE((pt.normal().norm() < 1 + 1e-6) &&
                (pt.normal().norm() > 1 - 1e-6));
  }
}  // TEST(pcl_point_cloud, normalizeIdx)

TEST(pcl_point_cloud, transformInPlace)
{
  size_t num_pts = 10;

  Point2dVectorCloud pc;
  PointNormal2dVectorCloud pc_n;
  pc.resize(num_pts);
  pc_n.resize(num_pts);

  for (Point2d& pt : pc)
  {
    pt.coordinates().x() = 9;
    pt.coordinates().y() = 5;
  }

  for (PointNormal2d& pt : pc_n)
  {
    pt.coordinates().x() = 9;
    pt.coordinates().y() = 5;
    pt.normal().x() = 9;
    pt.normal().y() = 5;
  }

  Isometry2d transformation = v2t(Vector3d(-1, 3, M_PIf64 * 0.5));

  pc.transformInPlace(transformation);
  pc_n.transformInPlace(transformation);

  for (Point2d& pt : pc)
  {
    EXPECT_NEAR(pt.coordinates().x(), -5 - 1, 1e-6);
    EXPECT_NEAR(pt.coordinates().y(), 9 + 3, 1e-6);
  }

  for (PointNormal2d& pt : pc_n)
  {
    EXPECT_NEAR(pt.coordinates().x(), -5 - 1, 1e-6);
    EXPECT_NEAR(pt.coordinates().y(), 9 + 3, 1e-6);
    EXPECT_NEAR(pt.normal().x(), -5, 1e-6);
    EXPECT_NEAR(pt.normal().y(), 9, 1e-6);
  }
}  // TEST(pcl_point_cloud, transformInPlace)

TEST(pcl_point_cloud, transformInPlaceIdx)
{
  size_t num_pts = 10;
  PointNormal2dVectorCloud pc_n;

  pc_n.resize(num_pts);

  for (PointNormal2d& pt : pc_n)
  {
    pt.coordinates().x() = 9;
    pt.coordinates().y() = 5;
    pt.normal().x() = 9;
    pt.normal().y() = 5;
  }

  Isometry2d transformation = v2t(Vector3d(-1, 3, M_PIf64 * 0.5));

  pc_n.transformInPlace<0>(transformation);

  for (PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE((pt.coordinates().x() < -5 - 1 + 1e-6) &&
                (pt.coordinates().x() > -5 - 1 - 1e-6));
    EXPECT_TRUE((pt.coordinates().y() < 9 + 3 + 1e-6) &&
                (pt.coordinates().y() > 9 + 3 - 1e-6));
    EXPECT_FALSE((pt.normal().x() < -5 + 1e-6) &&
                 (pt.normal().x() > -5 - 1e-6));
    EXPECT_FALSE((pt.normal().y() < 9 + 1e-6) && (pt.normal().y() > 9 - 1e-6));
  }

  pc_n.transformInPlace<1>(transformation);

  for (PointNormal2d& pt : pc_n)
  {
    EXPECT_TRUE((pt.coordinates().x() < -5 - 1 + 1e-6) &&
                (pt.coordinates().x() > -5 - 1 - 1e-6));
    EXPECT_TRUE((pt.coordinates().y() < 9 + 3 + 1e-6) &&
                (pt.coordinates().y() > 9 + 3 - 1e-6));
    EXPECT_TRUE((pt.normal().x() < -5 + 1e-6) && (pt.normal().x() > -5 - 1e-6));
    EXPECT_TRUE((pt.normal().y() < 9 + 1e-6) && (pt.normal().y() > 9 - 1e-6));
  }

}  // TEST(pcl_point_cloud, transformInPlaceIdx)

TEST(pcl_point_cloud, transform)
{
  size_t num_pts = 10;

  Point2dVectorCloud pc1, pc2;
  pc1.resize(num_pts);
  pc2.resize(num_pts);

  for (Point2d& pt : pc1)
  {
    pt.coordinates().x() = 9;
    pt.coordinates().y() = 5;
  }

  Isometry2d transformation = v2t(Vector3d(-1, 3, M_PIf64 * 0.5));

  pc1.transform(pc2.begin(), transformation);

  for (Point2d& pt : pc2)
  {
    EXPECT_NEAR(pt.coordinates().x(), -5 - 1, 1e-6);
    EXPECT_NEAR(pt.coordinates().y(), 9 + 3, 1e-6);
  }

  pc2.setZero();

  pc2 = pc1.transform(transformation);

  for (Point2d& pt : pc2)
  {
    EXPECT_NEAR(pt.coordinates().x(), -5 - 1, 1e-6);
    EXPECT_NEAR(pt.coordinates().y(), 9 + 3, 1e-6);
  }
}  // TEST(pcl_point_cloud, transform)

TEST(pcl_point_cloud, transformIdx)
{
  size_t num_pts = 10;
  PointNormal2dVectorCloud pc_n1, pc_n2;

  pc_n1.resize(num_pts);
  pc_n2.resize(num_pts);

  for (PointNormal2d& pt : pc_n1)
  {
    pt.coordinates().x() = 9;
    pt.coordinates().y() = 5;
    pt.normal().x() = 9;
    pt.normal().y() = 5;
  }

  Isometry2d transformation = v2t(Vector3d(-1, 3, M_PIf64 * 0.5));

  pc_n2.setZero();

  pc_n1.transform<1>(pc_n2.begin(), transformation);

  for (PointNormal2d& pt : pc_n2)
  {
    EXPECT_FALSE((pt.coordinates().x() < -5 - 1 + 1e-6) &&
                 (pt.coordinates().x() > -5 - 1 - 1e-6));
    EXPECT_FALSE((pt.coordinates().y() < 9 + 3 + 1e-6) &&
                 (pt.coordinates().y() > 9 + 3 - 1e-6));
    EXPECT_TRUE((pt.normal().x() < -5 + 1e-6) && (pt.normal().x() > -5 - 1e-6));
    EXPECT_TRUE((pt.normal().y() < 9 + 1e-6) && (pt.normal().y() > 9 - 1e-6));
  }

  pc_n2.setZero();

  pc_n1.transform<0>(pc_n2.begin(), transformation);

  for (PointNormal2d& pt : pc_n2)
  {
    EXPECT_TRUE((pt.coordinates().x() < -5 - 1 + 1e-6) &&
                (pt.coordinates().x() > -5 - 1 - 1e-6));
    EXPECT_TRUE((pt.coordinates().y() < 9 + 3 + 1e-6) &&
                (pt.coordinates().y() > 9 + 3 - 1e-6));
    EXPECT_FALSE((pt.normal().x() < -5 + 1e-6) &&
                 (pt.normal().x() > -5 - 1e-6));
    EXPECT_FALSE((pt.normal().y() < 9 + 1e-6) && (pt.normal().y() > 9 - 1e-6));
  }

}  // TEST(pcl_point_cloud, transformInPlaceIdx)

TEST(pcl_point_cloud, voxelize)
{
  Point2fVectorCloud pc, target;

  Point2fVectorCloud::PlainVectorType scales{0.01f, 0.01f};

  pc.resize(14);

  pc.at(0) = Point2f(Eigen::Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Eigen::Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Eigen::Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Eigen::Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Eigen::Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Eigen::Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Eigen::Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Eigen::Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Eigen::Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Eigen::Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Eigen::Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Eigen::Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Eigen::Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Eigen::Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize(target, scales);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  // test with invalid points (remaining a single point per voxel)

  target.clear();

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize(target, scales);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(2).m_status = PointStatus::kOutOfRange;

  pc.voxelize(target, scales);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;

  pc.voxelize(target, scales);

  EXPECT_EQ(target.size(), 6);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
  }

  for (size_t idx = 5; idx < pc.size(); idx++)
  {
    pc[idx].m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize(target, scales));

  EXPECT_EQ(target.size(), 4);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize(target, scales));

  EXPECT_EQ(target.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();
  target.clear();

  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize(target, scales);

  EXPECT_EQ(target.size(), 1);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(target.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.at(0).m_status = PointStatus::kOutOfRange;
  target.clear();

  pc.voxelize(target, scales);
  EXPECT_EQ(target.size(), 0);

  pc.at(0).m_status = PointStatus::kInvalid;
  target.clear();

  pc.voxelize(target, scales);
  EXPECT_EQ(target.size(), 0);
}  // TEST(pcl_point_cloud, voxelize)

TEST(pcl_point_cloud, voxelizeFieldIdx)
{
  PointNormal2fVectorCloud pc;
  Point2fVectorCloud target;

  Point2fVectorCloud::PlainVectorType scales{0.01f, 0.01f};

  pc.resize(14);

  pc.at(0) =
      PointNormal2f(Eigen::Vector2f(0.010f, 0.010f), Eigen::Vector2f(0.f, 0.f));
  pc.at(1) =
      PointNormal2f(Eigen::Vector2f(0.020f, 0.050f), Eigen::Vector2f(0.f, 0.f));
  pc.at(2) =
      PointNormal2f(Eigen::Vector2f(0.015f, 0.010f), Eigen::Vector2f(0.f, 0.f));
  pc.at(3) =
      PointNormal2f(Eigen::Vector2f(0.020f, 0.010f), Eigen::Vector2f(0.f, 0.f));
  pc.at(4) =
      PointNormal2f(Eigen::Vector2f(0.015f, 0.050f), Eigen::Vector2f(0.f, 0.f));
  pc.at(5) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.020f), Eigen::Vector2f(0.f, 0.f));
  pc.at(6) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.040f), Eigen::Vector2f(0.f, 0.f));
  pc.at(7) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.025f), Eigen::Vector2f(0.f, 0.f));
  pc.at(8) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.035f), Eigen::Vector2f(0.f, 0.f));
  pc.at(9) =
      PointNormal2f(Eigen::Vector2f(0.025f, 0.015f), Eigen::Vector2f(0.f, 0.f));
  pc.at(10) =
      PointNormal2f(Eigen::Vector2f(0.025f, 0.040f), Eigen::Vector2f(0.f, 0.f));
  pc.at(11) =
      PointNormal2f(Eigen::Vector2f(0.020f, 0.045f), Eigen::Vector2f(0.f, 0.f));
  pc.at(12) =
      PointNormal2f(Eigen::Vector2f(0.015f, 0.055f), Eigen::Vector2f(0.f, 0.f));
  pc.at(13) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.030f), Eigen::Vector2f(0.f, 0.f));

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize<0>(target, scales);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  target.clear();

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(target, scales);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(2).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(target, scales);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;

  pc.voxelize<0>(target, scales);

  EXPECT_EQ(target.size(), 6);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
  }

  for (size_t idx = 5; idx < pc.size(); idx++)
  {
    pc[idx].m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(target, scales));

  EXPECT_EQ(target.size(), 4);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(target, scales));

  EXPECT_EQ(target.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();
  target.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize<0>(target, scales);

  EXPECT_EQ(target.size(), 1);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(target.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.at(0).m_status = PointStatus::kOutOfRange;
  target.clear();

  pc.voxelize<0>(target, scales);
  EXPECT_EQ(target.size(), 0);

  pc.at(0).m_status = PointStatus::kInvalid;
  target.clear();

  pc.voxelize<0>(target, scales);
  EXPECT_EQ(target.size(), 0);
}  // TEST(pcl_point_cloud, voxelizeFieldIdx)

TEST(pcl_point_cloud, voxelizeInPlace)
{
  Point2fVectorCloud pc;

  Point2fVectorCloud::PlainVectorType scales{0.01f, 0.01f};

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize(scales);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  // test with invalid points (remaining a single point per voxel)

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize(scales);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;
  pc.at(2).m_status = PointStatus::kOutOfRange;
  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize(scales);

  EXPECT_EQ(pc.size(), 6);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize(scales));

  EXPECT_EQ(pc.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();
  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize(scales);

  EXPECT_EQ(pc.size(), 1);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.clear();
  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));
  pc.at(0).m_status = PointStatus::kOutOfRange;

  pc.voxelize(scales);

  EXPECT_EQ(pc.size(), 0);

  pc.clear();
  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));
  pc.at(0).m_status = PointStatus::kInvalid;

  pc.voxelize(scales);

  EXPECT_EQ(pc.size(), 0);
}  // TEST(pcl_point_cloud, voxelizeInPlace)

TEST(pcl_point_cloud, voxelizeFieldIdxInPlace)
{
  PointNormal2fVectorCloud pc;

  Point2fVectorCloud::PlainVectorType scales{0.01f, 0.01f};

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize<0>(scales);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scales);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  pc.at(2).m_status = PointStatus::kOutOfRange;
  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scales);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;
  pc.at(2).m_status = PointStatus::kOutOfRange;
  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scales);

  EXPECT_EQ(pc.size(), 6);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
  }

  for (size_t idx = 5; idx < pc.size(); idx++)
  {
    pc[idx].m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(scales));

  EXPECT_EQ(pc.size(), 4);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(scales));

  EXPECT_EQ(pc.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize<0>(scales);

  EXPECT_EQ(pc.size(), 1);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  pc.at(0).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scales);
  EXPECT_EQ(pc.size(), 0);

  pc.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  pc.at(0).m_status = PointStatus::kInvalid;

  pc.voxelize<0>(scales);
  EXPECT_EQ(pc.size(), 0);
}  // TEST(pcl_point_cloud, voxelizeFieldIdxInPlace)

TEST(pcl_point_cloud, voxelizeEqualScale)
{
  Point2fVectorCloud pc, target;

  Point2fVectorCloud::Scalar scale = 0.01f;

  pc.resize(14);

  pc.at(0) = Point2f(Eigen::Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Eigen::Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Eigen::Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Eigen::Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Eigen::Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Eigen::Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Eigen::Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Eigen::Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Eigen::Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Eigen::Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Eigen::Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Eigen::Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Eigen::Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Eigen::Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize(target, scale);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  // test with invalid points (remaining a single point per voxel)

  target.clear();

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize(target, scale);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(2).m_status = PointStatus::kOutOfRange;

  pc.voxelize(target, scale);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;

  pc.voxelize(target, scale);

  EXPECT_EQ(target.size(), 6);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
  }

  for (size_t idx = 5; idx < pc.size(); idx++)
  {
    pc[idx].m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize(target, scale));

  EXPECT_EQ(target.size(), 4);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize(target, scale));

  EXPECT_EQ(target.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();
  target.clear();

  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize(target, scale);

  EXPECT_EQ(target.size(), 1);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(target.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.at(0).m_status = PointStatus::kOutOfRange;
  target.clear();

  pc.voxelize(target, scale);
  EXPECT_EQ(target.size(), 0);

  pc.at(0).m_status = PointStatus::kInvalid;
  target.clear();

  pc.voxelize(target, scale);
  EXPECT_EQ(target.size(), 0);
}  // TEST(pcl_point_cloud, voxelizeEqualScale)

TEST(pcl_point_cloud, voxelizeEqualScaleFieldIdx)
{
  PointNormal2fVectorCloud pc;
  Point2fVectorCloud target;

  Point2fVectorCloud::Scalar scale = 0.01f;

  pc.resize(14);

  pc.at(0) =
      PointNormal2f(Eigen::Vector2f(0.010f, 0.010f), Eigen::Vector2f(0.f, 0.f));
  pc.at(1) =
      PointNormal2f(Eigen::Vector2f(0.020f, 0.050f), Eigen::Vector2f(0.f, 0.f));
  pc.at(2) =
      PointNormal2f(Eigen::Vector2f(0.015f, 0.010f), Eigen::Vector2f(0.f, 0.f));
  pc.at(3) =
      PointNormal2f(Eigen::Vector2f(0.020f, 0.010f), Eigen::Vector2f(0.f, 0.f));
  pc.at(4) =
      PointNormal2f(Eigen::Vector2f(0.015f, 0.050f), Eigen::Vector2f(0.f, 0.f));
  pc.at(5) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.020f), Eigen::Vector2f(0.f, 0.f));
  pc.at(6) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.040f), Eigen::Vector2f(0.f, 0.f));
  pc.at(7) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.025f), Eigen::Vector2f(0.f, 0.f));
  pc.at(8) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.035f), Eigen::Vector2f(0.f, 0.f));
  pc.at(9) =
      PointNormal2f(Eigen::Vector2f(0.025f, 0.015f), Eigen::Vector2f(0.f, 0.f));
  pc.at(10) =
      PointNormal2f(Eigen::Vector2f(0.025f, 0.040f), Eigen::Vector2f(0.f, 0.f));
  pc.at(11) =
      PointNormal2f(Eigen::Vector2f(0.020f, 0.045f), Eigen::Vector2f(0.f, 0.f));
  pc.at(12) =
      PointNormal2f(Eigen::Vector2f(0.015f, 0.055f), Eigen::Vector2f(0.f, 0.f));
  pc.at(13) =
      PointNormal2f(Eigen::Vector2f(0.030f, 0.030f), Eigen::Vector2f(0.f, 0.f));

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize<0>(target, scale);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  target.clear();

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(target, scale);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(2).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(target, scale);

  EXPECT_EQ(target.size(), 8);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(7).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;

  pc.voxelize<0>(target, scale);

  EXPECT_EQ(target.size(), 6);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(target.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(target.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(target.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(target.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(target.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(target.at(5).coordinates().y(), 0.0400f, 1e-6);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
  }

  for (size_t idx = 5; idx < pc.size(); idx++)
  {
    pc[idx].m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(target, scale));

  EXPECT_EQ(target.size(), 4);

  target.clear();

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(target, scale));

  EXPECT_EQ(target.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();
  target.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize<0>(target, scale);

  EXPECT_EQ(target.size(), 1);
  EXPECT_NEAR(target.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(target.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.at(0).m_status = PointStatus::kOutOfRange;
  target.clear();

  pc.voxelize<0>(target, scale);
  EXPECT_EQ(target.size(), 0);

  pc.at(0).m_status = PointStatus::kInvalid;
  target.clear();

  pc.voxelize<0>(target, scale);
  EXPECT_EQ(target.size(), 0);
}  // TEST(pcl_point_cloud, voxelizeEqualScaleFieldIdx)

TEST(pcl_point_cloud, voxelizeEqualScaleInPlace)
{
  Point2fVectorCloud pc;

  Point2fVectorCloud::Scalar scale = 0.01f;

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize(scale);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  // test with invalid points (remaining a single point per voxel)

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize(scale);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;
  pc.at(2).m_status = PointStatus::kOutOfRange;
  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize(scale);

  EXPECT_EQ(pc.size(), 6);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = Point2f(Vector2f(0.010f, 0.010f));   // voxel: (1,1)
  pc.at(1) = Point2f(Vector2f(0.020f, 0.050f));   // voxel: (2,5)
  pc.at(2) = Point2f(Vector2f(0.015f, 0.010f));   // voxel: (1,1)
  pc.at(3) = Point2f(Vector2f(0.020f, 0.010f));   // voxel: (2,1)
  pc.at(4) = Point2f(Vector2f(0.015f, 0.050f));   // voxel: (1,5)
  pc.at(5) = Point2f(Vector2f(0.030f, 0.020f));   // voxel: (3,2)
  pc.at(6) = Point2f(Vector2f(0.030f, 0.040f));   // voxel: (3,4)
  pc.at(7) = Point2f(Vector2f(0.030f, 0.025f));   // voxel: (3,2)
  pc.at(8) = Point2f(Vector2f(0.030f, 0.035f));   // voxel: (3,3)
  pc.at(9) = Point2f(Vector2f(0.025f, 0.015f));   // voxel: (2,1)
  pc.at(10) = Point2f(Vector2f(0.025f, 0.040f));  // voxel: (2,4)
  pc.at(11) = Point2f(Vector2f(0.020f, 0.045f));  // voxel: (2,4)
  pc.at(12) = Point2f(Vector2f(0.015f, 0.055f));  // voxel: (1,5)
  pc.at(13) = Point2f(Vector2f(0.030f, 0.030f));  // voxel: (3,3)

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize(scale));

  EXPECT_EQ(pc.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();
  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize(scale);

  EXPECT_EQ(pc.size(), 1);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.clear();
  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));
  pc.at(0).m_status = PointStatus::kOutOfRange;

  pc.voxelize(scale);

  EXPECT_EQ(pc.size(), 0);

  pc.clear();
  pc.emplace_back(Eigen::Vector2f(0.00f, 0.00f));
  pc.at(0).m_status = PointStatus::kInvalid;

  pc.voxelize(scale);

  EXPECT_EQ(pc.size(), 0);
}  // TEST(pcl_point_cloud, voxelizeEqualScaleInPlace)

TEST(pcl_point_cloud, voxelizeEqualScaleFieldIdxInPlace)
{
  PointNormal2fVectorCloud pc;

  Point2fVectorCloud::Scalar scale = 0.01f;

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  for (const auto& pt : pc)
  {
    ASSERT_TRUE(pt.m_status == PointStatus::kValid);
  }

  pc.voxelize<0>(scale);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);  // voxel: (1,1)
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);  // voxel: (1,5)
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,1)
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0225f, 1e-6);  // voxel: (2,4)
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);  // voxel: (2,5)
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,2)
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,3)
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);  // voxel: (3,4)

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0525f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0425f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0225f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0325f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  // std::cout << "Point Cloud (original)" << std::endl;
  // pc.toStream(std::cout) << std::endl;

  // std::cout << "Point Cloud (voxelized)" << std::endl;
  // target.toStream(std::cout) << std::endl << std::endl;

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scale);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0125f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  pc.at(2).m_status = PointStatus::kOutOfRange;
  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scale);

  EXPECT_EQ(pc.size(), 8);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(6).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(7).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  pc.at(0).m_status = PointStatus::kInvalid;
  pc.at(1).m_status = PointStatus::kInvalid;
  pc.at(2).m_status = PointStatus::kOutOfRange;
  pc.at(7).m_status = PointStatus::kOutOfRange;
  pc.at(9).m_status = PointStatus::kOutOfRange;
  pc.at(11).m_status = PointStatus::kOutOfRange;
  pc.at(12).m_status = PointStatus::kOutOfRange;
  pc.at(13).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scale);

  EXPECT_EQ(pc.size(), 6);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.0150f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().x(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().x(), 0.0250f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().x(), 0.0300f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().x(), 0.0300f, 1e-6);

  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.0500f, 1e-6);
  EXPECT_NEAR(pc.at(1).coordinates().y(), 0.0100f, 1e-6);
  EXPECT_NEAR(pc.at(2).coordinates().y(), 0.0400f, 1e-6);
  EXPECT_NEAR(pc.at(3).coordinates().y(), 0.0200f, 1e-6);
  EXPECT_NEAR(pc.at(4).coordinates().y(), 0.0350f, 1e-6);
  EXPECT_NEAR(pc.at(5).coordinates().y(), 0.0400f, 1e-6);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kValid;
  }

  for (size_t idx = 5; idx < pc.size(); idx++)
  {
    pc[idx].m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(scale));

  EXPECT_EQ(pc.size(), 4);

  pc.resize(14);

  pc.at(0) = PointNormal2f(Vector2f(0.010f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(1) = PointNormal2f(Vector2f(0.020f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(2) = PointNormal2f(Vector2f(0.015f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(3) = PointNormal2f(Vector2f(0.020f, 0.010f), Vector2f(0.f, 0.f));
  pc.at(4) = PointNormal2f(Vector2f(0.015f, 0.050f), Vector2f(0.f, 0.f));
  pc.at(5) = PointNormal2f(Vector2f(0.030f, 0.020f), Vector2f(0.f, 0.f));
  pc.at(6) = PointNormal2f(Vector2f(0.030f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(7) = PointNormal2f(Vector2f(0.030f, 0.025f), Vector2f(0.f, 0.f));
  pc.at(8) = PointNormal2f(Vector2f(0.030f, 0.035f), Vector2f(0.f, 0.f));
  pc.at(9) = PointNormal2f(Vector2f(0.025f, 0.015f), Vector2f(0.f, 0.f));
  pc.at(10) = PointNormal2f(Vector2f(0.025f, 0.040f), Vector2f(0.f, 0.f));
  pc.at(11) = PointNormal2f(Vector2f(0.020f, 0.045f), Vector2f(0.f, 0.f));
  pc.at(12) = PointNormal2f(Vector2f(0.015f, 0.055f), Vector2f(0.f, 0.f));
  pc.at(13) = PointNormal2f(Vector2f(0.030f, 0.030f), Vector2f(0.f, 0.f));

  for (auto& pt : pc)
  {
    pt.m_status = PointStatus::kOutOfRange;
  }

  EXPECT_NO_THROW(pc.voxelize<0>(scale));

  EXPECT_EQ(pc.size(), 0);

  // simple test one valid point versus one invalid point
  pc.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  ASSERT_EQ(pc.size(), 1);
  ASSERT_TRUE(pc.at(0).m_status == PointStatus::kValid);

  pc.voxelize<0>(scale);

  EXPECT_EQ(pc.size(), 1);
  EXPECT_NEAR(pc.at(0).coordinates().x(), 0.00f, 1e-6);
  EXPECT_NEAR(pc.at(0).coordinates().y(), 0.00f, 1e-6);

  pc.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  pc.at(0).m_status = PointStatus::kOutOfRange;

  pc.voxelize<0>(scale);
  EXPECT_EQ(pc.size(), 0);

  pc.clear();

  pc.emplace_back(PointNormal2f(Eigen::Vector2f(0.00f, 0.00f),
                                Eigen::Vector2f(0.00f, 0.00f)));

  pc.at(0).m_status = PointStatus::kInvalid;

  pc.voxelize<0>(scale);
  EXPECT_EQ(pc.size(), 0);
}  // TEST(pcl_point_cloud, voxelizeEqualScaleFieldIdxInPlace)

TEST(pcl_point_cloud, voxelizeInPlaceVersusStandard)
{
  size_t num_pts = 10000;
  Point2dVectorCloud pc_d, pctarget_d;

  Point2dVectorCloud::PlainVectorType scales{0.01, 0.01};

  createRandomPointCloud(pc_d, num_pts);

  std::cout << "createRandomPointCloud 10000 pts" << std::endl;
  PRINT_AND_LOG_TIME("std voxelize 0.01m", pc_d.voxelize(pctarget_d, scales));
  PRINT_AND_LOG_TIME("inplace voxelize 0.01m", pc_d.voxelize(scales));

  scales.x() = 0.001;
  scales.y() = 0.001;

  PRINT_AND_LOG_TIME("std voxelize 0.001m", pc_d.voxelize(pctarget_d, scales));
  PRINT_AND_LOG_TIME("inplace voxelize 0.001m", pc_d.voxelize(scales));

  num_pts = 100000;

  createRandomPointCloud(pc_d, num_pts);

  std::cout << "createRandomPointCloud 100000 pts" << std::endl;
  PRINT_AND_LOG_TIME("std voxelize 0.01m", pc_d.voxelize(pctarget_d, scales));
  PRINT_AND_LOG_TIME("inplace voxelize 0.01m", pc_d.voxelize(scales));

  scales.x() = 0.001;
  scales.y() = 0.001;

  PRINT_AND_LOG_TIME("std voxelize 0.001m", pc_d.voxelize(pctarget_d, scales));
  PRINT_AND_LOG_TIME("inplace voxelize 0.001m", pc_d.voxelize(scales));
}  // TEST(pcl_point_cloud, voxelizeInPlaceVersusStandard)

}  // namespace testing
