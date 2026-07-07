#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/pcl/point_normal.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point_utils.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;
using namespace pcl;

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

TEST(pcl_point_utils, wrapToRange)
{
  EXPECT_EQ(wrapToRange(0, 10), 0);
  EXPECT_EQ(wrapToRange(-1, 10), 9);
  EXPECT_EQ(wrapToRange(10, 10), 0);
  EXPECT_EQ(wrapToRange(4, 10), 4);
}  // TEST(pcl_point_utils, wrapToRange)

TEST(pcl_point_utils, pointFieldAccumulatorConstructor)
{
  PointFieldAccumulator_<0, PointNormal2d> pt_acc;

  EXPECT_TRUE(pt_acc.getAcc().isZero());
  EXPECT_TRUE(pt_acc.getMean().isZero());
  EXPECT_TRUE(pt_acc.getXXAcc().isZero());
  EXPECT_TRUE(pt_acc.getCovariance().isZero());
}  // TEST(pcl_point_utils, pointFieldAccumulatorConstructor)

TEST(pcl_point_utils, pointFieldAccumulatorGetAccMean)
{
  constexpr int fieldIdx = 0;

  using PointCloudType = PointNormal2dVectorCloud;
  using PointType = typename PointCloudType::value_type;
  using VectorType = typename PointCloudType::PointType::VectorType;

  PointFieldAccumulator_<fieldIdx, PointType> pt_acc;
  PointCloudType pc;

  createRandomPointNormalCloud(pc);

  pc.setZero<1>();

  for (int idx = 0; idx < static_cast<int>(pc.size()); idx++)
  {
    pt_acc.add(pc[idx]);

    VectorType check_accumulator = VectorType::Zero();

    for (int sub_idx = 0; sub_idx <= idx; sub_idx++)
    {
      check_accumulator += pc[sub_idx].value<fieldIdx>();
    }

    EXPECT_DOUBLE_EQ(pt_acc.getNumPts(), idx + 1);

    EXPECT_DOUBLE_EQ(check_accumulator.x(), pt_acc.getAcc().x());
    EXPECT_DOUBLE_EQ(check_accumulator.y(), pt_acc.getAcc().y());

    EXPECT_DOUBLE_EQ(check_accumulator.x() / (idx + 1), pt_acc.getMean().x());
    EXPECT_DOUBLE_EQ(check_accumulator.y() / (idx + 1), pt_acc.getMean().y());
  }

  for (int idx = 0; idx < static_cast<int>(pc.size()); idx++)
  {
    pt_acc.sub(pc[idx]);

    VectorType check_accumulator = VectorType::Zero();

    for (int sub_idx = idx + 1; sub_idx < static_cast<int>(pc.size());
         sub_idx++)
    {
      check_accumulator += pc[sub_idx].value<fieldIdx>();
    }

    int num_pts = static_cast<int>(pc.size()) - idx - 1;

    EXPECT_DOUBLE_EQ(pt_acc.getNumPts(), num_pts);

    EXPECT_DOUBLE_EQ(check_accumulator.x(), pt_acc.getAcc().x());
    EXPECT_DOUBLE_EQ(check_accumulator.y(), pt_acc.getAcc().y());

    if (num_pts > 0)
    {
      EXPECT_DOUBLE_EQ(check_accumulator.x() / num_pts, pt_acc.getMean().x());
      EXPECT_DOUBLE_EQ(check_accumulator.y() / num_pts, pt_acc.getMean().y());
    }
    else
    {
      EXPECT_DOUBLE_EQ(0, pt_acc.getMean().x());
      EXPECT_DOUBLE_EQ(0, pt_acc.getMean().y());
    }
  }

  EXPECT_TRUE(pt_acc.getAcc().isZero());
  EXPECT_TRUE(pt_acc.getMean().isZero());
}  // TEST(pcl_point_utils, pointFieldAccumulatorGetAccMean)

TEST(pcl_point_utils, pointFieldAccumulatorGetAccMeanAddOther)
{
  constexpr int fieldIdx = 0;
  constexpr int num_pts = 1000;
  constexpr int num_pts_subset = num_pts - 100;

  using PointCloudType = PointNormal2dVectorCloud;
  using PointType = typename PointCloudType::value_type;
  using VectorType = typename PointCloudType::PointType::VectorType;

  PointFieldAccumulator_<fieldIdx, PointType> pt_acc, pt_acc_other;
  PointCloudType pc;

  createRandomPointNormalCloud(pc, num_pts);

  pc.setZero<1>();

  for (int idx = 0; idx < num_pts_subset; idx++)
  {
    pt_acc.add(pc[idx]);
    EXPECT_DOUBLE_EQ(pt_acc.getNumPts(), idx + 1);
  }

  for (int idx = num_pts_subset; idx < num_pts; idx++)
  {
    pt_acc_other.add(pc[idx]);
  }

  pt_acc.add(pt_acc_other);

  VectorType check_accumulator = VectorType::Zero();

  for (int idx = 0; idx < num_pts; idx++)
  {
    check_accumulator += pc[idx].value<fieldIdx>();
  }

  EXPECT_DOUBLE_EQ(check_accumulator.x(), pt_acc.getAcc().x());
  EXPECT_DOUBLE_EQ(check_accumulator.y(), pt_acc.getAcc().y());

  EXPECT_DOUBLE_EQ(check_accumulator.x() / num_pts, pt_acc.getMean().x());
  EXPECT_DOUBLE_EQ(check_accumulator.y() / num_pts, pt_acc.getMean().y());
}  // TEST(pcl_point_utils, pointFieldAccumulatorGetAccMeanAddOther)

TEST(pcl_point_utils, pointFieldAccumulatorGetAccMeanSubOther)
{
  constexpr int fieldIdx = 0;
  constexpr int num_pts = 1000;
  constexpr int num_pts_subset = num_pts - 100;

  using PointCloudType = PointNormal2dVectorCloud;
  using PointType = typename PointCloudType::value_type;
  using VectorType = typename PointCloudType::PointType::VectorType;

  PointFieldAccumulator_<fieldIdx, PointType> pt_acc, pt_acc_other;
  PointCloudType pc;

  createRandomPointNormalCloud(pc, num_pts);

  pc.setZero<1>();

  for (int idx = 0; idx < num_pts; idx++)
  {
    pt_acc.add(pc[idx]);
    EXPECT_DOUBLE_EQ(pt_acc.getNumPts(), idx + 1);
  }

  for (int idx = num_pts_subset; idx < num_pts; idx++)
  {
    pt_acc_other.add(pc[idx]);
  }

  pt_acc.sub(pt_acc_other);

  VectorType check_accumulator = VectorType::Zero();

  for (int idx = 0; idx < num_pts_subset; idx++)
  {
    check_accumulator += pc[idx].value<fieldIdx>();
  }

  EXPECT_DOUBLE_EQ(check_accumulator.x(), pt_acc.getAcc().x());
  EXPECT_DOUBLE_EQ(check_accumulator.y(), pt_acc.getAcc().y());

  EXPECT_DOUBLE_EQ(check_accumulator.x() / num_pts_subset,
                   pt_acc.getMean().x());
  EXPECT_DOUBLE_EQ(check_accumulator.y() / num_pts_subset,
                   pt_acc.getMean().y());
}  // TEST(pcl_point_utils, pointFieldAccumulatorGetAccMeanSubOther)

TEST(pcl_point_utils, pointFieldAccumulatorGetAccCovariance)
{
  constexpr int fieldIdx = 0;
  constexpr int num_pts = 10000;

  using PointCloudType = PointNormal2dVectorCloud;
  using PointType = typename PointCloudType::value_type;
  using VectorType = typename PointCloudType::PointType::VectorType;
  using MatrixType = MatrixN_<PointType::ComponentsType, PointType::Dim>;

  PointFieldAccumulator_<fieldIdx, PointType> pt_acc;
  PointCloudType pc;

  createRandomPointNormalCloud(pc, num_pts);

  pc.setZero<1>();

  for (int idx = 0; idx < static_cast<int>(pc.size()); idx++)
  {
    int num_pts = idx + 1;

    pt_acc.add(pc[idx]);

    VectorType mean = VectorType::Zero();
    MatrixType cov = MatrixType::Zero();

    for (int sub_idx = 0; sub_idx <= idx; sub_idx++)
    {
      mean += pc[sub_idx].value<fieldIdx>();
    }

    mean /= num_pts;

    for (int sub_idx = 0; sub_idx <= idx; sub_idx++)
    {
      VectorType xmean = pc[sub_idx].value<fieldIdx>() - mean;
      cov += xmean * xmean.transpose();
    }

    cov /= num_pts;

    EXPECT_TRUE(pt_acc.getMean().isApprox(mean, 1e-6));
    EXPECT_TRUE(pt_acc.getCovariance().isApprox(cov, 1e-6));
  }

  for (int idx = 0; idx < static_cast<int>(pc.size()); idx++)
  {
    int num_pts = static_cast<int>(pc.size()) - idx - 1;

    pt_acc.sub(pc[idx]);

    if (num_pts == 0)
    {
      break;
    }

    VectorType mean = VectorType::Zero();
    MatrixType cov = MatrixType::Zero();

    for (int sub_idx = idx + 1; sub_idx < static_cast<int>(pc.size());
         sub_idx++)
    {
      mean += pc[sub_idx].value<fieldIdx>();
    }

    mean /= num_pts;

    for (int sub_idx = idx + 1; sub_idx < static_cast<int>(pc.size());
         sub_idx++)
    {
      VectorType xmean = pc[sub_idx].value<fieldIdx>() - mean;
      cov += xmean * xmean.transpose();
    }

    cov /= num_pts;

    EXPECT_TRUE(pt_acc.getMean().isApprox(mean, 1e-6));
    EXPECT_TRUE(pt_acc.getCovariance().isApprox(cov, 1e-6));
  }

  EXPECT_TRUE(pt_acc.getMean().isZero());
  EXPECT_TRUE(pt_acc.getCovariance().isZero());
}  // TEST(pcl_point_utils, pointFieldAccumulatorGetAccCovariance)

TEST(pcl_point_utils, pointFieldAccumulatorGetAccCovarianceAddOther)
{
  constexpr int fieldIdx = 0;
  constexpr int num_pts = 10000;
  constexpr int num_pts_subset = num_pts - 2000;

  using PointCloudType = PointNormal2dVectorCloud;
  using PointType = typename PointCloudType::value_type;
  using VectorType = typename PointCloudType::PointType::VectorType;
  using MatrixType = MatrixN_<PointType::ComponentsType, PointType::Dim>;

  PointFieldAccumulator_<fieldIdx, PointType> pt_acc, pt_acc_other;
  PointCloudType pc;

  createRandomPointNormalCloud(pc, num_pts);

  pc.setZero<1>();

  for (int idx = 0; idx < num_pts_subset; idx++)
  {
    pt_acc.add(pc[idx]);
    EXPECT_DOUBLE_EQ(pt_acc.getNumPts(), idx + 1);
  }

  for (int idx = num_pts_subset; idx < num_pts; idx++)
  {
    pt_acc_other.add(pc[idx]);
  }

  pt_acc.add(pt_acc_other);

  VectorType mean = VectorType::Zero();
  MatrixType cov = MatrixType::Zero();

  for (int idx = 0; idx < num_pts; idx++)
  {
    mean += pc[idx].value<fieldIdx>();
  }

  mean /= num_pts;

  for (int idx = 0; idx < num_pts; idx++)
  {
    VectorType xmean = pc[idx].value<fieldIdx>() - mean;
    cov += xmean * xmean.transpose();
  }

  cov /= num_pts;

  EXPECT_TRUE(pt_acc.getMean().isApprox(mean, 1e-6));
  EXPECT_TRUE(pt_acc.getCovariance().isApprox(cov, 1e-6));
}  // TEST(pcl_point_utils, pointFieldAccumulatorGetAccCovarianceAddOther)

TEST(pcl_point_utils, pointFieldAccumulatorGetAccCovarianceSubOther)
{
  constexpr int fieldIdx = 0;
  constexpr int num_pts = 10000;
  constexpr int num_pts_subset = num_pts - 2000;

  using PointCloudType = PointNormal2dVectorCloud;
  using PointType = typename PointCloudType::value_type;
  using VectorType = typename PointCloudType::PointType::VectorType;
  using MatrixType = MatrixN_<PointType::ComponentsType, PointType::Dim>;

  PointFieldAccumulator_<fieldIdx, PointType> pt_acc, pt_acc_other;
  PointCloudType pc;

  createRandomPointNormalCloud(pc, num_pts);

  pc.setZero<1>();

  for (int idx = 0; idx < num_pts; idx++)
  {
    pt_acc.add(pc[idx]);
    EXPECT_DOUBLE_EQ(pt_acc.getNumPts(), idx + 1);
  }

  for (int idx = num_pts_subset; idx < num_pts; idx++)
  {
    pt_acc_other.add(pc[idx]);
  }

  pt_acc.sub(pt_acc_other);

  VectorType mean = VectorType::Zero();
  MatrixType cov = MatrixType::Zero();

  for (int idx = 0; idx < num_pts_subset; idx++)
  {
    mean += pc[idx].value<fieldIdx>();
  }

  mean /= num_pts_subset;

  for (int idx = 0; idx < num_pts_subset; idx++)
  {
    VectorType xmean = pc[idx].value<fieldIdx>() - mean;
    cov += xmean * xmean.transpose();
  }

  cov /= num_pts_subset;

  EXPECT_TRUE(pt_acc.getMean().isApprox(mean, 1e-6));
  EXPECT_TRUE(pt_acc.getCovariance().isApprox(cov, 1e-6));
}  // TEST(pcl_point_utils, pointFieldAccumulatorGetAccCovarianceSubOther)

}  // namespace testing
