#include <gtest/gtest.h>

#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/triggering_criteria.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

TEST(slam_triggering_criteria, empty)
{
  using EstimateType = Eigen::Isometry2f;
  using TriggeringCriteriaType = TriggeringCriteriaEmpty_<EstimateType>;
  using Scalar = TriggeringCriteriaType::Scalar;
  using EstimateVectorType = geometry::Vector3_<Scalar>;

  TriggeringCriteriaType criteria;

  ASSERT_FALSE(criteria.isToTrigger());
  ASSERT_FALSE(
      criteria.isToTrigger(msg::LaserScan(), EstimateType::Identity(), -1.f));

  EstimateType pose;
  msg::LaserScan msg;

  for (int idx = 0; idx < 100; idx++)
  {
    pose = geometry::v2t<Scalar>(EstimateVectorType::Random());

    criteria.compute(msg, pose, Scalar(-1.));
    EXPECT_FALSE(criteria.isToTrigger());
    EXPECT_FALSE(criteria.isToTrigger(msg, pose, Scalar(-1.)));
  }

  criteria.forceTrigger();
  criteria.compute(msg, pose, Scalar(-1.));
  EXPECT_TRUE(criteria.isToTrigger());
  // though trigger forced, isToTrigger only evaluates based on criteria param
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, Scalar(-1.)));
  criteria.compute(msg, pose, Scalar(-1.));
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, Scalar(-1.)));

  std::cout << "Triggering criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_triggering_criteria, empty)

TEST(slam_triggering_criteria, rotation)
{
  using EstimateType = Eigen::Isometry2f;
  using TriggeringCriteriaType = TriggeringCriteriaRotation_<EstimateType>;
  using Scalar = TriggeringCriteriaType::Scalar;

  TriggeringCriteriaType criteria;

  criteria.m_min_travel_heading = geometry::degToRad(5.0f);

  ASSERT_FALSE(criteria.isToTrigger());
  ASSERT_FALSE(
      criteria.isToTrigger(msg::LaserScan(), EstimateType::Identity(), -1.f));

  msg::LaserScan msg;
  EstimateType pose = EstimateType::Identity();
  Scalar inliers_ratio = -1.f;

  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  criteria.forceTrigger();
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  // though trigger forced, isToTrigger only evaluates based on criteria param
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = EstimateType::Identity();
  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  criteria.reset(msg, pose);

  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(93.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(96.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(87.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(84.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  std::cout << "Triggering criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_triggering_criteria, rotation)

TEST(slam_triggering_criteria, translation)
{
  using EstimateType = Eigen::Isometry2f;
  using TriggeringCriteriaType = TriggeringCriteriaTranslation_<EstimateType>;
  using Scalar = TriggeringCriteriaType::Scalar;

  TriggeringCriteriaType criteria;

  criteria.m_min_travel_dist = 0.10f;

  ASSERT_FALSE(criteria.isToTrigger());
  ASSERT_FALSE(
      criteria.isToTrigger(msg::LaserScan(), EstimateType::Identity(), -1.f));

  msg::LaserScan msg;
  EstimateType pose = EstimateType::Identity();
  Scalar inliers_ratio = -1.f;

  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  criteria.forceTrigger();
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  // though trigger forced, isToTrigger only evaluates based on criteria param
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(9.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-9.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = EstimateType::Identity();
  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.00f, 0.06f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, 0.12f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, -0.06f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, -0.12f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.06f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.12f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.06f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.12f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.05f, 0.05f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.10f, 0.10f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.05f, -0.05f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.10f, -0.10f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  criteria.reset(msg, pose);

  pose = geometry::v2t(Eigen::Vector3f(5.05f, 5.05f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.10f, 5.10f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(4.95f, 4.95f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(4.90f, 4.90f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  std::cout << "Triggering criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_triggering_criteria, translation)

TEST(slam_triggering_criteria, viewpoint)
{
  using EstimateType = Eigen::Isometry2f;
  using TriggeringCriteriaType = TriggeringCriteriaViewpoint_<EstimateType>;
  using Scalar = TriggeringCriteriaType::Scalar;

  TriggeringCriteriaType criteria;

  criteria.m_min_travel_dist = 0.10f;
  criteria.m_min_travel_heading = geometry::degToRad(5.0f);

  ASSERT_FALSE(criteria.isToTrigger());
  ASSERT_FALSE(
      criteria.isToTrigger(msg::LaserScan(), EstimateType::Identity(), -1.f));

  msg::LaserScan msg;
  EstimateType pose = EstimateType::Identity();
  Scalar inliers_ratio = -1.f;

  pose = EstimateType::Identity();
  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  criteria.forceTrigger();
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  // though trigger forced, isToTrigger only evaluates based on criteria param
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.00f, 0.06f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, 0.12f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, -0.06f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, -0.12f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.06f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.12f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.06f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.12f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.05f, 0.05f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.10f, 0.10f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.05f, -0.05f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.10f, -0.10f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  criteria.reset(msg, pose);

  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(93.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(96.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(87.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(84.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(5.05f, 5.05f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.10f, 5.10f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(4.95f, 4.95f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(4.90f, 4.90f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  std::cout << "Triggering criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_triggering_criteria, viewpoint)

TEST(slam_triggering_criteria, gridcell)
{
  using EstimateType = Eigen::Isometry2f;
  using TriggeringCriteriaType = TriggeringCriteriaGridCell_<EstimateType>;
  using Scalar = TriggeringCriteriaType::Scalar;

  TriggeringCriteriaType criteria;

  criteria.m_cell_size_2 = 0.10f;

  ASSERT_FALSE(criteria.isToTrigger());
  ASSERT_FALSE(
      criteria.isToTrigger(msg::LaserScan(), EstimateType::Identity(), -1.f));

  msg::LaserScan msg;
  EstimateType pose = EstimateType::Identity();
  Scalar inliers_ratio = -1.f;

  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  criteria.forceTrigger();
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  // though trigger forced, isToTrigger only evaluates based on criteria param
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(9.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-3.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-9.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = EstimateType::Identity();
  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.00f, 0.06f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, 0.12f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, -0.06f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.00f, -0.12f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.06f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.12f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.06f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.12f, 0.00f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.05f, 0.05f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.10f, 0.10f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.05f, -0.05f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.10f, -0.10f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  criteria.reset(msg, pose);

  pose = geometry::v2t(Eigen::Vector3f(5.05f, 5.05f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.10f, 5.10f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(5.11f, 5.11f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(4.95f, 4.95f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(4.90f, 4.90f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(4.89f, 4.89f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  pose = geometry::v2t(Eigen::Vector3f(0.95f, 0.95f, geometry::degToRad(90.f)));
  criteria.reset(msg, pose);

  pose = geometry::v2t(Eigen::Vector3f(1.00f, 1.00f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(1.10f, 1.10f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(1.11f, 1.11f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.90f, 0.90f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.80f, 0.80f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.79f, 0.79f, geometry::degToRad(90.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  std::cout << "Triggering criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_triggering_criteria, gridcell)

TEST(slam_triggering_criteria, inliers_ratio)
{
  using EstimateType = Eigen::Isometry2f;
  using TriggeringCriteriaType = TriggeringCriteriaInliersRatio_<EstimateType>;
  using Scalar = TriggeringCriteriaType::Scalar;

  TriggeringCriteriaType criteria;

  criteria.m_min_travel_dist = 0.10f;
  criteria.m_min_travel_heading = geometry::degToRad(5.0f);
  criteria.m_max_ratio_inliers = 0.80f;
  criteria.m_max_travel_dist = -1.0f;
  criteria.m_max_travel_heading = -1.0f;

  ASSERT_FALSE(criteria.isToTrigger());
  ASSERT_FALSE(
      criteria.isToTrigger(msg::LaserScan(), EstimateType::Identity(), -1.f));

  msg::LaserScan msg;
  EstimateType pose = EstimateType::Identity();
  Scalar inliers_ratio = -1.f;

  pose = EstimateType::Identity();
  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  criteria.forceTrigger();
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  // though trigger forced, isToTrigger only evaluates based on criteria param
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  inliers_ratio = 0.8f;

  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(75.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-75.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  inliers_ratio = 0.5f;

  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(75.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-75.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  inliers_ratio = 0.5f;

  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(2.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(4.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-2.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-4.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f)));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.08f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.08f, 0.08f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, 0.08f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.08f, 0.f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(-0.08f, -0.08f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  pose = geometry::v2t(Eigen::Vector3f(0.f, -0.08f, 0.f));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  std::cout << "Triggering criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_triggering_criteria, inliers_ratio)

TEST(slam_triggering_criteria, timestamp)
{
  using EstimateType = Eigen::Isometry2f;
  using TriggeringCriteriaType = TriggeringCriteriaTime_<EstimateType>;
  using Scalar = TriggeringCriteriaType::Scalar;

  TriggeringCriteriaType criteria;

  criteria.m_min_time_interval = 1.0;
  criteria.m_max_ratio_inliers = 0.80f;

  ASSERT_FALSE(criteria.isToTrigger());
  ASSERT_FALSE(
      criteria.isToTrigger(msg::LaserScan(), EstimateType::Identity(), 1.f));

  msg::LaserScan msg;
  EstimateType pose = EstimateType::Identity();
  Scalar inliers_ratio = -1.f;

  auto current_time = std::chrono::system_clock::now();
  auto duration_in_seconds =
      std::chrono::duration<double>(current_time.time_since_epoch());

  msg.m_header.m_t = duration_in_seconds.count();

  inliers_ratio = 0.80f;

  criteria.reset(msg, pose);
  ASSERT_NO_THROW(criteria.compute(msg, pose, inliers_ratio));
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.forceTrigger();
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  // though trigger forced, isToTrigger only evaluates based on criteria param
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));

  msg.m_header.m_t += criteria.m_min_time_interval + 0.1;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.reset(msg, pose);
  msg.m_header.m_t += 1.0;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  msg.m_header.m_t -= 2.0;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_FALSE(criteria.isToTrigger());
  EXPECT_FALSE(criteria.isToTrigger(msg, pose, inliers_ratio));
  msg.m_header.m_t += 6.001;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.reset(msg, pose);

  inliers_ratio = 0.50f;

  criteria.reset(msg, pose);
  ASSERT_NO_THROW(criteria.compute(msg, pose, inliers_ratio));
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.forceTrigger();
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.reset(msg, pose);
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));

  msg.m_header.m_t += criteria.m_min_time_interval + 0.1;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.reset(msg, pose);
  msg.m_header.m_t += 1.0;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  msg.m_header.m_t -= 2.0;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  msg.m_header.m_t += 6.001;
  criteria.compute(msg, pose, inliers_ratio);
  EXPECT_TRUE(criteria.isToTrigger());
  EXPECT_TRUE(criteria.isToTrigger(msg, pose, inliers_ratio));
  criteria.reset(msg, pose);

  std::cout << "Triggering criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_triggering_criteria, timestamp)

}  // namespace testing