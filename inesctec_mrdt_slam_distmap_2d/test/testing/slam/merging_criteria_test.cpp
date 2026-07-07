#include <gtest/gtest.h>

#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/merging_criteria.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

class MergingCriteriaTrackerTest
{
 public:

  struct Stats
  {
    float m_inliers_ratio;  //!< inliers ratio
  };  // struct MergingCriteriaTrackerTest::Stats

 public:

  using EstimateType = Eigen::Isometry2f;
  using AlignerBaseTypeStats = Stats;

 protected:

  EstimateType m_robot_in_local_ = EstimateType::Identity();
  AlignerBaseTypeStats m_stats_ = {0.f};

  msg::LaserScanPtr m_scan_ = nullptr;  //!< latest laser scan message

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline const EstimateType& getRobotInLocalMap() const
  {
    return m_robot_in_local_;
  }

  inline void setRobotInLocalMap(const EstimateType& robot_in_local)
  {
    m_robot_in_local_ = robot_in_local;
  }

  inline const AlignerBaseTypeStats& getAlignerStats() const
  {
    return m_stats_;
  }

  inline void setStats(const AlignerBaseTypeStats& stats) { m_stats_ = stats; }

  inline msg::LaserScan* getScan() const { return m_scan_.get(); }
  inline const msg::LaserScanPtr& getScanPtr() const { return m_scan_; }
  inline void setScanPtr(const msg::LaserScanPtr& scan) { m_scan_ = scan; }

};  // class MergingCriteriaTrackerTest

TEST(slam_merging_criteria, rotation)
{
  using TrackerType = MergingCriteriaTrackerTest;
  using MergingCriteriaType =
      MergingCriteriaRotation_<MergingCriteriaTrackerTest>;
  using EstimateType = MergingCriteriaType::EstimateType;

  TrackerType tracker;
  MergingCriteriaType criteria;

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_travel_heading, geometry::degToRad(5.0f));
  ASSERT_TRUE(criteria.getRobotInMapInit().isApprox(EstimateType::Identity()));
  ASSERT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.reset();
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  criteria.forceMerge();
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  criteria.reset();
  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(3.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-3.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  criteria.reset();
  EXPECT_TRUE(criteria.getRobotInMapInit().isApprox(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f)))));

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(93.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(96.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(87.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(84.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  std::cout << "Merging criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_merging_criteria, rotation)

TEST(slam_merging_criteria, translation)
{
  using TrackerType = MergingCriteriaTrackerTest;
  using MergingCriteriaType =
      MergingCriteriaTranslation_<MergingCriteriaTrackerTest>;
  using EstimateType = MergingCriteriaType::EstimateType;

  TrackerType tracker;
  MergingCriteriaType criteria;

  criteria.setTracker(&tracker);
  criteria.reset();

  ASSERT_FLOAT_EQ(criteria.m_min_travel_dist, 0.10f);
  ASSERT_TRUE(criteria.getRobotInMapInit().isApprox(EstimateType::Identity()));
  ASSERT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.reset();
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  criteria.forceMerge();
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(3.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(9.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-3.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-9.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  criteria.reset();
  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.00f, 0.06f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.00f, 0.12f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.00f, -0.06f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.00f, -0.12f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.06f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.12f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.06f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.12f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.05f, 0.05f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.10f, 0.10f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.05f, -0.05f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.10f, -0.10f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  criteria.reset();
  EXPECT_TRUE(criteria.getRobotInMapInit().isApprox(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f)))));

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.05f, 5.05f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.10f, 5.10f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(4.95f, 4.95f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(4.90f, 4.90f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  std::cout << "Merging criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_merging_criteria, translation)

TEST(slam_merging_criteria, viewpoint)
{
  using TrackerType = MergingCriteriaTrackerTest;
  using MergingCriteriaType =
      MergingCriteriaViewpoint_<MergingCriteriaTrackerTest>;
  using EstimateType = MergingCriteriaType::EstimateType;

  TrackerType tracker;
  MergingCriteriaType criteria;

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_travel_dist, 0.10f);
  ASSERT_FLOAT_EQ(criteria.m_min_travel_heading, geometry::degToRad(5.0f));
  ASSERT_TRUE(criteria.getRobotInMapInit().isApprox(EstimateType::Identity()));
  ASSERT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.reset();
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  criteria.forceMerge();
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.00f, 0.06f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.00f, 0.12f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.00f, -0.06f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.00f, -0.12f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.06f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.12f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.06f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.12f, 0.00f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.05f, 0.05f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.10f, 0.10f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.05f, -0.05f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.10f, -0.10f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(3.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-3.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  criteria.reset();
  EXPECT_TRUE(criteria.getRobotInMapInit().isApprox(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(90.f)))));

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(93.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(96.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(87.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.f, 5.f, geometry::degToRad(84.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.05f, 5.05f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(5.10f, 5.10f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(4.95f, 4.95f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(4.90f, 4.90f, geometry::degToRad(90.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  std::cout << "Merging criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_merging_criteria, viewpoint)

TEST(slam_merging_criteria, inliers_ratio)
{
  using TrackerType = MergingCriteriaTrackerTest;
  using AlignerBaseTypeStats = MergingCriteriaTrackerTest::AlignerBaseTypeStats;
  using MergingCriteriaType =
      MergingCriteriaInliersRatio_<MergingCriteriaTrackerTest>;
  using EstimateType = MergingCriteriaType::EstimateType;

  TrackerType tracker;
  MergingCriteriaType criteria;

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_travel_dist, 0.10f);
  ASSERT_FLOAT_EQ(criteria.m_min_travel_heading, geometry::degToRad(5.0f));
  ASSERT_FLOAT_EQ(criteria.m_max_ratio_inliers, 0.80f);
  ASSERT_FLOAT_EQ(criteria.m_max_travel_dist, -1.0f);
  ASSERT_FLOAT_EQ(criteria.m_max_travel_heading, -1.0f);
  ASSERT_FALSE(criteria.isToMerge());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  criteria.forceMerge();
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  tracker.setStats(AlignerBaseTypeStats({0.8f}));
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(75.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-75.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  tracker.setStats(AlignerBaseTypeStats({0.5f}));
  ASSERT_FLOAT_EQ(tracker.getAlignerStats().m_inliers_ratio, 0.5f);

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(75.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-75.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  ASSERT_FLOAT_EQ(tracker.getAlignerStats().m_inliers_ratio, 0.5f);

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(2.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(4.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-2.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-4.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.08f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.08f, 0.08f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.08f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-0.08f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.08f, -0.08f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -0.08f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  std::cout << "Merging criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_merging_criteria, inliers_ratio)

TEST(slam_merging_criteria, timestamp)
{
  using TrackerType = MergingCriteriaTrackerTest;
  using AlignerBaseTypeStats = MergingCriteriaTrackerTest::AlignerBaseTypeStats;
  using MergingCriteriaType = MergingCriteriaTime_<MergingCriteriaTrackerTest>;
  using EstimateType = MergingCriteriaType::EstimateType;

  TrackerType tracker;
  MergingCriteriaType criteria;

  ASSERT_ANY_THROW(criteria.compute());

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_time_interval, 1.0);
  ASSERT_FLOAT_EQ(criteria.m_max_ratio_inliers, 0.80f);
  ASSERT_FALSE(criteria.isToMerge());
  ASSERT_ANY_THROW(criteria.compute());
  ASSERT_FALSE(criteria.isToMerge());

  msg::LaserScanPtr msg = std::make_shared<msg::LaserScan>();

  auto current_time = std::chrono::system_clock::now();
  auto duration_in_seconds =
      std::chrono::duration<double>(current_time.time_since_epoch());

  msg->m_header.m_t = duration_in_seconds.count();

  tracker.setStats(AlignerBaseTypeStats({0.80f}));
  tracker.setRobotInLocalMap(EstimateType::Identity());
  tracker.setScanPtr(msg);
  criteria.reset();
  ASSERT_NO_THROW(criteria.compute());
  EXPECT_FALSE(criteria.isToMerge());
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  criteria.forceMerge();
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.reset();
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());

  msg->m_header.m_t += criteria.m_min_time_interval + 0.1;
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.reset();
  msg->m_header.m_t += 1.0;
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  msg->m_header.m_t -= 2.0;
  criteria.compute();
  EXPECT_FALSE(criteria.isToMerge());
  msg->m_header.m_t += 6.001;
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.reset();

  tracker.setStats(AlignerBaseTypeStats({0.50f}));
  ASSERT_FLOAT_EQ(tracker.getAlignerStats().m_inliers_ratio, 0.5f);
  criteria.reset();
  ASSERT_NO_THROW(criteria.compute());
  EXPECT_TRUE(criteria.isToMerge());
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.forceMerge();
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.reset();
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());

  msg->m_header.m_t += criteria.m_min_time_interval + 0.1;
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.reset();
  msg->m_header.m_t += 1.0;
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  msg->m_header.m_t -= 2.0;
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  msg->m_header.m_t += 6.001;
  criteria.compute();
  EXPECT_TRUE(criteria.isToMerge());
  criteria.reset();

  std::cout << "Merging criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_merging_criteria, timestamp)

}  // namespace testing
