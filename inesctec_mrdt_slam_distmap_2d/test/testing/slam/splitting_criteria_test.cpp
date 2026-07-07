#include <gtest/gtest.h>

#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/splitting_criteria.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

class SplittingCriteriaTrackerTest
{
 public:

  struct Stats
  {
    float m_inliers_ratio;  //!< inliers ratio
  };  // struct SplittingCriteriaTrackerTest::Stats

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

};  // class SplittingCriteriaTrackerTest

TEST(slam_splitting_criteria, rotation)
{
  using TrackerType = SplittingCriteriaTrackerTest;
  using SplittingCriteriaType =
      SplittingCriteriaRotation_<SplittingCriteriaTrackerTest>;
  using EstimateType = SplittingCriteriaType::EstimateType;

  TrackerType tracker;
  SplittingCriteriaType criteria;

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_travel_heading, geometry::degToRad(30.0f));
  ASSERT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  criteria.forceSplit();
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());

  std::cout << "Splitting criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_splitting_criteria, rotation)

TEST(slam_splitting_criteria, translation)
{
  using TrackerType = SplittingCriteriaTrackerTest;
  using SplittingCriteriaType =
      SplittingCriteriaTranslation_<SplittingCriteriaTrackerTest>;
  using EstimateType = SplittingCriteriaType::EstimateType;

  TrackerType tracker;
  SplittingCriteriaType criteria;

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_travel_dist, 0.50f);
  ASSERT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  criteria.forceSplit();
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(75.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-75.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.4f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.6f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -0.4f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -0.6f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.4f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.6f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-0.4f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-0.6f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.25f, 0.25f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.50f, 0.50f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.25f, -0.25f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.50f, -0.50f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());

  std::cout << "Splitting criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_splitting_criteria, translation)

TEST(slam_splitting_criteria, viewpoint)
{
  using TrackerType = SplittingCriteriaTrackerTest;
  using SplittingCriteriaType =
      SplittingCriteriaViewpoint_<SplittingCriteriaTrackerTest>;
  using EstimateType = SplittingCriteriaType::EstimateType;

  TrackerType tracker;
  SplittingCriteriaType criteria;

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_travel_heading, geometry::degToRad(30.0f));
  ASSERT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  criteria.forceSplit();
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());

  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.4f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.6f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -0.4f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -0.6f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.4f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.6f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-0.4f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-0.6f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.25f, 0.25f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.50f, 0.50f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.25f, -0.25f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.50f, -0.50f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());

  std::cout << "Splitting criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_splitting_criteria, viewpoint)

TEST(slam_splitting_criteria, inliers_ratio)
{
  using TrackerType = SplittingCriteriaTrackerTest;
  using AlignerBaseTypeStats =
      SplittingCriteriaTrackerTest::AlignerBaseTypeStats;
  using SplittingCriteriaType =
      SplittingCriteriaInliersRatio_<SplittingCriteriaTrackerTest>;
  using EstimateType = SplittingCriteriaType::EstimateType;

  TrackerType tracker;
  SplittingCriteriaType criteria;

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_travel_dist, 0.10f);
  ASSERT_FLOAT_EQ(criteria.m_min_travel_heading, geometry::degToRad(5.0f));
  ASSERT_FLOAT_EQ(criteria.m_max_ratio_inliers, 0.60f);
  ASSERT_FLOAT_EQ(criteria.m_max_travel_dist, -1.0f);
  ASSERT_FLOAT_EQ(criteria.m_max_travel_heading, -1.0f);
  ASSERT_FALSE(criteria.isToSplit());

  tracker.setRobotInLocalMap(EstimateType::Identity());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  criteria.forceSplit();
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setStats(AlignerBaseTypeStats({0.8f}));
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(75.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-75.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  tracker.setStats(AlignerBaseTypeStats({0.5f}));
  ASSERT_FLOAT_EQ(tracker.getAlignerStats().m_inliers_ratio, 0.5f);

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(25.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(75.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-25.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-50.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-75.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(1.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-1.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -1.f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());

  ASSERT_FLOAT_EQ(tracker.getAlignerStats().m_inliers_ratio, 0.5f);

  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(2.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(4.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-2.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-4.f))));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(0.f, 0.f, geometry::degToRad(-6.f))));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.08f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.08f, 0.08f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, 0.08f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(-0.08f, 0.f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  tracker.setRobotInLocalMap(
      geometry::v2t(Eigen::Vector3f(-0.08f, -0.08f, 0.f)));
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  tracker.setRobotInLocalMap(geometry::v2t(Eigen::Vector3f(0.f, -0.08f, 0.f)));
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  std::cout << "Splitting criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_splitting_criteria, inliers_ratio)

TEST(slam_splitting_criteria, timestamp)
{
  using TrackerType = SplittingCriteriaTrackerTest;
  using AlignerBaseTypeStats =
      SplittingCriteriaTrackerTest::AlignerBaseTypeStats;
  using SplittingCriteriaType =
      SplittingCriteriaTime_<SplittingCriteriaTrackerTest>;
  using EstimateType = SplittingCriteriaType::EstimateType;

  TrackerType tracker;
  SplittingCriteriaType criteria;

  ASSERT_ANY_THROW(criteria.compute());

  criteria.setTracker(&tracker);

  ASSERT_FLOAT_EQ(criteria.m_min_time_interval, 5.0);
  ASSERT_FLOAT_EQ(criteria.m_max_ratio_inliers, 0.60f);
  ASSERT_FALSE(criteria.isToSplit());
  ASSERT_ANY_THROW(criteria.compute());
  ASSERT_FALSE(criteria.isToSplit());

  msg::LaserScanPtr msg = std::make_shared<msg::LaserScan>();

  auto current_time = std::chrono::system_clock::now();
  auto duration_in_seconds =
      std::chrono::duration<double>(current_time.time_since_epoch());

  msg->m_header.m_t = duration_in_seconds.count();

  tracker.setStats(AlignerBaseTypeStats({0.60f}));
  tracker.setRobotInLocalMap(EstimateType::Identity());
  tracker.setScanPtr(msg);
  criteria.reset();
  ASSERT_NO_THROW(criteria.compute());
  EXPECT_FALSE(criteria.isToSplit());
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  criteria.forceSplit();
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.reset();
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());

  msg->m_header.m_t += criteria.m_min_time_interval + 0.1;
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.reset();
  msg->m_header.m_t += 1.0;
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  msg->m_header.m_t -= 2.0;
  criteria.compute();
  EXPECT_FALSE(criteria.isToSplit());
  msg->m_header.m_t += 6.001;
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.reset();

  tracker.setStats(AlignerBaseTypeStats({0.50f}));
  ASSERT_FLOAT_EQ(tracker.getAlignerStats().m_inliers_ratio, 0.5f);
  criteria.reset();
  ASSERT_NO_THROW(criteria.compute());
  EXPECT_TRUE(criteria.isToSplit());
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.forceSplit();
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.reset();
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());

  msg->m_header.m_t += criteria.m_min_time_interval + 0.1;
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.reset();
  msg->m_header.m_t += 1.0;
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  msg->m_header.m_t -= 2.0;
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  msg->m_header.m_t += 6.001;
  criteria.compute();
  EXPECT_TRUE(criteria.isToSplit());
  criteria.reset();

  std::cout << "Splitting criteria parametrization:" << std::endl << criteria;
}  // TEST(slam_splitting_criteria, timestamp)

}  // namespace testing
