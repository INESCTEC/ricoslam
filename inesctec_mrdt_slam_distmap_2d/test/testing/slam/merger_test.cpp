#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/merger.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

using VariableType =
    srrg2_solver::VariableSE2DistanceMapNanoStaticPoint2fVectorCloudRight;
using VariableTypePtr = std::shared_ptr<VariableType>;

using EstimateType = VariableType::EstimateType;

using MeasurementType = typename VariableType::MeasurementType;
using MeasurementTypeElement = typename MeasurementType::element_type;

using PointCloudType = VariableType::MeasurementOwnerType::PointCloudType;
using PointCloudTypePtr = VariableType::MeasurementOwnerType::PointCloudTypePtr;

using DistanceMapType = VariableType::MeasurementOwnerType::DistanceMapType;

using MergerType = Merger_<EstimateType, DistanceMapType, PointCloudType>;
using MergerTypeParam = MergerType::Param;

TEST(slam_merger, constructor)
{
  MergerTypeParam merger_param;
  merger_param.m_ray_min_dist_add_new = 0.10f;
  merger_param.m_ray_min_dist_through = 0.10f;
  merger_param.m_enable_avg = false;
  merger_param.m_weight_new = -1.f;
  merger_param.m_weight_old = -1.f;

  ASSERT_NO_THROW(MergerType merger(merger_param));

  MergerType merger;
  ASSERT_NO_THROW(merger.setParam(merger_param));

  merger_param.m_enable_avg = true;

  EXPECT_ANY_THROW(MergerType merger(merger_param));
  EXPECT_ANY_THROW(merger.setParam(merger_param));

  merger_param.m_weight_new = 1.f;
  merger_param.m_weight_old = 0.5f;

  EXPECT_ANY_THROW(MergerType merger(merger_param));
  EXPECT_ANY_THROW(merger.setParam(merger_param));

  merger_param.m_weight_new = 0.5f;
  merger_param.m_weight_old = 1.f;

  EXPECT_ANY_THROW(MergerType merger(merger_param));
  EXPECT_ANY_THROW(merger.setParam(merger_param));

  merger_param.m_weight_new = 2.f;
  merger_param.m_weight_old = -1.f;

  EXPECT_ANY_THROW(MergerType merger(merger_param));
  EXPECT_ANY_THROW(merger.setParam(merger_param));

  merger_param.m_weight_new = -1.f;
  merger_param.m_weight_old = 2.f;

  EXPECT_ANY_THROW(MergerType merger(merger_param));
  EXPECT_ANY_THROW(merger.setParam(merger_param));

  merger_param.m_weight_new = 0.5f;
  merger_param.m_weight_old = 0.5f;

  EXPECT_NO_THROW(MergerType merger(merger_param));
  EXPECT_NO_THROW(merger.setParam(merger_param));

  merger_param.m_weight_new = 0.25f;
  merger_param.m_weight_old = 0.75f;

  EXPECT_NO_THROW(MergerType merger(merger_param));
  EXPECT_NO_THROW(merger.setParam(merger_param));

  merger_param.m_weight_new = 0.75f;
  merger_param.m_weight_old = 0.25f;

  EXPECT_NO_THROW(MergerType merger(merger_param));
  EXPECT_NO_THROW(merger.setParam(merger_param));

  merger_param.m_ray_min_dist_add_new = -0.10f;  // add new disabled

  EXPECT_NO_THROW(MergerType merger(merger_param));
  EXPECT_NO_THROW(merger.setParam(merger_param));

  merger_param.m_ray_min_dist_through = -0.10f;  // see through disabled

  EXPECT_NO_THROW(MergerType merger(merger_param));
  EXPECT_NO_THROW(merger.setParam(merger_param));

  merger_param.m_ray_min_dist_add_new = 0.10f;
  merger_param.m_ray_min_dist_through = 0.10f;
  merger_param.m_enable_avg = false;
  merger_param.m_weight_new = -1.f;
  merger_param.m_weight_old = -1.f;
}  // TEST(slam_merger, constructor)

TEST(slam_merger, merge)
{
  using LaserScanMsgType = MergerType::LaserScanMsgType;

  constexpr float kDistMapRes = 0.03f;
  constexpr float kDistMapSize = 10.f;
  constexpr float kDistMapMaxDist = 0.5f;

  const float angle_min = geometry::degToRad(-135.0f);
  const float angle_max = geometry::degToRad(135.0f);
  const float angle_increment = geometry::degToRad(0.25f);
  // constexpr int num_rays = 1081;
  constexpr float time_increment = -1.f;
  constexpr float scan_time = -1.f;
  constexpr float range_min = 0.06f;
  constexpr float range_max = 10.0f;

  constexpr float radius = 2.5f;

  std::random_device rd;
  std::mt19937 rng(rd());

  MergerTypeParam merger_param;
  merger_param.m_ray_min_dist_add_new = 0.10f;
  merger_param.m_ray_min_dist_through = 0.10f;
  merger_param.m_enable_avg = false;
  merger_param.m_weight_new = -1.f;
  merger_param.m_weight_old = -1.f;

  LaserScanMsgType scan({}, angle_min, angle_max, angle_increment,
                        time_increment, scan_time, range_min, range_max, {},
                        {});

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudType pc_orig;

  test::getLaser2DPointCloudCircle(*pc, radius, scan.m_angle_inc,
                                   scan.m_angle_max - scan.m_angle_min,
                                   scan.m_range_max, rng);

  pc_orig = *pc;

  ASSERT_EQ(pc_orig.size(), pc->size());

  VariableTypePtr local_map = std::make_shared<VariableType>();
  local_map->setMeasurement(std::make_shared<MeasurementTypeElement>());
  local_map->setSensorInRobot(EstimateType::Identity());
  local_map->measurement()->m_sensor_in_robot = EstimateType::Identity();
  local_map->measurement()->m_pc = pc;
  local_map->measurement()->getDistanceMap()->setGoals(
      *pc, kDistMapRes, kDistMapSize, kDistMapMaxDist);

  MergerType merger(merger_param);

  EXPECT_NO_THROW(merger.merge(*pc, scan, local_map->getSensorInRobot(),
                               EstimateType::Identity(),
                               local_map->measurement()->getPointCloud(),
                               local_map->measurement()->getDistanceMap()));

  EXPECT_EQ(pc_orig.size(), local_map->measurement()->getPointCloud()->size());

  for (size_t idx = 0; idx < local_map->measurement()->getPointCloud()->size();
       idx++)
  {
    EXPECT_TRUE(pc_orig[idx].coordinates().isApprox(local_map->measurement()
                                                        ->getPointCloud()
                                                        ->operator[](idx)
                                                        .coordinates()));
  }
}

}  // namespace testing
