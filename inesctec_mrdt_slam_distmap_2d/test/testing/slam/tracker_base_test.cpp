#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/slam/tracker.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

using VariableType =
    srrg2_solver::VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight;
using EstimateType = VariableType::EstimateType;

using TrackerType = Tracker_<VariableType>;
using TrackerTypeParam = TrackerType::Param;

using PointCloudType = TrackerType::PointCloudType;
using PointCloudTypePtr = TrackerType::PointCloudTypePtr;

using AlignerType = AlignerDistanceMapPointPoint_<VariableType>;
using AlignerTypePtr = std::shared_ptr<AlignerType>;

using AlignerSolverParamType = AlignerType::Param;
using AlignerDistMapParamType = AlignerType::DistMapParam;

using MergerType = TrackerType::MergerType;
using MergerTypePtr = TrackerType::MergerTypePtr;
using MergerTypeParam = MergerType::Param;

TEST(slam_tracker_base, constructor)
{
  using SplittingCriteriaType = TriggeringCriteriaTranslation_<EstimateType>;
  using SplittingCriteriaTypePtr = std::shared_ptr<SplittingCriteriaType>;

  using MergingCriteriaType = TriggeringCriteriaTranslation_<EstimateType>;
  using MergingCriteriaTypePtr = std::shared_ptr<MergingCriteriaType>;

  EXPECT_ANY_THROW(TrackerType tracker);

  AlignerSolverParamType aligner_param_solver;
  aligner_param_solver.m_debug = false;
  aligner_param_solver.m_solver_verbose = true;
  aligner_param_solver.m_solver_damping = 1.f;
  aligner_param_solver.m_solver_max_iterations = 25;
  aligner_param_solver.m_solver_criteria_type = "stepnorm";
  aligner_param_solver.m_solver_criteria_epsilon = 1e-5;
  aligner_param_solver.m_solver_robustifier_type = "cauchy";
  aligner_param_solver.m_solver_robustifier_num_iterations_coarse = 10;
  aligner_param_solver.m_solver_robustifier_threshold_coarse = 0.025f;
  aligner_param_solver.m_solver_robustifier_threshold_fine = 0.0025f;
  aligner_param_solver.m_failure_ratio_inliers = 0.35f;
  aligner_param_solver.m_failure_chi_inliers = -1.0f;
  aligner_param_solver.m_dbg_ogl_interval = -1;
  aligner_param_solver.m_dbg_ogl_mode = "";

  AlignerDistMapParamType aligner_param_dist_map;
  aligner_param_dist_map.m_dist_map_res = 0.01f;
  aligner_param_dist_map.m_dist_map_size = -1.f;
  aligner_param_dist_map.m_dist_map_max_dist = 0.2f;

  AlignerTypePtr aligner;

  ASSERT_NO_THROW(aligner = std::make_shared<AlignerType>(
                      aligner_param_solver, aligner_param_dist_map));

  TrackerTypeParam tracker_param;

  tracker_param.m_aligner = aligner;

  EXPECT_NO_THROW(TrackerType tracker(tracker_param));

  SplittingCriteriaTypePtr splitting_criteria =
      std::make_shared<SplittingCriteriaType>();

  splitting_criteria->m_min_travel_dist = 0.5f;

  tracker_param.m_splitting_criteria = splitting_criteria;

  EXPECT_NO_THROW(TrackerType tracker(tracker_param));

  MergingCriteriaTypePtr merging_criteria =
      std::make_shared<MergingCriteriaType>();

  merging_criteria->m_min_travel_dist = 0.1f;

  tracker_param.m_merging_criteria = merging_criteria;

  EXPECT_ANY_THROW(TrackerType tracker(tracker_param));

  MergerTypeParam merger_param;
  merger_param.m_ray_min_dist_through = 0.1f;
  merger_param.m_ray_min_dist_add_new = 0.1f;
  merger_param.m_enable_avg = false;
  merger_param.m_weight_old = -1.f;
  merger_param.m_weight_new = -1.f;

  MergerTypePtr merger = std::make_shared<MergerType>(merger_param);

  tracker_param.m_merger = merger;

  EXPECT_NO_THROW(TrackerType tracker(tracker_param));

  std::cout << "Tracker parametrization:" << std::endl << tracker_param;
}  // TEST(slam_tracker_base, constructor)

/* TEST(slam_tracker_base, splitTranslationNoMerge)
{
  using SplittingCriteriaType = SplittingCriteriaTranslation_<TrackerType>;
  using SplittingCriteriaTypePtr = std::shared_ptr<SplittingCriteriaType>;

  TrackerTypeParam tracker_param;
  tracker_param.m_use_ext_odom_as_init = false;
  tracker_param.m_use_ext_odom_as_prior = false;
  tracker_param.m_enable_voxelization = true;
  tracker_param.m_voxel_scale = -1;

  ASSERT_NO_THROW(tracker_param.m_aligner = std::make_shared<AlignerType>());
  ASSERT_NO_THROW(tracker_param.m_sensor =
                      std::make_shared<SensorModelType>(SensorModelTypeParam(
                          geometry::degToRad(-180.f), geometry::degToRad(180.f),
                          geometry::degToRad(360.f) / 1440, 1441, -1.f, -1.f,
                          0.06f, 10.f)));

  tracker_param.m_merging_criteria = nullptr;
  tracker_param.m_merger = nullptr;

  SplittingCriteriaTypePtr split_translation =
      std::make_shared<SplittingCriteriaType>();

  split_translation->m_min_travel_dist = 0.5f;

  tracker_param.m_splitting_criteria = split_translation;

  ASSERT_NO_THROW(TrackerType tracker(tracker_param));
  TrackerType tracker(tracker_param);

  EstimateType gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_moving = std::make_shared<PointCloudType>();

  test::getPointCloudSquare(*pc, 4.f, 400);

  pc_moving->resize(pc->size());
  pc->copyTo(pc_moving->begin());

  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initializing);
  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initialized);

  for (float motion_x = 0.f; motion_x < 0.5f; motion_x += 0.1f)
  {
    gt = geometry::v2t(geometry::Vector3f(motion_x, 0.f, 0.f));

    *pc_moving = pc->transform(gt.inverse());

    EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
    EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
    EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

    // std::cout << "gt  : " << geometry::t2v(gt) << std::endl;
    // std::cout << "odom: " << geometry::t2v(tracker.getRobotInOdom())
    //           << std::endl;

}

gt = geometry::v2t(geometry::Vector3f(0.55f, 0.f, 0.f));

*pc_moving = pc->transform(gt.inverse());

EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
EXPECT_EQ(tracker.getStatus(), TrackerType::Status::TrackingMeasurementSplit);
EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

gt = geometry::v2t(geometry::Vector3f(0.70f, 0.f, 0.f));

*pc_moving = pc->transform(gt.inverse());

EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));
}  // TEST(slam_tracker_base, splitTranslationNoMerge)

TEST(slam_tracker_base, splitRotationNoMerge)
{
  using SplittingCriteriaType = SplittingCriteriaRotation_<TrackerType>;
  using SplittingCriteriaTypePtr = std::shared_ptr<SplittingCriteriaType>;

  TrackerTypeParam tracker_param;
  tracker_param.m_use_ext_odom_as_init = false;
  tracker_param.m_use_ext_odom_as_prior = false;
  tracker_param.m_enable_voxelization = true;
  tracker_param.m_voxel_scale = -1;

  AlignerDistMapParamType distmap_param;
  distmap_param.m_dist_map_max_dist = 0.3f;

  ASSERT_NO_THROW(tracker_param.m_aligner = std::make_shared<AlignerType>(
                      AlignerSolverParamType(), distmap_param));
  ASSERT_NO_THROW(tracker_param.m_sensor =
                      std::make_shared<SensorModelType>(SensorModelTypeParam(
                          geometry::degToRad(-180.f), geometry::degToRad(180.f),
                          geometry::degToRad(360.f) / 1440, 1441, -1.f, -1.f,
                          0.06f, 10.f)));

  tracker_param.m_merging_criteria = nullptr;
  tracker_param.m_merger = nullptr;

  SplittingCriteriaTypePtr split_rotation =
      std::make_shared<SplittingCriteriaType>();

  split_rotation->m_min_travel_heading = geometry::degToRad(45.f);

  tracker_param.m_splitting_criteria = split_rotation;

  ASSERT_NO_THROW(TrackerType tracker(tracker_param));
  TrackerType tracker(tracker_param);

  EstimateType gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_moving = std::make_shared<PointCloudType>();

  test::getPointCloudSquare(*pc, 4.f, 400);

  pc_moving->resize(pc->size());
  pc->copyTo(pc_moving->begin());

  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initializing);
  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initialized);

  for (float motion_th = 0.f; motion_th < 45.0f; motion_th += 4.0f)
  {
    gt = geometry::v2t(
        geometry::Vector3f(0.f, 0.f, geometry::degToRad(motion_th)));

    *pc_moving = pc->transform(gt.inverse());

    EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
    EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
    EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

    // std::cout << "gt  : " << geometry::t2v(gt) << std::endl;
    // std::cout << "odom: " << geometry::t2v(tracker.getRobotInOdom())
    //           << std::endl;
  }

  gt = geometry::v2t(geometry::Vector3f(0.f, 0.f, geometry::degToRad(48.f)));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::TrackingMeasurementSplit);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

  for (float motion_th = 0.f; motion_th < 45.0f; motion_th += 4.0f)
  {
    gt = geometry::v2t(
        geometry::Vector3f(0.f, 0.f, geometry::degToRad(48.f + motion_th)));

    *pc_moving = pc->transform(gt.inverse());

    EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
    EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
    EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

    // std::cout << "gt  : " << geometry::t2v(gt) << std::endl;
    // std::cout << "odom: " << geometry::t2v(tracker.getRobotInOdom())
    //           << std::endl;
  }

  gt = geometry::v2t(
      geometry::Vector3f(0.f, 0.f, geometry::degToRad(48.f + 48.f)));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::TrackingMeasurementSplit);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

  gt = geometry::v2t(
      geometry::Vector3f(0.f, 0.f, geometry::degToRad(48.f + 52.f)));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));
}  // TEST(slam_tracker_base, splitRotationNoMerge)

TEST(slam_tracker_base, splitViewpointNoMerge)
{
  using SplittingCriteriaType = SplittingCriteriaViewpoint_<TrackerType>;
  using SplittingCriteriaTypePtr = std::shared_ptr<SplittingCriteriaType>;

  TrackerTypeParam tracker_param;
  tracker_param.m_use_ext_odom_as_init = false;
  tracker_param.m_use_ext_odom_as_prior = false;
  tracker_param.m_enable_voxelization = true;
  tracker_param.m_voxel_scale = -1;

  ASSERT_NO_THROW(tracker_param.m_aligner = std::make_shared<AlignerType>());
  ASSERT_NO_THROW(tracker_param.m_sensor =
                      std::make_shared<SensorModelType>(SensorModelTypeParam(
                          geometry::degToRad(-180.f), geometry::degToRad(180.f),
                          geometry::degToRad(360.f) / 1440, 1441, -1.f, -1.f,
                          0.06f, 10.f)));

  tracker_param.m_merging_criteria = nullptr;
  tracker_param.m_merger = nullptr;

  SplittingCriteriaTypePtr split_viewpoint =
      std::make_shared<SplittingCriteriaType>();

  split_viewpoint->m_min_travel_dist = 0.5f;
  split_viewpoint->m_min_travel_heading = geometry::degToRad(45.f);

  tracker_param.m_splitting_criteria = split_viewpoint;

  ASSERT_NO_THROW(TrackerType tracker(tracker_param));
  TrackerType tracker(tracker_param);

  EstimateType gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_moving = std::make_shared<PointCloudType>();

  test::getPointCloudSquare(*pc, 4.f, 400);

  pc_moving->resize(pc->size());
  pc->copyTo(pc_moving->begin());

  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initializing);
  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initialized);

  for (float motion_x = 0.f; motion_x < 0.5f; motion_x += 0.1f)
  {
    gt = geometry::v2t(geometry::Vector3f(motion_x, 0.f, 0.f));

    *pc_moving = pc->transform(gt.inverse());

    EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
    EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
    EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

    // std::cout << "gt  : " << geometry::t2v(gt) << std::endl;
    // std::cout << "odom: " << geometry::t2v(tracker.getRobotInOdom())
    //           << std::endl;
  }

  gt = geometry::v2t(geometry::Vector3f(0.55f, 0.f, 0.f));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::TrackingMeasurementSplit);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

  gt = geometry::v2t(geometry::Vector3f(0.60f, 0.f, 0.f));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.025f));

  for (float motion_th = 0.f; motion_th < 45.0f; motion_th += 4.0f)
  {
    gt = geometry::v2t(
        geometry::Vector3f(0.60f, 0.f, geometry::degToRad(motion_th)));

    *pc_moving = pc->transform(gt.inverse());

    EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
    EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
    EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.025f));

    // std::cout << "gt  : " << geometry::t2v(gt) << std::endl;
    // std::cout << "odom: " << geometry::t2v(tracker.getRobotInOdom())
    //           << std::endl;
  }

  gt = geometry::v2t(geometry::Vector3f(0.60f, 0.f, geometry::degToRad(48.f)));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::TrackingMeasurementSplit);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.025f));
}  // TEST(slam_tracker_base, splitViewpointNoMerge)

TEST(slam_tracker_base, splitInliersRatioNoMerge)
{
  using SplittingCriteriaType = SplittingCriteriaInliersRatio_<TrackerType>;
  using SplittingCriteriaTypePtr = std::shared_ptr<SplittingCriteriaType>;

  TrackerTypeParam tracker_param;
  tracker_param.m_use_ext_odom_as_init = false;
  tracker_param.m_use_ext_odom_as_prior = false;
  tracker_param.m_enable_voxelization = true;
  tracker_param.m_voxel_scale = -1;

  ASSERT_NO_THROW(tracker_param.m_aligner = std::make_shared<AlignerType>());
  ASSERT_NO_THROW(tracker_param.m_sensor =
                      std::make_shared<SensorModelType>(SensorModelTypeParam(
                          geometry::degToRad(-180.f), geometry::degToRad(180.f),
                          geometry::degToRad(360.f) / 1440, 1441, -1.f, -1.f,
                          0.06f, 10.f)));

  tracker_param.m_merging_criteria = nullptr;
  tracker_param.m_merger = nullptr;

  SplittingCriteriaTypePtr split_inliers =
      std::make_shared<SplittingCriteriaType>();

  split_inliers->m_min_travel_dist = 0.05f;
  split_inliers->m_min_travel_heading = geometry::degToRad(2.5f);
  split_inliers->m_max_travel_dist = 2.5f;
  split_inliers->m_max_ratio_inliers = 0.5f;

  tracker_param.m_splitting_criteria = split_inliers;

  ASSERT_NO_THROW(TrackerType tracker(tracker_param));
  TrackerType tracker(tracker_param);

  EstimateType gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_moving = std::make_shared<PointCloudType>();

  test::getPointCloudSquare(*pc, 4.f, 400);

  pc_moving->resize(pc->size());
  pc->copyTo(pc_moving->begin());

  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initializing);
  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Initialized);

  for (float motion_x = 0.f; motion_x < 2.5f; motion_x += 0.3f)
  {
    gt = geometry::v2t(geometry::Vector3f(motion_x, 0.f, 0.f));

    *pc_moving = pc->transform(gt.inverse());

    EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
    EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
    EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

    // std::cout << "gt  : " << geometry::t2v(gt) << std::endl;
    // std::cout << "odom: " << geometry::t2v(tracker.getRobotInOdom())
    //           << std::endl;
  }

  gt = geometry::v2t(geometry::Vector3f(2.7f, 0.f, 0.f));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::TrackingMeasurementSplit);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));

  gt = geometry::v2t(geometry::Vector3f(3.0f, 0.f, 0.f));

  *pc_moving = pc->transform(gt.inverse());

  EXPECT_NO_THROW(tracker.processPointCloud(pc_moving));
  EXPECT_EQ(tracker.getStatus(), TrackerType::Status::Tracking);
  EXPECT_TRUE(tracker.getRobotInOdom().isApprox(gt, 0.01f));
}  // TEST(slam_tracker_base, splitInliersRatioNoMerge) */

}  // namespace testing
