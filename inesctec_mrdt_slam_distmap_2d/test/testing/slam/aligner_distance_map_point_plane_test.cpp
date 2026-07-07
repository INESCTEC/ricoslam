#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/normal_computator.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/aligner_distance_map_point_plane.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;
using namespace pcl;
using namespace slam;
using namespace srrg2_solver;

TEST(slam_aligner_distance_map_point_plane, constructor)
{
  using VariableType =
      VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight;
  using AlignerType = AlignerDistanceMapPointPlane_<VariableType>;
  using AlignerSolverParamType = AlignerType::Param;
  using AlignerDistMapParamType = AlignerType::DistMapParam;

  AlignerSolverParamType param_solver;
  AlignerDistMapParamType param_dist_map;

  ASSERT_NO_THROW(AlignerType aligner);
  ASSERT_NO_THROW(AlignerType aligner(param_solver));
  ASSERT_NO_THROW(AlignerType aligner(param_solver, param_dist_map));

  param_dist_map.m_dist_map_res = -1.f;
  EXPECT_ANY_THROW(AlignerType aligner(param_solver, param_dist_map));
}  // TEST(slam_aligner_distance_map_point_plane, constructor)

TEST(slam_aligner_distance_map_point_plane, align_distance_map_static)
{
  using VariableType =
      VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight;
  using EstimateType = VariableType::EstimateType;
  using AlignerType = AlignerDistanceMapPointPlane_<VariableType>;
  using AlignerSolverParamType = AlignerType::Param;
  using AlignerDistMapParamType = AlignerType::DistMapParam;
  using FactorType = AlignerType::FactorType;
  using MovingPointCloudType = FactorType::MovingContainerType;
  using FixedPointCloudType = AlignerType::PointCloudType;

  using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
  using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

  using NormalComputatorType =
      NormalComputatorNearestPoints1D_<MovingPointCloudType>;

  std::random_device rd;
  std::mt19937 rng(rd());

  NormalComputatorType normal_computator;

  normal_computator.m_min_pts = 3;
  normal_computator.m_radius = 0.15f;

  AlignerSolverParamType param_solver;
  AlignerDistMapParamType param_dist_map;

  param_solver.m_solver_verbose = true;

  AlignerType aligner(param_solver, param_dist_map);

  FixedPointCloudTypePtr pc_fixed = std::make_shared<FixedPointCloudType>();
  MovingPointCloudTypePtr pc_moving = std::make_shared<MovingPointCloudType>();

  constexpr float square_side = 10.0;
  constexpr int square_side_pts = 250;

  float noise_std = param_dist_map.m_dist_map_res;

  test::getPointCloudSquare(*pc_fixed, square_side, square_side_pts, rng,
                            noise_std);
  test::getPointCloudSquare(*pc_moving, square_side, square_side_pts, rng,
                            noise_std);

  normal_computator.computeNormals(*pc_fixed);
  normal_computator.computeNormals(*pc_moving);

  const Eigen::Vector3f initial_estimate = Eigen::Vector3f::Random() * 0.05f;

  aligner.setSource(pc_moving, EstimateType::Identity());
  aligner.setSourceInTarget(geometry::v2t(initial_estimate));
  aligner.setTarget(pc_fixed, EstimateType::Identity());
  aligner.setPriorEnabled(false);
  aligner.compute();

  std::cout << "aligner distance map point2point parametrization:" << std::endl
            << aligner.getParam() << aligner.getDistMapParam();
  std::cout << "alignment initial estimation:" << std::endl
            << initial_estimate.transpose() << std::endl
            << "alignment solution:" << std::endl
            << geometry::t2v(aligner.getSourceInTarget()).transpose()
            << std::endl;
  std::cout << "alignment stats:" << std::endl
            << aligner.getStats() << std::endl;

  auto orig_stats = aligner.getStats();

  aligner.computeStatsOnly();

  auto computed_stats = aligner.getStats();

  EXPECT_FLOAT_EQ(computed_stats.m_num_moving_pts, orig_stats.m_num_moving_pts);
  EXPECT_FLOAT_EQ(computed_stats.m_num_inliers, orig_stats.m_num_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_outliers, orig_stats.m_num_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_kernelized, orig_stats.m_num_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_inliers, orig_stats.m_chi_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_outliers, orig_stats.m_chi_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_kernelized, orig_stats.m_chi_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_inliers_ratio, orig_stats.m_inliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_outliers_ratio, orig_stats.m_outliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_suppressed_ratio,
                  orig_stats.m_suppressed_ratio);
}  // TEST(slam_aligner_distance_map_point_plane, align_distance_map_static)

TEST(slam_aligner_distance_map_point_plane, align_distance_map_sparse_static)
{
  using VariableType =
      VariableSE2DistanceMapSparseStaticPointNormal2fVectorCloudRight;
  using EstimateType = VariableType::EstimateType;
  using AlignerType = AlignerDistanceMapPointPlane_<VariableType>;
  using AlignerSolverParamType = AlignerType::Param;
  using AlignerDistMapParamType = AlignerType::DistMapParam;
  using FactorType = AlignerType::FactorType;
  using MovingPointCloudType = FactorType::MovingContainerType;
  using FixedPointCloudType = AlignerType::PointCloudType;

  using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
  using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

  using NormalComputatorType =
      NormalComputatorNearestPoints1D_<MovingPointCloudType>;

  std::random_device rd;
  std::mt19937 rng(rd());

  NormalComputatorType normal_computator;

  normal_computator.m_min_pts = 3;
  normal_computator.m_radius = 0.15f;

  AlignerSolverParamType param_solver;
  AlignerDistMapParamType param_dist_map;

  param_solver.m_solver_verbose = true;

  AlignerType aligner(param_solver, param_dist_map);

  FixedPointCloudTypePtr pc_fixed = std::make_shared<FixedPointCloudType>();
  MovingPointCloudTypePtr pc_moving = std::make_shared<MovingPointCloudType>();

  constexpr float square_side = 10.0;
  constexpr int square_side_pts = 250;

  float noise_std = param_dist_map.m_dist_map_res;

  test::getPointCloudSquare(*pc_fixed, square_side, square_side_pts, rng,
                            noise_std);
  test::getPointCloudSquare(*pc_moving, square_side, square_side_pts, rng,
                            noise_std);

  normal_computator.computeNormals(*pc_fixed);
  normal_computator.computeNormals(*pc_moving);

  const Eigen::Vector3f initial_estimate = Eigen::Vector3f::Random() * 0.05f;

  aligner.setSource(pc_moving, EstimateType::Identity());
  aligner.setSourceInTarget(geometry::v2t(initial_estimate));
  aligner.setTarget(pc_fixed, EstimateType::Identity());
  aligner.setPriorEnabled(false);
  aligner.compute();

  std::cout << "aligner distance map sparse point2point parametrization:"
            << std::endl
            << aligner.getParam() << aligner.getDistMapParam();
  std::cout << "alignment initial estimation:" << std::endl
            << initial_estimate.transpose() << std::endl
            << "alignment solution:" << std::endl
            << geometry::t2v(aligner.getSourceInTarget()).transpose()
            << std::endl;
  std::cout << "alignment stats:" << std::endl
            << aligner.getStats() << std::endl;

  auto orig_stats = aligner.getStats();

  aligner.computeStatsOnly();

  auto computed_stats = aligner.getStats();

  EXPECT_FLOAT_EQ(computed_stats.m_num_moving_pts, orig_stats.m_num_moving_pts);
  EXPECT_FLOAT_EQ(computed_stats.m_num_inliers, orig_stats.m_num_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_outliers, orig_stats.m_num_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_kernelized, orig_stats.m_num_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_inliers, orig_stats.m_chi_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_outliers, orig_stats.m_chi_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_kernelized, orig_stats.m_chi_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_inliers_ratio, orig_stats.m_inliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_outliers_ratio, orig_stats.m_outliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_suppressed_ratio,
                  orig_stats.m_suppressed_ratio);
}  // TEST(slam_aligner_distance_map_point_plane,
   // align_distance_map_sparse_static)

TEST(slam_aligner_distance_map_point_plane, align_distance_map_nano_static)
{
  using VariableType =
      VariableSE2DistanceMapNanoStaticPointNormal2fVectorCloudRight;
  using EstimateType = VariableType::EstimateType;
  using AlignerType = AlignerDistanceMapPointPlane_<VariableType>;
  using AlignerSolverParamType = AlignerType::Param;
  using AlignerDistMapParamType = AlignerType::DistMapParam;
  using FactorType = AlignerType::FactorType;
  using MovingPointCloudType = FactorType::MovingContainerType;
  using FixedPointCloudType = AlignerType::PointCloudType;

  using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
  using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

  using NormalComputatorType =
      NormalComputatorNearestPoints1D_<MovingPointCloudType>;

  std::random_device rd;
  std::mt19937 rng(rd());

  NormalComputatorType normal_computator;

  normal_computator.m_min_pts = 3;
  normal_computator.m_radius = 0.15f;

  AlignerSolverParamType param_solver;
  AlignerDistMapParamType param_dist_map;

  param_solver.m_solver_verbose = true;

  AlignerType aligner(param_solver, param_dist_map);

  FixedPointCloudTypePtr pc_fixed = std::make_shared<FixedPointCloudType>();
  MovingPointCloudTypePtr pc_moving = std::make_shared<MovingPointCloudType>();

  constexpr float square_side = 10.0;
  constexpr int square_side_pts = 250;

  float noise_std = param_dist_map.m_dist_map_res;

  test::getPointCloudSquare(*pc_fixed, square_side, square_side_pts, rng,
                            noise_std);
  test::getPointCloudSquare(*pc_moving, square_side, square_side_pts, rng,
                            noise_std);

  normal_computator.computeNormals(*pc_fixed);
  normal_computator.computeNormals(*pc_moving);

  const Eigen::Vector3f initial_estimate = Eigen::Vector3f::Random() * 0.05f;

  aligner.setSource(pc_moving, EstimateType::Identity());
  aligner.setSourceInTarget(geometry::v2t(initial_estimate));
  aligner.setTarget(pc_fixed, EstimateType::Identity());
  aligner.setPriorEnabled(false);
  aligner.compute();

  std::cout << "aligner distance map nano point2point parametrization:"
            << std::endl
            << aligner.getParam() << aligner.getDistMapParam();
  std::cout << "alignment initial estimation:" << std::endl
            << initial_estimate.transpose() << std::endl
            << "alignment solution:" << std::endl
            << geometry::t2v(aligner.getSourceInTarget()).transpose()
            << std::endl;
  std::cout << "alignment stats:" << std::endl
            << aligner.getStats() << std::endl;

  auto orig_stats = aligner.getStats();

  aligner.computeStatsOnly();

  auto computed_stats = aligner.getStats();

  EXPECT_FLOAT_EQ(computed_stats.m_num_moving_pts, orig_stats.m_num_moving_pts);
  EXPECT_FLOAT_EQ(computed_stats.m_num_inliers, orig_stats.m_num_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_outliers, orig_stats.m_num_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_kernelized, orig_stats.m_num_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_inliers, orig_stats.m_chi_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_outliers, orig_stats.m_chi_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_kernelized, orig_stats.m_chi_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_inliers_ratio, orig_stats.m_inliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_outliers_ratio, orig_stats.m_outliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_suppressed_ratio,
                  orig_stats.m_suppressed_ratio);
}  // TEST(slam_aligner_distance_map_point_plane,
   // align_distance_map_nano_static)

TEST(slam_aligner_distance_map_point_plane,
     align_distance_map_static_with_2nd_deriv)
{
  using VariableType =
      VariableSE2DistanceMapStaticWith2ndDerivPointNormal2fVectorCloudRight;
  using EstimateType = VariableType::EstimateType;
  using AlignerType = AlignerDistanceMapPointPlane_<VariableType>;
  using AlignerSolverParamType = AlignerType::Param;
  using AlignerDistMapParamType = AlignerType::DistMapParam;
  using FactorType = AlignerType::FactorType;
  using MovingPointCloudType = FactorType::MovingContainerType;
  using FixedPointCloudType = AlignerType::PointCloudType;

  using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
  using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

  using NormalComputatorType =
      NormalComputatorNearestPoints1D_<MovingPointCloudType>;

  std::random_device rd;
  std::mt19937 rng(rd());

  NormalComputatorType normal_computator;

  normal_computator.m_min_pts = 3;
  normal_computator.m_radius = 0.15f;

  AlignerSolverParamType param_solver;
  AlignerDistMapParamType param_dist_map;

  param_solver.m_solver_verbose = true;

  AlignerType aligner(param_solver, param_dist_map);

  FixedPointCloudTypePtr pc_fixed = std::make_shared<FixedPointCloudType>();
  MovingPointCloudTypePtr pc_moving = std::make_shared<MovingPointCloudType>();

  constexpr float square_side = 10.0;
  constexpr int square_side_pts = 250;

  float noise_std = param_dist_map.m_dist_map_res;

  test::getPointCloudSquare(*pc_fixed, square_side, square_side_pts, rng,
                            noise_std);
  test::getPointCloudSquare(*pc_moving, square_side, square_side_pts, rng,
                            noise_std);

  normal_computator.computeNormals(*pc_fixed);
  normal_computator.computeNormals(*pc_moving);

  const Eigen::Vector3f initial_estimate = Eigen::Vector3f::Random() * 0.05f;

  aligner.setSource(pc_moving, EstimateType::Identity());
  aligner.setSourceInTarget(geometry::v2t(initial_estimate));
  aligner.setTarget(pc_fixed, EstimateType::Identity());
  aligner.setPriorEnabled(false);
  aligner.compute();

  std::cout << "aligner distance map point2point parametrization:" << std::endl
            << aligner.getParam() << aligner.getDistMapParam();
  std::cout << "alignment initial estimation:" << std::endl
            << initial_estimate.transpose() << std::endl
            << "alignment solution:" << std::endl
            << geometry::t2v(aligner.getSourceInTarget()).transpose()
            << std::endl;
  std::cout << "alignment stats:" << std::endl
            << aligner.getStats() << std::endl;

  auto orig_stats = aligner.getStats();

  aligner.computeStatsOnly();

  auto computed_stats = aligner.getStats();

  EXPECT_FLOAT_EQ(computed_stats.m_num_moving_pts, orig_stats.m_num_moving_pts);
  EXPECT_FLOAT_EQ(computed_stats.m_num_inliers, orig_stats.m_num_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_outliers, orig_stats.m_num_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_kernelized, orig_stats.m_num_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_inliers, orig_stats.m_chi_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_outliers, orig_stats.m_chi_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_kernelized, orig_stats.m_chi_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_inliers_ratio, orig_stats.m_inliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_outliers_ratio, orig_stats.m_outliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_suppressed_ratio,
                  orig_stats.m_suppressed_ratio);
}  // TEST(slam_aligner_distance_map_point_plane,
   // align_distance_map_static_with_2nd_deriv)

TEST(slam_aligner_distance_map_point_plane,
     align_distance_map_sparse_static_with_2nd_deriv)
{
  using VariableType =
      VariableSE2DistanceMapSparseStaticWith2ndDerivPointNormal2fVectorCloudRight;
  using EstimateType = VariableType::EstimateType;
  using AlignerType = AlignerDistanceMapPointPlane_<VariableType>;
  using AlignerSolverParamType = AlignerType::Param;
  using AlignerDistMapParamType = AlignerType::DistMapParam;
  using FactorType = AlignerType::FactorType;
  using MovingPointCloudType = FactorType::MovingContainerType;
  using FixedPointCloudType = AlignerType::PointCloudType;

  using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
  using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

  using NormalComputatorType =
      NormalComputatorNearestPoints1D_<MovingPointCloudType>;

  std::random_device rd;
  std::mt19937 rng(rd());

  NormalComputatorType normal_computator;

  normal_computator.m_min_pts = 3;
  normal_computator.m_radius = 0.15f;

  AlignerSolverParamType param_solver;
  AlignerDistMapParamType param_dist_map;

  param_solver.m_solver_verbose = true;

  AlignerType aligner(param_solver, param_dist_map);

  FixedPointCloudTypePtr pc_fixed = std::make_shared<FixedPointCloudType>();
  MovingPointCloudTypePtr pc_moving = std::make_shared<MovingPointCloudType>();

  constexpr float square_side = 10.0;
  constexpr int square_side_pts = 250;

  float noise_std = param_dist_map.m_dist_map_res;

  test::getPointCloudSquare(*pc_fixed, square_side, square_side_pts, rng,
                            noise_std);
  test::getPointCloudSquare(*pc_moving, square_side, square_side_pts, rng,
                            noise_std);

  normal_computator.computeNormals(*pc_fixed);
  normal_computator.computeNormals(*pc_moving);

  const Eigen::Vector3f initial_estimate = Eigen::Vector3f::Random() * 0.05f;

  aligner.setSource(pc_moving, EstimateType::Identity());
  aligner.setSourceInTarget(geometry::v2t(initial_estimate));
  aligner.setTarget(pc_fixed, EstimateType::Identity());
  aligner.setPriorEnabled(false);
  aligner.compute();

  std::cout << "aligner distance map point2point parametrization:" << std::endl
            << aligner.getParam() << aligner.getDistMapParam();
  std::cout << "alignment initial estimation:" << std::endl
            << initial_estimate.transpose() << std::endl
            << "alignment solution:" << std::endl
            << geometry::t2v(aligner.getSourceInTarget()).transpose()
            << std::endl;
  std::cout << "alignment stats:" << std::endl
            << aligner.getStats() << std::endl;

  auto orig_stats = aligner.getStats();

  aligner.computeStatsOnly();

  auto computed_stats = aligner.getStats();

  EXPECT_FLOAT_EQ(computed_stats.m_num_moving_pts, orig_stats.m_num_moving_pts);
  EXPECT_FLOAT_EQ(computed_stats.m_num_inliers, orig_stats.m_num_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_outliers, orig_stats.m_num_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_num_kernelized, orig_stats.m_num_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_inliers, orig_stats.m_chi_inliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_outliers, orig_stats.m_chi_outliers);
  EXPECT_FLOAT_EQ(computed_stats.m_chi_kernelized, orig_stats.m_chi_kernelized);
  EXPECT_FLOAT_EQ(computed_stats.m_inliers_ratio, orig_stats.m_inliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_outliers_ratio, orig_stats.m_outliers_ratio);
  EXPECT_FLOAT_EQ(computed_stats.m_suppressed_ratio,
                  orig_stats.m_suppressed_ratio);
}  // TEST(slam_aligner_distance_map_point_plane,
   // align_distance_map_sparse_static_with_2nd_deriv)

}  // namespace testing
