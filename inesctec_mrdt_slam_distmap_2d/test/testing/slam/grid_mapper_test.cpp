#include <gtest/gtest.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/solver.h>
#include <srrg2_solver/solver_core/variable.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/slam/grid_mapper.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_geodesic_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/laser_scan_examples.hpp>
#include <memory>
#include <thread>

using FactorGraphType = srrg2_solver::FactorGraph;
using FactorType = srrg2_solver::
    SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor;
using VariableType = FactorType::VariableType;
using EstimateType = VariableType::EstimateType;
using Scalar =
    VariableType::MeasurementOwnerType::MeasurementObjType::ScalarType;

using InformationMatrixType = FactorType::InformationMatrixType;

using FactorGraphTypePtr = std::shared_ptr<FactorGraphType>;
using FactorTypePtr = std::shared_ptr<FactorType>;
using VariableTypePtr = std::shared_ptr<VariableType>;

using IterationAlgorithm = srrg2_solver::IterationAlgorithmGN;

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;
using namespace slam;

TEST(slam_grid_mapper, constructor)
{
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(GridMapper::Param()));

  GridMapper::Param param;
  param.m_res = 0.05f;
  param.m_usable_range_min = 0.f;
  param.m_usable_range_max = 10.f;
  param.m_range_max = 12.f;
  param.m_thresh_free = 0.1f;
  param.m_thresh_occ = 0.85f;

  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));

  param.m_res = 0.f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));
  param.m_res = -0.05f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));

  param.m_res = 0.05f;
  param.m_usable_range_min = -1.f;
  param.m_usable_range_max = -1.f;
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));
  param.m_usable_range_min = -1.f;
  param.m_usable_range_max = 8.f;
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));
  param.m_usable_range_min = 0.5f;
  param.m_usable_range_max = -1.f;
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));
  param.m_usable_range_min = 0.5f;
  param.m_usable_range_max = 8.f;
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));
  param.m_usable_range_min = 8.f;
  param.m_usable_range_max = 0.5f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));
  param.m_usable_range_min = 0.f;
  param.m_usable_range_max = 12.f;
  param.m_range_max = 10.f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));
  param.m_usable_range_min = 0.f;
  param.m_usable_range_max = 10.f;
  param.m_range_max = 12.f;
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));

  param.m_usable_range_min = 0.f;
  param.m_usable_range_max = 10.f;
  param.m_range_max = 12.f;
  param.m_thresh_free = 0.1f;
  param.m_thresh_occ = 0.85f;
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));
  /* special case of binary probabilistic formulation */
  param.m_thresh_free = -1.f;
  param.m_thresh_occ = 0.85f;
  EXPECT_NO_THROW(GridMapper mapper = GridMapper(param));
  param.m_thresh_free = 2.f;
  param.m_thresh_occ = 0.85f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));
  param.m_thresh_free = 0.1f;
  param.m_thresh_occ = -1.f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));
  param.m_thresh_free = 0.1f;
  param.m_thresh_occ = 2.f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));
  param.m_thresh_free = 0.85f;
  param.m_thresh_occ = 0.1f;
  EXPECT_ANY_THROW(GridMapper mapper = GridMapper(param));
}  // TEST(slam_grid_mapper, constructor)

TEST(slam_grid_mapper, updateMapEmpty)
{
  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  VariableTypePtr v = std::make_shared<VariableType>();
  v->setGraphId(0);
  v->setEstimate(EstimateType::Identity());
  v->setStatus(srrg2_solver::VariableBase::Fixed);

  graph->addVariable(v);

  v = std::make_shared<VariableType>();
  v->setGraphId(1);
  v->setEstimate(geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(1), static_cast<Scalar>(0), static_cast<Scalar>(0))));
  v->setStatus(srrg2_solver::VariableBase::Fixed);

  graph->addVariable(v);

  ASSERT_EQ(graph->variables().size(), 2);
  ASSERT_TRUE(graph->variables().find(0) != graph->variables().end());
  ASSERT_TRUE(graph->variables().find(1) != graph->variables().end());

  srrg2_solver::Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(100);
  solver.param_algorithm.setValue(std::make_shared<IterationAlgorithm>());
  dynamic_cast<IterationAlgorithm*>(solver.param_algorithm.value().get())
      ->param_damping.setValue(1.f);

  solver.setGraph(graph);

  GridMapper mapper = GridMapper(GridMapper::Param());
  mapper.setGraph(graph.get());
  mapper.updateMap();
}  // TEST(slam_grid_mapper, updateMapEmpty)

TEST(slam_grid_mapper, updateMapCircle)
{
  using LaserMsgType = msg::LaserScan;
  using LaserMsgTypePtr = msg::LaserScanPtr;

  std::random_device rd;
  std::mt19937 rng(rd());

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  VariableTypePtr v0 = std::make_shared<VariableType>();
  v0->setGraphId(0);
  v0->setEstimate(EstimateType::Identity());
  v0->setStatus(srrg2_solver::VariableBase::Fixed);

  graph->addVariable(v0);

  ASSERT_EQ(graph->variables().size(), 1);
  ASSERT_TRUE(graph->variables().find(0) != graph->variables().end());

  srrg2_solver::Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(100);
  solver.param_algorithm.setValue(std::make_shared<IterationAlgorithm>());
  dynamic_cast<IterationAlgorithm*>(solver.param_algorithm.value().get())
      ->param_damping.setValue(1.f);

  solver.setGraph(graph);

  GridMapper mapper = GridMapper(GridMapper::Param(
      0.03f /* res */, 0.f /* usable_range_min */, 8.f /* usable_range_max */,
      10.f /* range_max */, 0.1f /* thresh_free */, 0.85f /* thresh_occ */));
  mapper.setGraph(graph.get());

  // Generate laser scans associated to each variable
  constexpr float radius = 5.f;
  constexpr float max_range = 10.f;
  constexpr float noise_std = 0.f;

  const float angle_res = geometry::degToRad(1.f);
  const float fov = geometry::degToRad(360.f);

  EstimateType scan_0_in_w = EstimateType::Identity();
  EstimateType scan_1_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(0), static_cast<Scalar>(-7),
      static_cast<Scalar>(geometry::degToRad(90.f))));
  EstimateType scan_2_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(0), static_cast<Scalar>(7),
      static_cast<Scalar>(geometry::degToRad(-90.f))));
  EstimateType scan_3_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(7), static_cast<Scalar>(0),
      static_cast<Scalar>(geometry::degToRad(-180.f))));
  EstimateType scan_4_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(-7), static_cast<Scalar>(0),
      static_cast<Scalar>(geometry::degToRad(0.f))));

  LaserMsgTypePtr msg = std::make_shared<LaserMsgType>();

  test::getLaser2DLaserScanMsgCircle(*msg, EstimateType::Identity(), radius,
                                     angle_res, fov, max_range, rng, noise_std);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_0_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaser2DLaserScanMsgCircle(*msg, scan_1_in_w, radius, angle_res, fov,
                                     max_range, rng, noise_std);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_1_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaser2DLaserScanMsgCircle(*msg, scan_2_in_w, radius, angle_res, fov,
                                     max_range, rng, noise_std);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_2_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaser2DLaserScanMsgCircle(*msg, scan_3_in_w, radius, angle_res, fov,
                                     max_range, rng, noise_std);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_3_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaser2DLaserScanMsgCircle(*msg, scan_4_in_w, radius, angle_res, fov,
                                     max_range, rng, noise_std);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_4_in_w,
                          v0->graphId());

  ASSERT_EQ(mapper.getProcessedScans().size(), 5);

  mapper.updateMap();

  UCharImage img;
  mapper.getGridMapContainer()->toImageFrequencyMap(img);

  cv::imshow("occupancy grid map", img);
  cv::waitKey(100);
  cv::imwrite(__FILE__ ".updateMapCircle.png", img);
  cv::waitKey(0);
}  // TEST(slam_grid_mapper, updateMapCircle)

TEST(slam_grid_mapper, updateMapSquare)
{
  using LaserMsgType = msg::LaserScan;
  using LaserMsgTypePtr = msg::LaserScanPtr;

  std::random_device rd;
  std::mt19937 rng(rd());

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  VariableTypePtr v0 = std::make_shared<VariableType>();
  v0->setGraphId(0);
  v0->setEstimate(EstimateType::Identity());
  v0->setStatus(srrg2_solver::VariableBase::Fixed);

  graph->addVariable(v0);

  ASSERT_EQ(graph->variables().size(), 1);
  ASSERT_TRUE(graph->variables().find(0) != graph->variables().end());

  srrg2_solver::Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(100);
  solver.param_algorithm.setValue(std::make_shared<IterationAlgorithm>());
  dynamic_cast<IterationAlgorithm*>(solver.param_algorithm.value().get())
      ->param_damping.setValue(1.f);

  solver.setGraph(graph);

  GridMapper mapper = GridMapper(GridMapper::Param(
      0.03f /* res */, 0.f /* usable_range_min */, 8.f /* usable_range_max */,
      10.f /* range_max */, 0.1f /* thresh_free */, 0.85f /* thresh_occ */));
  mapper.setGraph(graph.get());

  // Generate laser scans associated to each variable
  constexpr float max_range = 10.f;
  constexpr float side = 5.f;

  const float angle_res = geometry::degToRad(0.5f);
  const float fov = geometry::degToRad(360.f);

  EstimateType scan_0_in_w = EstimateType::Identity();
  EstimateType scan_1_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(7), static_cast<Scalar>(7), static_cast<Scalar>(0)));
  EstimateType scan_2_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(-7), static_cast<Scalar>(7), static_cast<Scalar>(0)));
  EstimateType scan_3_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(-7), static_cast<Scalar>(-7),
      static_cast<Scalar>(0)));
  EstimateType scan_4_in_w = geometry::v2t(geometry::Vector3_<Scalar>(
      static_cast<Scalar>(7), static_cast<Scalar>(-7), static_cast<Scalar>(0)));

  LaserMsgTypePtr msg = std::make_shared<LaserMsgType>();

  test::getLaserScanMsgSquare(*msg, EstimateType::Identity(), angle_res, fov,
                              max_range, side, rng);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_0_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaserScanMsgSquare(*msg, scan_1_in_w, angle_res, fov, max_range,
                              side, rng);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_1_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaserScanMsgSquare(*msg, scan_2_in_w, angle_res, fov, max_range,
                              side, rng);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_2_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaserScanMsgSquare(*msg, scan_3_in_w, angle_res, fov, max_range,
                              side, rng);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_3_in_w,
                          v0->graphId());

  msg = std::make_shared<LaserMsgType>();

  test::getLaserScanMsgSquare(*msg, scan_4_in_w, angle_res, fov, max_range,
                              side, rng);
  mapper.pushLaserScanMsg(msg, EstimateType::Identity(), scan_4_in_w,
                          v0->graphId());

  ASSERT_EQ(mapper.getProcessedScans().size(), 5);

  mapper.updateMap();

  UCharImage img;
  mapper.getGridMapContainer()->toImageFrequencyMap(img);

  cv::imshow("occupancy grid map", img);
  cv::waitKey(100);
  cv::imwrite(__FILE__ ".updateMapSquare.png", img);
  cv::waitKey(0);
}  // TEST(slam_grid_mapper, updateMapSquare)

}  // namespace testing
