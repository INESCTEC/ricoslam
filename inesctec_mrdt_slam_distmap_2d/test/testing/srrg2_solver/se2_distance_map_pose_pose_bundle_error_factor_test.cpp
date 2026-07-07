#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry3d.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky_cholmod.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/solver.h>

#include <Eigen/StdVector>
#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/image_defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_bundle_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/solver_actions.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

using FactorGraphType = srrg2_solver::FactorGraph;

using FactorType = srrg2_solver::
    SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseBundleWithSensorErrorFactorCorrespondenceFreeDriven;
using VariableType = FactorType::VariableType;
using EstimateType = VariableType::EstimateType;
using InformationMatrixType = FactorType::InformationMatrixType;

using FactorGraphTypePtr = std::shared_ptr<srrg2_solver::FactorGraph>;
using VariableTypePtr = std::shared_ptr<VariableType>;
using FactorTypePtr = std::shared_ptr<FactorType>;

using MovingPointCloudType = FactorType::MovingContainerType;
using MovingPointType = FactorType::MovingItemType;

using MeasurementType = VariableType::MeasurementOwnerType::MeasurementObjType;
using DistanceMapType = VariableType::MeasurementOwnerType::DistanceMapType;
using DistanceMapTypePtr =
    VariableType::MeasurementOwnerType::DistanceMapTypePtr;
using FixedPointCloudType = VariableType::MeasurementOwnerType::PointCloudType;
using FixedPointType = FixedPointCloudType::PointType;

using MovingPointCloudTypePtr = std::shared_ptr<MovingPointCloudType>;
using FixedPointCloudTypePtr = std::shared_ptr<FixedPointCloudType>;

using TrajectoryType =
    std::vector<EstimateType, Eigen::aligned_allocator<EstimateType>>;

std::shared_ptr<srrg2_core::ViewerCanvas> srrg_canvas = nullptr;

const EstimateType sensor_in_robot =
    inesctec_mrdt_slam_distmap_2d::geometry::v2t(
        Eigen::Vector3f(0.2f, 0.1f, M_PI_4));

namespace testing
{

void viewGraph(const srrg2_core::ViewerCanvasPtr& canvas,
               const FactorGraphTypePtr& graph, int64_t sleep_ms = 50)
{
  while (!canvas->_setup())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  for (auto var : graph->variables())
  {
    const VariableType* v = static_cast<const VariableType*>(var.second);

    v->_drawImpl(canvas);
  }
  for (auto fact : graph->factors())
  {
    const FactorType* f = static_cast<const FactorType*>(fact.second);

    srrg2_core::Vector3f segments[2];

    const VariableType* v_from =
        static_cast<const VariableType*>(f->variable(0));
    const VariableType* v_to = static_cast<const VariableType*>(f->variable(1));

    if (!v_from || !v_to)
    {
      continue;
    }

    const EstimateType& from = v_from->estimate();
    const EstimateType& to = v_to->estimate();

    segments[0] << from.translation().x(), from.translation().y(), 0.f;
    segments[1] << to.translation().x(), to.translation().y(), 0.f;

    canvas->putSegment(2, segments, 0);
  }
  canvas->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
}  // void viewGraph(const srrg2_core::ViewerCanvasPtr&, const
   // FactorGraphTypePtr&, int64_t = 50)

/**
 * @brief creaate a grid manhattan-like trajectory, to better simulate the
 *        bundle adjustment
 * @param[out] trajectory sequence of Isometry SE(2) poses
 */
void createGTTrajectoryManhattan(TrajectoryType& trajectory, int grid_size)
{
  trajectory.clear();

  for (int r = -grid_size; r <= grid_size; r++)
  {
    for (int c = -grid_size; c <= grid_size; c++)
    {
      EstimateType pose;

      pose.setIdentity();
      pose.translation().x() = r;
      pose.translation().y() = c;

      if ((abs(r) % 2 == 0) && (abs(c) % 2 == 0))
        pose.linear() = srrg2_core::geometry2d::a2r<float>(0.f);
      else if ((abs(r) % 2 == 0) && (abs(c) % 2 == 1))
        pose.linear() = srrg2_core::geometry2d::a2r<float>(M_PIf32 * 0.5f);
      else if ((abs(r) % 2 == 1) && (abs(c) % 2 == 0))
        pose.linear() = srrg2_core::geometry2d::a2r<float>(-M_PIf32);
      else
        pose.linear() = srrg2_core::geometry2d::a2r<float>(-M_PIf32 * 0.5f);

      trajectory.emplace_back(pose);
    }
  }
}  // void createGTTrajectoryManhattan(TrajectoryType&, int, int, int, float =
   // 0.5f)

/**
 * @brief create the graph by adding the odometry factor between consecutive
 *        variables and loop closure factors in a certain range
 * @param[in] trajectory sequence of Isometry SE(2) poses
 * @param[in,out] graph factor graph
 * @param[in] loop_range loop search radius
 * @param[in] pc reference point cloud (to be propagated to the other variables)
 */
void createGraphTrajectory(const TrajectoryType& trajectory,
                           const FactorGraphTypePtr& graph, float loop_range,
                           const float res, const float max_dist,
                           const float map_size, const float square_side,
                           const float square_side_pts, const float noise_std)
{
  std::random_device rd;
  std::mt19937 rng(rd());

  VariableTypePtr v;

  FixedPointCloudType pc_reference;

  for (size_t idx = 0; idx < trajectory.size(); idx++)
  {
    v = std::make_shared<VariableType>();

    v->setGraphId(idx);
    v->setEstimate(trajectory[idx]);
    v->setMeasurement(std::make_shared<MeasurementType>());
    v->setSensorInRobot(sensor_in_robot);

    v->measurement()->m_dist_map = std::make_shared<DistanceMapType>();
    v->measurement()->m_pc = std::make_shared<FixedPointCloudType>();

    inesctec_mrdt_slam_distmap_2d::test::getPointCloudSquare(
        *v->measurement()->m_pc, square_side, square_side_pts, rng, noise_std);

    v->measurement()->m_pc->transformInPlace(sensor_in_robot.inverse() *
                                             v->estimate().inverse());

    v->measurement()->getDistanceMap()->setGoals(*v->measurement()->m_pc, res,
                                                 map_size, max_dist);

    graph->addVariable(v);
  }

  graph->variable(0)->setStatus(VariableType::Status::Fixed);

  for (const auto& v1 : graph->variables())
  {
    for (const auto& v2 : graph->variables())
    {
      if (v1.second->graphId() == v2.second->graphId())
      {
        continue;
      }

      if ((static_cast<VariableType*>(v1.second)->estimate().translation() -
           static_cast<VariableType*>(v2.second)->estimate().translation())
              .norm() > loop_range)
      {
        continue;
      }

      FactorTypePtr f = std::make_shared<FactorType>();

      f->setVariableId(0, v1.second->graphId());
      f->setVariableId(1, v2.second->graphId());
      f->setInformationMatrix(InformationMatrixType::Identity());
      f->setSensorInRobot(sensor_in_robot);

      f->setMoving(*static_cast<VariableType*>(v1.second)
                        ->measurement()
                        ->getPointCloud());

      graph->addFactor(f);
    }
  }
}  // void createGraphTrajectory(const TrajectoryType& trajectory, const
   // FactorGraphTypePtr&, float)

}  // namespace testing

int main(int argc, char** argv)
{
  std::shared_ptr<QApplication> qapp =
      std::make_shared<QApplication>(argc, argv);
  std::shared_ptr<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom> srrg_viewer =
      std::make_shared<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom>(
          argc, argv, qapp.get(), BUFFER_SIZE_50MEGABYTE, 3, 25, false);

  srrg_canvas = srrg_viewer->getCanvas(
      "srrg2_solver_se2_distance_map_pose_pose_bundle_error_factor");
  srrg_canvas->setName(
      "srrg2_solver_se2_distance_map_pose_pose_bundle_error_factor");

  std::thread run_tests(
      []()
      {
        constexpr int num_iterations = 100;

        constexpr float res = 0.010f;
        constexpr float max_dist = 0.5f;
        constexpr float map_size = -1.f;

        constexpr float square_side = 10.0;
        constexpr int square_side_pts = 500;

        constexpr float noise_std = 0.f;

        TrajectoryType trajectory;

        testing::createGTTrajectoryManhattan(trajectory, 5);

        std::cout << "Manhattan-like Trajectory:" << std::endl
                  << "- #poses: " << trajectory.size() << std::endl;

        FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

        testing::createGraphTrajectory(trajectory, graph, 2.5f, res, max_dist,
                                       map_size, square_side, square_side_pts,
                                       noise_std);

        std::cout
            << "srrg2_solver_se2_distance_map_pose_pose_bundle_error_factor::"
               "bundleAdjustment | "
               "viewGraph(srrg_canvas, graph), last test..."
            << std::endl;

        std::cout << "ground-truth graph..." << std::endl;
        testing::viewGraph(srrg_canvas, graph);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        srrg2_solver::Solver solver;

        solver.setGraph(graph);
        solver.param_verbose.setValue(true);
        solver.param_termination_criteria.setValue(nullptr);
        solver.param_max_iterations.setValue({num_iterations});

        solver.param_actions.pushBack(
            std::make_shared<srrg2_solver::SolverVerboseAction>());

        srrg2_solver::SolverGenericAtIterationEndActionPtr view_graph_dynamic =
            std::make_shared<srrg2_solver::SolverGenericAtIterationEndAction>();

        view_graph_dynamic->m_action = [&graph]()
        { testing::viewGraph(srrg_canvas, graph); };

        solver.param_actions.pushBack(view_graph_dynamic);

        solver.param_algorithm.setValue(
            std::make_shared<srrg2_solver::IterationAlgorithmGN>());
        dynamic_cast<srrg2_solver::IterationAlgorithmGN*>(
            solver.param_algorithm.value().get())
            ->param_damping.setValue(1.0f);

        solver.param_linear_solver.setValue(
            std::make_shared<srrg2_solver::SparseBlockLinearSolverCholesky>());

        std::cout << "initial estimation graph... (all nodes in the origin)"
                  << std::endl;
        for (auto var : graph->variables())
        {
          VariableType* v = static_cast<VariableType*>(var.second);

          if (v->status() == VariableType::Status::Fixed)
          {
            continue;
          }

          Eigen::Vector3f pert = 0.100f * Eigen::Vector3f::Random();
          EstimateType iso = v->estimate();
          v->setEstimate(srrg2_core::geometry2d::v2t(pert) * iso);
        }
        testing::viewGraph(srrg_canvas, graph);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        std::cout << "solver compute global bundle adjustment graph "
                     "optimization (gba)..."
                  << std::endl;
        solver.compute();
        testing::viewGraph(srrg_canvas, graph);
        std::this_thread::sleep_for(std::chrono::seconds(5));
      });

  srrg_viewer->startViewerServer();

  run_tests.join();

  return 0;
}  // int main(int, char**)
