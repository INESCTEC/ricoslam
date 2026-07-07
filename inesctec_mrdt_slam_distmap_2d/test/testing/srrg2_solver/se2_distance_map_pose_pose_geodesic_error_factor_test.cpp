#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_viewer/viewer_core/color_palette.h>
#include <srrg2_qgl_viewport/viewer_core_shared_qgl.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/solver.h>
#include <srrg2_solver/variables_and_factors/types_2d/instances.h>

#include <Eigen/StdVector>
#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_qgl_viewport/viewer_core_shared_qgl_custom.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_pose_pose_geodesic_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

using FactorGraphType = srrg2_solver::FactorGraph;
using FactorType = srrg2_solver::
    SE2DistanceMapStaticPoint2fVectorCloudRightPosePoseGeodesicErrorFactor;
using VariableType = FactorType::VariableType;
using EstimateType = VariableType::EstimateType;

using InformationMatrixType = FactorType::InformationMatrixType;

using VariableTypePtr = std::shared_ptr<VariableType>;
using FactorTypePtr = std::shared_ptr<FactorType>;
using FactorGraphTypePtr = std::shared_ptr<srrg2_solver::FactorGraph>;

using MeasurementType = VariableType::MeasurementOwnerType::MeasurementObjType;

using TrajectoryType =
    std::vector<EstimateType, Eigen::aligned_allocator<EstimateType>>;

std::shared_ptr<srrg2_core::ViewerCanvas> srrg_canvas = nullptr;

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
    f->_drawImpl(canvas);
  }
  canvas->flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
}

/**
 * @brief based on trajectory random generator for Manhattan-like trajectories
 *        from SRRG software (srrg2_solver > plgo_simulator >
 *        trajectory_generator)
 * @param[out] trajectory sequence of Isometry SE(2) poses
 */
void createGTTrajectoryManhattan(TrajectoryType& trajectory, int grid_size,
                                 int simulation_steps, int simulation_step_size,
                                 float probability_turn = 0.5f)
{
  int row = 0;  // row column coordinate
  int col = 0;  // cell column coordinate
  int dir = 0;  // 0: right, 1: front, 2: left, 3: back

  trajectory.clear();

  // Radom generator, mersenne_twister_engine seeded with rd
  // (uniform dist between [0.0,1.0[ >> open interval due to impl lmts...)
  std::random_device rd;
  std::mt19937 rand_gen(rd());
  std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

  auto canMove = [&row, &col, &dir, &grid_size, &simulation_step_size]() -> bool
  {
    switch (dir)
    {
      case 0:
        if (row > grid_size - simulation_step_size - 1)
        {
          return false;
        }
        ++row;
        return true;
      case 1:
        if (col > grid_size - simulation_step_size - 1)
        {
          return false;
        }
        ++col;
        return true;
      case 2:
        if (row < -grid_size + simulation_step_size + 1)
        {
          return false;
        }
        --row;
        return true;
      case 3:
        if (col < -grid_size + simulation_step_size + 1)
        {
          return false;
        }
        --col;
        return true;
      default:
        return false;
    }
  };

  auto addPose = [&row, &col, &dir, &trajectory]() -> void
  {
    EstimateType pose;
    pose.setIdentity();
    pose.translation().x() = row;
    pose.translation().y() = col;
    pose.linear() = srrg2_core::geometry2d::a2r<float>(dir * M_PIf32 * 0.5f);
    trajectory.emplace_back(pose);
  };

  auto move = [&dir, &simulation_step_size, &probability_turn, &rand_gen,
               &uniform_dist, &canMove, &addPose]() -> void
  {
    float prob = uniform_dist(rand_gen);

    if (prob > probability_turn)
    {
      bool move_result = false;

      for (int idx = 0; idx < simulation_step_size; idx++)
      {
        move_result = canMove();
        if (move_result)
        {
          addPose();
        }
      }

      if (!move_result)
      {
        dir += 2;
        dir %= 4;
        addPose();
      }

      return;
    }

    prob = uniform_dist(rand_gen);

    if (prob > 0.5f)
      ++dir;
    else
      --dir;

    dir = (dir + 4) % 4;

    addPose();
  };

  for (int idx = 0; idx < simulation_steps; idx++)
  {
    move();
  }
}  // void createGTTrajectoryManhattan(TrajectoryType&, int, int, int, float =
   // 0.5f)

/**
 * @brief create the graph by adding the odometry factor between consecutive
 *        variables and loop closure factors in a certain range
 * @param[in] trajectory sequence of Isometry SE(2) poses
 * @param[in,out] graph factor graph
 * @param[in] loop_range loop search radius
 */
void createGraphTrajectoryManhattan(const TrajectoryType& trajectory,
                                    const FactorGraphTypePtr& graph,
                                    float loop_range)
{
  VariableTypePtr v = std::make_shared<VariableType>();

  v->setGraphId(0);
  v->setEstimate(trajectory.at(0));
  v->setStatus(srrg2_solver::VariableBase::Fixed);
  v->setMeasurement(std::make_shared<MeasurementType>());

  graph->addVariable(v);

  for (size_t idx = 1; idx < trajectory.size(); idx++)
  {
    v = std::make_shared<VariableType>();

    v->setGraphId(idx);
    v->setEstimate(trajectory[idx]);
    v->setMeasurement(std::make_shared<MeasurementType>());

    graph->addVariable(v);

    VariableType* prev_v = static_cast<VariableType*>(graph->variable(idx - 1));

    FactorTypePtr f = std::make_shared<FactorType>();

    f->setVariableId(0, prev_v->graphId());
    f->setVariableId(1, v->graphId());
    f->setInformationMatrix(InformationMatrixType::Identity());
    f->setMeasurement(prev_v->estimate().inverse() * v->estimate());

    graph->addFactor(f);

    for (const auto& loop_candidate : graph->variables())
    {
      if (loop_candidate.first == v->graphId())
      {
        continue;
      }

      const VariableType* loop_v =
          static_cast<const VariableType*>(loop_candidate.second);
      const EstimateType loop_d = loop_v->estimate().inverse() * v->estimate();

      const Eigen::Vector3f loop_d_v = srrg2_core::geometry2d::t2v(loop_d);

      if (loop_d_v.head(2).norm() <= loop_range)
      {
        FactorTypePtr loop_f = std::make_shared<FactorType>();

        loop_f->setVariableId(0, loop_v->graphId());
        loop_f->setVariableId(1, v->graphId());
        loop_f->setInformationMatrix(InformationMatrixType::Identity());
        loop_f->setMeasurement(loop_d);
      }
    }
  }
}  // void createGraphTrajectoryManhattan(const TrajectoryType& trajectory,
   // const FactorGraphTypePtr&, float)

}  // namespace testing

int main(int argc, char** argv)
{
  std::shared_ptr<QApplication> qapp =
      std::make_shared<QApplication>(argc, argv);
  std::shared_ptr<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom> srrg_viewer =
      std::make_shared<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom>(
          argc, argv, qapp.get(), BUFFER_SIZE_50MEGABYTE, 3, 25, false);

  srrg_canvas = srrg_viewer->getCanvas(
      "srrg2_solver_se2_distance_map_pose_pose_geodesic_error_factor");
  srrg_canvas->setName(
      "srrg2_solver_se2_distance_map_pose_pose_geodesic_error_factor");

  std::thread run_tests(
      [&srrg_viewer]()
      {
        TrajectoryType trajectory;

        testing::createGTTrajectoryManhattan(trajectory, 50, 1000, 5, 0.50f);

        FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

        testing::createGraphTrajectoryManhattan(trajectory, graph, 5.f);

        std::cout
            << "srrg2_solver_se2_distance_map_pose_pose_geodesic_error_factor::"
               "manhattanTrajectory | "
               "viewGraph(srrg_canvas, graph), last test..."
            << std::endl;

        std::cout << "ground-truth graph..." << std::endl;
        testing::viewGraph(srrg_canvas, graph);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        srrg2_solver::Solver solver;

        solver.param_termination_criteria.setValue(nullptr);
        solver.param_max_iterations.pushBack(1000);
        dynamic_cast<srrg2_solver::IterationAlgorithmGN*>(
            solver.param_algorithm.value().get())
            ->param_damping.setValue(1.f);
        solver.setGraph(graph);

        std::cout << "initial estimation graph... (all nodes in the origin)"
                  << std::endl;
        for (auto var : graph->variables())
        {
          VariableType* v = static_cast<VariableType*>(var.second);
          // v->setEstimate(EstimateType::Identity());
          Eigen::Vector3f pert = 0.005f * Eigen::Vector3f::Random();
          EstimateType iso = v->estimate();
          v->setEstimate(srrg2_core::geometry2d::v2t(pert) * iso);
        }
        testing::viewGraph(srrg_canvas, graph);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        std::cout << "solver compute pose graph optimization (pgo)..."
                  << std::endl;
        solver.compute();
        testing::viewGraph(srrg_canvas, graph);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        while (srrg_viewer->isRunning())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      });

  srrg_viewer->startViewerServer();

  run_tests.join();

  return 0;
}  // int main(int, char**)
