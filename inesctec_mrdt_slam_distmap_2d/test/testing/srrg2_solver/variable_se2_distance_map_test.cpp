#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry3d.h>
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
               const FactorGraphTypePtr& graph)
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
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void createGTTrajectoryRandom(TrajectoryType& trajectory, size_t num_poses)
{
  trajectory.clear();
  trajectory.reserve(num_poses);

  std::random_device rd;
  std::mt19937 rand_gen(rd());
  std::uniform_real_distribution<float> uniform_dist(0.0f, 10.0f);

  trajectory.emplace_back(EstimateType::Identity());

  for (size_t idx = 0; idx < num_poses; idx++)
  {
    const EstimateType pose = srrg2_core::geometry2d::v2t(srrg2_core::Vector3f(
        uniform_dist(rand_gen), uniform_dist(rand_gen), 0.0f));

    trajectory.emplace_back(pose);
  }

  assert(trajectory.size() == num_poses + 1);
}

void createGraphTrajectoryRandom(TrajectoryType& trajectory,
                                 const FactorGraphTypePtr& graph)
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
    v->setStatus(srrg2_solver::VariableBase::Fixed);
    v->setMeasurement(std::make_shared<MeasurementType>());

    graph->addVariable(v);

    VariableType* prev_v = static_cast<VariableType*>(graph->variable(idx - 1));

    FactorTypePtr f = std::make_shared<FactorType>();

    f->setVariableId(0, prev_v->graphId());
    f->setVariableId(1, v->graphId());
    f->setInformationMatrix(InformationMatrixType::Identity());
    f->setMeasurement(prev_v->estimate().inverse() * v->estimate());

    graph->addFactor(f);
  }
}

/* TEST_F(srrg2_solver_variable_se2_distance_map, constructor)
{
  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

  VariableTypePtr v = std::make_shared<VariableType>();

  v->setGraphId(0);
  v->setEstimate(EstimateType::Identity());
  v->setStatus(srrg2_solver::VariableBase::Fixed);
  v->setMeasurement(std::make_shared<MeasurementType>());

  graph->addVariable(v);

  std::cout << "srrg2_solver_variable_se2_distance_map::constructor | "
               "viewGraph(srrg_canvas, graph), waiting 5s..."
            << std::endl;

  viewGraph(srrg_canvas, graph);

  std::this_thread::sleep_for(std::chrono::seconds(5));
}  // TEST_F(srrg2_solver_variable_se2_distance_map, constructor) */

}  // namespace testing

int main(int argc, char** argv)
{
  std::shared_ptr<QApplication> qapp =
      std::make_shared<QApplication>(argc, argv);
  std::shared_ptr<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom> srrg_viewer =
      std::make_shared<srrg2_qgl_viewport::ViewerCoreSharedQGLCustom>(
          argc, argv, qapp.get(), BUFFER_SIZE_50MEGABYTE, 3, 25, false);

  srrg_canvas =
      srrg_viewer->getCanvas("srrg2_solver_variable_se2_distance_map");
  srrg_canvas->setName("srrg2_solver_variable_se2_distance_map");

  std::thread run_tests(
      [&srrg_viewer]()
      {
        TrajectoryType trajectory;

        testing::createGTTrajectoryRandom(trajectory, 10);

        FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();

        testing::createGraphTrajectoryRandom(trajectory, graph);

        std::cout
            << "srrg2_solver_variable_se2_distance_map::randomTrajectory | "
               "viewGraph(srrg_canvas, graph), last test..."
            << std::endl;

        testing::viewGraph(srrg_canvas, graph);

        while (srrg_viewer->isRunning())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      });

  srrg_viewer->startViewerServer();

  run_tests.join();

  return 0;
}  // int main(int, char**)
