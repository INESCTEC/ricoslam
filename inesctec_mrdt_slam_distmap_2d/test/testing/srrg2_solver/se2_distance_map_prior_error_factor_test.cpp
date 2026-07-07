#include <srrg2_core/srrg_geometry/geometry2d.h>
#include <srrg2_core/srrg_geometry/geometry3d.h>
#include <srrg2_solver/solver_core/factor_graph.h>
#include <srrg2_solver/solver_core/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/instances.h>
#include <srrg2_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholesky.h>
#include <srrg2_solver/solver_core/iteration_algorithm_gn.h>
#include <srrg2_solver/solver_core/solver.h>

#include <Eigen/Eigenvalues>
#include <chrono>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/se2_distance_map_prior_error_factor.hpp>
#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/variable_se2_distance_map.hpp>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

using FactorGraphType = srrg2_solver::FactorGraph;

using FactorType =
    srrg2_solver::SE2DistanceMapStaticPoint2fVectorCloudRightPriorErrorFactor;
using VariableType = FactorType::VariableType;
using EstimateType = VariableType::EstimateType;
using InformationMatrixType = FactorType::InformationMatrixType;

using FactorGraphTypePtr = std::shared_ptr<srrg2_solver::FactorGraph>;
using VariableTypePtr = std::shared_ptr<VariableType>;
using FactorTypePtr = std::shared_ptr<FactorType>;

using namespace srrg2_core;

int main(int argc, char **argv)
{
  srand48(time(0));

  constexpr bool solver_verbose = false;
  constexpr int num_iterations = 25;
  constexpr int num_attempts = 10;

  FactorGraphTypePtr graph = std::make_shared<FactorGraphType>();
  VariableTypePtr variable = std::make_shared<VariableType>();
  FactorTypePtr factor = std::make_shared<FactorType>();

  variable->setEstimate(EstimateType::Identity());
  variable->setGraphId(0);

  factor->setVariableId(0, variable->graphId());
  factor->setInformationMatrix(InformationMatrixType::Identity());

  graph->addVariable(variable);
  graph->addFactor(factor);

  srrg2_solver::Solver solver;

  solver.setGraph(graph);
  solver.param_verbose.setValue(solver_verbose);
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.setValue({num_iterations});

  if (solver_verbose)
  {
    solver.param_actions.pushBack(
        std::make_shared<srrg2_solver::SolverVerboseAction>());
  }

  solver.param_algorithm.setValue(
      std::make_shared<srrg2_solver::IterationAlgorithmGN>());
  dynamic_cast<srrg2_solver::IterationAlgorithmGN *>(
      solver.param_algorithm.value().get())
      ->param_damping.setValue(1.0f);

  solver.param_linear_solver.setValue(
      std::make_shared<srrg2_solver::SparseBlockLinearSolverCholesky>());

  for (int idx = 0; idx < num_attempts; idx++)
  {
    Eigen::Vector3f prior_estimate_0 = Eigen::Vector3f::Random();

    variable->setEstimate(geometry2d::v2t(prior_estimate_0));
    factor->setMeasurement(EstimateType::Identity());

    solver.compute();

    std::cout << std::endl
              << "srrg2_solver_se2_distance_map_prior_error_factor::compute:"
              << std::endl
              << "- ground-truth      : "
              << geometry2d::t2v(factor->measurement()).transpose() << std::endl
              << "- initial estimation: " << prior_estimate_0.transpose()
              << std::endl
              << "- final estimation  : "
              << geometry2d::t2v(variable->estimate()).transpose() << std::endl;
  }
}  // int main(int, char**)
