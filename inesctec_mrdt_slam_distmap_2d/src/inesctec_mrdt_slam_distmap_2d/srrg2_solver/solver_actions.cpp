#include "inesctec_mrdt_slam_distmap_2d/srrg2_solver/solver_actions.hpp"

#include <thread>

// SRRG Software
#include <srrg2_solver/solver_core/solver.h>

namespace srrg2_solver
{

using namespace srrg2_core;
using namespace srrg2_solver;

SolverVerboseLastAction::SolverVerboseLastAction()
{
  param_event.setValue(Solver::SolverEvent::ComputeEnd);
}

void SolverVerboseLastAction::doAction()
{
  std::cerr << _solver_ptr->iterationStats().back() << std::endl;
}

SolverDynamicRobustifierAction::SolverDynamicRobustifierAction()
{
  param_event.setValue(Solver::SolverEvent::IterationStart);
}

void SolverDynamicRobustifierAction::doAction()
{
  assert(m_robustifier &&
         "SolverDynamicRobustifierAction::doAction | "
         "robustifier not set...");
  assert(m_num_iterations_coarse > 0 &&
         "SolverDynamicRobustifierAction::doAction | "
         "number of coarse iterations must be greater than 0");
  assert(m_robustifier_threshold_coarse > 0 &&
         "SolverDynamicRobustifierAction::doAction | "
         "robustifier threshold for coarse iterations must be greater than 0");
  assert(m_robustifier_threshold_fine > 0 &&
         "SolverDynamicRobustifierAction::doAction | "
         "robustifier threshold for fine iterations must be greater than 0");

  if (_solver_ptr->iterationStats().empty())
  {
    m_robustifier->param_chi_threshold.setValue(m_robustifier_threshold_coarse);
  }
  else if (_solver_ptr->iterationStats().back().iteration <
           m_num_iterations_coarse)
  {
    m_robustifier->param_chi_threshold.setValue(m_robustifier_threshold_coarse);
  }
  else
  {
    m_robustifier->param_chi_threshold.setValue(m_robustifier_threshold_fine);
  }
}

SolverGenericAtComputeStartAction::SolverGenericAtComputeStartAction()
{
  param_event.setValue(Solver::SolverEvent::ComputeStart);

  m_action = []()
  {
    std::cerr << "SolverGenericAtComputeStartAction::doAction | "
                 "not initialized..."
              << std::endl;
  };
}

void SolverGenericAtComputeStartAction::doAction()
{
  m_action();

  if (m_sleep_ms > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_ms));
  }
}

SolverGenericAtComputeEndAction::SolverGenericAtComputeEndAction()
{
  param_event.setValue(Solver::SolverEvent::ComputeEnd);

  m_action = []()
  {
    std::cerr << "SolverGenericAtComputeEndAction::doAction | "
                 "not initialized..."
              << std::endl;
  };
}

void SolverGenericAtComputeEndAction::doAction()
{
  m_action();

  if (m_sleep_ms > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_ms));
  }
}

SolverGenericAtIterationStartAction::SolverGenericAtIterationStartAction()
{
  param_event.setValue(Solver::SolverEvent::IterationStart);

  m_action = []()
  {
    std::cerr << "SolverGenericAtIterationStartAction::doAction | "
                 "not initialized..."
              << std::endl;
  };
}

void SolverGenericAtIterationStartAction::doAction()
{
  m_action();

  if (m_sleep_ms > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_ms));
  }
}

SolverGenericAtIterationEndAction::SolverGenericAtIterationEndAction()
{
  param_event.setValue(Solver::SolverEvent::IterationEnd);

  m_action = []()
  {
    std::cerr << "SolverGenericAtIterationEndAction::doAction | "
                 "not initialized..."
              << std::endl;
  };
}

void SolverGenericAtIterationEndAction::doAction()
{
  m_action();

  if (m_sleep_ms > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(m_sleep_ms));
  }
}

}  // namespace srrg2_solver
