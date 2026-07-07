#pragma once

#include <chrono>
#include <functional>
#include <memory>

// SRRG Software
#include <srrg2_solver/solver_core/robustifier.h>
#include <srrg2_solver/solver_core/solver_action_base.h>

namespace srrg2_solver
{

/**
 * @brief only print the last iteration stats at the computation end event
 */
class SolverVerboseLastAction : public SolverActionBase
{
 public:

  using BaseType = SolverActionBase;
  using ThisType = SolverVerboseLastAction;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverVerboseLastAction();

  void doAction() override;

};  // class SolverVerboseLastAction : public SolverActionBase

class SolverDynamicRobustifierAction : public SolverActionBase
{
 public:

  using BaseType = SolverActionBase;
  using ThisType = SolverDynamicRobustifierAction;

 public:

  RobustifierBase* m_robustifier = nullptr;

  int m_num_iterations_coarse = -1;

  float m_robustifier_threshold_coarse = -1.f;
  float m_robustifier_threshold_fine = -1.f;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverDynamicRobustifierAction();

  void doAction() override;

};  // class SolverDynamicRobustifierAction : public SolverActionBase

/**
 * @brief generic action event (w/ void() functional member variable)
 *        at compute start
 */
class SolverGenericAtComputeStartAction : public SolverActionBase
{
 public:

  using BaseType = SolverActionBase;
  using ThisType = SolverGenericAtComputeStartAction;

 public:

  std::function<void()> m_action;
  int64_t m_sleep_ms = -1;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverGenericAtComputeStartAction();

  void doAction() override;

};  // class SolverGenericAtComputeStartAction : public SolverActionBase

/**
 * @brief generic action event (w/ void() functional member variable)
 *        at compute end
 */
class SolverGenericAtComputeEndAction : public SolverActionBase
{
 public:

  using BaseType = SolverActionBase;
  using ThisType = SolverGenericAtComputeEndAction;

 public:

  std::function<void()> m_action;
  int64_t m_sleep_ms = -1;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverGenericAtComputeEndAction();

  void doAction() override;

};  // class SolverGenericAtComputeEndAction : public SolverActionBase

/**
 * @brief generic action event (w/ void() functional member variable)
 *        at iteration start
 */
class SolverGenericAtIterationStartAction : public SolverActionBase
{
 public:

  using BaseType = SolverActionBase;
  using ThisType = SolverGenericAtIterationStartAction;

 public:

  std::function<void()> m_action;
  int64_t m_sleep_ms = -1;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverGenericAtIterationStartAction();

  void doAction() override;

};  // class SolverGenericAtIterationStartAction : public SolverActionBase

/**
 * @brief generic action event (w/ void() functional member variable)
 *        at iteration end
 */
class SolverGenericAtIterationEndAction : public SolverActionBase
{
 public:

  using BaseType = SolverActionBase;
  using ThisType = SolverGenericAtIterationEndAction;

 public:

  std::function<void()> m_action;
  int64_t m_sleep_ms = -1;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SolverGenericAtIterationEndAction();

  void doAction() override;

};  // class SolverGenericAtIterationEndAction : public SolverActionBase

using SolverVerboseLastActionPtr = std::shared_ptr<SolverVerboseLastAction>;
using SolverDynamicRobustifierActionPtr =
    std::shared_ptr<SolverDynamicRobustifierAction>;
using SolverGenericAtComputeStartActionPtr =
    std::shared_ptr<SolverGenericAtComputeStartAction>;
using SolverGenericAtComputeEndActionPtr =
    std::shared_ptr<SolverGenericAtComputeEndAction>;
using SolverGenericAtIterationEndActionPtr =
    std::shared_ptr<SolverGenericAtIterationEndAction>;
using SolverGenericAtIterationStartActionPtr =
    std::shared_ptr<SolverGenericAtIterationStartAction>;

}  // namespace srrg2_solver
