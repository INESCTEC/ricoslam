#include <gtest/gtest.h>
#include <srrg2_solver/solver_core/error_factor.h>
#include <srrg2_solver/solver_core/robustifier.h>
#include <srrg2_solver/variables_and_factors/types_2d/variable_se2.h>

#include <srrg2_solver/solver_core/error_factor_impl.hpp>

namespace srrg2_solver
{

template <typename VariableType_>
class RobustifierTestErrorFactor_ : public ErrorFactor_<1, VariableType_>
{
 public:

  using ThisType = RobustifierTestErrorFactor_<VariableType_>;
  using BaseType = ErrorFactor_<1, VariableType_>;
  using VariableType = VariableType_;
  using EstimateType = typename VariableType::EstimateType;

 public:

  float m_error = 0.f;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void errorAndJacobian(bool error_only = false) override
  {
    this->_is_valid = true;
    this->_e(0, 0) = this->m_error;

    if (error_only)
    {
      return;
    }

    this->_J.setZero();
  }
};  // class RobustifierTestErrorFactor_<VariableType_> : public ErrorFactor_<1,
    // VariableType_>

using RobustifierTestErrorFactor =
    RobustifierTestErrorFactor_<srrg2_solver::VariableSE2Right>;

}  // namespace srrg2_solver

namespace testing
{

constexpr float distance_threshold = 0.06f;
constexpr float kernel_threshold = distance_threshold * distance_threshold;

TEST(srrg2_solver_robustifier, saturated)
{
  using FactorType = srrg2_solver::RobustifierTestErrorFactor;
  using RobustifierType = srrg2_solver::RobustifierSaturated;

  FactorType f;
  RobustifierType r;

  r.param_chi_threshold.setValue(kernel_threshold);

  f.setRobustifier(&r);
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 0.95f * distance_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 1.05f * distance_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Kernelized);
}  // TEST(srrg2_solver_robustifier, saturated)

TEST(srrg2_solver_robustifier, cauchy)
{
  using FactorType = srrg2_solver::RobustifierTestErrorFactor;
  using RobustifierType = srrg2_solver::RobustifierCauchy;

  FactorType f;
  RobustifierType r;

  r.param_chi_threshold.setValue(kernel_threshold);

  f.setRobustifier(&r);
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 0.95f * distance_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 1.05f * distance_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Kernelized);
}  // TEST(srrg2_solver_robustifier, cauchy)

TEST(srrg2_solver_robustifier, clamp)
{
  using FactorType = srrg2_solver::RobustifierTestErrorFactor;
  using RobustifierType = srrg2_solver::RobustifierClamp;

  FactorType f;
  RobustifierType r;

  r.param_chi_threshold.setValue(kernel_threshold);

  f.setRobustifier(&r);
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 0.95f * distance_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 1.05f * distance_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Kernelized);
}  // TEST(srrg2_solver_robustifier, clamp)

TEST(srrg2_solver_robustifier, huber)
{
  /**
   * @note seems that Huber robustifier considers the chi threshold as the error
   *       thhreshold...
   *       (decides to kernelize considering the residual norm of chi)
   */

  using FactorType = srrg2_solver::RobustifierTestErrorFactor;
  using RobustifierType = srrg2_solver::RobustifierHuber;

  FactorType f;
  RobustifierType r;

  r.param_chi_threshold.setValue(kernel_threshold);

  f.setRobustifier(&r);
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 0.95f * kernel_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Inlier);

  f.m_error = 1.05f * kernel_threshold;
  f.compute(true, true);

  EXPECT_EQ(f.stats().status, srrg2_solver::FactorStats::Status::Kernelized);
}  // TEST(srrg2_solver_robustifier, huber)

}  // namespace testing
