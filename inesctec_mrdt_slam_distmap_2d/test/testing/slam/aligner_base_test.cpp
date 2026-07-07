#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/slam/aligner.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;

class AlignerBaseTest : public AlignerBase
{
 public:

  AlignerBaseTest() = default;
  AlignerBaseTest(const Param& param) : AlignerBase(param) {}

  void compute() override {}
  void computeStatsOnly() override {}

};  // class AlignerBaseTest : public AlignerBase

TEST(slam_aligner_base, constructor)
{
  AlignerBaseTest::Param param;

  ASSERT_NO_THROW(AlignerBaseTest aligner);
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));  // default values

  AlignerBaseTest aligner;
  EXPECT_NO_THROW(
      aligner.setParam(AlignerBaseTest::Param()));  // default values

  param.m_solver_damping = -1.f;
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_solver_damping = 0.f;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));

  param.m_solver_max_iterations = 0;
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_solver_max_iterations = 30;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));

  param.m_solver_criteria_type = "";
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));
  param.m_solver_criteria_type = "stepnorm";
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));

  param.m_solver_criteria_epsilon = -1.f;
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_solver_criteria_epsilon = 1e-5;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));

  param.m_solver_robustifier_type = "";
  param.m_solver_robustifier_num_iterations_coarse = -1.f;
  param.m_solver_robustifier_threshold_coarse = -1.f;
  param.m_solver_robustifier_threshold_fine = -1.f;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));
  param.m_solver_robustifier_type = "cauchy";
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_solver_robustifier_num_iterations_coarse = 0;
  param.m_solver_robustifier_threshold_coarse = -1.f;
  param.m_solver_robustifier_threshold_fine = 0.0025f;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));
  param.m_solver_robustifier_num_iterations_coarse = 10;
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_solver_robustifier_threshold_coarse = 0.025f;
  param.m_solver_robustifier_num_iterations_coarse = 40;
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_solver_robustifier_num_iterations_coarse = 10;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));

  param.m_debug = true;
  param.m_dbg_ogl_mode = "";
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_dbg_ogl_mode = "iterationend";
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));
  param.m_dbg_ogl_interval = -10;
  EXPECT_ANY_THROW(AlignerBaseTest aligner(param));
  EXPECT_ANY_THROW(aligner.setParam(param));
  param.m_dbg_ogl_interval = 0;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));
  param.m_dbg_ogl_interval = 10;
  EXPECT_NO_THROW(AlignerBaseTest aligner(param));
  EXPECT_NO_THROW(aligner.setParam(param));

  std::cout << "Test print parametrization in std::cout" << std::endl
            << aligner.getParam() << std::endl;
}  // TEST(slam_aligner_base, constructor)

TEST(slam_aligner_base, constructor_default)
{
  AlignerBaseTest aligner;

  EXPECT_FALSE(aligner.getSolver()->param_verbose.value());
  EXPECT_FLOAT_EQ(dynamic_cast<AlignerBaseTest::IterationAlgorithm*>(
                      aligner.getSolver()->param_algorithm.value().get())
                      ->param_damping.value(),
                  1.f);
  EXPECT_EQ(aligner.getSolver()->param_max_iterations.value().front(), 25);
  EXPECT_TRUE(aligner.getSolver()->param_termination_criteria.name().compare(
      "stepnorm"));
  EXPECT_TRUE(aligner.getSolver()
                  ->param_robustifier_policies.value(0)
                  ->param_robustifier.name()
                  .compare("cauchy"));

  aligner.setParam(AlignerBaseTest::Param());

  EXPECT_FALSE(aligner.getSolver()->param_verbose.value());
  EXPECT_FLOAT_EQ(dynamic_cast<AlignerBaseTest::IterationAlgorithm*>(
                      aligner.getSolver()->param_algorithm.value().get())
                      ->param_damping.value(),
                  1.f);
  EXPECT_EQ(aligner.getSolver()->param_max_iterations.value().front(), 25);
  EXPECT_TRUE(aligner.getSolver()->param_termination_criteria.name().compare(
      "stepnorm"));
  EXPECT_TRUE(aligner.getSolver()
                  ->param_robustifier_policies.value(0)
                  ->param_robustifier.name()
                  .compare("cauchy"));

  AlignerBaseTest::Param param;

  param.m_debug = true;
  param.m_solver_verbose = true;
  param.m_dbg_ogl_mode = "iterationend";
  param.m_dbg_ogl_interval = 0;
  param.m_solver_damping = 2.f;
  param.m_solver_robustifier_type = "";
  param.m_solver_criteria_type = "";
  param.m_solver_max_iterations = 10;

  aligner.setParam(param);

  EXPECT_TRUE(aligner.getSolver()->param_verbose.value());
  EXPECT_FLOAT_EQ(dynamic_cast<AlignerBaseTest::IterationAlgorithm*>(
                      aligner.getSolver()->param_algorithm.value().get())
                      ->param_damping.value(),
                  2.f);
  EXPECT_EQ(aligner.getSolver()->param_max_iterations.value().front(), 10);
  EXPECT_FALSE(aligner.getSolver()->param_termination_criteria.value());
  EXPECT_TRUE(aligner.getSolver()->param_robustifier_policies.empty());

}  // TEST(slam_aligner_base, constructor_default)

}  // namespace testing
