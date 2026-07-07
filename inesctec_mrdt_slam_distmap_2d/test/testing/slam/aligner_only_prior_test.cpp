#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/slam/aligner.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace slam;
using namespace srrg2_solver;

class AlignerOnlyPriorTest
    : public Aligner_<VariableSE2DistanceMapStaticPointNormal2fVectorCloudRight>
{
 public:

  AlignerOnlyPriorTest() = default;
  AlignerOnlyPriorTest(const Param& param) : Aligner_(param) {}

  virtual void setSource(const PointCloudTypePtr& source,
                         const EstimateType& sensor_in_robot) override
  {
  }
  virtual void setSource(VariableType* const source) override {}
  virtual void setTarget(const PointCloudTypePtr& target,
                         const EstimateType& sensor_in_robot) override
  {
  }
  virtual void setTarget(VariableType* const target) override {}

  virtual void computeStatsOnly() override {}

  virtual void initCorrespondencesRegularizer() override {}
  virtual void updateCorrespondencesRegularizer() override {}

};  // class AlignerOnlyPriorTest : public
    // Aligner_<VariableSE2DistanceMapStaticRight>

TEST(slam_aligner_only_prior, constructor)
{
  constexpr int num_attempts = 10;

  EXPECT_NO_THROW(AlignerOnlyPriorTest aligner);

  srand48(time(0));

  AlignerOnlyPriorTest aligner;

  for (int idx = 0; idx < num_attempts; idx++)
  {
    Eigen::Vector3f prior_estimate_0 = Eigen::Vector3f::Random();

    aligner.setPrior(AlignerOnlyPriorTest::EstimateType::Identity());
    aligner.setSourceInTarget(geometry2d::v2t(prior_estimate_0));
    aligner.compute();

    EXPECT_TRUE(aligner.getSourceInTarget().isApprox(
        AlignerOnlyPriorTest::EstimateType::Identity()));

    /* std::cout << std::endl
              << "slam_aligner_only_prior::constructor:" << std::endl
              << "- ground-truth      : "
                  << geometry2d::t2v(
                        AlignerOnlyPriorTest::EstimateType::Identity()).transpose()
                  << std::endl
              << "- initial estimation: "
                  << prior_estimate_0.transpose() << std::endl
              << "- final estimation  : "
                  << geometry2d::t2v(aligner.getSourceInTarget()).transpose()
                  << std::endl; */
  }
}  // TEST(slam_aligner_only_prior, constructor)

TEST(slam_aligner_only_prior, max_iterations)
{
  constexpr int num_attempts = 10;

  AlignerOnlyPriorTest::Param param;
  param.m_failure_ratio_inliers = -1.f;
  param.m_failure_chi_inliers = -1.f;
  param.m_solver_criteria_type = "";

  AlignerOnlyPriorTest aligner(param);

  for (int idx = 0; idx < num_attempts; idx++)
  {
    Eigen::Vector3f prior_estimate_0 = Eigen::Vector3f::Random();

    aligner.setPrior(AlignerOnlyPriorTest::EstimateType::Identity());
    aligner.setSourceInTarget(geometry2d::v2t(prior_estimate_0));
    aligner.compute();

    EXPECT_TRUE(aligner.getSourceInTarget().isApprox(
        AlignerOnlyPriorTest::EstimateType::Identity()));
    EXPECT_EQ(aligner.getSolver()->lastIterationStats().iteration,
              param.m_solver_max_iterations - 1);
  }
}  // TEST(slam_aligner_only_prior, max_iterations)

}  // namespace testing
