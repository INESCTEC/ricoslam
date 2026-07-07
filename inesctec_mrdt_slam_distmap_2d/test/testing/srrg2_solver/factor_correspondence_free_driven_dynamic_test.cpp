#include <gtest/gtest.h>
#include <srrg2_solver/variables_and_factors/types_2d/se2_point2point_error_factor.h>

#include <inesctec_mrdt_slam_distmap_2d/srrg2_solver/factor_correspondence_free_driven_dynamic.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;

TEST(srrg2_solver_factor_correspondence_free_driven_dynamic, interface)
{
  using FactorBaseType = srrg2_solver::SE2Point2PointErrorFactor;
  using MovingType = FactorBaseType::MovingType;
  using MovingContainerType = srrg2_core::Point2fVectorCloud;
  using Scalar = MovingType::Scalar;

  using FactorType =
      srrg2_solver::FactorCorrespondenceFreeDrivenDynamic_<FactorBaseType,
                                                           MovingContainerType>;

  std::random_device rd;
  std::mt19937 rng(rd());

  MovingContainerType pc;

  Scalar ang_res = geometry::degToRad(static_cast<Scalar>(1.0));
  Scalar fov = geometry::degToRad(static_cast<Scalar>(360.0));
  Scalar max_range = Scalar(10.0);

  test::getLaser2DPointCloudRandom(pc, ang_res, fov, max_range, rng);

  FactorType f;

  f.setMoving(pc);

  EXPECT_FALSE(f.isEnd());
  EXPECT_EQ(f.size(), pc.size());

  for (int idx = 0; idx < static_cast<int>(pc.size()); idx++)
  {
    MovingType moving = (*f.getMovingIt());

    EXPECT_TRUE(f.get());

    EXPECT_EQ(moving.coordinates().x(), pc[idx].coordinates().x());
    EXPECT_EQ(moving.coordinates().y(), pc[idx].coordinates().y());

    f.next();
  }

  EXPECT_TRUE(f.isEnd());
  EXPECT_FALSE(f.get());

  f.setBegin();

  EXPECT_FALSE(f.isEnd());

  size_t idx = 1;  // required given that getNext updates the iterator
                   // automatically... not possible to test with a getMoving due
                   // to SRRG not having that method...

  srrg2_solver::FactorBase* f_base;

  while (f.getNext(f_base))
  {
    if (idx == pc.size())
    {
      EXPECT_TRUE(f.isEnd());
      EXPECT_FALSE(f.get());
      break;
    }

    MovingType moving = (*f.getMovingIt());

    EXPECT_EQ(moving.coordinates().x(), pc[idx].coordinates().x());
    EXPECT_EQ(moving.coordinates().y(), pc[idx].coordinates().y());

    idx++;
  }
}  // TEST(srrg2_solver_factor_correspondence_free_driven_dynamic, interface)

}  // namespace testing
