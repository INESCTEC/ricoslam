#include <gtest/gtest.h>

#include <Eigen/Core>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point_field_direction.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace pcl;

using Normal2f = PointFieldDirection_<float, 2>;
using Normal2fTraits = Normal2f::TraitsType;

using Normal3f = PointFieldDirection_<float, 3>;
using Normal3fTraits = Normal3f::TraitsType;

TEST(pcl_point_field_direction, traits)
{
  EXPECT_EQ(Normal2f::Dim, 2);
  EXPECT_EQ(Normal2f::TraitsType::Dim, 2);
}  // TEST(pcl_point_field_direction, traits)

TEST(pcl_point_field_direction, setZero)
{
  Normal2f pt1, pt2, pt3;

  Normal2fTraits::setZero(pt1.m_value);
  Normal2fTraits::setZero(pt2.m_value);
  Normal2fTraits::setZero(pt3.m_value);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), 0.f);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), 0.f);
  EXPECT_FLOAT_EQ(pt2.m_value.x(), 0.f);
  EXPECT_FLOAT_EQ(pt2.m_value.y(), 0.f);
  EXPECT_FLOAT_EQ(pt3.m_value.x(), 0.f);
  EXPECT_FLOAT_EQ(pt3.m_value.y(), 0.f);
}  // TEST(pcl_point_field_direction, setZero)

TEST(pcl_point_field_direction, copyTo)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;

  Normal2f pt1, pt2;

  Normal2fTraits::setZero(pt1.m_value);
  Normal2fTraits::setZero(pt2.m_value);

  pt2.m_value.x() = x;
  pt2.m_value.y() = y;

  Normal2fTraits::copyTo(pt1.m_value, pt2.m_value);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y);
}  // TEST(pcl_point_field_direction, copyTo)

TEST(pcl_point_field_direction, add)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;
  constexpr float x2 = 1.f;
  constexpr float y2 = -7.f;

  Normal2f pt1, pt2, pt3;

  Normal2fTraits::setZero(pt1.m_value);
  Normal2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;
  pt2.m_value.x() = x2;
  pt2.m_value.y() = y2;

  Normal2fTraits::add(pt3.m_value, pt1.m_value, pt2.m_value);

  EXPECT_FLOAT_EQ(pt3.m_value.x(), x1 + x2);
  EXPECT_FLOAT_EQ(pt3.m_value.y(), y1 + y2);
}  // TEST(pcl_point_field_direction, add)

TEST(pcl_point_field_direction, addInPlace)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;
  constexpr float x2 = 1.f;
  constexpr float y2 = -7.f;

  Normal2f pt1, pt2;

  Normal2fTraits::setZero(pt1.m_value);
  Normal2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;
  pt2.m_value.x() = x2;
  pt2.m_value.y() = y2;

  Normal2fTraits::addInPlace(pt1.m_value, pt2.m_value);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1 + x2);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1 + y2);
}  // TEST(pcl_point_field_direction, addInPlace)

TEST(pcl_point_field_direction, scale)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;
  constexpr float s = 0.8f;

  Normal2f pt1, pt2;

  Normal2fTraits::setZero(pt1.m_value);
  Normal2fTraits::setZero(pt2.m_value);

  pt2.m_value.x() = x;
  pt2.m_value.y() = y;

  Normal2fTraits::scale(pt1.m_value, pt2.m_value, s);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x * s);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y * s);
}  // TEST(pcl_point_field_direction, scale)

TEST(pcl_point_field_direction, scaleInPlace)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;
  constexpr float s = 0.8f;

  Normal2f pt1;

  Normal2fTraits::setZero(pt1.m_value);

  pt1.m_value.x() = x;
  pt1.m_value.y() = y;

  Normal2fTraits::scaleInPlace(pt1.m_value, s);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x * s);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y * s);
}  // TEST(pcl_point_field_direction, scaleInPlace)

TEST(pcl_point_field_direction, addAndScaleInPlace)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;
  constexpr float x2 = 1.f;
  constexpr float y2 = -7.f;
  constexpr float s2 = 0.8f;

  Normal2f pt1, pt2;

  Normal2fTraits::setZero(pt1.m_value);
  Normal2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;
  pt2.m_value.x() = x2;
  pt2.m_value.y() = y2;

  Normal2fTraits::addAndScaleInPlace(pt1.m_value, pt2.m_value, s2);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1 + x2 * s2);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1 + y2 * s2);
}  // TEST(pcl_point_field_direction, addAndScaleInPlace)

TEST(pcl_point_field_direction, transform)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;

  Eigen::Isometry2f transformation = Eigen::Isometry2f::Identity();

  Normal2f pt1, pt2;

  Normal2fTraits::setZero(pt1.m_value);
  Normal2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Normal2fTraits::transform(pt2.m_value, pt1.m_value, transformation);

  EXPECT_FLOAT_EQ(pt2.m_value.x(), pt1.m_value.x());
  EXPECT_FLOAT_EQ(pt2.m_value.y(), pt1.m_value.y());

  Normal2fTraits::transform(pt2.m_value, pt1.m_value,
                            geometry::v2t(Eigen::Vector3f(x1, y1, 0)));

  //! direction is not affected by translation!!!!

  EXPECT_FLOAT_EQ(pt2.m_value.x(), pt1.m_value.x());
  EXPECT_FLOAT_EQ(pt2.m_value.y(), pt1.m_value.y());

  Normal2fTraits::transform(
      pt2.m_value, pt1.m_value,
      geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.m_value.x(), -pt1.m_value.y(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), pt1.m_value.x(), 1e-5);

  Normal2fTraits::transform(
      pt2.m_value, pt1.m_value,
      geometry::v2t(Eigen::Vector3f(0, 0, -M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.m_value.x(), pt1.m_value.y(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -pt1.m_value.x(), 1e-5);

  Normal2fTraits::transform(pt2.m_value, pt1.m_value,
                            geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32)));

  EXPECT_NEAR(pt2.m_value.x(), -pt1.m_value.x(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -pt1.m_value.y(), 1e-5);

  Normal2fTraits::transform(
      pt2.m_value, pt1.m_value,
      geometry::v2t(Eigen::Vector3f(x1, y1, M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.m_value.x(), -pt1.m_value.y(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), pt1.m_value.x(), 1e-5);

  Normal2fTraits::transform(
      pt2.m_value, pt1.m_value,
      geometry::v2t(Eigen::Vector3f(x1, y1, -M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.m_value.x(), pt1.m_value.y(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -pt1.m_value.x(), 1e-5);

  Normal2fTraits::transform(pt2.m_value, pt1.m_value,
                            geometry::v2t(Eigen::Vector3f(x1, y1, M_PIf32)));

  EXPECT_NEAR(pt2.m_value.x(), -pt1.m_value.x(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -pt1.m_value.y(), 1e-5);
}  // TEST(pcl_point_field_direction, transform)

TEST(pcl_point_field_direction, transformInPlace)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;

  Eigen::Isometry2f transformation = Eigen::Isometry2f::Identity();

  Normal2f pt1;

  Normal2fTraits::setZero(pt1.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Normal2fTraits::transformInPlace(pt1.m_value, transformation);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1);

  Normal2fTraits::transformInPlace(pt1.m_value,
                                   geometry::v2t(Eigen::Vector3f(x1, y1, 0)));

  //! direction is not affected by translation!!!!

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Normal2fTraits::transformInPlace(
      pt1.m_value, geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt1.m_value.x(), -y1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), x1, 1e-5);

  Normal2fTraits::transformInPlace(
      pt1.m_value, geometry::v2t(Eigen::Vector3f(0, 0, -M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt1.m_value.x(), x1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), y1, 1e-5);

  Normal2fTraits::transformInPlace(
      pt1.m_value, geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32)));

  EXPECT_NEAR(pt1.m_value.x(), -x1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -y1, 1e-5);
}  // TEST(pcl_point_field_direction, transformInPlace)

TEST(pcl_point_field_direction, normalize)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;

  float norm1 = sqrt(x1 * x1 + y1 * y1);

  Normal2f pt1;

  Normal2fTraits::setZero(pt1.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Normal2fTraits::normalize(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), x1 / norm1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), y1 / norm1, 1e-5);
}  // TEST(pcl_point_field_direction, normalize)

TEST(pcl_point_field_direction, toStream)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;

  Normal2f pt1;

  pt1.m_value.x() = x;
  pt1.m_value.y() = y;

  // Normal2fTraits::toStream(std::cout, pt1.m_value);
  // std::cout << std::endl;
}  // TEST(pcl_point_field_direction, toStream)

}  // namespace testing
