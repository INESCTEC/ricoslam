#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point_derived.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace geometry;
using namespace pcl;

using PointBaseExampleType =
    PointDerived_<Point3f, PointFieldDirection_<float, 3>>;

using PointBaseTransformExampleType =
    PointDerived_<Point2f, PointFieldDirection_<float, 2>>;

TEST(pcl_point_derived, definition)
{
  EXPECT_EQ(PointBaseExampleType::BaseType::kNumFields, 2);
  EXPECT_EQ(PointBaseExampleType::BaseType::kNumOuterFields, 1);

  EXPECT_EQ(PointBaseExampleType::BaseType::Dim, 3);

  EXPECT_EQ(PointBaseTransformExampleType::BaseType::kNumFields, 2);
  EXPECT_EQ(PointBaseTransformExampleType::BaseType::kNumOuterFields, 1);

  EXPECT_EQ(PointBaseTransformExampleType::BaseType::Dim, 2);

  EXPECT_TRUE(
      (std::is_same<PointBaseExampleType::ValueTypeAt<0>, Vector3f>::value));
  EXPECT_TRUE(
      (std::is_same<PointBaseExampleType::ValueTypeAt<1>, Vector3f>::value));
  EXPECT_TRUE((std::is_same<PointBaseTransformExampleType::ValueTypeAt<0>,
                            Vector2f>::value));
  EXPECT_TRUE((std::is_same<PointBaseTransformExampleType::ValueTypeAt<1>,
                            Vector2f>::value));
}  // TEST(pcl_point_derived, definition)

TEST(pcl_point_derived, constructor)
{
  PointBaseExampleType pt1;
  PointBaseExampleType pt2(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));
}  // TEST(pcl_point_derived, constructor)

TEST(pcl_point_derived, valueAt)
{
  PointBaseExampleType pt(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));

  EXPECT_FLOAT_EQ(pt.value<0>().x(), 1.f);
  EXPECT_FLOAT_EQ(pt.value<0>().y(), 2.f);
  EXPECT_FLOAT_EQ(pt.value<0>().z(), 3.f);
  EXPECT_FLOAT_EQ(pt.value<1>().x(), -3.f);
  EXPECT_FLOAT_EQ(pt.value<1>().y(), -2.f);
  EXPECT_FLOAT_EQ(pt.value<1>().z(), -1.f);
}  // TEST(pcl_point_derived, valueAt)

TEST(pcl_point_derived, setZero)
{
  PointBaseExampleType pt1(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));

  ASSERT_FLOAT_EQ(pt1.value<0>().x(), 1.f);
  ASSERT_FLOAT_EQ(pt1.value<0>().y(), 2.f);
  ASSERT_FLOAT_EQ(pt1.value<0>().z(), 3.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().x(), -3.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().y(), -2.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().z(), -1.f);

  pt1.setZero<0>();

  EXPECT_FLOAT_EQ(pt1.value<0>().x(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<0>().y(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<0>().z(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<1>().x(), -3.f);
  EXPECT_FLOAT_EQ(pt1.value<1>().y(), -2.f);
  EXPECT_FLOAT_EQ(pt1.value<1>().z(), -1.f);

  pt1.setZero<1>();

  EXPECT_FLOAT_EQ(pt1.value<0>().x(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<0>().y(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<0>().z(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<1>().x(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<1>().y(), 0.f);
  EXPECT_FLOAT_EQ(pt1.value<1>().z(), 0.f);

  PointBaseExampleType pt2(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));

  pt2.setZero();

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), 0.f);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), 0.f);
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), 0.f);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), 0.f);
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), 0.f);
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), 0.f);
}  // TEST(pcl_point_derived, setZero)

TEST(pcl_point_derived, copyTo)
{
  PointBaseExampleType pt1(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));

  ASSERT_FLOAT_EQ(pt1.value<0>().x(), 1.f);
  ASSERT_FLOAT_EQ(pt1.value<0>().y(), 2.f);
  ASSERT_FLOAT_EQ(pt1.value<0>().z(), 3.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().x(), -3.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().y(), -2.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().z(), -1.f);

  PointBaseExampleType pt2;

  pt2.setZero();

  ASSERT_FLOAT_EQ(pt2.value<0>().x(), 0.f);
  ASSERT_FLOAT_EQ(pt2.value<0>().y(), 0.f);
  ASSERT_FLOAT_EQ(pt2.value<0>().z(), 0.f);
  ASSERT_FLOAT_EQ(pt2.value<1>().x(), 0.f);
  ASSERT_FLOAT_EQ(pt2.value<1>().y(), 0.f);
  ASSERT_FLOAT_EQ(pt2.value<1>().z(), 0.f);

  pt2.copyFrom(pt1);

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), 1.f);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), 2.f);
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), 3.f);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), -3.f);
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), -2.f);
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), -1.f);
}  // TEST(pcl_point_derived, copyTo)

TEST(pcl_point_derived, add)
{
  PointBaseExampleType pt1(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));

  PointBaseExampleType pt2(
      PointFieldCoordinates_<float, 3>(Vector3f(4.f, -1.f, 10.f)),
      PointFieldDirection_<float, 3>(Vector3f(-7.f, -8.f, -9.f)));

  PointBaseExampleType pt3;

  pt3.setZero();

  pt3 = pt1 + pt2;

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() + pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() + pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() + pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x() + pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y() + pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z() + pt2.value<1>().z());

  pt3 += pt1;

  EXPECT_FLOAT_EQ(pt3.value<0>().x(),
                  pt1.value<0>().x() + pt1.value<0>().x() + pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(),
                  pt1.value<0>().y() + pt1.value<0>().y() + pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(),
                  pt1.value<0>().z() + pt1.value<0>().z() + pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(),
                  pt1.value<1>().x() + pt1.value<1>().x() + pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(),
                  pt1.value<1>().y() + pt1.value<1>().y() + pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(),
                  pt1.value<1>().z() + pt1.value<1>().z() + pt2.value<1>().z());

  pt1 = PointBaseExampleType(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));

  pt2 = PointBaseExampleType(
      PointFieldCoordinates_<float, 3>(Vector3f(4.f, -1.f, 10.f)),
      PointFieldDirection_<float, 3>(Vector3f(-7.f, -8.f, -9.f)));

  ASSERT_FLOAT_EQ(pt1.value<0>().x(), 1.f);
  ASSERT_FLOAT_EQ(pt1.value<0>().y(), 2.f);
  ASSERT_FLOAT_EQ(pt1.value<0>().z(), 3.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().x(), -3.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().y(), -2.f);
  ASSERT_FLOAT_EQ(pt1.value<1>().z(), -1.f);

  ASSERT_FLOAT_EQ(pt2.value<0>().x(), 4.f);
  ASSERT_FLOAT_EQ(pt2.value<0>().y(), -1.f);
  ASSERT_FLOAT_EQ(pt2.value<0>().z(), 10.f);
  ASSERT_FLOAT_EQ(pt2.value<1>().x(), -7.f);
  ASSERT_FLOAT_EQ(pt2.value<1>().y(), -8.f);
  ASSERT_FLOAT_EQ(pt2.value<1>().z(), -9.f);

  pt3.setZero();

  pt3 = pt1.operator+ <0>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() + pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() + pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() + pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z());

  pt3 = pt1.operator+ <1>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x() + pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y() + pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z() + pt2.value<1>().z());

  pt3 = pt1;

  pt3.operator+= <0>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() + pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() + pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() + pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z());

  pt3.operator+= <1>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() + pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() + pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() + pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x() + pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y() + pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z() + pt2.value<1>().z());
}  // TEST(pcl_point_derived, add)

TEST(pcl_point_derived, scale)
{
  PointBaseExampleType pt1(
      PointFieldCoordinates_<float, 3>(Vector3f(1.f, 2.f, 3.f)),
      PointFieldDirection_<float, 3>(Vector3f(-3.f, -2.f, -1.f)));

  PointBaseExampleType pt2;

  pt2.setZero();

  pt2 = pt1 * 9.33;

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), pt1.value<0>().z() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), pt1.value<1>().z() * 9.33);

  pt2 = pt1;

  pt2 *= 0.33;

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x() * 0.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y() * 0.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), pt1.value<0>().z() * 0.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x() * 0.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y() * 0.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), pt1.value<1>().z() * 0.33);

  pt2.setZero();

  pt2 = pt1.operator* <0>(9.33);

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), pt1.value<0>().z() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y());
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), pt1.value<1>().z());

  pt2 = pt1.operator* <1>(9.33);

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x());
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y());
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), pt1.value<0>().z());
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), pt1.value<1>().z() * 9.33);

  pt2 = pt1;

  pt2.operator*= <0>(9.33);

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), pt1.value<0>().z() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y());
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), pt1.value<1>().z());

  pt2.operator*= <1>(0.33);

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<0>().z(), pt1.value<0>().z() * 9.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x() * 0.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y() * 0.33);
  EXPECT_FLOAT_EQ(pt2.value<1>().z(), pt1.value<1>().z() * 0.33);
}  // TEST(pcl_point_derived, scale)

TEST(pcl_point_derived, subtract)
{
  PointBaseExampleType pt1(
      PointFieldCoordinates_<float, 3>(Vector3f(1.11f, 2.22f, 3.33f)),
      PointFieldDirection_<float, 3>(Vector3f(-33.3f, -22.2f, -11.1f)));

  PointBaseExampleType pt2(
      PointFieldCoordinates_<float, 3>(Vector3f(4.44f, -1.11f, 10.321f)),
      PointFieldDirection_<float, 3>(Vector3f(-72.4f, -8.12f, -9.12f)));

  PointBaseExampleType pt3;

  pt3.setZero();

  pt3 = pt1 - pt2;

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() - pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() - pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() - pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x() - pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y() - pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z() - pt2.value<1>().z());

  pt3 -= pt1;

  EXPECT_FLOAT_EQ(pt3.value<0>().x(),
                  pt1.value<0>().x() - pt1.value<0>().x() - pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(),
                  pt1.value<0>().y() - pt1.value<0>().y() - pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(),
                  pt1.value<0>().z() - pt1.value<0>().z() - pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(),
                  pt1.value<1>().x() - pt1.value<1>().x() - pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(),
                  pt1.value<1>().y() - pt1.value<1>().y() - pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(),
                  pt1.value<1>().z() - pt1.value<1>().z() - pt2.value<1>().z());

  pt1 = PointBaseExampleType(
      PointFieldCoordinates_<float, 3>(Vector3f(1.11f, 2.22f, 3.33f)),
      PointFieldDirection_<float, 3>(Vector3f(-33.3f, -22.2f, -11.1f)));

  pt2 = PointBaseExampleType(
      PointFieldCoordinates_<float, 3>(Vector3f(4.44f, -1.11f, 10.321f)),
      PointFieldDirection_<float, 3>(Vector3f(-72.4f, -8.12f, -9.12f)));

  ASSERT_FLOAT_EQ(pt1.value<0>().x(), 1.11f);
  ASSERT_FLOAT_EQ(pt1.value<0>().y(), 2.22f);
  ASSERT_FLOAT_EQ(pt1.value<0>().z(), 3.33f);
  ASSERT_FLOAT_EQ(pt1.value<1>().x(), -33.3f);
  ASSERT_FLOAT_EQ(pt1.value<1>().y(), -22.2f);
  ASSERT_FLOAT_EQ(pt1.value<1>().z(), -11.1f);

  ASSERT_FLOAT_EQ(pt2.value<0>().x(), 4.44f);
  ASSERT_FLOAT_EQ(pt2.value<0>().y(), -1.11f);
  ASSERT_FLOAT_EQ(pt2.value<0>().z(), 10.321f);
  ASSERT_FLOAT_EQ(pt2.value<1>().x(), -72.4f);
  ASSERT_FLOAT_EQ(pt2.value<1>().y(), -8.12f);
  ASSERT_FLOAT_EQ(pt2.value<1>().z(), -9.12f);

  pt3.setZero();

  pt3 = pt1.operator- <0>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() - pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() - pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() - pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z());

  pt3 = pt1.operator- <1>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x() - pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y() - pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z() - pt2.value<1>().z());

  pt3 = pt1;

  pt3.operator-= <0>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() - pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() - pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() - pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z());

  pt3.operator-= <1>(pt2);

  EXPECT_FLOAT_EQ(pt3.value<0>().x(), pt1.value<0>().x() - pt2.value<0>().x());
  EXPECT_FLOAT_EQ(pt3.value<0>().y(), pt1.value<0>().y() - pt2.value<0>().y());
  EXPECT_FLOAT_EQ(pt3.value<0>().z(), pt1.value<0>().z() - pt2.value<0>().z());
  EXPECT_FLOAT_EQ(pt3.value<1>().x(), pt1.value<1>().x() - pt2.value<1>().x());
  EXPECT_FLOAT_EQ(pt3.value<1>().y(), pt1.value<1>().y() - pt2.value<1>().y());
  EXPECT_FLOAT_EQ(pt3.value<1>().z(), pt1.value<1>().z() - pt2.value<1>().z());
}  // TEST(pcl_point_derived, subtract)

TEST(pcl_point_derived, normalize)
{
  constexpr float pt_coord_x = 1.11f;
  constexpr float pt_coord_y = 2.22f;
  constexpr float pt_coord_z = 3.33f;

  constexpr float pt_normal_x = -33.3f;
  constexpr float pt_normal_y = -22.2f;
  constexpr float pt_normal_z = -11.1f;

  const float pt_coord_norm =
      sqrt(pt_coord_x * pt_coord_x + pt_coord_y * pt_coord_y +
           pt_coord_z * pt_coord_z);

  const float pt_normal_norm =
      sqrt(pt_normal_x * pt_normal_x + pt_normal_y * pt_normal_y +
           pt_normal_z * pt_normal_z);

  PointBaseExampleType pt(PointFieldCoordinates_<float, 3>(
                              Vector3f(pt_coord_x, pt_coord_y, pt_coord_z)),
                          PointFieldDirection_<float, 3>(
                              Vector3f(pt_normal_x, pt_normal_y, pt_normal_z)));

  pt.normalize();

  EXPECT_NEAR(pt.value<0>().x(), pt_coord_x / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<0>().y(), pt_coord_y / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<0>().z(), pt_coord_z / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<1>().x(), pt_normal_x / pt_normal_norm, 1e-5);
  EXPECT_NEAR(pt.value<1>().y(), pt_normal_y / pt_normal_norm, 1e-5);
  EXPECT_NEAR(pt.value<1>().z(), pt_normal_z / pt_normal_norm, 1e-5);

  pt = PointBaseExampleType(PointFieldCoordinates_<float, 3>(
                                Vector3f(pt_coord_x, pt_coord_y, pt_coord_z)),
                            PointFieldDirection_<float, 3>(Vector3f(
                                pt_normal_x, pt_normal_y, pt_normal_z)));

  pt.normalize<0>();

  EXPECT_NEAR(pt.value<0>().x(), pt_coord_x / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<0>().y(), pt_coord_y / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<0>().z(), pt_coord_z / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<1>().x(), pt_normal_x, 1e-5);
  EXPECT_NEAR(pt.value<1>().y(), pt_normal_y, 1e-5);
  EXPECT_NEAR(pt.value<1>().z(), pt_normal_z, 1e-5);

  pt.normalize<1>();

  EXPECT_NEAR(pt.value<0>().x(), pt_coord_x / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<0>().y(), pt_coord_y / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<0>().z(), pt_coord_z / pt_coord_norm, 1e-5);
  EXPECT_NEAR(pt.value<1>().x(), pt_normal_x / pt_normal_norm, 1e-5);
  EXPECT_NEAR(pt.value<1>().y(), pt_normal_y / pt_normal_norm, 1e-5);
  EXPECT_NEAR(pt.value<1>().z(), pt_normal_z / pt_normal_norm, 1e-5);
}  // TEST(pcl_point_derived, normalize)

TEST(pcl_point_derived, transform)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;

  PointBaseTransformExampleType pt1(
      PointFieldCoordinates_<float, 2>(Vector2f(x, y)),
      PointFieldDirection_<float, 2>(Vector2f(x, y)));

  PointBaseTransformExampleType pt2;

  Eigen::Isometry2f transformation = Eigen::Isometry2f::Identity();

  pt2 = pt1.transform(transformation);

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x());
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y());
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y());

  pt2 = pt1.transform(geometry::v2t(Eigen::Vector3f(x, y, 0)));

  //! direction is not affected by translation!!!!

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x() + x);
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y() + y);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y());

  pt2 = pt1.transform(geometry::v2t(Eigen::Vector3f(x, y, M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.value<0>().x(), -pt1.value<0>().y() + x, 1e-5);
  EXPECT_NEAR(pt2.value<0>().y(), pt1.value<0>().x() + y, 1e-5);
  EXPECT_NEAR(pt2.value<1>().x(), -pt1.value<0>().y(), 1e-5);
  EXPECT_NEAR(pt2.value<1>().y(), pt1.value<0>().x(), 1e-5);

  pt2 = pt1.transform<1>(geometry::v2t(Eigen::Vector3f(x, y, M_PIf32 * 0.5f)));

  EXPECT_FLOAT_EQ(pt2.value<0>().x(), pt1.value<0>().x());
  EXPECT_FLOAT_EQ(pt2.value<0>().y(), pt1.value<0>().y());
  EXPECT_NEAR(pt2.value<1>().x(), -pt1.value<0>().y(), 1e-5);
  EXPECT_NEAR(pt2.value<1>().y(), pt1.value<0>().x(), 1e-5);

  pt2 = pt1.transform<0>(geometry::v2t(Eigen::Vector3f(x, y, M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.value<0>().x(), -pt1.value<0>().y() + x, 1e-5);
  EXPECT_NEAR(pt2.value<0>().y(), pt1.value<0>().x() + y, 1e-5);
  EXPECT_FLOAT_EQ(pt2.value<1>().x(), pt1.value<1>().x());
  EXPECT_FLOAT_EQ(pt2.value<1>().y(), pt1.value<1>().y());
}  // TEST(pcl_point_derived, transform)

TEST(pcl_point_derived, transformInPlace)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;

  PointBaseTransformExampleType pt(
      PointFieldCoordinates_<float, 2>(Vector2f(x, y)),
      PointFieldDirection_<float, 2>(Vector2f(x, y)));

  pt.transformInPlace(Eigen::Isometry2f::Identity());

  EXPECT_FLOAT_EQ(pt.value<0>().x(), x);
  EXPECT_FLOAT_EQ(pt.value<0>().y(), y);
  EXPECT_FLOAT_EQ(pt.value<1>().x(), x);
  EXPECT_FLOAT_EQ(pt.value<1>().y(), y);

  pt.transformInPlace(geometry::v2t(Eigen::Vector3f(x, y, 0)));

  EXPECT_FLOAT_EQ(pt.value<0>().x(), x + x);
  EXPECT_FLOAT_EQ(pt.value<0>().y(), y + y);
  EXPECT_FLOAT_EQ(pt.value<1>().x(), x);
  EXPECT_FLOAT_EQ(pt.value<1>().y(), y);

  pt.transformInPlace(geometry::v2t(Eigen::Vector3f(x, y, M_PIf32 * 0.5f)));

  EXPECT_FLOAT_EQ(pt.value<0>().x(), -(y + y) + x);
  EXPECT_FLOAT_EQ(pt.value<0>().y(), (x + x) + y);
  EXPECT_FLOAT_EQ(pt.value<1>().x(), -y);
  EXPECT_FLOAT_EQ(pt.value<1>().y(), x);

  pt = PointBaseTransformExampleType(
      PointFieldCoordinates_<float, 2>(Vector2f(x, y)),
      PointFieldDirection_<float, 2>(Vector2f(x, y)));

  pt.transformInPlace<1>(geometry::v2t(Eigen::Vector3f(x, y, M_PIf32 * 0.5f)));

  EXPECT_FLOAT_EQ(pt.value<0>().x(), x);
  EXPECT_FLOAT_EQ(pt.value<0>().y(), y);
  EXPECT_FLOAT_EQ(pt.value<1>().x(), -y);
  EXPECT_FLOAT_EQ(pt.value<1>().y(), x);

  pt.transformInPlace<0>(geometry::v2t(Eigen::Vector3f(x, y, M_PIf32 * 0.5f)));

  EXPECT_FLOAT_EQ(pt.value<0>().x(), -y + x);
  EXPECT_FLOAT_EQ(pt.value<0>().y(), x + y);
  EXPECT_FLOAT_EQ(pt.value<1>().x(), -y);
  EXPECT_FLOAT_EQ(pt.value<1>().y(), x);
}  // TEST(pcl_point_derived, transformInPlace)

TEST(pcl_point_derived, euclideanToPolar)
{
  constexpr float x = 1.f;
  constexpr float y = 1.f;

  PointBaseTransformExampleType pt(
      PointFieldCoordinates_<float, 2>(Vector2f(x, y)),
      PointFieldDirection_<float, 2>(Vector2f(x, y)));

  PointBaseTransformExampleType pt_polar = pt.euclideanToPolar();

  EXPECT_NEAR(pt_polar.value<0>().x(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt_polar.value<0>().y(), sqrt(2), 1e-5);
  EXPECT_FLOAT_EQ(pt_polar.value<1>().x(), x);
  EXPECT_FLOAT_EQ(pt_polar.value<1>().y(), y);
}  // TEST(pcl_point_derived, euclideanToPolar)

TEST(pcl_point_derived, polarToEuclidean)
{
  const float x = M_PIf32 * 0.25f;
  const float y = sqrt(2);

  PointBaseTransformExampleType pt(
      PointFieldCoordinates_<float, 2>(Vector2f(x, y)),
      PointFieldDirection_<float, 2>(Vector2f(x, y)));

  PointBaseTransformExampleType pt_euclidean = pt.polarToEuclidean();

  EXPECT_NEAR(pt_euclidean.value<0>().x(), 1.f, 1e-5);
  EXPECT_NEAR(pt_euclidean.value<0>().y(), 1.f, 1e-5);
  EXPECT_FLOAT_EQ(pt_euclidean.value<1>().x(), x);
  EXPECT_FLOAT_EQ(pt_euclidean.value<1>().y(), y);
}  // TEST(pcl_point_derived, polarToEuclidean)

TEST(pcl_point_derived, toStream)
{
  PointBaseExampleType pt1(
      PointFieldCoordinates_<float, 3>(Vector3f(4.f, -1.f, 10.f)),
      PointFieldDirection_<float, 3>(Vector3f(-7.f, -8.f, -9.f)));

  PointBaseTransformExampleType pt2(
      PointFieldCoordinates_<float, 2>(Vector2f(1.f, 1.f)),
      PointFieldDirection_<float, 2>(Vector2f(1.f, 1.f)));

  // pt1.toStream(std::cout) << std::endl;
  // pt2.toStream(std::cout) << std::endl;
}  // TEST(pcl_point_derived, toStream)

}  // namespace testing
