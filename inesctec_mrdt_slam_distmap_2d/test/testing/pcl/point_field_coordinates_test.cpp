#include <gtest/gtest.h>

#include <Eigen/Core>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/pcl/point_field_coordinates.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace pcl;

using Point2f = PointFieldCoordinates_<float, 2>;
using Point2fTraits = Point2f::TraitsType;

using Point3f = PointFieldCoordinates_<float, 3>;
using Point3fTraits = Point3f::TraitsType;

TEST(pcl_point_field_coordinates, traits)
{
  EXPECT_EQ(Point2f::Dim, 2);
  EXPECT_EQ(Point2f::TraitsType::Dim, 2);
}  // TEST(pcl_point_field_coordinates, traits)

TEST(pcl_point_field_coordinates, setZero)
{
  Point2f pt1, pt2, pt3;

  Point2fTraits::setZero(pt1.m_value);
  Point2fTraits::setZero(pt2.m_value);
  Point2fTraits::setZero(pt3.m_value);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), 0.f);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), 0.f);
  EXPECT_FLOAT_EQ(pt2.m_value.x(), 0.f);
  EXPECT_FLOAT_EQ(pt2.m_value.y(), 0.f);
  EXPECT_FLOAT_EQ(pt3.m_value.x(), 0.f);
  EXPECT_FLOAT_EQ(pt3.m_value.y(), 0.f);
}  // TEST(pcl_point_field_coordinates, setZero)

TEST(pcl_point_field_coordinates, copyTo)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;

  Point2f pt1, pt2;

  Point2fTraits::setZero(pt1.m_value);
  Point2fTraits::setZero(pt2.m_value);

  pt2.m_value.x() = x;
  pt2.m_value.y() = y;

  Point2fTraits::copyTo(pt1.m_value, pt2.m_value);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y);
}  // TEST(pcl_point_field_coordinates, copyTo)

TEST(pcl_point_field_coordinates, add)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;
  constexpr float x2 = 1.f;
  constexpr float y2 = -7.f;

  Point2f pt1, pt2, pt3;

  Point2fTraits::setZero(pt1.m_value);
  Point2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;
  pt2.m_value.x() = x2;
  pt2.m_value.y() = y2;

  Point2fTraits::add(pt3.m_value, pt1.m_value, pt2.m_value);

  EXPECT_FLOAT_EQ(pt3.m_value.x(), x1 + x2);
  EXPECT_FLOAT_EQ(pt3.m_value.y(), y1 + y2);
}  // TEST(pcl_point_field_coordinates, add)

TEST(pcl_point_field_coordinates, addInPlace)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;
  constexpr float x2 = 1.f;
  constexpr float y2 = -7.f;

  Point2f pt1, pt2;

  Point2fTraits::setZero(pt1.m_value);
  Point2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;
  pt2.m_value.x() = x2;
  pt2.m_value.y() = y2;

  Point2fTraits::addInPlace(pt1.m_value, pt2.m_value);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1 + x2);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1 + y2);
}  // TEST(pcl_point_field_coordinates, addInPlace)

TEST(pcl_point_field_coordinates, scale)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;
  constexpr float s = 0.8f;

  Point2f pt1, pt2;

  Point2fTraits::setZero(pt1.m_value);
  Point2fTraits::setZero(pt2.m_value);

  pt2.m_value.x() = x;
  pt2.m_value.y() = y;

  Point2fTraits::scale(pt1.m_value, pt2.m_value, s);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x * s);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y * s);
}  // TEST(pcl_point_field_coordinates, scale)

TEST(pcl_point_field_coordinates, scaleInPlace)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;
  constexpr float s = 0.8f;

  Point2f pt1;

  Point2fTraits::setZero(pt1.m_value);

  pt1.m_value.x() = x;
  pt1.m_value.y() = y;

  Point2fTraits::scaleInPlace(pt1.m_value, s);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x * s);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y * s);
}  // TEST(pcl_point_field_coordinates, scaleInPlace)

TEST(pcl_point_field_coordinates, addAndScaleInPlace)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;
  constexpr float x2 = 1.f;
  constexpr float y2 = -7.f;
  constexpr float s2 = 0.8f;

  Point2f pt1, pt2;

  Point2fTraits::setZero(pt1.m_value);
  Point2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;
  pt2.m_value.x() = x2;
  pt2.m_value.y() = y2;

  Point2fTraits::addAndScaleInPlace(pt1.m_value, pt2.m_value, s2);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1 + x2 * s2);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1 + y2 * s2);
}  // TEST(pcl_point_field_coordinates, addAndScaleInPlace)

TEST(pcl_point_field_coordinates, transform)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;

  Eigen::Isometry2f transformation = Eigen::Isometry2f::Identity();

  Point2f pt1, pt2;

  Point2fTraits::setZero(pt1.m_value);
  Point2fTraits::setZero(pt2.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Point2fTraits::transform(pt2.m_value, pt1.m_value, transformation);

  EXPECT_FLOAT_EQ(pt2.m_value.x(), pt1.m_value.x());
  EXPECT_FLOAT_EQ(pt2.m_value.y(), pt1.m_value.y());

  Point2fTraits::transform(pt2.m_value, pt1.m_value,
                           geometry::v2t(Eigen::Vector3f(x1, y1, 0)));

  EXPECT_FLOAT_EQ(pt2.m_value.x(), pt1.m_value.x() + x1);
  EXPECT_FLOAT_EQ(pt2.m_value.y(), pt1.m_value.y() + y1);

  Point2fTraits::transform(
      pt2.m_value, pt1.m_value,
      geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.m_value.x(), -pt1.m_value.y(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), pt1.m_value.x(), 1e-5);

  Point2fTraits::transform(
      pt2.m_value, pt1.m_value,
      geometry::v2t(Eigen::Vector3f(0, 0, -M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt2.m_value.x(), pt1.m_value.y(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -pt1.m_value.x(), 1e-5);

  Point2fTraits::transform(pt2.m_value, pt1.m_value,
                           geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32)));

  EXPECT_NEAR(pt2.m_value.x(), -pt1.m_value.x(), 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -pt1.m_value.y(), 1e-5);
}  // TEST(pcl_point_field_coordinates, transform)

TEST(pcl_point_field_coordinates, transformInPlace)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;

  Eigen::Isometry2f transformation = Eigen::Isometry2f::Identity();

  Point2f pt1;

  Point2fTraits::setZero(pt1.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Point2fTraits::transformInPlace(pt1.m_value, transformation);

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1);

  Point2fTraits::transformInPlace(pt1.m_value,
                                  geometry::v2t(Eigen::Vector3f(x1, y1, 0)));

  EXPECT_FLOAT_EQ(pt1.m_value.x(), x1 + x1);
  EXPECT_FLOAT_EQ(pt1.m_value.y(), y1 + y1);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Point2fTraits::transformInPlace(
      pt1.m_value, geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt1.m_value.x(), -y1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), x1, 1e-5);

  Point2fTraits::transformInPlace(
      pt1.m_value, geometry::v2t(Eigen::Vector3f(0, 0, -M_PIf32 * 0.5f)));

  EXPECT_NEAR(pt1.m_value.x(), x1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), y1, 1e-5);

  Point2fTraits::transformInPlace(
      pt1.m_value, geometry::v2t(Eigen::Vector3f(0, 0, M_PIf32)));

  EXPECT_NEAR(pt1.m_value.x(), -x1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -y1, 1e-5);
}  // TEST(pcl_point_field_coordinates, transformInPlace)

TEST(pcl_point_field_coordinates, normalize)
{
  constexpr float x1 = 5.f;
  constexpr float y1 = 3.f;

  float norm1 = sqrt(x1 * x1 + y1 * y1);

  Point2f pt1;

  Point2fTraits::setZero(pt1.m_value);

  pt1.m_value.x() = x1;
  pt1.m_value.y() = y1;

  Point2fTraits::normalize(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), x1 / norm1, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), y1 / norm1, 1e-5);
}  // TEST(pcl_point_field_coordinates, normalize)

TEST(pcl_point_field_coordinates, euclideanToPolar)
{
  Point2f pt1;
  Point2f pt2;

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = 1.f;

  Point2fTraits::euclideanToPolar(pt2.m_value, pt1.m_value);

  EXPECT_NEAR(pt2.m_value.x(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), sqrt(2), 1e-5);
}  // TEST(pcl_point_field_coordinates, euclideanToPolar)

TEST(pcl_point_field_coordinates, euclideanToPolarInPlace)
{
  Point2f pt1;

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = 1.f;

  Point2fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), sqrt(2), 1e-5);

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = -1.f;

  Point2fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), sqrt(2), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = 1.f;

  Point2fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), sqrt(2), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = -1.f;

  Point2fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), sqrt(2), 1e-5);

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = 0.f;

  Point2fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = 0.f;

  Point2fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);
}  // TEST(pcl_point_field_coordinates, euclideanToPolarInPlace)

TEST(pcl_point_field_coordinates, polarToEuclidean)
{
  Point2f pt1;
  Point2f pt2;

  pt1.m_value.x() = M_PIf32 * 0.25f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt2.m_value, pt1.m_value);

  EXPECT_NEAR(pt2.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), 1.f, 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.25f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt2.m_value, pt1.m_value);

  EXPECT_NEAR(pt2.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -1.f, 1e-5);

  pt1.m_value.x() = M_PIf32 * 0.75f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt2.m_value, pt1.m_value);

  EXPECT_NEAR(pt2.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), 1.f, 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.75f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt2.m_value, pt1.m_value);

  EXPECT_NEAR(pt2.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), -1.f, 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = 1.f;

  Point2fTraits::polarToEuclidean(pt2.m_value, pt1.m_value);

  EXPECT_NEAR(pt2.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), 0.f, 1e-5);

  pt1.m_value.x() = M_PIf32;
  pt1.m_value.y() = 1.f;

  Point2fTraits::polarToEuclidean(pt2.m_value, pt1.m_value);

  EXPECT_NEAR(pt2.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt2.m_value.y(), 0.f, 1e-5);
}  // TEST(pcl_point_field_coordinates, polarToEuclidean)

TEST(pcl_point_field_coordinates, polarToEuclideanInPlace)
{
  Point2f pt1;

  pt1.m_value.x() = M_PIf32 * 0.25f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.25f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);

  pt1.m_value.x() = M_PIf32 * 0.75f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.75f;
  pt1.m_value.y() = sqrt(2);

  Point2fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = 1.f;

  Point2fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);

  pt1.m_value.x() = M_PIf32;
  pt1.m_value.y() = 1.f;

  Point2fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
}  // TEST(pcl_point_field_coordinates, polarToEuclideanInPlace)

TEST(pcl_point_field_coordinates, euclideanToPolarInPlace3D)
{
  Point3f pt1;

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = 1.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = 1.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = -1.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = -1.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = 1.f;
  pt1.m_value.z() = -sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = 1.f;
  pt1.m_value.z() = -sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = -1.f;
  pt1.m_value.z() = -sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = -1.f;
  pt1.m_value.z() = -sqrt(2.f);

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(4.f), 1e-5);

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = 1.f;
  pt1.m_value.z() = 0.f;

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = 1.f;
  pt1.m_value.z() = 0.f;

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = -1.f;
  pt1.m_value.y() = -1.f;
  pt1.m_value.z() = 0.f;

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.75f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = 1.f;
  pt1.m_value.y() = -1.f;
  pt1.m_value.z() = 0.f;

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -M_PIf32 * 0.25f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = 1.f;

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), M_PIf32 * 0.5f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 1.f, 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = -1.f;

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -M_PIf32 * 0.5f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 1.f, 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = 0.f;

  Point3fTraits::euclideanToPolar(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 0.f, 1e-5);
}  // TEST(pcl_point_field_coordinates, euclideanToPolarInPlace3D)

TEST(pcl_point_field_coordinates, polarToEuclideanInPlace3D)
{
  Point3f pt1;

  pt1.m_value.x() = M_PIf32 * 0.25f;
  pt1.m_value.y() = M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = M_PIf32 * 0.75f;
  pt1.m_value.y() = M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.75f;
  pt1.m_value.y() = M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.25f;
  pt1.m_value.y() = M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), sqrt(2.f), 1e-5);

  pt1.m_value.x() = M_PIf32 * 0.25f;
  pt1.m_value.y() = -M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), -sqrt(2.f), 1e-5);

  pt1.m_value.x() = M_PIf32 * 0.75f;
  pt1.m_value.y() = -M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), -sqrt(2.f), 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.75f;
  pt1.m_value.y() = -M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), -sqrt(2.f), 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.25f;
  pt1.m_value.y() = -M_PIf32 * 0.25f;
  pt1.m_value.z() = sqrt(4.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), -sqrt(2.f), 1e-5);

  pt1.m_value.x() = M_PIf32 * 0.25f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 0.f, 1e-5);

  pt1.m_value.x() = M_PIf32 * 0.75f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 0.f, 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.75f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 0.f, 1e-5);

  pt1.m_value.x() = -M_PIf32 * 0.25f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = sqrt(2.f);

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), -1.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 0.f, 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = M_PIf32 * 0.5f;
  pt1.m_value.z() = 1.f;

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 1.f, 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = -M_PIf32 * 0.5f;
  pt1.m_value.z() = 1.f;

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), -1.f, 1e-5);

  pt1.m_value.x() = 0.f;
  pt1.m_value.y() = 0.f;
  pt1.m_value.z() = 0.f;

  Point3fTraits::polarToEuclidean(pt1.m_value);

  EXPECT_NEAR(pt1.m_value.x(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.y(), 0.f, 1e-5);
  EXPECT_NEAR(pt1.m_value.z(), 0.f, 1e-5);
}  // TEST(pcl_point_field_coordinates, polarToEuclideanInPlace3D)

TEST(pcl_point_field_coordinates, toStream)
{
  constexpr float x = 5.f;
  constexpr float y = 3.f;

  Point2f pt1;

  pt1.m_value.x() = x;
  pt1.m_value.y() = y;

  // Point2fTraits::toStream(std::cout, pt1.m_value);
  // std::cout << std::endl;
}  // TEST(pcl_point_field_coordinates, toStream)

}  // namespace testing
