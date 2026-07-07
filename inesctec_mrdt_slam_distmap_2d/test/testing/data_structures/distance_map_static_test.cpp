#include <gtest/gtest.h>
#include <srrg2_core/srrg_boss/deserializer.h>
#include <srrg2_core/srrg_boss/serializer.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_static.hpp>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/instances.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>
#include <inesctec_mrdt_slam_distmap_2d/test/point_cloud_examples.hpp>

// #define DISTANCE_MAP_TEST_OPENCV_VIZ

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_distance_map_static_generic, constructor)
{
  DistanceMapStaticFloat dist_map_std_f;
  DistanceMapStaticDouble dist_map_std_d;

  DistanceMapStaticWith2ndDerivFloat dist_map_with_d2_f;
  DistanceMapStaticWith2ndDerivDouble dist_map_with_d2_d;

  EXPECT_TRUE(dist_map_std_f.getMap());
  EXPECT_TRUE(dist_map_std_d.getMap());
  EXPECT_TRUE(dist_map_with_d2_f.getMap());
  EXPECT_TRUE(dist_map_with_d2_d.getMap());

  EXPECT_EQ(dist_map_std_f.getType(), DistanceMapImplType::kStandard);
  EXPECT_EQ(dist_map_std_d.getType(), DistanceMapImplType::kStandard);
  EXPECT_EQ(dist_map_with_d2_f.getType(), DistanceMapImplType::kWith2ndDeriv);
  EXPECT_EQ(dist_map_with_d2_d.getType(), DistanceMapImplType::kWith2ndDeriv);
}  // TEST(data_structures_distance_map_static_generic, constructor)

TEST(data_structures_distance_map_static_generic, findNeighbors)
{
  constexpr int rows = 50;
  constexpr int cols = 100;

  DistanceMapStaticFloat map_std;
  DistanceMapStaticWith2ndDerivFloat map_with_d2;

  ASSERT_NO_FATAL_FAILURE(map_std.getMap()->resize(rows, cols));
  ASSERT_NO_FATAL_FAILURE(map_with_d2.getMap()->resize(rows, cols));

  DistanceMapStaticFloat::CellType* nn_std[8];
  DistanceMapStaticWith2ndDerivFloat::CellType* nn_with_d2[8];

  EXPECT_EQ(map_std.findNeighbors(nn_std, 0, 0), 3);
  EXPECT_EQ(map_std.findNeighbors(nn_std, rows - 1, cols - 1), 3);
  EXPECT_EQ(map_std.findNeighbors(nn_std, 0, cols - 1), 3);
  EXPECT_EQ(map_std.findNeighbors(nn_std, rows - 1, 0), 3);
  EXPECT_EQ(map_std.findNeighbors(nn_std, 1, 1), 8);

  EXPECT_EQ(map_with_d2.findNeighbors(nn_with_d2, 0, 0), 3);
  EXPECT_EQ(map_with_d2.findNeighbors(nn_with_d2, rows - 1, cols - 1), 3);
  EXPECT_EQ(map_with_d2.findNeighbors(nn_with_d2, 0, cols - 1), 3);
  EXPECT_EQ(map_with_d2.findNeighbors(nn_with_d2, rows - 1, 0), 3);
  EXPECT_EQ(map_with_d2.findNeighbors(nn_with_d2, 1, 1), 8);
}  // TEST(data_structures_distance_map_static_generic, findNeighbors)

TEST(data_structures_distance_map_static_standard, setGoalsSimpleParam)
{
  using DistanceMapType = DistanceMapStaticFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;
  using Scalar = DistanceMapType::Scalar;

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  pc->clear();
  pc->emplace_back(PointType(PointVectorType(0.f, 0.f)));

  ASSERT_EQ(pc->size(), 1);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), 0.f);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), 0.f);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(10.0);
  Scalar max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_FLOAT_EQ(map.getRes(), res);
  EXPECT_FLOAT_EQ(map.getMapSize(), map_size);
  EXPECT_FLOAT_EQ(map.getMaxDist(), max_dist);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size));

  EXPECT_FLOAT_EQ(map.getRes(), res);
  EXPECT_FLOAT_EQ(map.getMapSize(), map_size);
  EXPECT_FLOAT_EQ(map.getMaxDist(), -1.f);

  EXPECT_NO_THROW(map.setGoals(*pc, res));

  EXPECT_FLOAT_EQ(map.getRes(), res);
  EXPECT_FLOAT_EQ(map.getMapSize(), -1.f);
  EXPECT_FLOAT_EQ(map.getMaxDist(), -1.f);
}  // TEST(data_structures_distance_map_static_standard, setGoalsSimpleParam)

TEST(data_structures_distance_map_static_with_2nd_deriv, setGoalsSimpleParam)
{
  using DistanceMapType = DistanceMapStaticWith2ndDerivFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;
  using Scalar = DistanceMapType::Scalar;

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  pc->clear();
  pc->emplace_back(PointType(PointVectorType(0.f, 0.f)));

  ASSERT_EQ(pc->size(), 1);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), 0.f);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), 0.f);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(10.0);
  Scalar max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_FLOAT_EQ(map.getRes(), res);
  EXPECT_FLOAT_EQ(map.getMapSize(), map_size);
  EXPECT_FLOAT_EQ(map.getMaxDist(), max_dist);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size));

  EXPECT_FLOAT_EQ(map.getRes(), res);
  EXPECT_FLOAT_EQ(map.getMapSize(), map_size);
  EXPECT_FLOAT_EQ(map.getMaxDist(), -1.f);

  EXPECT_NO_THROW(map.setGoals(*pc, res));

  EXPECT_FLOAT_EQ(map.getRes(), res);
  EXPECT_FLOAT_EQ(map.getMapSize(), -1.f);
  EXPECT_FLOAT_EQ(map.getMaxDist(), -1.f);
}  // TEST(data_structures_distance_map_static_with_2nd_deriv,
   // setGoalsSimpleParam)

TEST(data_structures_distance_map_static_standard, setGoalsSimpleSize)
{
  using DistanceMapType = DistanceMapStaticFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;
  using Scalar = DistanceMapType::Scalar;

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(5.0);
  Scalar max_dist = Scalar(1.0);

  // empty point cloud >>> expected empty distance map

  pc->clear();

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 0);
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());

  // distance map with 1 point w/ size constrain

  pc->clear();
  pc->emplace_back(PointType(PointVectorType(0.f, 0.f)));

  ASSERT_EQ(pc->size(), 1);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), 0.f);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), 0.f);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_EQ((*map.getMap()).rows(),
            1001);  // map size fixates the distance map size
  EXPECT_EQ((*map.getMap()).cols(), 1001);
  EXPECT_TRUE(
      map.getPointCloudMin().isApprox(PointVectorType(-map_size, -map_size)));

  // distance map with 1 point wo/ size constrain

  EXPECT_NO_THROW(map.setGoals(*pc, res, Scalar(-1.0), max_dist));

  EXPECT_EQ((*map.getMap()).rows(),
            201);  // given only 1 point, max distance sets the size
  EXPECT_EQ((*map.getMap()).cols(), 201);
  EXPECT_TRUE(map.getPointCloudMin().isApprox(
      PointVectorType(pc->at(0).coordinates().x() - max_dist,
                      pc->at(0).coordinates().y() - max_dist)));

  // distance map w/ invalid point

  pc->at(0).m_status = pcl::PointStatus::kOutOfRange;

  EXPECT_NO_THROW(map.setGoals(*pc, res, Scalar(-1.0), max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 0);  // given out of range point, 0 size
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());

  pc->at(0).m_status = pcl::PointStatus::kInvalid;

  EXPECT_NO_THROW(map.setGoals(*pc, res, Scalar(-1.0), max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 0);  // given out of range point, 0 size
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());

  // distance map w/ 1 point compute all

  EXPECT_NO_THROW(map.setGoals(*pc, res));

  EXPECT_EQ(
      (*map.getMap()).rows(),
      0);  // though compute all map with no restrictions, only has 1 point
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());
}  // TEST(data_structures_distance_map_static_standard, setGoalsSimpleSize)

TEST(data_structures_distance_map_static_with_2nd_deriv, setGoalsSimpleSize)
{
  using DistanceMapType = DistanceMapStaticWith2ndDerivFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;
  using Scalar = DistanceMapType::Scalar;

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(5.0);
  Scalar max_dist = Scalar(1.0);

  // empty point cloud >>> expected empty distance map

  pc->clear();

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 0);
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());

  // distance map with 1 point w/ size constrain

  pc->clear();
  pc->emplace_back(PointType(PointVectorType(0.f, 0.f)));

  ASSERT_EQ(pc->size(), 1);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), 0.f);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), 0.f);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_EQ((*map.getMap()).rows(),
            1001);  // map size fixates the distance map size
  EXPECT_EQ((*map.getMap()).cols(), 1001);
  EXPECT_TRUE(
      map.getPointCloudMin().isApprox(PointVectorType(-map_size, -map_size)));

  // distance map with 1 point wo/ size constrain

  EXPECT_NO_THROW(map.setGoals(*pc, res, Scalar(-1.0), max_dist));

  EXPECT_EQ((*map.getMap()).rows(),
            201);  // given only 1 point, max distance sets the size
  EXPECT_EQ((*map.getMap()).cols(), 201);
  EXPECT_TRUE(map.getPointCloudMin().isApprox(
      PointVectorType(pc->at(0).coordinates().x() - max_dist,
                      pc->at(0).coordinates().y() - max_dist)));

  // distance map w/ invalid point

  pc->at(0).m_status = pcl::PointStatus::kOutOfRange;

  EXPECT_NO_THROW(map.setGoals(*pc, res, Scalar(-1.0), max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 0);  // given out of range point, 0 size
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());

  pc->at(0).m_status = pcl::PointStatus::kInvalid;

  EXPECT_NO_THROW(map.setGoals(*pc, res, Scalar(-1.0), max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 0);  // given out of range point, 0 size
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());

  // distance map w/ 1 point compute all

  EXPECT_NO_THROW(map.setGoals(*pc, res));

  EXPECT_EQ(
      (*map.getMap()).rows(),
      0);  // though compute all map with no restrictions, only has 1 point
  EXPECT_EQ((*map.getMap()).cols(), 0);
  EXPECT_TRUE(map.getPointCloudMin().isZero());
}  // TEST(data_structures_distance_map_static_with_2nd_deriv,
   // setGoalsSimpleSize)

TEST(data_structures_distance_map_static_standard, setGoalsSimple)
{
  using DistanceMapType = DistanceMapStaticFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;
  using Scalar = DistanceMapType::Scalar;
  using CellType = DistanceMapType::CellType;

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  pc->clear();
  pc->emplace_back(PointType(PointVectorType(0.f, 0.f)));

  ASSERT_EQ(pc->size(), 1);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), 0.f);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), 0.f);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 201);
  EXPECT_EQ((*map.getMap()).cols(), 201);

  EXPECT_NEAR((*map.getMap())(100, 100).m_dist, 0.f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(100, 99).m_dist, .01f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(100, 101).m_dist, .01f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(99, 100).m_dist, .01f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(101, 100).m_dist, .01f, res * 1e-2);

  EXPECT_NEAR((*map.getMap())(100, 98).m_dist, .02f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(100, 102).m_dist, .02f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(98, 100).m_dist, .02f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(102, 100).m_dist, .02f, res * 1e-2);

  EXPECT_NEAR(
      (*map.getMap())(100, 98).m_dist_d(0),
      ((*map.getMap())(101, 98).m_dist - (*map.getMap())(99, 98).m_dist) * .5f /
          res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(100, 102).m_dist_d(0),
      ((*map.getMap())(101, 102).m_dist - (*map.getMap())(99, 102).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(98, 100).m_dist_d(0),
      ((*map.getMap())(99, 100).m_dist - (*map.getMap())(97, 100).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(102, 100).m_dist_d(0),
      ((*map.getMap())(103, 100).m_dist - (*map.getMap())(101, 100).m_dist) *
          .5f / res,
      res * 1e-2);

  EXPECT_NEAR(
      (*map.getMap())(100, 98).m_dist_d(1),
      ((*map.getMap())(100, 99).m_dist - (*map.getMap())(100, 97).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(100, 102).m_dist_d(1),
      ((*map.getMap())(100, 103).m_dist - (*map.getMap())(100, 101).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(98, 100).m_dist_d(1),
      ((*map.getMap())(98, 101).m_dist - (*map.getMap())(98, 99).m_dist) * .5f /
          res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(102, 100).m_dist_d(1),
      ((*map.getMap())(102, 101).m_dist - (*map.getMap())(102, 99).m_dist) *
          .5f / res,
      res * 1e-2);

  EXPECT_NEAR((*map.getMap())(0, 0).m_dist, 1.f, res * 1e-2);

  for (int r = 0; r < map.getMap()->rows(); r++)
  {
    for (int c = 0; c < map.getMap()->cols(); c++)
    {
      if ((*map.getMap())(r, c).m_nn_idx >= 0)
      {
        EXPECT_EQ((*map.getMap())(r, c).m_nn_idx, 0);  // just one point in pc
      }
    }
  }

  CellType map_cell;

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.f, 100.f));

  EXPECT_NEAR(map_cell.m_dist, 0.f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0), 0.f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1), 0.f, res * 1e-2);

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.5f, 100.f));

  EXPECT_NEAR(map_cell.m_dist, 0.005f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0), 0.005f / res, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1), 0.f, res * 1e-2);

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.f, 100.5f));

  EXPECT_NEAR(map_cell.m_dist, 0.005f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0), 0.f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1), 0.005f / res, res * 1e-2);

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.5f, 100.5f));

  EXPECT_NEAR(
      map_cell.m_dist,
      0.25f *
          ((*map.getMap())(100, 100).m_dist + (*map.getMap())(101, 100).m_dist +
           (*map.getMap())(100, 101).m_dist + (*map.getMap())(101, 101).m_dist),
      res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0),
              0.25f * ((*map.getMap())(100, 100).m_dist_d(0) +
                       (*map.getMap())(101, 100).m_dist_d(0) +
                       (*map.getMap())(100, 101).m_dist_d(0) +
                       (*map.getMap())(101, 101).m_dist_d(0)),
              res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1),
              0.25f * ((*map.getMap())(100, 100).m_dist_d(1) +
                       (*map.getMap())(101, 100).m_dist_d(1) +
                       (*map.getMap())(100, 101).m_dist_d(1) +
                       (*map.getMap())(101, 101).m_dist_d(1)),
              res * 1e-2);

  EXPECT_NO_THROW(map.setGoals(*pc, res));

  EXPECT_EQ((*map.getMap()).rows(), 1);
  EXPECT_EQ((*map.getMap()).cols(), 1);

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  RGBImage idx_img;
  RGBImage dist_img;
  RGBImage dist_deriv_x_img;
  RGBImage dist_deriv_y_img;
  RGBImage dist_deriv_2_xx_img;
  RGBImage dist_deriv_2_xy_img;
  RGBImage dist_deriv_2_yx_img;
  RGBImage dist_deriv_2_yy_img;

  res = Scalar(0.01);
  map_size = Scalar(-1.0);
  max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("std::setGoalsSimple: index");
  cv::namedWindow("std::setGoalsSimple: distance");
  cv::namedWindow("std::setGoalsSimple: deriv x");
  cv::namedWindow("std::setGoalsSimple: deriv y");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv yy");

  cv::imshow("std::setGoalsSimple: index", idx_img);
  cv::imshow("std::setGoalsSimple: distance", dist_img);
  cv::imshow("std::setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("std::setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

  res = Scalar(0.01);
  map_size = Scalar(2.0);
  max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("std::setGoalsSimple: index");
  cv::namedWindow("std::setGoalsSimple: distance");
  cv::namedWindow("std::setGoalsSimple: deriv x");
  cv::namedWindow("std::setGoalsSimple: deriv y");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv yy");

  cv::imshow("std::setGoalsSimple: index", idx_img);
  cv::imshow("std::setGoalsSimple: distance", dist_img);
  cv::imshow("std::setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("std::setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

  res = Scalar(0.01);
  map_size = Scalar(2.0);
  max_dist = Scalar(-1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("std::setGoalsSimple: index");
  cv::namedWindow("std::setGoalsSimple: distance");
  cv::namedWindow("std::setGoalsSimple: deriv x");
  cv::namedWindow("std::setGoalsSimple: deriv y");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("std::setGoalsSimple: 2nd deriv yy");

  cv::imshow("std::setGoalsSimple: index", idx_img);
  cv::imshow("std::setGoalsSimple: distance", dist_img);
  cv::imshow("std::setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("std::setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("std::setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_static_standard, setGoalsSimple)

TEST(data_structures_distance_map_static_with_2nd_deriv, setGoalsSimple)
{
  using DistanceMapType = DistanceMapStaticWith2ndDerivFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;
  using Scalar = DistanceMapType::Scalar;
  using CellType = DistanceMapType::CellType;

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  pc->clear();
  pc->emplace_back(PointType(PointVectorType(0.f, 0.f)));

  ASSERT_EQ(pc->size(), 1);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), 0.f);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), 0.f);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 201);
  EXPECT_EQ((*map.getMap()).cols(), 201);

  EXPECT_NEAR((*map.getMap())(100, 100).m_dist, 0.f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(100, 99).m_dist, .01f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(100, 101).m_dist, .01f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(99, 100).m_dist, .01f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(101, 100).m_dist, .01f, res * 1e-2);

  EXPECT_NEAR((*map.getMap())(100, 98).m_dist, .02f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(100, 102).m_dist, .02f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(98, 100).m_dist, .02f, res * 1e-2);
  EXPECT_NEAR((*map.getMap())(102, 100).m_dist, .02f, res * 1e-2);

  EXPECT_NEAR(
      (*map.getMap())(100, 98).m_dist_d(0),
      ((*map.getMap())(101, 98).m_dist - (*map.getMap())(99, 98).m_dist) * .5f /
          res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(100, 102).m_dist_d(0),
      ((*map.getMap())(101, 102).m_dist - (*map.getMap())(99, 102).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(98, 100).m_dist_d(0),
      ((*map.getMap())(99, 100).m_dist - (*map.getMap())(97, 100).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(102, 100).m_dist_d(0),
      ((*map.getMap())(103, 100).m_dist - (*map.getMap())(101, 100).m_dist) *
          .5f / res,
      res * 1e-2);

  EXPECT_NEAR(
      (*map.getMap())(100, 98).m_dist_d(1),
      ((*map.getMap())(100, 99).m_dist - (*map.getMap())(100, 97).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(100, 102).m_dist_d(1),
      ((*map.getMap())(100, 103).m_dist - (*map.getMap())(100, 101).m_dist) *
          .5f / res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(98, 100).m_dist_d(1),
      ((*map.getMap())(98, 101).m_dist - (*map.getMap())(98, 99).m_dist) * .5f /
          res,
      res * 1e-2);
  EXPECT_NEAR(
      (*map.getMap())(102, 100).m_dist_d(1),
      ((*map.getMap())(102, 101).m_dist - (*map.getMap())(102, 99).m_dist) *
          .5f / res,
      res * 1e-2);

  EXPECT_NEAR((*map.getMap())(0, 0).m_dist, 1.f, res * 1e-2);

  for (int r = 0; r < map.getMap()->rows(); r++)
  {
    for (int c = 0; c < map.getMap()->cols(); c++)
    {
      if ((*map.getMap())(r, c).m_nn_idx >= 0)
      {
        EXPECT_EQ((*map.getMap())(r, c).m_nn_idx, 0);  // just one point in pc
      }
    }
  }

  CellType map_cell;

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.f, 100.f));

  EXPECT_NEAR(map_cell.m_dist, 0.f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0), 0.f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1), 0.f, res * 1e-2);

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.5f, 100.f));

  EXPECT_NEAR(map_cell.m_dist, 0.005f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0), 0.005f / res, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1), 0.f, res * 1e-2);

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.f, 100.5f));

  EXPECT_NEAR(map_cell.m_dist, 0.005f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0), 0.f, res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1), 0.005f / res, res * 1e-2);

  (*map.getMap()).getSubPxValue(map_cell, PointVectorType(100.5f, 100.5f));

  EXPECT_NEAR(
      map_cell.m_dist,
      0.25f *
          ((*map.getMap())(100, 100).m_dist + (*map.getMap())(101, 100).m_dist +
           (*map.getMap())(100, 101).m_dist + (*map.getMap())(101, 101).m_dist),
      res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(0),
              0.25f * ((*map.getMap())(100, 100).m_dist_d(0) +
                       (*map.getMap())(101, 100).m_dist_d(0) +
                       (*map.getMap())(100, 101).m_dist_d(0) +
                       (*map.getMap())(101, 101).m_dist_d(0)),
              res * 1e-2);
  EXPECT_NEAR(map_cell.m_dist_d(1),
              0.25f * ((*map.getMap())(100, 100).m_dist_d(1) +
                       (*map.getMap())(101, 100).m_dist_d(1) +
                       (*map.getMap())(100, 101).m_dist_d(1) +
                       (*map.getMap())(101, 101).m_dist_d(1)),
              res * 1e-2);

  EXPECT_NO_THROW(map.setGoals(*pc, res));

  EXPECT_EQ((*map.getMap()).rows(), 1);
  EXPECT_EQ((*map.getMap()).cols(), 1);

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  RGBImage idx_img;
  RGBImage dist_img;
  RGBImage dist_deriv_x_img;
  RGBImage dist_deriv_y_img;
  RGBImage dist_deriv_2_xx_img;
  RGBImage dist_deriv_2_xy_img;
  RGBImage dist_deriv_2_yx_img;
  RGBImage dist_deriv_2_yy_img;

  res = Scalar(0.01);
  map_size = Scalar(-1.0);
  max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("2nd::setGoalsSimple: index");
  cv::namedWindow("2nd::setGoalsSimple: distance");
  cv::namedWindow("2nd::setGoalsSimple: deriv x");
  cv::namedWindow("2nd::setGoalsSimple: deriv y");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv yy");

  cv::imshow("2nd::setGoalsSimple: index", idx_img);
  cv::imshow("2nd::setGoalsSimple: distance", dist_img);
  cv::imshow("2nd::setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("2nd::setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

  res = Scalar(0.01);
  map_size = Scalar(2.0);
  max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("2nd::setGoalsSimple: index");
  cv::namedWindow("2nd::setGoalsSimple: distance");
  cv::namedWindow("2nd::setGoalsSimple: deriv x");
  cv::namedWindow("2nd::setGoalsSimple: deriv y");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv yy");

  cv::imshow("2nd::setGoalsSimple: index", idx_img);
  cv::imshow("2nd::setGoalsSimple: distance", dist_img);
  cv::imshow("2nd::setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("2nd::setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

  res = Scalar(0.01);
  map_size = Scalar(2.0);
  max_dist = Scalar(-1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("2nd::setGoalsSimple: index");
  cv::namedWindow("2nd::setGoalsSimple: distance");
  cv::namedWindow("2nd::setGoalsSimple: deriv x");
  cv::namedWindow("2nd::setGoalsSimple: deriv y");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("2nd::setGoalsSimple: 2nd deriv yy");

  cv::imshow("2nd::setGoalsSimple: index", idx_img);
  cv::imshow("2nd::setGoalsSimple: distance", dist_img);
  cv::imshow("2nd::setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("2nd::setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("2nd::setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_static_with_2nd_deriv, setGoalsSimple)

TEST(data_structures_distance_map_static_standard, getValueWith2ndDeriv)
{
  using Scalar = float;

  using DistanceMapGTType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudGTType = pcl::Point2VectorCloud_<DistanceMapGTType::Scalar>;
  using PointCloudGTTypePtr = std::shared_ptr<PointCloudGTType>;
  using PointGTType = PointCloudGTType::PointType;
  using PointGTVectorType = DistanceMapGTType::PointVectorType;

  using DistanceMapType = DistanceMapStatic_<DistanceMapCell_<Scalar>>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;

  using CellType = DistanceMapWith2ndDerivCell_<Scalar>;

  constexpr Scalar pt1_x = 0.5f;
  constexpr Scalar pt1_y = 0.f;
  constexpr Scalar pt2_x = -0.5f;
  constexpr Scalar pt2_y = 0.f;

  DistanceMapType map;
  DistanceMapGTType map_gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudGTTypePtr pc_gt = std::make_shared<PointCloudGTType>();

  pc->clear();
  pc_gt->clear();

  pc->emplace_back(PointType(PointVectorType(pt1_x, pt1_y)));
  pc->emplace_back(PointType(PointVectorType(pt2_x, pt2_y)));

  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt1_x, pt1_y)));
  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt2_x, pt2_y)));

  ASSERT_EQ(pc->size(), 2);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().y(), pt2_y);

  ASSERT_EQ(pc_gt->size(), 2);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().y(), pt2_y);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  ASSERT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));
  ASSERT_NO_THROW(map_gt.setGoals(*pc_gt, res, map_size, max_dist));

  ASSERT_EQ((*map.getMap()).rows(), (*map_gt.getMap()).rows());
  ASSERT_EQ((*map.getMap()).cols(), (*map_gt.getMap()).cols());

  for (Scalar pt_x = map.getPointCloudMin().x();
       pt_x <= map.getMap()->rows() * map.getRes(); pt_x += map.getRes())
  {
    for (Scalar pt_y = map.getPointCloudMin().y();
         pt_y <= map.getMap()->cols() * map.getRes(); pt_y += map.getRes())
    {
      CellType cell;
      CellType cell_gt;

      EXPECT_EQ(
          map.getValueWith2ndDeriv(cell, PointVectorType(pt_x, pt_y), *pc),
          map_gt.getValueWith2ndDeriv(cell_gt, PointGTVectorType(pt_x, pt_y),
                                      *pc_gt));

      if (!map_gt.getValueWith2ndDeriv(cell_gt, PointGTVectorType(pt_x, pt_y),
                                       *pc_gt))
      {
        continue;
      }

      EXPECT_FLOAT_EQ(cell.m_dist, cell_gt.m_dist);
      EXPECT_FLOAT_EQ(cell.m_dist_d(0), cell_gt.m_dist_d(0));
      EXPECT_FLOAT_EQ(cell.m_dist_d(1), cell_gt.m_dist_d(1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 0), cell_gt.m_dist_d2(0, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 1), cell_gt.m_dist_d2(0, 1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 0), cell_gt.m_dist_d2(1, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 1), cell_gt.m_dist_d2(1, 1));
    }
  }

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  RGBImage idx_img;
  RGBImage dist_img;
  RGBImage dist_deriv_x_img;
  RGBImage dist_deriv_y_img;
  RGBImage dist_deriv_2_xx_img;
  RGBImage dist_deriv_2_xy_img;
  RGBImage dist_deriv_2_yx_img;
  RGBImage dist_deriv_2_yy_img;

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("std::getValueWith2ndDeriv: index");
  cv::namedWindow("std::getValueWith2ndDeriv: distance");
  cv::namedWindow("std::getValueWith2ndDeriv: deriv x");
  cv::namedWindow("std::getValueWith2ndDeriv: deriv y");
  cv::namedWindow("std::getValueWith2ndDeriv: 2nd deriv xx");
  cv::namedWindow("std::getValueWith2ndDeriv: 2nd deriv xy");
  cv::namedWindow("std::getValueWith2ndDeriv: 2nd deriv yx");
  cv::namedWindow("std::getValueWith2ndDeriv: 2nd deriv yy");

  cv::imshow("std::getValueWith2ndDeriv: index", idx_img);
  cv::imshow("std::getValueWith2ndDeriv: distance", dist_img);
  cv::imshow("std::getValueWith2ndDeriv: deriv x", dist_deriv_x_img);
  cv::imshow("std::getValueWith2ndDeriv: deriv y", dist_deriv_y_img);
  cv::imshow("std::getValueWith2ndDeriv: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("std::getValueWith2ndDeriv: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("std::getValueWith2ndDeriv: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("std::getValueWith2ndDeriv: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_static_standard, getValueWith2ndDeriv)

TEST(data_structures_distance_map_static_with_2nd_deriv, getValueWith2ndDeriv)
{
  using Scalar = float;

  using DistanceMapGTType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudGTType = pcl::Point2VectorCloud_<DistanceMapGTType::Scalar>;
  using PointCloudGTTypePtr = std::shared_ptr<PointCloudGTType>;
  using PointGTType = PointCloudGTType::PointType;
  using PointGTVectorType = DistanceMapGTType::PointVectorType;

  using DistanceMapType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;

  using CellType = DistanceMapWith2ndDerivCell_<Scalar>;

  constexpr Scalar pt1_x = 0.5f;
  constexpr Scalar pt1_y = 0.f;
  constexpr Scalar pt2_x = -0.5f;
  constexpr Scalar pt2_y = 0.f;

  DistanceMapType map;
  DistanceMapGTType map_gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudGTTypePtr pc_gt = std::make_shared<PointCloudGTType>();

  pc->clear();
  pc_gt->clear();

  pc->emplace_back(PointType(PointVectorType(pt1_x, pt1_y)));
  pc->emplace_back(PointType(PointVectorType(pt2_x, pt2_y)));

  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt1_x, pt1_y)));
  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt2_x, pt2_y)));

  ASSERT_EQ(pc->size(), 2);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().y(), pt2_y);

  ASSERT_EQ(pc_gt->size(), 2);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().y(), pt2_y);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  ASSERT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));
  ASSERT_NO_THROW(map_gt.setGoals(*pc_gt, res, map_size, max_dist));

  ASSERT_EQ((*map.getMap()).rows(), (*map_gt.getMap()).rows());
  ASSERT_EQ((*map.getMap()).cols(), (*map_gt.getMap()).cols());

  for (Scalar pt_x = map.getPointCloudMin().x();
       pt_x <= map.getMap()->rows() * map.getRes(); pt_x += map.getRes())
  {
    for (Scalar pt_y = map.getPointCloudMin().y();
         pt_y <= map.getMap()->cols() * map.getRes(); pt_y += map.getRes())
    {
      CellType cell;
      CellType cell_gt;

      EXPECT_EQ(
          map.getValueWith2ndDeriv(cell, PointVectorType(pt_x, pt_y), *pc),
          map_gt.getValueWith2ndDeriv(cell_gt, PointGTVectorType(pt_x, pt_y),
                                      *pc_gt));

      if (!map_gt.getValueWith2ndDeriv(cell_gt, PointGTVectorType(pt_x, pt_y),
                                       *pc_gt))
      {
        continue;
      }

      EXPECT_FLOAT_EQ(cell.m_dist, cell_gt.m_dist);
      EXPECT_FLOAT_EQ(cell.m_dist_d(0), cell_gt.m_dist_d(0));
      EXPECT_FLOAT_EQ(cell.m_dist_d(1), cell_gt.m_dist_d(1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 0), cell_gt.m_dist_d2(0, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 1), cell_gt.m_dist_d2(0, 1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 0), cell_gt.m_dist_d2(1, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 1), cell_gt.m_dist_d2(1, 1));
    }
  }

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  RGBImage idx_img;
  RGBImage dist_img;
  RGBImage dist_deriv_x_img;
  RGBImage dist_deriv_y_img;
  RGBImage dist_deriv_2_xx_img;
  RGBImage dist_deriv_2_xy_img;
  RGBImage dist_deriv_2_yx_img;
  RGBImage dist_deriv_2_yy_img;

  map.toImageIdx(idx_img);
  map.toImageDist(dist_img, *pc);
  map.toImageDistDx(dist_deriv_x_img, *pc);
  map.toImageDistDy(dist_deriv_y_img, *pc);
  map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc);
  map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc);
  map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc);
  map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc);

  map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0));
  map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0));

  cv::namedWindow("2nd::getValueWith2ndDeriv: index");
  cv::namedWindow("2nd::getValueWith2ndDeriv: distance");
  cv::namedWindow("2nd::getValueWith2ndDeriv: deriv x");
  cv::namedWindow("2nd::getValueWith2ndDeriv: deriv y");
  cv::namedWindow("2nd::getValueWith2ndDeriv: 2nd deriv xx");
  cv::namedWindow("2nd::getValueWith2ndDeriv: 2nd deriv xy");
  cv::namedWindow("2nd::getValueWith2ndDeriv: 2nd deriv yx");
  cv::namedWindow("2nd::getValueWith2ndDeriv: 2nd deriv yy");

  cv::imshow("2nd::getValueWith2ndDeriv: index", idx_img);
  cv::imshow("2nd::getValueWith2ndDeriv: distance", dist_img);
  cv::imshow("2nd::getValueWith2ndDeriv: deriv x", dist_deriv_x_img);
  cv::imshow("2nd::getValueWith2ndDeriv: deriv y", dist_deriv_y_img);
  cv::imshow("2nd::getValueWith2ndDeriv: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("2nd::getValueWith2ndDeriv: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("2nd::getValueWith2ndDeriv: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("2nd::getValueWith2ndDeriv: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_static_with_2nd_deriv,
   // getValueWith2ndDeriv)

TEST(data_structures_distance_map_static_standard, getValueSubPxWith2ndDeriv)
{
  using Scalar = float;

  using DistanceMapGTType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudGTType = pcl::Point2VectorCloud_<DistanceMapGTType::Scalar>;
  using PointCloudGTTypePtr = std::shared_ptr<PointCloudGTType>;
  using PointGTType = PointCloudGTType::PointType;
  using PointGTVectorType = DistanceMapGTType::PointVectorType;

  using DistanceMapType = DistanceMapStatic_<DistanceMapCell_<Scalar>>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;

  using CellType = DistanceMapWith2ndDerivCell_<Scalar>;

  constexpr Scalar pt1_x = 0.5f;
  constexpr Scalar pt1_y = 0.f;
  constexpr Scalar pt2_x = -0.5f;
  constexpr Scalar pt2_y = 0.f;

  DistanceMapType map;
  DistanceMapGTType map_gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudGTTypePtr pc_gt = std::make_shared<PointCloudGTType>();

  pc->clear();
  pc_gt->clear();

  pc->emplace_back(PointType(PointVectorType(pt1_x, pt1_y)));
  pc->emplace_back(PointType(PointVectorType(pt2_x, pt2_y)));

  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt1_x, pt1_y)));
  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt2_x, pt2_y)));

  ASSERT_EQ(pc->size(), 2);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().y(), pt2_y);

  ASSERT_EQ(pc_gt->size(), 2);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().y(), pt2_y);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  ASSERT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));
  ASSERT_NO_THROW(map_gt.setGoals(*pc_gt, res, map_size, max_dist));

  ASSERT_EQ((*map.getMap()).rows(), (*map_gt.getMap()).rows());
  ASSERT_EQ((*map.getMap()).cols(), (*map_gt.getMap()).cols());

  for (Scalar pt_x = map.getPointCloudMin().x();
       pt_x <= map.getMap()->rows() * map.getRes() * 0.25f;
       pt_x += map.getRes())
  {
    for (Scalar pt_y = map.getPointCloudMin().y();
         pt_y <= map.getMap()->cols() * map.getRes() * 0.25f;
         pt_y += map.getRes())
    {
      CellType cell;
      CellType cell_gt;

      EXPECT_EQ(
          map.getValueSubPxWith2ndDeriv(cell, PointVectorType(pt_x, pt_y), *pc),
          map_gt.getValueSubPxWith2ndDeriv(
              cell_gt, PointGTVectorType(pt_x, pt_y), *pc_gt));

      if (!map_gt.getValueSubPx(cell_gt, PointGTVectorType(pt_x, pt_y), *pc_gt))
      {
        continue;
      }

      EXPECT_FLOAT_EQ(cell.m_dist, cell_gt.m_dist);
      EXPECT_FLOAT_EQ(cell.m_dist_d(0), cell_gt.m_dist_d(0));
      EXPECT_FLOAT_EQ(cell.m_dist_d(1), cell_gt.m_dist_d(1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 0), cell_gt.m_dist_d2(0, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 1), cell_gt.m_dist_d2(0, 1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 0), cell_gt.m_dist_d2(1, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 1), cell_gt.m_dist_d2(1, 1));
    }
  }
}  // TEST(data_structures_distance_map_static_standard,
   // getValueSubPxWith2ndDeriv)

TEST(data_structures_distance_map_static_with_2nd_deriv,
     getValueSubPxWith2ndDeriv)
{
  using Scalar = float;

  using DistanceMapGTType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudGTType = pcl::Point2VectorCloud_<DistanceMapGTType::Scalar>;
  using PointCloudGTTypePtr = std::shared_ptr<PointCloudGTType>;
  using PointGTType = PointCloudGTType::PointType;
  using PointGTVectorType = DistanceMapGTType::PointVectorType;

  using DistanceMapType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;

  using CellType = DistanceMapWith2ndDerivCell_<Scalar>;

  constexpr Scalar pt1_x = 0.5f;
  constexpr Scalar pt1_y = 0.f;
  constexpr Scalar pt2_x = -0.5f;
  constexpr Scalar pt2_y = 0.f;

  DistanceMapType map;
  DistanceMapGTType map_gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudGTTypePtr pc_gt = std::make_shared<PointCloudGTType>();

  pc->clear();
  pc_gt->clear();

  pc->emplace_back(PointType(PointVectorType(pt1_x, pt1_y)));
  pc->emplace_back(PointType(PointVectorType(pt2_x, pt2_y)));

  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt1_x, pt1_y)));
  pc_gt->emplace_back(PointGTType(PointGTVectorType(pt2_x, pt2_y)));

  ASSERT_EQ(pc->size(), 2);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc->at(1).coordinates().y(), pt2_y);

  ASSERT_EQ(pc_gt->size(), 2);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().x(), pt1_x);
  ASSERT_FLOAT_EQ(pc_gt->at(0).coordinates().y(), pt1_y);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().x(), pt2_x);
  ASSERT_FLOAT_EQ(pc_gt->at(1).coordinates().y(), pt2_y);

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  ASSERT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));
  ASSERT_NO_THROW(map_gt.setGoals(*pc_gt, res, map_size, max_dist));

  ASSERT_EQ((*map.getMap()).rows(), (*map_gt.getMap()).rows());
  ASSERT_EQ((*map.getMap()).cols(), (*map_gt.getMap()).cols());

  for (Scalar pt_x = map.getPointCloudMin().x();
       pt_x <= map.getMap()->rows() * map.getRes() * 0.25f;
       pt_x += map.getRes())
  {
    for (Scalar pt_y = map.getPointCloudMin().y();
         pt_y <= map.getMap()->cols() * map.getRes() * 0.25f;
         pt_y += map.getRes())
    {
      CellType cell;
      CellType cell_gt;

      EXPECT_EQ(
          map.getValueSubPxWith2ndDeriv(cell, PointVectorType(pt_x, pt_y), *pc),
          map_gt.getValueSubPxWith2ndDeriv(
              cell_gt, PointGTVectorType(pt_x, pt_y), *pc_gt));

      if (!map_gt.getValueSubPxWith2ndDeriv(
              cell_gt, PointGTVectorType(pt_x, pt_y), *pc_gt))
      {
        continue;
      }

      EXPECT_FLOAT_EQ(cell.m_dist, cell_gt.m_dist);
      EXPECT_FLOAT_EQ(cell.m_dist_d(0), cell_gt.m_dist_d(0));
      EXPECT_FLOAT_EQ(cell.m_dist_d(1), cell_gt.m_dist_d(1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 0), cell_gt.m_dist_d2(0, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(0, 1), cell_gt.m_dist_d2(0, 1));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 0), cell_gt.m_dist_d2(1, 0));
      EXPECT_FLOAT_EQ(cell.m_dist_d2(1, 1), cell_gt.m_dist_d2(1, 1));
    }
  }
}  // TEST(data_structures_distance_map_static_with_2nd_deriv,
   // getValueSubPxWith2ndDeriv)

TEST(data_structures_distance_map_static_standard, setGoalsRandom)
{
  using Scalar = float;

  using PointCloudType = pcl::PointNormal2VectorCloud_<Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

  using DistanceMapType = DistanceMapStatic_<DistanceMapCell_<Scalar>>;

  std::random_device rd;
  std::mt19937 rng(rd());

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.05);
  Scalar map_size = Scalar(-1.0);
  Scalar max_range = Scalar(10.0);
  Scalar max_dist = Scalar(2.5);

  Scalar ang_res = geometry::degToRad(static_cast<Scalar>(1.0));
  Scalar fov = geometry::degToRad(static_cast<Scalar>(360.0));

  test::getLaser2DPointCloudRandom(*pc, ang_res, fov, max_range, rng);

  ASSERT_EQ(pc->size(), 360);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  RGBImage idx_img;
  RGBImage dist_img;
  RGBImage dist_deriv_x_img;
  RGBImage dist_deriv_y_img;
  RGBImage dist_deriv_2_xx_img;
  RGBImage dist_deriv_2_xy_img;
  RGBImage dist_deriv_2_yx_img;
  RGBImage dist_deriv_2_yy_img;

  EXPECT_NO_THROW(map.toImageIdx(idx_img));
  EXPECT_NO_THROW(map.toImageDist(dist_img, *pc));
  EXPECT_NO_THROW(map.toImageDistDx(dist_deriv_x_img, *pc));
  EXPECT_NO_THROW(map.toImageDistDy(dist_deriv_y_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc));

  EXPECT_NO_THROW(map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0)));

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  cv::namedWindow("std::setGoalsRandom: index");
  cv::namedWindow("std::setGoalsRandom: distance");
  cv::namedWindow("std::setGoalsRandom: deriv x");
  cv::namedWindow("std::setGoalsRandom: deriv y");
  cv::namedWindow("std::setGoalsRandom: 2nd deriv xx");
  cv::namedWindow("std::setGoalsRandom: 2nd deriv xy");
  cv::namedWindow("std::setGoalsRandom: 2nd deriv yx");
  cv::namedWindow("std::setGoalsRandom: 2nd deriv yy");

  cv::imshow("std::setGoalsRandom: index", idx_img);
  cv::imshow("std::setGoalsRandom: distance", dist_img);
  cv::imshow("std::setGoalsRandom: deriv x", dist_deriv_x_img);
  cv::imshow("std::setGoalsRandom: deriv y", dist_deriv_y_img);
  cv::imshow("std::setGoalsRandom: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("std::setGoalsRandom: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("std::setGoalsRandom: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("std::setGoalsRandom: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_static_standard, setGoalsRandom)

TEST(data_structures_distance_map_static_with_2nd_deriv, setGoalsRandom)
{
  using Scalar = float;

  using PointCloudType = pcl::PointNormal2VectorCloud_<Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

  using DistanceMapType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;

  std::random_device rd;
  std::mt19937 rng(rd());

  DistanceMapType map;
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.05);
  Scalar map_size = Scalar(-1.0);
  Scalar max_range = Scalar(10.0);
  Scalar max_dist = Scalar(2.5);

  Scalar ang_res = geometry::degToRad(static_cast<Scalar>(1.0));
  Scalar fov = geometry::degToRad(static_cast<Scalar>(360.0));

  test::getLaser2DPointCloudRandom(*pc, ang_res, fov, max_range, rng);

  ASSERT_EQ(pc->size(), 360);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  RGBImage idx_img;
  RGBImage dist_img;
  RGBImage dist_deriv_x_img;
  RGBImage dist_deriv_y_img;
  RGBImage dist_deriv_2_xx_img;
  RGBImage dist_deriv_2_xy_img;
  RGBImage dist_deriv_2_yx_img;
  RGBImage dist_deriv_2_yy_img;

  EXPECT_NO_THROW(map.toImageIdx(idx_img));
  EXPECT_NO_THROW(map.toImageDist(dist_img, *pc));
  EXPECT_NO_THROW(map.toImageDistDx(dist_deriv_x_img, *pc));
  EXPECT_NO_THROW(map.toImageDistDy(dist_deriv_y_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dxx(dist_deriv_2_xx_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dxy(dist_deriv_2_xy_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dyx(dist_deriv_2_yx_img, *pc));
  EXPECT_NO_THROW(map.toImageDistD2Dyy(dist_deriv_2_yy_img, *pc));

  EXPECT_NO_THROW(map.drawPoints(idx_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(map.drawPoints(dist_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(map.drawPoints(dist_deriv_x_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(map.drawPoints(dist_deriv_y_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_xx_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_xy_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_yx_img, *pc, cv::Scalar(255, 0, 0)));
  EXPECT_NO_THROW(
      map.drawPoints(dist_deriv_2_yy_img, *pc, cv::Scalar(255, 0, 0)));

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  cv::namedWindow("2nd::setGoalsRandom: index");
  cv::namedWindow("2nd::setGoalsRandom: distance");
  cv::namedWindow("2nd::setGoalsRandom: deriv x");
  cv::namedWindow("2nd::setGoalsRandom: deriv y");
  cv::namedWindow("2nd::setGoalsRandom: 2nd deriv xx");
  cv::namedWindow("2nd::setGoalsRandom: 2nd deriv xy");
  cv::namedWindow("2nd::setGoalsRandom: 2nd deriv yx");
  cv::namedWindow("2nd::setGoalsRandom: 2nd deriv yy");

  cv::imshow("2nd::setGoalsRandom: index", idx_img);
  cv::imshow("2nd::setGoalsRandom: distance", dist_img);
  cv::imshow("2nd::setGoalsRandom: deriv x", dist_deriv_x_img);
  cv::imshow("2nd::setGoalsRandom: deriv y", dist_deriv_y_img);
  cv::imshow("2nd::setGoalsRandom: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("2nd::setGoalsRandom: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("2nd::setGoalsRandom: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("2nd::setGoalsRandom: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_static_with_2nd_deriv, setGoalsRandom)

TEST(data_structures_distance_map_static_standard, validation)
{
  using Scalar = float;

  using DistanceMapGTType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudGTType = pcl::Point2VectorCloud_<DistanceMapGTType::Scalar>;
  using PointCloudGTTypePtr = std::shared_ptr<PointCloudGTType>;

  using DistanceMapType = DistanceMapStatic_<DistanceMapCell_<Scalar>>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;

  using CellType = DistanceMapWith2ndDerivCell_<Scalar>;

  std::random_device rd;
  std::mt19937 rng(rd());

  DistanceMapType map;
  DistanceMapGTType map_gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudGTTypePtr pc_gt = std::make_shared<PointCloudGTType>();

  PointCloudTypePtr pc_diff_get_dist = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_dx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_dy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dxx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dxy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dyx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dyy = std::make_shared<PointCloudType>();

  PointCloudTypePtr pc_diff_map_dist = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_dx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_dy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dxx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dxy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dyx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dyy = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.05);
  Scalar map_size = Scalar(-1.0);
  Scalar max_range = Scalar(10.0);
  Scalar max_dist = Scalar(2.5);

  Scalar ang_res = geometry::degToRad(static_cast<Scalar>(1.0));
  Scalar fov = geometry::degToRad(static_cast<Scalar>(360.0));

  test::getLaser2DPointCloudRandom(*pc, ang_res, fov, max_range, rng);
  pc_gt->resize(pc->size());
  pc->copyTo<0>(pc_gt->begin());

  ASSERT_EQ(pc->size(), 360);
  ASSERT_EQ(pc_gt->size(), pc->size());

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));
  EXPECT_NO_THROW(map_gt.setGoals(*pc_gt, res, map_size, max_dist));

  for (int r = 0; r < (*map.getMap()).rows(); r++)
  {
    for (int c = 0; c < (*map.getMap()).cols(); c++)
    {
      CellType cell;
      Eigen::Vector2i index(r, c);

      if (!map.getValueWith2ndDeriv(cell, index, *pc))
      {
        continue;
      }

      EXPECT_NEAR((*map.getMap())(r, c).m_dist, (*map_gt.getMap())(r, c).m_dist,
                  res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d(0),
                  (*map_gt.getMap())(r, c).m_dist_d(0), res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d(1),
                  (*map_gt.getMap())(r, c).m_dist_d(1), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(0, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 0), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(0, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 1), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(1, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 0), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(1, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 1), res * 1e-2);

      if ((*map.getMap())(r, c).m_dist != (*map_gt.getMap())(r, c).m_dist)
        pc_diff_map_dist->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d(0) !=
          (*map_gt.getMap())(r, c).m_dist_d(0))
        pc_diff_map_dist_dx->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d(1) !=
          (*map_gt.getMap())(r, c).m_dist_d(1))
        pc_diff_map_dist_dy->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(0, 0) != (*map_gt.getMap())(r, c).m_dist_d2(0, 0))
        pc_diff_map_dist_d2dxx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(0, 1) != (*map_gt.getMap())(r, c).m_dist_d2(0, 1))
        pc_diff_map_dist_d2dxy->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(1, 0) != (*map_gt.getMap())(r, c).m_dist_d2(1, 0))
        pc_diff_map_dist_d2dyx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(1, 1) != (*map_gt.getMap())(r, c).m_dist_d2(1, 1))
        pc_diff_map_dist_d2dyy->emplace_back(PointType(PointVectorType(r, c)));

      EXPECT_NEAR(cell.m_dist, (*map_gt.getMap())(r, c).m_dist, res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d(0), (*map_gt.getMap())(r, c).m_dist_d(0),
                  res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d(1), (*map_gt.getMap())(r, c).m_dist_d(1),
                  res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(0, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 0), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(0, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 1), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(1, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 0), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(1, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 1), res * 1e-2);

      if (cell.m_dist != (*map_gt.getMap())(r, c).m_dist)
        pc_diff_get_dist->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d(0) != (*map_gt.getMap())(r, c).m_dist_d(0))
        pc_diff_get_dist_dx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d(1) != (*map_gt.getMap())(r, c).m_dist_d(1))
        pc_diff_get_dist_dy->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(0, 0) != (*map_gt.getMap())(r, c).m_dist_d2(0, 0))
        pc_diff_get_dist_d2dxx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(0, 1) != (*map_gt.getMap())(r, c).m_dist_d2(0, 1))
        pc_diff_get_dist_d2dxy->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(1, 0) != (*map_gt.getMap())(r, c).m_dist_d2(1, 0))
        pc_diff_get_dist_d2dyx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(1, 1) != (*map_gt.getMap())(r, c).m_dist_d2(1, 1))
        pc_diff_get_dist_d2dyy->emplace_back(PointType(PointVectorType(r, c)));
    }
  }

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  auto computeAndShowAllImgs =
      [](RGBImage& img_eval, RGBImage& img_gt, RGBImage& img_diff_get,
         RGBImage& img_diff_map, DistanceMapType& dmap_eval,
         DistanceMapGTType& dmap_gt, const PointCloudType& pc_eval,
         const PointCloudType& pc_gt, const PointCloudType& pc_diff_get,
         const PointCloudType& pc_diff_map,
         DistanceMapType::Channel dmap_eval_ch,
         DistanceMapGTType::Channel dmap_gt_ch)
  {
    EXPECT_NO_THROW(dmap_eval.toImage(img_eval, dmap_eval_ch, pc_eval));
    EXPECT_NO_THROW(dmap_gt.toImage(img_gt, dmap_gt_ch, pc_gt));
    EXPECT_NO_THROW(dmap_eval.toImage(img_diff_get, dmap_eval_ch, pc_eval));
    EXPECT_NO_THROW(dmap_eval.toImage(img_diff_map, dmap_eval_ch, pc_eval));
    EXPECT_NO_THROW(
        dmap_eval.drawPoints(img_eval, pc_eval, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(dmap_gt.drawPoints(img_gt, pc_gt, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(
        dmap_eval.drawPoints(img_diff_get, pc_eval, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(
        dmap_eval.drawPoints(img_diff_map, pc_eval, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(dmap_eval.drawPxFromMapIdx(img_diff_get, pc_diff_get,
                                               cv::Scalar(255, 255, 255)));
    EXPECT_NO_THROW(dmap_eval.drawPxFromMapIdx(img_diff_map, pc_diff_map,
                                               cv::Scalar(255, 255, 255)));

    std::string img_descrip;

    switch (dmap_eval_ch)
    {
      case DistanceMapType::Channel::kDist:
        img_descrip = "distance";
        break;
      case DistanceMapType::Channel::kDistDx:
        img_descrip = "deriv x";
        break;
      case DistanceMapType::Channel::kDistDy:
        img_descrip = "deriv y";
        break;
      case DistanceMapType::Channel::kDistD2Dxx:
        img_descrip = "2nd deriv xx";
        break;
      case DistanceMapType::Channel::kDistD2Dxy:
        img_descrip = "2nd deriv xy";
        break;
      case DistanceMapType::Channel::kDistD2Dyx:
        img_descrip = "2nd deriv yx";
        break;
      case DistanceMapType::Channel::kDistD2Dyy:
        img_descrip = "2nd deriv yy";
        break;
    }

    cv::namedWindow("std::validation: " + img_descrip);
    cv::namedWindow("std::validation: " + img_descrip + " gt");
    cv::namedWindow("std::validation: " + img_descrip + " diff getValue");
    cv::namedWindow("std::validation: " + img_descrip + " diff getMap");
    cv::imshow("std::validation: " + img_descrip, img_eval);
    cv::imshow("std::validation: " + img_descrip + " gt", img_gt);
    cv::imshow("std::validation: " + img_descrip + " diff getValue",
               img_diff_get);
    cv::imshow("std::validation: " + img_descrip + " diff getMap",
               img_diff_map);
    cv::waitKey(0);
    cv::destroyAllWindows();
  };

  RGBImage dist_img, dist_deriv_x_img, dist_deriv_y_img, dist_deriv_2_xx_img,
      dist_deriv_2_xy_img, dist_deriv_2_yx_img, dist_deriv_2_yy_img;
  RGBImage dist_img_gt, dist_deriv_x_img_gt, dist_deriv_y_img_gt,
      dist_deriv_2_xx_img_gt, dist_deriv_2_xy_img_gt, dist_deriv_2_yx_img_gt,
      dist_deriv_2_yy_img_gt;
  RGBImage dist_img_diff_get, dist_deriv_x_img_diff_get,
      dist_deriv_y_img_diff_get, dist_deriv_2_xx_img_diff_get,
      dist_deriv_2_xy_img_diff_get, dist_deriv_2_yx_img_diff_get,
      dist_deriv_2_yy_img_diff_get;
  RGBImage dist_img_diff_map, dist_deriv_x_img_diff_map,
      dist_deriv_y_img_diff_map, dist_deriv_2_xx_img_diff_map,
      dist_deriv_2_xy_img_diff_map, dist_deriv_2_yx_img_diff_map,
      dist_deriv_2_yy_img_diff_map;

  computeAndShowAllImgs(
      dist_img, dist_img_gt, dist_img_diff_get, dist_img_diff_map, map, map_gt,
      *pc, *pc_gt, *pc_diff_get_dist, *pc_diff_map_dist,
      DistanceMapType::Channel::kDist, DistanceMapGTType::Channel::kDist);

  computeAndShowAllImgs(dist_deriv_x_img, dist_deriv_x_img_gt,
                        dist_deriv_x_img_diff_get, dist_deriv_x_img_diff_map,
                        map, map_gt, *pc, *pc_gt, *pc_diff_get_dist_dx,
                        *pc_diff_map_dist_dx, DistanceMapType::Channel::kDistDx,
                        DistanceMapGTType::Channel::kDistDx);

  computeAndShowAllImgs(dist_deriv_y_img, dist_deriv_y_img_gt,
                        dist_deriv_y_img_diff_get, dist_deriv_y_img_diff_map,
                        map, map_gt, *pc, *pc_gt, *pc_diff_get_dist_dy,
                        *pc_diff_map_dist_dy, DistanceMapType::Channel::kDistDy,
                        DistanceMapGTType::Channel::kDistDy);

  computeAndShowAllImgs(dist_deriv_2_xx_img, dist_deriv_2_xx_img_gt,
                        dist_deriv_2_xx_img_diff_get,
                        dist_deriv_2_xx_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dxx, *pc_diff_map_dist_d2dxx,
                        DistanceMapType::Channel::kDistD2Dxx,
                        DistanceMapGTType::Channel::kDistD2Dxx);

  computeAndShowAllImgs(dist_deriv_2_xy_img, dist_deriv_2_xy_img_gt,
                        dist_deriv_2_xy_img_diff_get,
                        dist_deriv_2_xy_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dxy, *pc_diff_map_dist_d2dxy,
                        DistanceMapType::Channel::kDistD2Dxy,
                        DistanceMapGTType::Channel::kDistD2Dxy);

  computeAndShowAllImgs(dist_deriv_2_yx_img, dist_deriv_2_yx_img_gt,
                        dist_deriv_2_yx_img_diff_get,
                        dist_deriv_2_yx_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dyx, *pc_diff_map_dist_d2dyx,
                        DistanceMapType::Channel::kDistD2Dyx,
                        DistanceMapGTType::Channel::kDistD2Dyx);

  computeAndShowAllImgs(dist_deriv_2_yy_img, dist_deriv_2_yy_img_gt,
                        dist_deriv_2_yy_img_diff_get,
                        dist_deriv_2_yy_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dyy, *pc_diff_map_dist_d2dyy,
                        DistanceMapType::Channel::kDistD2Dyy,
                        DistanceMapGTType::Channel::kDistD2Dyy);

#endif

}  // TEST(data_structures_distance_map_static_standard, validation)

TEST(data_structures_distance_map_static_with_2nd_deriv, validation)
{
  using Scalar = float;

  using DistanceMapGTType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudGTType = pcl::Point2VectorCloud_<DistanceMapGTType::Scalar>;
  using PointCloudGTTypePtr = std::shared_ptr<PointCloudGTType>;

  using DistanceMapType =
      DistanceMapStatic_<DistanceMapWith2ndDerivCell_<Scalar>>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;

  using CellType = DistanceMapWith2ndDerivCell_<Scalar>;

  std::random_device rd;
  std::mt19937 rng(rd());

  DistanceMapType map;
  DistanceMapGTType map_gt;

  PointCloudTypePtr pc = std::make_shared<PointCloudType>();
  PointCloudGTTypePtr pc_gt = std::make_shared<PointCloudGTType>();

  PointCloudTypePtr pc_diff_get_dist = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_dx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_dy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dxx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dxy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dyx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_get_dist_d2dyy = std::make_shared<PointCloudType>();

  PointCloudTypePtr pc_diff_map_dist = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_dx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_dy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dxx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dxy = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dyx = std::make_shared<PointCloudType>();
  PointCloudTypePtr pc_diff_map_dist_d2dyy = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.05);
  Scalar map_size = Scalar(-1.0);
  Scalar max_range = Scalar(10.0);
  Scalar max_dist = Scalar(2.5);

  Scalar ang_res = geometry::degToRad(static_cast<Scalar>(1.0));
  Scalar fov = geometry::degToRad(static_cast<Scalar>(360.0));

  test::getLaser2DPointCloudRandom(*pc, ang_res, fov, max_range, rng);
  pc_gt->resize(pc->size());
  pc->copyTo<0>(pc_gt->begin());

  ASSERT_EQ(pc->size(), 360);
  ASSERT_EQ(pc_gt->size(), pc->size());

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));
  EXPECT_NO_THROW(map_gt.setGoals(*pc_gt, res, map_size, max_dist));

  for (int r = 0; r < (*map.getMap()).rows(); r++)
  {
    for (int c = 0; c < (*map.getMap()).cols(); c++)
    {
      CellType cell;
      Eigen::Vector2i index(r, c);

      if (!map.getValueWith2ndDeriv(cell, index, *pc))
      {
        continue;
      }

      EXPECT_NEAR((*map.getMap())(r, c).m_dist, (*map_gt.getMap())(r, c).m_dist,
                  res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d(0),
                  (*map_gt.getMap())(r, c).m_dist_d(0), res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d(1),
                  (*map_gt.getMap())(r, c).m_dist_d(1), res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d2(0, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 0), res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d2(0, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 1), res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d2(1, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 0), res * 1e-2);
      EXPECT_NEAR((*map.getMap())(r, c).m_dist_d2(1, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 1), res * 1e-2);

      if ((*map.getMap())(r, c).m_dist != (*map_gt.getMap())(r, c).m_dist)
        pc_diff_map_dist->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d(0) !=
          (*map_gt.getMap())(r, c).m_dist_d(0))
        pc_diff_map_dist_dx->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d(1) !=
          (*map_gt.getMap())(r, c).m_dist_d(1))
        pc_diff_map_dist_dy->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d2(0, 0) !=
          (*map_gt.getMap())(r, c).m_dist_d2(0, 0))
        pc_diff_map_dist_d2dxx->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d2(0, 1) !=
          (*map_gt.getMap())(r, c).m_dist_d2(0, 1))
        pc_diff_map_dist_d2dxy->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d2(1, 0) !=
          (*map_gt.getMap())(r, c).m_dist_d2(1, 0))
        pc_diff_map_dist_d2dyx->emplace_back(PointType(PointVectorType(r, c)));
      if ((*map.getMap())(r, c).m_dist_d2(1, 1) !=
          (*map_gt.getMap())(r, c).m_dist_d2(1, 1))
        pc_diff_map_dist_d2dyy->emplace_back(PointType(PointVectorType(r, c)));

      EXPECT_NEAR(cell.m_dist, (*map_gt.getMap())(r, c).m_dist, res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d(0), (*map_gt.getMap())(r, c).m_dist_d(0),
                  res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d(1), (*map_gt.getMap())(r, c).m_dist_d(1),
                  res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(0, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 0), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(0, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(0, 1), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(1, 0),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 0), res * 1e-2);
      EXPECT_NEAR(cell.m_dist_d2(1, 1),
                  (*map_gt.getMap())(r, c).m_dist_d2(1, 1), res * 1e-2);

      if (cell.m_dist != (*map_gt.getMap())(r, c).m_dist)
        pc_diff_get_dist->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d(0) != (*map_gt.getMap())(r, c).m_dist_d(0))
        pc_diff_get_dist_dx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d(1) != (*map_gt.getMap())(r, c).m_dist_d(1))
        pc_diff_get_dist_dy->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(0, 0) != (*map_gt.getMap())(r, c).m_dist_d2(0, 0))
        pc_diff_get_dist_d2dxx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(0, 1) != (*map_gt.getMap())(r, c).m_dist_d2(0, 1))
        pc_diff_get_dist_d2dxy->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(1, 0) != (*map_gt.getMap())(r, c).m_dist_d2(1, 0))
        pc_diff_get_dist_d2dyx->emplace_back(PointType(PointVectorType(r, c)));
      if (cell.m_dist_d2(1, 1) != (*map_gt.getMap())(r, c).m_dist_d2(1, 1))
        pc_diff_get_dist_d2dyy->emplace_back(PointType(PointVectorType(r, c)));
    }
  }

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  auto computeAndShowAllImgs =
      [](RGBImage& img_eval, RGBImage& img_gt, RGBImage& img_diff_get,
         RGBImage& img_diff_map, DistanceMapType& dmap_eval,
         DistanceMapGTType& dmap_gt, const PointCloudType& pc_eval,
         const PointCloudType& pc_gt, const PointCloudType& pc_diff_get,
         const PointCloudType& pc_diff_map,
         DistanceMapType::Channel dmap_eval_ch,
         DistanceMapGTType::Channel dmap_gt_ch)
  {
    EXPECT_NO_THROW(dmap_eval.toImage(img_eval, dmap_eval_ch, pc_eval));
    EXPECT_NO_THROW(dmap_gt.toImage(img_gt, dmap_gt_ch, pc_gt));
    EXPECT_NO_THROW(dmap_eval.toImage(img_diff_get, dmap_eval_ch, pc_eval));
    EXPECT_NO_THROW(dmap_eval.toImage(img_diff_map, dmap_eval_ch, pc_eval));
    EXPECT_NO_THROW(
        dmap_eval.drawPoints(img_eval, pc_eval, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(dmap_gt.drawPoints(img_gt, pc_gt, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(
        dmap_eval.drawPoints(img_diff_get, pc_eval, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(
        dmap_eval.drawPoints(img_diff_map, pc_eval, cv::Scalar(255, 0, 0)));
    EXPECT_NO_THROW(dmap_eval.drawPxFromMapIdx(img_diff_get, pc_diff_get,
                                               cv::Scalar(255, 255, 255)));
    EXPECT_NO_THROW(dmap_eval.drawPxFromMapIdx(img_diff_map, pc_diff_map,
                                               cv::Scalar(255, 255, 255)));

    std::string img_descrip;

    switch (dmap_eval_ch)
    {
      case DistanceMapType::Channel::kDist:
        img_descrip = "distance";
        break;
      case DistanceMapType::Channel::kDistDx:
        img_descrip = "deriv x";
        break;
      case DistanceMapType::Channel::kDistDy:
        img_descrip = "deriv y";
        break;
      case DistanceMapType::Channel::kDistD2Dxx:
        img_descrip = "2nd deriv xx";
        break;
      case DistanceMapType::Channel::kDistD2Dxy:
        img_descrip = "2nd deriv xy";
        break;
      case DistanceMapType::Channel::kDistD2Dyx:
        img_descrip = "2nd deriv yx";
        break;
      case DistanceMapType::Channel::kDistD2Dyy:
        img_descrip = "2nd deriv yy";
        break;
    }

    cv::namedWindow("2nd::validation: " + img_descrip);
    cv::namedWindow("2nd::validation: " + img_descrip + " gt");
    cv::namedWindow("2nd::validation: " + img_descrip + " diff getValue");
    cv::namedWindow("2nd::validation: " + img_descrip + " diff getMap");
    cv::imshow("2nd::validation: " + img_descrip, img_eval);
    cv::imshow("2nd::validation: " + img_descrip + " gt", img_gt);
    cv::imshow("2nd::validation: " + img_descrip + " diff getValue",
               img_diff_get);
    cv::imshow("2nd::validation: " + img_descrip + " diff getMap",
               img_diff_map);
    cv::waitKey(0);
    cv::destroyAllWindows();
  };

  RGBImage dist_img, dist_deriv_x_img, dist_deriv_y_img, dist_deriv_2_xx_img,
      dist_deriv_2_xy_img, dist_deriv_2_yx_img, dist_deriv_2_yy_img;
  RGBImage dist_img_gt, dist_deriv_x_img_gt, dist_deriv_y_img_gt,
      dist_deriv_2_xx_img_gt, dist_deriv_2_xy_img_gt, dist_deriv_2_yx_img_gt,
      dist_deriv_2_yy_img_gt;
  RGBImage dist_img_diff_get, dist_deriv_x_img_diff_get,
      dist_deriv_y_img_diff_get, dist_deriv_2_xx_img_diff_get,
      dist_deriv_2_xy_img_diff_get, dist_deriv_2_yx_img_diff_get,
      dist_deriv_2_yy_img_diff_get;
  RGBImage dist_img_diff_map, dist_deriv_x_img_diff_map,
      dist_deriv_y_img_diff_map, dist_deriv_2_xx_img_diff_map,
      dist_deriv_2_xy_img_diff_map, dist_deriv_2_yx_img_diff_map,
      dist_deriv_2_yy_img_diff_map;

  computeAndShowAllImgs(
      dist_img, dist_img_gt, dist_img_diff_get, dist_img_diff_map, map, map_gt,
      *pc, *pc_gt, *pc_diff_get_dist, *pc_diff_map_dist,
      DistanceMapType::Channel::kDist, DistanceMapGTType::Channel::kDist);

  computeAndShowAllImgs(dist_deriv_x_img, dist_deriv_x_img_gt,
                        dist_deriv_x_img_diff_get, dist_deriv_x_img_diff_map,
                        map, map_gt, *pc, *pc_gt, *pc_diff_get_dist_dx,
                        *pc_diff_map_dist_dx, DistanceMapType::Channel::kDistDx,
                        DistanceMapGTType::Channel::kDistDx);

  computeAndShowAllImgs(dist_deriv_y_img, dist_deriv_y_img_gt,
                        dist_deriv_y_img_diff_get, dist_deriv_y_img_diff_map,
                        map, map_gt, *pc, *pc_gt, *pc_diff_get_dist_dy,
                        *pc_diff_map_dist_dy, DistanceMapType::Channel::kDistDy,
                        DistanceMapGTType::Channel::kDistDy);

  computeAndShowAllImgs(dist_deriv_2_xx_img, dist_deriv_2_xx_img_gt,
                        dist_deriv_2_xx_img_diff_get,
                        dist_deriv_2_xx_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dxx, *pc_diff_map_dist_d2dxx,
                        DistanceMapType::Channel::kDistD2Dxx,
                        DistanceMapGTType::Channel::kDistD2Dxx);

  computeAndShowAllImgs(dist_deriv_2_xy_img, dist_deriv_2_xy_img_gt,
                        dist_deriv_2_xy_img_diff_get,
                        dist_deriv_2_xy_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dxy, *pc_diff_map_dist_d2dxy,
                        DistanceMapType::Channel::kDistD2Dxy,
                        DistanceMapGTType::Channel::kDistD2Dxy);

  computeAndShowAllImgs(dist_deriv_2_yx_img, dist_deriv_2_yx_img_gt,
                        dist_deriv_2_yx_img_diff_get,
                        dist_deriv_2_yx_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dyx, *pc_diff_map_dist_d2dyx,
                        DistanceMapType::Channel::kDistD2Dyx,
                        DistanceMapGTType::Channel::kDistD2Dyx);

  computeAndShowAllImgs(dist_deriv_2_yy_img, dist_deriv_2_yy_img_gt,
                        dist_deriv_2_yy_img_diff_get,
                        dist_deriv_2_yy_img_diff_map, map, map_gt, *pc, *pc_gt,
                        *pc_diff_get_dist_d2dyy, *pc_diff_map_dist_d2dyy,
                        DistanceMapType::Channel::kDistD2Dyy,
                        DistanceMapGTType::Channel::kDistD2Dyy);

#endif

}  // TEST(data_structures_distance_map_static_with_2nd_deriv, validation)

TEST(data_structures_distance_map_static_standard, serializerBoss)
{
  using DistanceMapType = DistanceMapStaticFloat;
  using DistanceMapTypePtr = std::shared_ptr<DistanceMapType>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using Scalar = DistanceMapType::Scalar;

  srrg2_solver::inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes();

  DistanceMapTypePtr map = std::make_shared<DistanceMapType>();
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(10.0);
  Scalar max_dist = Scalar(-1.0);

  ASSERT_NO_THROW(map->setGoals(*pc, res, map_size, max_dist));

  ASSERT_FLOAT_EQ(map->getRes(), res);
  ASSERT_FLOAT_EQ(map->getMapSize(), map_size);
  ASSERT_FLOAT_EQ(map->getMaxDist(), max_dist);

  std::string filename = std::string(__FILE__) + ".json";

  srrg2_core::Serializer serializer;

  serializer.setFilePath(filename);
  serializer.setBinaryPath(filename +
                           ".d/<classname>.<nameAttribute>.<id>.<ext>");

  serializer.writeObject(*map);

  map.reset();

  ASSERT_FALSE(map);

  srrg2_core::Deserializer deserializer;
  srrg2_core::SerializablePtr obj;

  deserializer.setFilePath(filename);

  while ((obj = deserializer.readObjectShared()))
  {
    DistanceMapTypePtr obj_ptr =
        std::dynamic_pointer_cast<DistanceMapType>(obj);

    if (obj_ptr)
    {
      map = obj_ptr;
      break;
    }
  }

  EXPECT_TRUE(map);

  EXPECT_FLOAT_EQ(map->getRes(), res);
  EXPECT_FLOAT_EQ(map->getMapSize(), map_size);
  EXPECT_FLOAT_EQ(map->getMaxDist(), max_dist);

  /**
   * @note also, do not forget to do $ export LC_ALL=C (SRRG BOSS stuff....)
   */
}  // TEST(data_structures_distance_map_static_standard, serializerBoss)

TEST(data_structures_distance_map_static_with_2nd_deriv, serializerBoss)
{
  using DistanceMapType = DistanceMapStaticWith2ndDerivFloat;
  using DistanceMapTypePtr = std::shared_ptr<DistanceMapType>;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using Scalar = DistanceMapType::Scalar;

  srrg2_solver::inesctec_mrdt_slam_distmap_2d_data_structures_registerTypes();

  DistanceMapTypePtr map = std::make_shared<DistanceMapType>();
  PointCloudTypePtr pc = std::make_shared<PointCloudType>();

  Scalar res = Scalar(0.01);
  Scalar map_size = Scalar(10.0);
  Scalar max_dist = Scalar(-1.0);

  ASSERT_NO_THROW(map->setGoals(*pc, res, map_size, max_dist));

  ASSERT_FLOAT_EQ(map->getRes(), res);
  ASSERT_FLOAT_EQ(map->getMapSize(), map_size);
  ASSERT_FLOAT_EQ(map->getMaxDist(), max_dist);

  std::string filename = std::string(__FILE__) + "_with_2nd_deriv.json";

  srrg2_core::Serializer serializer;

  serializer.setFilePath(filename);
  serializer.setBinaryPath(filename +
                           ".d/<classname>.<nameAttribute>.<id>.<ext>");

  serializer.writeObject(*map);

  map.reset();

  ASSERT_FALSE(map);

  srrg2_core::Deserializer deserializer;
  srrg2_core::SerializablePtr obj;

  deserializer.setFilePath(filename);

  while ((obj = deserializer.readObjectShared()))
  {
    DistanceMapTypePtr obj_ptr =
        std::dynamic_pointer_cast<DistanceMapType>(obj);

    if (obj_ptr)
    {
      map = obj_ptr;
      break;
    }
  }

  EXPECT_TRUE(map);

  EXPECT_FLOAT_EQ(map->getRes(), res);
  EXPECT_FLOAT_EQ(map->getMapSize(), map_size);
  EXPECT_FLOAT_EQ(map->getMaxDist(), max_dist);

  /**
   * @note also, do not forget to do $ export LC_ALL=C (SRRG BOSS stuff....)
   */
}  // TEST(data_structures_distance_map_static_with_2nd_deriv, serializerBoss)

}  // namespace testing
