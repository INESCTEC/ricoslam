#include <gtest/gtest.h>
#include <srrg2_core/srrg_boss/deserializer.h>
#include <srrg2_core/srrg_boss/serializer.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_nano_static.hpp>
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

TEST(data_structures_distance_map_nano_static, constructor)
{
  DistanceMapNanoStaticFloat dist_map_f;
  DistanceMapNanoStaticDouble dist_map_d;

  EXPECT_TRUE(dist_map_f.getMap());
  EXPECT_TRUE(dist_map_d.getMap());

  EXPECT_EQ(dist_map_f.getType(), DistanceMapImplType::kNNPointParent);
  EXPECT_EQ(dist_map_d.getType(), DistanceMapImplType::kNNPointParent);
}  // TEST(data_structures_distance_map_nano_static, constructor)

TEST(data_structures_distance_map_nano_static, findNeighbors)
{
  constexpr int rows = 50;
  constexpr int cols = 100;

  DistanceMapNanoStaticFloat map;

  ASSERT_NO_FATAL_FAILURE(map.getMap()->resize(rows, cols));

  DistanceMapNanoStaticFloat::CellType* nn_std[8];

  EXPECT_EQ(map.findNeighbors(nn_std, 0, 0), 3);
  EXPECT_EQ(map.findNeighbors(nn_std, rows - 1, cols - 1), 3);
  EXPECT_EQ(map.findNeighbors(nn_std, 0, cols - 1), 3);
  EXPECT_EQ(map.findNeighbors(nn_std, rows - 1, 0), 3);
  EXPECT_EQ(map.findNeighbors(nn_std, 1, 1), 8);
}  // TEST(data_structures_distance_map_nano_static, findNeighbors)

TEST(data_structures_distance_map_nano_static, setGoalsSimpleParam)
{
  using DistanceMapType = DistanceMapNanoStaticFloat;
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
}  // TEST(data_structures_distance_map_nano_static, setGoalsSimpleParam)

TEST(data_structures_distance_map_nano_static, setGoalsSimpleSize)
{
  using DistanceMapType = DistanceMapNanoStaticFloat;
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
}  // TEST(data_structures_distance_map_nano_static, setGoalsSimpleSize)

TEST(data_structures_distance_map_nano_static, global2local)
{
  using DistanceMapType = DistanceMapNanoStaticFloat;
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
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  ASSERT_EQ((*map.getMap()).rows(), 201);
  ASSERT_EQ((*map.getMap()).cols(), 201);

  ASSERT_FLOAT_EQ(map.getPointCloudMin().x(), -1.f);
  ASSERT_FLOAT_EQ(map.getPointCloudMin().y(), -1.f);

  PointVectorType map_pc_min = map.getPointCloudMin();

  int rows = (*map.getMap()).rows();
  int cols = (*map.getMap()).cols();

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      PointVectorType pt(std::round(map_pc_min.x() / res + r) * res,
                         std::round(map_pc_min.y() / res + c) * res);

      EXPECT_NEAR(map.global2local(pt).x(), r, 1);
      EXPECT_NEAR(map.global2local(pt).y(), c, 1);

      EXPECT_NEAR(map.local2global(map.global2local(pt)).x(), pt.x(), res);
      EXPECT_NEAR(map.local2global(map.global2local(pt)).y(), pt.y(), res);

      EXPECT_NEAR(map.local2global(Eigen::Vector2i(r, c)).x(), pt.x(), res);
      EXPECT_NEAR(map.local2global(Eigen::Vector2i(r, c)).y(), pt.y(), res);
    }
  }
}  // TEST(data_structures_distance_map_nano_static, global2local)

TEST(data_structures_distance_map_nano_static, getValue)
{
  using DistanceMapType = DistanceMapNanoStaticFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using PointType = PointCloudType::PointType;
  using PointVectorType = DistanceMapType::PointVectorType;
  using Scalar = DistanceMapType::Scalar;
  using CellTypeStd = DistanceMapType::CellTypeStd;
  using CellTypeWith2ndDeriv = DistanceMapType::CellTypeWith2ndDeriv;
  using VectorType = CellTypeWith2ndDeriv::VectorType;
  using MatrixType = CellTypeWith2ndDeriv::MatrixType;

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

  ASSERT_EQ((*map.getMap()).rows(), 201);
  ASSERT_EQ((*map.getMap()).cols(), 201);

  ASSERT_FLOAT_EQ(map.getPointCloudMin().x(), -1.f);
  ASSERT_FLOAT_EQ(map.getPointCloudMin().y(), -1.f);

  PointVectorType map_pc_min = map.getPointCloudMin();

  int rows = (*map.getMap()).rows();
  int cols = (*map.getMap()).cols();

  CellTypeStd cell_std;
  CellTypeWith2ndDeriv cell_2nd;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      PointVectorType pt(std::round(map_pc_min.x() / res + r) * res,
                         std::round(map_pc_min.y() / res + c) * res);

      Eigen::Vector2i pt_idx = map.global2local(pt);

      if (!map.getMap()->isInside(pt_idx))
      {
        continue;
      }

      // if (std::sqrt(pt.x()*pt.x()+pt.y()*pt.y()) < max_dist - res)
      if ((*map.getMap())(pt_idx.x(), pt_idx.y()) == 0)
      {
        EXPECT_TRUE(map.getValue(cell_std, pt, *pc));
        EXPECT_TRUE(map.getValueWith2ndDeriv(cell_2nd, pt, *pc));
        EXPECT_TRUE(map.getValueSubPx(cell_std, pt, *pc));
        EXPECT_TRUE(map.getValueSubPxWith2ndDeriv(cell_2nd, pt, *pc));

        EXPECT_EQ(map.getValue(cell_std, pt_idx, *pc),
                  map.getValue(cell_std, pt, *pc));
        EXPECT_EQ(map.getValueWith2ndDeriv(cell_2nd, pt_idx, *pc),
                  map.getValueWith2ndDeriv(cell_2nd, pt, *pc));
        EXPECT_EQ(map.getValue(cell_std, pt, *pc),
                  map.getValueSubPx(cell_std, pt, *pc));
        EXPECT_EQ(map.getValueWith2ndDeriv(cell_2nd, pt, *pc),
                  map.getValueSubPxWith2ndDeriv(cell_2nd, pt, *pc));

        if ((!map.getValue(cell_std, pt, *pc)) ||
            (!map.getValueWith2ndDeriv(cell_2nd, pt, *pc)) ||
            (!map.getValue(cell_std, pt_idx, *pc)) ||
            (!map.getValueWith2ndDeriv(cell_2nd, pt_idx, *pc)) ||
            (!map.getValueSubPx(cell_std, pt, *pc)) ||
            (!map.getValueSubPxWith2ndDeriv(cell_2nd, pt, *pc)))
        {
          std::cout << "data_structures_distance_map_nano_static::getValue: "
                       "EXPECT_TRUE(map.getValue(cell_std, pt))"
                       " invalid..."
                    << std::endl
                    << "- point    : "
                    << PointVectorType(pt.x(), pt.y()).transpose() << std::endl
                    << "- distance : " << cell_std.m_dist << std::endl
                    << "- 1st-deriv: " << cell_std.m_dist_d.transpose()
                    << std::endl;
        }

        map.getValue(cell_std, pt, *pc);
        map.getValueWith2ndDeriv(cell_2nd, pt, *pc);

        VectorType dist_d_approx(
            pt.x() / std::sqrt(pt.x() * pt.x() + pt.y() * pt.y()),
            pt.y() / std::sqrt(pt.x() * pt.x() + pt.y() * pt.y()));
        MatrixType dist_d2_approx;
        dist_d2_approx << pt.y() * pt.y() /
                              std::pow(pt.x() * pt.x() + pt.y() * pt.y(), 1.5),
            -pt.x() * pt.y() / std::pow(pt.x() * pt.x() + pt.y() * pt.y(), 1.5),
            -pt.x() * pt.y() / std::pow(pt.x() * pt.x() + pt.y() * pt.y(), 1.5),
            pt.x() * pt.x() / std::pow(pt.x() * pt.x() + pt.y() * pt.y(), 1.5);

        EXPECT_FLOAT_EQ(cell_std.m_dist,
                        std::sqrt(pt.x() * pt.x() + pt.y() * pt.y()));
        EXPECT_FLOAT_EQ(cell_2nd.m_dist,
                        std::sqrt(pt.x() * pt.x() + pt.y() * pt.y()));

        if (pt.x() == 0 && pt.y() == 0)
        {
          EXPECT_FLOAT_EQ(cell_std.m_dist_d(0), 0.f);
          EXPECT_FLOAT_EQ(cell_std.m_dist_d(1), 0.f);
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d(0), 0.f);
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d(1), 0.f);
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(0, 0), 0.f);
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(0, 1), 0.f);
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(1, 0), 0.f);
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(1, 1), 0.f);
        }
        else
        {
          EXPECT_FLOAT_EQ(cell_std.m_dist_d(0), dist_d_approx(0));
          EXPECT_FLOAT_EQ(cell_std.m_dist_d(1), dist_d_approx(1));
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d(0), dist_d_approx(0));
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d(1), dist_d_approx(1));
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(0, 0), dist_d2_approx(0, 0));
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(0, 1), dist_d2_approx(0, 1));
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(1, 0), dist_d2_approx(1, 0));
          EXPECT_FLOAT_EQ(cell_2nd.m_dist_d2(1, 1), dist_d2_approx(1, 1));
        }
      }
      // else if (std::sqrt(pt.x()*pt.x()+pt.y()*pt.y()) > max_dist + res)
      else
      {
        EXPECT_FALSE(map.getValue(cell_std, pt, *pc));
        EXPECT_FALSE(map.getValueWith2ndDeriv(cell_2nd, pt, *pc));

        if ((map.getValue(cell_std, pt, *pc)) ||
            (map.getValueWith2ndDeriv(cell_2nd, pt, *pc)))
        {
          std::cout << "data_structures_distance_map_nano_static::getValue: "
                       "EXPECT_FALSE(map.getValue(cell_std, pt))"
                       " invalid..."
                    << std::endl
                    << "- point    : "
                    << PointVectorType(pt.x(), pt.y()).transpose() << std::endl
                    << "- distance : " << cell_std.m_dist << std::endl
                    << "- 1st-deriv: " << cell_std.m_dist_d.transpose()
                    << std::endl;
        }
      }
    }
  }
}  // TEST(data_structures_distance_map_nano_static, getValue)

TEST(data_structures_distance_map_nano_static, setGoalsSimple)
{
  using DistanceMapType = DistanceMapNanoStaticFloat;
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
  Scalar map_size = Scalar(-1.0);
  Scalar max_dist = Scalar(1.0);

  EXPECT_NO_THROW(map.setGoals(*pc, res, map_size, max_dist));

  EXPECT_EQ((*map.getMap()).rows(), 201);
  EXPECT_EQ((*map.getMap()).cols(), 201);

  EXPECT_EQ((*map.getMap())(100, 100), 0);
  EXPECT_EQ((*map.getMap())(100, 99), 0);
  EXPECT_EQ((*map.getMap())(100, 101), 0);
  EXPECT_EQ((*map.getMap())(99, 100), 0);
  EXPECT_EQ((*map.getMap())(101, 100), 0);

  EXPECT_EQ((*map.getMap())(100, 98), 0);
  EXPECT_EQ((*map.getMap())(100, 102), 0);
  EXPECT_EQ((*map.getMap())(98, 100), 0);
  EXPECT_EQ((*map.getMap())(102, 100), 0);

  EXPECT_EQ((*map.getMap())(0, 0), -1);

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

  cv::namedWindow("setGoalsSimple: index");
  cv::namedWindow("setGoalsSimple: distance");
  cv::namedWindow("setGoalsSimple: deriv x");
  cv::namedWindow("setGoalsSimple: deriv y");
  cv::namedWindow("setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("setGoalsSimple: 2nd deriv yy");

  cv::imshow("setGoalsSimple: index", idx_img);
  cv::imshow("setGoalsSimple: distance", dist_img);
  cv::imshow("setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

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

  cv::namedWindow("setGoalsSimple: index");
  cv::namedWindow("setGoalsSimple: distance");
  cv::namedWindow("setGoalsSimple: deriv x");
  cv::namedWindow("setGoalsSimple: deriv y");
  cv::namedWindow("setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("setGoalsSimple: 2nd deriv yy");

  cv::imshow("setGoalsSimple: index", idx_img);
  cv::imshow("setGoalsSimple: distance", dist_img);
  cv::imshow("setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

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

  cv::namedWindow("setGoalsSimple: index");
  cv::namedWindow("setGoalsSimple: distance");
  cv::namedWindow("setGoalsSimple: deriv x");
  cv::namedWindow("setGoalsSimple: deriv y");
  cv::namedWindow("setGoalsSimple: 2nd deriv xx");
  cv::namedWindow("setGoalsSimple: 2nd deriv xy");
  cv::namedWindow("setGoalsSimple: 2nd deriv yx");
  cv::namedWindow("setGoalsSimple: 2nd deriv yy");

  cv::imshow("setGoalsSimple: index", idx_img);
  cv::imshow("setGoalsSimple: distance", dist_img);
  cv::imshow("setGoalsSimple: deriv x", dist_deriv_x_img);
  cv::imshow("setGoalsSimple: deriv y", dist_deriv_y_img);
  cv::imshow("setGoalsSimple: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("setGoalsSimple: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("setGoalsSimple: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("setGoalsSimple: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_nano_static, setGoalsSimple)

TEST(data_structures_distance_map_nano_static, setGoalsRandom)
{
  using Scalar = float;

  using PointCloudType = pcl::PointNormal2VectorCloud_<Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;

  using DistanceMapType = DistanceMapNanoStaticFloat;

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

  cv::namedWindow("setGoalsRandom: index");
  cv::namedWindow("setGoalsRandom: distance");
  cv::namedWindow("setGoalsRandom: deriv x");
  cv::namedWindow("setGoalsRandom: deriv y");
  cv::namedWindow("setGoalsRandom: 2nd deriv xx");
  cv::namedWindow("setGoalsRandom: 2nd deriv xy");
  cv::namedWindow("setGoalsRandom: 2nd deriv yx");
  cv::namedWindow("setGoalsRandom: 2nd deriv yy");

  cv::imshow("setGoalsRandom: index", idx_img);
  cv::imshow("setGoalsRandom: distance", dist_img);
  cv::imshow("setGoalsRandom: deriv x", dist_deriv_x_img);
  cv::imshow("setGoalsRandom: deriv y", dist_deriv_y_img);
  cv::imshow("setGoalsRandom: 2nd deriv xx", dist_deriv_2_xx_img);
  cv::imshow("setGoalsRandom: 2nd deriv xy", dist_deriv_2_xy_img);
  cv::imshow("setGoalsRandom: 2nd deriv yx", dist_deriv_2_yx_img);
  cv::imshow("setGoalsRandom: 2nd deriv yy", dist_deriv_2_yy_img);

  cv::waitKey(0);

  cv::destroyAllWindows();

#endif
}  // TEST(data_structures_distance_map_nano_static, setGoalsRandom)

TEST(data_structures_distance_map_nano_static, setGoalsRandomValidation)
{
  using DistanceMapType = DistanceMapNanoStaticFloat;
  using PointCloudType = pcl::Point2VectorCloud_<DistanceMapType::Scalar>;
  using PointCloudTypePtr = std::shared_ptr<PointCloudType>;
  using Scalar = DistanceMapType::Scalar;
  using CellTypeStd = DistanceMapType::CellTypeStd;

  using DistanceMapTestType = DistanceMapStatic_<CellTypeStd>;

  std::random_device rd;
  std::mt19937 rng(rd());

  DistanceMapType map;
  DistanceMapTestType map_test;

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
  EXPECT_NO_THROW(map_test.setGoals(*pc, res, map_size, max_dist));

  ASSERT_EQ((*map.getMap()).rows(), (*map_test.getMap()).rows());
  ASSERT_EQ((*map.getMap()).cols(), (*map_test.getMap()).cols());

  RGBImage dist_img_diff;
  dist_img_diff.create(map.getMap()->cols(), map.getMap()->rows());
  dist_img_diff = cv::Vec3b(0, 0, 0);

  Scalar max_dist_error = 0;
  Eigen::Vector2i max_dist_error_idx;

  for (int r = 0; r < (*map.getMap()).rows(); r++)
  {
    for (int c = 0; c < (*map.getMap()).cols(); c++)
    {
      Eigen::Vector2i pt_idx(r, c);
      CellTypeStd cell_std, cell_std_test;

      bool cell_std_isok = map.getValue(cell_std, pt_idx, *pc);
      bool cell_std_test_isok = map_test.getValue(cell_std_test, pt_idx, *pc);

      if (!cell_std_isok || !cell_std_test_isok)
      {
        continue;
      }

      if (std::fabs(cell_std.m_dist - cell_std_test.m_dist) > max_dist_error)
      {
        max_dist_error = std::fabs(cell_std.m_dist - cell_std_test.m_dist);
        max_dist_error_idx = pt_idx;
      }
    }
  }

  std::cerr << "max dist error    : " << max_dist_error << std::endl
            << "max dist error idx: ( " << max_dist_error_idx.transpose()
            << " )" << std::endl
            << "dist map res      : " << res << std::endl;

  for (int r = 0; r < (*map.getMap()).rows(); r++)
  {
    for (int c = 0; c < (*map.getMap()).cols(); c++)
    {
      Eigen::Vector2i pt_idx(r, c);
      CellTypeStd cell_std, cell_std_test;

      bool cell_std_isok = map.getValue(cell_std, pt_idx, *pc);
      bool cell_std_test_isok = map_test.getValue(cell_std_test, pt_idx, *pc);

      if (!cell_std_isok || !cell_std_test_isok)
      {
        continue;
      }

      Scalar dist_error = std::fabs(cell_std.m_dist - cell_std_test.m_dist);

      EXPECT_NEAR(dist_error, Scalar(0), res * 2);

      unsigned char v = static_cast<unsigned char>(
          std::round(255 * dist_error / max_dist_error));

      dist_img_diff.at<cv::Vec3b>(c, r) = cv::Vec3b(v, v, v);
    }
  }

#ifdef DISTANCE_MAP_TEST_OPENCV_VIZ

  cv::namedWindow("setGoalsRandomValidation: distance");
  cv::imshow("setGoalsRandomValidation: distance", dist_img_diff);
  cv::waitKey(0);
  cv::destroyAllWindows();

#endif

}  // TEST(data_structures_distance_map_nano_static, setGoalsRandomValidation)

TEST(data_structures_distance_map_nano_static, serializerBoss)
{
  using DistanceMapType = DistanceMapNanoStaticFloat;
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
}  // TEST(data_structures_distance_map_nano_static, serializerBoss)

}  // namespace testing
