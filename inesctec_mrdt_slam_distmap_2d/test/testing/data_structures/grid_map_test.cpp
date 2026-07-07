#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/grid_map.hpp>
#include <inesctec_mrdt_slam_distmap_2d/geometry/geometry_2d.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

class GridMap2DHeaderTest : public GridMap2DHeader
{
 public:

  void resize(const IndexType& size) final {}
};

using EstimateType = GridMap2DHeaderTest::EstimateType;
using IndexType = GridMap2DHeaderTest::IndexType;
using CoordinatesType = GridMap2DHeaderTest::CoordinatesType;

class data_structures_grid_map_header : public testing::Test
{
 public:

  GridMap2DHeaderTest m_header;

 protected:

  data_structures_grid_map_header() {}

  ~data_structures_grid_map_header() override {}

  void SetUp() override
  {
    m_header.setOrigin(EstimateType::Identity());
    m_header.setRes(0.05f);
    m_header.setSize(Eigen::Vector2i{2000, 2000});

    ASSERT_TRUE(m_header.getOrigin().isApprox(EstimateType::Identity()));
    ASSERT_TRUE(m_header.getOriginInv().isApprox(EstimateType::Identity()));
    ASSERT_EQ(m_header.getWidth(), 2000);
    ASSERT_EQ(m_header.getHeight(), 2000);
    ASSERT_FLOAT_EQ(m_header.getRes(), 0.05f);
    ASSERT_FLOAT_EQ(m_header.getResInv(), 1 / 0.05f);
  }

  void TearDown() override {}
};

TEST_F(data_structures_grid_map_header, global2local)
{
  EXPECT_FLOAT_EQ(this->m_header.global2local({0.f, 0.f}).x(), 0.f);
  EXPECT_FLOAT_EQ(this->m_header.global2local({0.f, 0.f}).y(), 0.f);

  EXPECT_FLOAT_EQ(this->m_header.global2local({10.f, 10.f}).x(), 10.f);
  EXPECT_FLOAT_EQ(this->m_header.global2local({10.f, 10.f}).y(), 10.f);

  this->m_header.setOrigin(geometry::v2t(Eigen::Vector3f{4.f, 5.f, 0.f}));

  EXPECT_FLOAT_EQ(this->m_header.global2local({0.f, 0.f}).x(), -4.f);
  EXPECT_FLOAT_EQ(this->m_header.global2local({0.f, 0.f}).y(), -5.f);

  EXPECT_FLOAT_EQ(this->m_header.global2local({10.f, 10.f}).x(), 6.f);
  EXPECT_FLOAT_EQ(this->m_header.global2local({10.f, 10.f}).y(), 5.f);
}  // TEST_F(data_structures_grid_map_header, global2local)

TEST_F(data_structures_grid_map_header, local2global)
{
  EXPECT_FLOAT_EQ(this->m_header.local2global({0.f, 0.f}).x(), 0.f);
  EXPECT_FLOAT_EQ(this->m_header.local2global({0.f, 0.f}).y(), 0.f);

  EXPECT_FLOAT_EQ(this->m_header.local2global({10.f, 10.f}).x(), 10.f);
  EXPECT_FLOAT_EQ(this->m_header.local2global({10.f, 10.f}).y(), 10.f);

  this->m_header.setOrigin(geometry::v2t(Eigen::Vector3f{4.f, 5.f, 0.f}));

  EXPECT_FLOAT_EQ(this->m_header.local2global({0.f, 0.f}).x(), 4.f);
  EXPECT_FLOAT_EQ(this->m_header.local2global({0.f, 0.f}).y(), 5.f);

  EXPECT_FLOAT_EQ(this->m_header.local2global({10.f, 10.f}).x(), 14.f);
  EXPECT_FLOAT_EQ(this->m_header.local2global({10.f, 10.f}).y(), 15.f);
}  // TEST_F(data_structures_grid_map_header, local2global)

TEST_F(data_structures_grid_map_header, global2Idx)
{
  EXPECT_EQ(this->m_header.global2Idx({0.f, 0.f}).x(), 0);
  EXPECT_EQ(this->m_header.global2Idx({0.f, 0.f}).y(), 0);

  EXPECT_EQ(this->m_header.global2Idx({10.f, 6.f}).x(), 200);
  EXPECT_EQ(this->m_header.global2Idx({10.f, 6.f}).y(), 120);

  EXPECT_EQ(this->m_header.global2Idx({10.05f, 6.05f}).x(), 201);
  EXPECT_EQ(this->m_header.global2Idx({10.05f, 6.05f}).y(), 121);
  EXPECT_EQ(this->m_header.global2Idx({10.10f, 6.10f}).x(), 202);
  EXPECT_EQ(this->m_header.global2Idx({10.10f, 6.10f}).y(), 122);
  EXPECT_EQ(this->m_header.global2Idx({10.07f, 6.07f}).x(), 201);
  EXPECT_EQ(this->m_header.global2Idx({10.07f, 6.07f}).y(), 121);
  EXPECT_EQ(this->m_header.global2Idx({10.12f, 6.12f}).x(), 202);
  EXPECT_EQ(this->m_header.global2Idx({10.12f, 6.12f}).y(), 122);

  this->m_header.setOrigin(geometry::v2t(Eigen::Vector3f{4.f, 5.f, 0.f}));

  EXPECT_EQ(this->m_header.global2Idx({0.f, 0.f}).x(), -80);
  EXPECT_EQ(this->m_header.global2Idx({0.f, 0.f}).y(), -100);

  EXPECT_EQ(this->m_header.global2Idx({10.f, 6.f}).x(), 120);
  EXPECT_EQ(this->m_header.global2Idx({10.f, 6.f}).y(), 20);
}  // TEST_F(data_structures_grid_map_header, global2Idx)

TEST_F(data_structures_grid_map_header, idx2global)
{
  EXPECT_FLOAT_EQ(this->m_header.idx2global({0, 0}).x(), 0.f);
  EXPECT_FLOAT_EQ(this->m_header.idx2global({0, 0}).y(), 0.f);

  EXPECT_FLOAT_EQ(this->m_header.idx2global({200, 120}).x(), 10.f);
  EXPECT_FLOAT_EQ(this->m_header.idx2global({200, 120}).y(), 6.f);

  EXPECT_FLOAT_EQ(this->m_header.idx2global({201, 121}).x(), 10.05f);
  EXPECT_FLOAT_EQ(this->m_header.idx2global({201, 121}).y(), 6.05f);
  EXPECT_FLOAT_EQ(this->m_header.idx2global({202, 122}).x(), 10.10f);
  EXPECT_FLOAT_EQ(this->m_header.idx2global({202, 122}).y(), 6.10f);

  EXPECT_NE(this->m_header.idx2global({201, 121}).x(), 10.07f);
  EXPECT_NE(this->m_header.idx2global({201, 121}).y(), 6.07f);
  EXPECT_NE(this->m_header.idx2global({202, 122}).x(), 10.12f);
  EXPECT_NE(this->m_header.idx2global({202, 122}).y(), 6.12f);

  EXPECT_FLOAT_EQ(this->m_header.floatIdx2global({201.4f, 121.4f}).x(), 10.07f);
  EXPECT_FLOAT_EQ(this->m_header.floatIdx2global({201.4f, 121.4f}).y(), 6.07f);
  EXPECT_FLOAT_EQ(this->m_header.floatIdx2global({202.4f, 122.4f}).x(), 10.12f);
  EXPECT_FLOAT_EQ(this->m_header.floatIdx2global({202.4f, 122.4f}).y(), 6.12f);

  this->m_header.setOrigin(geometry::v2t(Eigen::Vector3f{4.f, 5.f, 0.f}));

  EXPECT_FLOAT_EQ(this->m_header.idx2global({-80, -100}).x(), 0.f);
  EXPECT_FLOAT_EQ(this->m_header.idx2global({-80, -100}).y(), 0.f);

  EXPECT_FLOAT_EQ(this->m_header.idx2global({120, 20}).x(), 10.f);
  EXPECT_FLOAT_EQ(this->m_header.idx2global({120, 20}).y(), 6.f);
}  // TEST_F(data_structures_grid_map_header, idx2global)

TEST_F(data_structures_grid_map_header, boundingBox)
{
  EXPECT_FLOAT_EQ(this->m_header.boundingBox().x(), 100.f);
  EXPECT_FLOAT_EQ(this->m_header.boundingBox().y(), 100.f);

  this->m_header.setSize(CoordinatesType{300.f, 100.f});

  EXPECT_FLOAT_EQ(this->m_header.boundingBox().x(), 300.f);
  EXPECT_FLOAT_EQ(this->m_header.boundingBox().y(), 100.f);
}  // TEST_F(data_structures_grid_map_header, boundingBox)

TEST_F(data_structures_grid_map_header, corners)
{
  Eigen::Matrix<float, 2, 4> pts;

  this->m_header.corners(pts);

  EXPECT_FLOAT_EQ(pts.col(0).x(), 0.f);
  EXPECT_FLOAT_EQ(pts.col(0).y(), 0.f);
  EXPECT_FLOAT_EQ(pts.col(1).x(), 99.95f);
  EXPECT_FLOAT_EQ(pts.col(1).y(), 0.f);
  EXPECT_FLOAT_EQ(pts.col(2).x(), 99.95f);
  EXPECT_FLOAT_EQ(pts.col(2).y(), 99.95f);
  EXPECT_FLOAT_EQ(pts.col(3).x(), 0);
  EXPECT_FLOAT_EQ(pts.col(3).y(), 99.95f);
}  // TEST_F(data_structures_grid_map_header, corners)

TEST(data_structures_grid_map, constructor)
{
  using GridMapType =
      GridMap2D_<OccupancyGridMap, OccupancyImage, FrequencyIntGridMap>;

  ASSERT_NO_THROW(GridMapType map);
  ASSERT_NO_THROW(GridMapType map(std::make_shared<OccupancyGridMap>(),
                                  std::make_shared<OccupancyImage>(),
                                  std::make_shared<FrequencyIntGridMap>()));
  ASSERT_NO_THROW(
      GridMapType map(std::move(std::make_shared<OccupancyGridMap>()),
                      std::move(std::make_shared<OccupancyImage>()),
                      std::move(std::make_shared<FrequencyIntGridMap>())));

  GridMapType map_default;

  EXPECT_TRUE(map_default.getOccupancyMapPtr());
  EXPECT_TRUE(map_default.getOccupancyImgPtr());
  EXPECT_TRUE(map_default.getFrequencyMapPtr());

  OccupancyGridMapPtr occ = std::make_shared<OccupancyGridMap>();
  OccupancyImagePtr img = std::make_shared<OccupancyImage>();
  FrequencyIntGridMapPtr freq = std::make_shared<FrequencyIntGridMap>();

  GridMapType map_shared(occ, img, freq);

  EXPECT_TRUE(map_shared.getOccupancyMapPtr() == occ);
  EXPECT_TRUE(map_shared.getOccupancyImgPtr() == img);
  EXPECT_TRUE(map_shared.getFrequencyMapPtr() == freq);

  GridMapType map_mv(std::move(std::make_shared<OccupancyGridMap>()),
                     std::move(std::make_shared<OccupancyImage>()),
                     std::move(std::make_shared<FrequencyIntGridMap>()));

  EXPECT_TRUE(map_mv.getOccupancyMapPtr());
  EXPECT_TRUE(map_mv.getOccupancyImgPtr());
  EXPECT_TRUE(map_mv.getFrequencyMapPtr());

  GridMapType map_mv_2(std::move(occ), std::move(img), std::move(freq));

  EXPECT_TRUE(map_mv_2.getOccupancyMapPtr());
  EXPECT_TRUE(map_mv_2.getOccupancyImgPtr());
  EXPECT_TRUE(map_mv_2.getFrequencyMapPtr());

  GridMapType map_occ(std::make_shared<OccupancyGridMap>(), nullptr, nullptr);
  GridMapType map_img(nullptr, std::make_shared<OccupancyImage>(), nullptr);
  GridMapType map_freq(nullptr, nullptr,
                       std::make_shared<FrequencyIntGridMap>());

  EXPECT_TRUE(map_occ.getOccupancyMapPtr());
  EXPECT_FALSE(map_occ.getOccupancyImgPtr());
  EXPECT_FALSE(map_occ.getFrequencyMapPtr());

  EXPECT_FALSE(map_img.getOccupancyMapPtr());
  EXPECT_TRUE(map_img.getOccupancyImgPtr());
  EXPECT_FALSE(map_img.getFrequencyMapPtr());

  EXPECT_FALSE(map_freq.getOccupancyMapPtr());
  EXPECT_FALSE(map_freq.getOccupancyImgPtr());
  EXPECT_TRUE(map_freq.getFrequencyMapPtr());
}  // TEST(data_structures_grid_map, constructor)

TEST(data_structures_grid_map, setSize)
{
  using GridMapType =
      GridMap2D_<OccupancyGridMap, OccupancyImage, FrequencyIntGridMap>;

  GridMapType map(std::move(std::make_shared<OccupancyGridMap>()),
                  std::move(std::make_shared<OccupancyImage>()),
                  std::move(std::make_shared<FrequencyIntGridMap>()));

  ASSERT_TRUE(map.getOccupancyMapPtr());
  ASSERT_TRUE(map.getOccupancyImgPtr());
  ASSERT_TRUE(map.getFrequencyMapPtr());

  EXPECT_EQ(map.getOccupancyMap()->rows(), 0);
  EXPECT_EQ(map.getOccupancyMap()->cols(), 0);
  EXPECT_EQ(map.getOccupancyImg()->rows(), 0);
  EXPECT_EQ(map.getOccupancyImg()->cols(), 0);
  EXPECT_EQ(map.getFrequencyMap()->rows(), 0);
  EXPECT_EQ(map.getFrequencyMap()->cols(), 0);

  map.setSize(GridMapType::IndexType(100, 200));

  EXPECT_EQ(map.getOccupancyMap()->rows(), 100);
  EXPECT_EQ(map.getOccupancyMap()->cols(), 200);
  EXPECT_EQ(map.getOccupancyImg()->rows(), 100);
  EXPECT_EQ(map.getOccupancyImg()->cols(), 200);
  EXPECT_EQ(map.getFrequencyMap()->rows(), 100);
  EXPECT_EQ(map.getFrequencyMap()->cols(), 200);

  map.setRes(0.01f);
  map.setSize(GridMapType::CoordinatesType(100.f, 50.f));

  EXPECT_EQ(map.getOccupancyMap()->rows(), 10000);
  EXPECT_EQ(map.getOccupancyMap()->cols(), 5000);
  EXPECT_EQ(map.getOccupancyImg()->rows(), 10000);
  EXPECT_EQ(map.getOccupancyImg()->cols(), 5000);
  EXPECT_EQ(map.getFrequencyMap()->rows(), 10000);
  EXPECT_EQ(map.getFrequencyMap()->cols(), 5000);
}  // TEST(data_structures_grid_map, resize)

}  // namespace testing
