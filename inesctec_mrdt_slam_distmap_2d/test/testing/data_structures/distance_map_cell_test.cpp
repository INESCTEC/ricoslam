#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_cell.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_distance_map_cell, constructor)
{
  using DistanceMapCellType = DistanceMapCellFloat;
  using ScalarType = DistanceMapCellType::Scalar;
  using VectorType = DistanceMapCellType::VectorType;

  DistanceMapCellType cell_1(1.f, -1, VectorType(2.f, 3.f));

  EXPECT_FLOAT_EQ(cell_1.m_dist, 1.f);
  EXPECT_FLOAT_EQ(cell_1.m_dist_d(0), 2.f);
  EXPECT_FLOAT_EQ(cell_1.m_dist_d(1), 3.f);

  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  DistanceMapCellType cell_2(5.f, -1, VectorType(3.f, 6.f));

  EXPECT_FLOAT_EQ(cell_2.m_dist, 5.f);
  EXPECT_FLOAT_EQ(cell_2.m_dist_d(0), 3.f);
  EXPECT_FLOAT_EQ(cell_2.m_dist_d(1), 6.f);

  EXPECT_TRUE(cell_2.m_nn_idx < 0);

  DistanceMapCellType cell_3;

  EXPECT_FLOAT_EQ(cell_3.m_dist, std::numeric_limits<ScalarType>::max());
  EXPECT_FLOAT_EQ(cell_3.m_dist_d(0), 0.f);
  EXPECT_FLOAT_EQ(cell_3.m_dist_d(1), 0.f);

  EXPECT_TRUE(cell_3.m_nn_idx < 0);
}  // TEST(data_structures_distance_map_cell, constructor)

TEST(data_structures_distance_map_cell, traits)
{
  using DistanceMapCellType = DistanceMapCellFloat;
  using CellTraits = DistanceMapCellType::Traits;

  DistanceMapCellType cell_1(1.f, -1,
                             DistanceMapCellFloat::VectorType(2.f, 3.f));
  DistanceMapCellType cell_2(5.f, 0,
                             DistanceMapCellFloat::VectorType(3.f, 6.f));

  CellTraits::sumAndScale(cell_1, cell_1, 0.5f);
  CellTraits::sumAndScale(cell_2, cell_2, 0.5f);

  EXPECT_FLOAT_EQ(cell_1.m_dist, 1.5f);
  EXPECT_FLOAT_EQ(cell_1.m_dist_d(0), 3.f);
  EXPECT_FLOAT_EQ(cell_1.m_dist_d(1), 4.5f);
  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  EXPECT_FLOAT_EQ(cell_2.m_dist, 7.5f);
  EXPECT_FLOAT_EQ(cell_2.m_dist_d(0), 4.5f);
  EXPECT_FLOAT_EQ(cell_2.m_dist_d(1), 9.f);
  EXPECT_TRUE(cell_2.m_nn_idx >= 0);

  CellTraits::postInterpolate(cell_1);  // expected to not doing anything...
  CellTraits::postInterpolate(cell_2);

  EXPECT_FLOAT_EQ(cell_1.m_dist, 1.5f);
  EXPECT_FLOAT_EQ(cell_1.m_dist_d(0), 3.f);
  EXPECT_FLOAT_EQ(cell_1.m_dist_d(1), 4.5f);
  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  EXPECT_FLOAT_EQ(cell_2.m_dist, 7.5f);
  EXPECT_FLOAT_EQ(cell_2.m_dist_d(0), 4.5f);
  EXPECT_FLOAT_EQ(cell_2.m_dist_d(1), 9.f);
  EXPECT_TRUE(cell_2.m_nn_idx >= 0);

  CellTraits::setZero(cell_1);
  CellTraits::setZero(cell_2);

  EXPECT_FLOAT_EQ(cell_1.m_dist, 0.f);
  EXPECT_TRUE(cell_1.m_dist_d.isZero());
  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  EXPECT_FLOAT_EQ(cell_2.m_dist, 0.f);
  EXPECT_TRUE(cell_2.m_dist_d.isZero());
  EXPECT_TRUE(cell_2.m_nn_idx >= 0);

}  // TEST(data_structures_distance_map_cell, traits)

TEST(data_structures_distance_map_cell, searchQueue)
{
  DistanceMapCellFloatSearchQueue queue;

  DistanceMapCellFloat cell_1(0.1f);
  DistanceMapCellFloat cell_2(0.2f);
  DistanceMapCellFloat cell_3(0.3f);

  ASSERT_FLOAT_EQ(cell_1.m_dist, 0.1f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_1.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_2.m_dist, 0.2f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_2.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_3.m_dist, 0.3f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_3.m_nn_idx < 0);

  queue.push(cell_1);
  queue.push(cell_2);
  queue.push(cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);

  queue.push(cell_3);
  queue.push(cell_2);
  queue.push(cell_1);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_cell, searchQueue)

TEST(data_structures_distance_map_cell, searchQueueNearLimits)
{
  DistanceMapCellFloatSearchQueue queue;

  DistanceMapCellFloat cell_1(std::numeric_limits<float>::max() - 10.0f);
  DistanceMapCellFloat cell_2(std::numeric_limits<float>::max());
  DistanceMapCellFloat cell_3(std::numeric_limits<float>::max() - 20.0f);

  queue.push(&cell_1);
  queue.push(&cell_2);
  queue.push(&cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_3.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_1.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_2.m_dist);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_cell, searchQueueNearLimits)

TEST(data_structures_distance_map_cell, searchQueueSparse)
{
  DistanceMapSparseCellFloatSearchQueue queue;

  DistanceMapCellFloat cell_1(0.1f);
  DistanceMapCellFloat cell_2(0.2f);
  DistanceMapCellFloat cell_3(0.3f);

  ASSERT_FLOAT_EQ(cell_1.m_dist, 0.1f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_1.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_2.m_dist, 0.2f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_2.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_3.m_dist, 0.3f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_3.m_nn_idx < 0);

  queue.push(&cell_1);
  queue.push(&cell_2);
  queue.push(&cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);

  queue.push(&cell_3);
  queue.push(&cell_2);
  queue.push(&cell_1);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_cell, searchQueueSparse)

TEST(data_structures_distance_map_cell, searchQueueSparseNearLimits)
{
  DistanceMapSparseCellFloatSearchQueue queue;

  DistanceMapCellFloat cell_1(std::numeric_limits<float>::max() - 10.0f);
  DistanceMapCellFloat cell_2(std::numeric_limits<float>::max());
  DistanceMapCellFloat cell_3(std::numeric_limits<float>::max() - 20.0f);

  queue.push(cell_1);
  queue.push(cell_2);
  queue.push(cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_3.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_1.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_2.m_dist);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_cell, searchQueueSparseNearLimits)

TEST(data_structures_distance_map_with_2nd_deriv_cell, constructor)
{
  using DistanceMapCellType = DistanceMapWith2ndDerivCellFloat;
  using ScalarType = DistanceMapCellType::Scalar;
  using VectorType = DistanceMapCellType::VectorType;
  using MatrixType = DistanceMapCellType::MatrixType;

  ScalarType dist_0 = 1.f;
  VectorType dist_d_0(2.f, 3.f);
  MatrixType dist_d2_0(MatrixType::Random());

  DistanceMapCellType cell_1(dist_0, -1, dist_d_0, dist_d2_0);

  EXPECT_FLOAT_EQ(cell_1.m_dist, dist_0);
  EXPECT_TRUE(cell_1.m_dist_d.isApprox(dist_d_0));
  EXPECT_TRUE(cell_1.m_dist_d2.isApprox(dist_d2_0));

  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  DistanceMapCellType cell_2(dist_0, 0, dist_d_0, dist_d2_0);

  EXPECT_FLOAT_EQ(cell_2.m_dist, dist_0);
  EXPECT_TRUE(cell_2.m_dist_d.isApprox(dist_d_0));
  EXPECT_TRUE(cell_2.m_dist_d2.isApprox(dist_d2_0));

  EXPECT_TRUE(cell_2.m_nn_idx >= 0);

  DistanceMapCellType cell_3;

  EXPECT_FLOAT_EQ(cell_3.m_dist, std::numeric_limits<float>::max());
  EXPECT_TRUE(cell_3.m_dist_d.isZero());
  EXPECT_TRUE(cell_3.m_dist_d2.isZero());

  EXPECT_TRUE(cell_3.m_nn_idx < 0);
}  // TEST(data_structures_distance_map_with_2nd_deriv_cell, constructor)

TEST(data_structures_distance_map_with_2nd_deriv_cell, traits)
{
  using DistanceMapCellType = DistanceMapWith2ndDerivCellFloat;
  using ScalarType = DistanceMapCellType::Scalar;
  using VectorType = DistanceMapCellType::VectorType;
  using MatrixType = DistanceMapCellType::MatrixType;
  using CellTraits = DistanceMapCellType::Traits;

  ScalarType dist_0 = 1.f;
  VectorType dist_d_0(2.f, 3.f);
  MatrixType dist_d2_0(MatrixType::Random());

  DistanceMapCellType cell_1(dist_0, -1, dist_d_0, dist_d2_0);
  DistanceMapCellType cell_2(dist_0, 0, dist_d_0, dist_d2_0);

  CellTraits::sumAndScale(cell_1, cell_1, 0.5f);
  CellTraits::sumAndScale(cell_2, cell_2, 0.5f);

  EXPECT_FLOAT_EQ(cell_1.m_dist, dist_0 * 1.5f);
  EXPECT_TRUE(cell_1.m_dist_d.isApprox(dist_d_0 * 1.5f));
  EXPECT_TRUE(cell_1.m_dist_d2.isApprox(dist_d2_0 * 1.5f));
  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  EXPECT_FLOAT_EQ(cell_2.m_dist, dist_0 * 1.5f);
  EXPECT_TRUE(cell_2.m_dist_d.isApprox(dist_d_0 * 1.5f));
  EXPECT_TRUE(cell_2.m_dist_d2.isApprox(dist_d2_0 * 1.5f));
  EXPECT_TRUE(cell_2.m_nn_idx >= 0);

  CellTraits::postInterpolate(cell_1);  // expected to not doing anything...
  CellTraits::postInterpolate(cell_2);

  EXPECT_FLOAT_EQ(cell_1.m_dist, dist_0 * 1.5f);
  EXPECT_TRUE(cell_1.m_dist_d.isApprox(dist_d_0 * 1.5f));
  EXPECT_TRUE(cell_1.m_dist_d2.isApprox(dist_d2_0 * 1.5f));
  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  EXPECT_FLOAT_EQ(cell_2.m_dist, dist_0 * 1.5f);
  EXPECT_TRUE(cell_2.m_dist_d.isApprox(dist_d_0 * 1.5f));
  EXPECT_TRUE(cell_2.m_dist_d2.isApprox(dist_d2_0 * 1.5f));
  EXPECT_TRUE(cell_2.m_nn_idx >= 0);

  CellTraits::setZero(cell_1);
  CellTraits::setZero(cell_2);

  EXPECT_FLOAT_EQ(cell_1.m_dist, 0.f);
  EXPECT_TRUE(cell_1.m_dist_d.isZero());
  EXPECT_TRUE(cell_1.m_dist_d2.isZero());
  EXPECT_TRUE(cell_1.m_nn_idx < 0);

  EXPECT_FLOAT_EQ(cell_2.m_dist, 0.f);
  EXPECT_TRUE(cell_2.m_dist_d.isZero());
  EXPECT_TRUE(cell_2.m_dist_d2.isZero());
  EXPECT_TRUE(cell_2.m_nn_idx >= 0);
}  // TEST(data_structures_distance_map_with_2nd_deriv_cell, traits)

TEST(data_structures_distance_map_with_2nd_deriv_cell, searchQueue)
{
  DistanceMapWith2ndDerivCellFloatSearchQueue queue;

  DistanceMapWith2ndDerivCellFloat cell_1(0.1f);
  DistanceMapWith2ndDerivCellFloat cell_2(0.2f);
  DistanceMapWith2ndDerivCellFloat cell_3(0.3f);

  ASSERT_FLOAT_EQ(cell_1.m_dist, 0.1f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_1.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_2.m_dist, 0.2f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_2.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_3.m_dist, 0.3f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_3.m_nn_idx < 0);

  queue.push(cell_1);
  queue.push(cell_2);
  queue.push(cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);

  queue.push(cell_3);
  queue.push(cell_2);
  queue.push(cell_1);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_with_2nd_deriv_cell, searchQueue)

TEST(data_structures_distance_map_with_2nd_deriv_cell, searchQueueNearLimits)
{
  DistanceMapWith2ndDerivCellFloatSearchQueue queue;

  DistanceMapWith2ndDerivCellFloat cell_1(std::numeric_limits<float>::max() -
                                          10.0f);
  DistanceMapWith2ndDerivCellFloat cell_2(std::numeric_limits<float>::max());
  DistanceMapWith2ndDerivCellFloat cell_3(std::numeric_limits<float>::max() -
                                          20.0f);

  queue.push(&cell_1);
  queue.push(&cell_2);
  queue.push(&cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_3.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_1.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_2.m_dist);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_with_2nd_deriv_cell,
   // searchQueueNearLimits)

TEST(data_structures_distance_map_with_2nd_deriv_cell, searchQueueSparse)
{
  DistanceMapSparseWith2ndDerivCellFloatSearchQueue queue;

  DistanceMapWith2ndDerivCellFloat cell_1(0.1f);
  DistanceMapWith2ndDerivCellFloat cell_2(0.2f);
  DistanceMapWith2ndDerivCellFloat cell_3(0.3f);

  ASSERT_FLOAT_EQ(cell_1.m_dist, 0.1f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_1.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_1.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_2.m_dist, 0.2f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_2.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_2.m_nn_idx < 0);

  ASSERT_FLOAT_EQ(cell_3.m_dist, 0.3f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(0), 0.f);
  ASSERT_FLOAT_EQ(cell_3.m_dist_d(1), 0.f);
  ASSERT_TRUE(cell_3.m_nn_idx < 0);

  queue.push(&cell_1);
  queue.push(&cell_2);
  queue.push(&cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);

  queue.push(&cell_3);
  queue.push(&cell_2);
  queue.push(&cell_1);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.1f);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.2f);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, 0.3f);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_with_2nd_deriv_cell, searchQueueSparse)

TEST(data_structures_distance_map_with_2nd_deriv_cell,
     searchQueueSparseNearLimits)
{
  DistanceMapSparseWith2ndDerivCellFloatSearchQueue queue;

  DistanceMapWith2ndDerivCellFloat cell_1(std::numeric_limits<float>::max() -
                                          10.0f);
  DistanceMapWith2ndDerivCellFloat cell_2(std::numeric_limits<float>::max());
  DistanceMapWith2ndDerivCellFloat cell_3(std::numeric_limits<float>::max() -
                                          20.0f);

  queue.push(cell_1);
  queue.push(cell_2);
  queue.push(cell_3);

  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_3.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_1.m_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell->m_dist, cell_2.m_dist);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_with_2nd_deriv_cell,
   // searchQueueSparseNearLimits)

TEST(data_structures_distance_map_nn_point_idx_cell, constructor)
{
  using DistanceMapCellType = DistanceMapNNPointIdxCell;

  DistanceMapCellType cell_1(3);

  EXPECT_EQ(cell_1.m_nn_idx, 3);
  EXPECT_TRUE(cell_1.m_nn_idx >= 0);

  DistanceMapCellType cell_2;

  EXPECT_EQ(cell_2.m_nn_idx, -1);
  EXPECT_TRUE(cell_2.m_nn_idx < 0);
}  // TEST(data_structures_distance_map_nn_point_idx_cell, constructor)

TEST(data_structures_distance_map_nn_point_idx_cell, traits)
{
  using DistanceMapCellType = DistanceMapNNPointIdxCell;
  using CellTraits = DistanceMapCellType::Traits;

  DistanceMapCellType cell_1(1);
  DistanceMapCellType cell_2(2);

  CellTraits::sumAndScale(cell_1, cell_1,
                          0.5f);  // expected to not doing anything...
  CellTraits::sumAndScale(cell_2, cell_2, 0.5f);

  EXPECT_EQ(cell_1.m_nn_idx, 1);
  EXPECT_EQ(cell_2.m_nn_idx, 2);

  CellTraits::postInterpolate(cell_1);  // expected to not doing anything...
  CellTraits::postInterpolate(cell_2);

  EXPECT_EQ(cell_1.m_nn_idx, 1);
  EXPECT_EQ(cell_2.m_nn_idx, 2);

  CellTraits::setZero(cell_1);  // expected to not doing anything...
  CellTraits::setZero(cell_2);

  EXPECT_EQ(cell_1.m_nn_idx, 1);
  EXPECT_EQ(cell_2.m_nn_idx, 2);

}  // TEST(data_structures_distance_map_nn_point_idx_cell, traits)

TEST(data_structures_distance_map_nn_point_idx_cell, searchQueue)
{
  using DistanceMapCellType = DistanceMapNNPointIdxCell;
  using DistanceMapSearchQueueCellType =
      DistanceMapNNPointIdxCellSearchWithDistFloatQueue;
  using DistanceMapSearchCellType = DistanceMapSearchQueueCellType::value_type;

  constexpr float cell_1_dist = 0.1f;
  constexpr float cell_2_dist = 0.2f;
  constexpr float cell_3_dist = 0.3f;

  DistanceMapNNPointIdxCellSearchWithDistFloatQueue queue;

  DistanceMapCellType cell_1;
  DistanceMapCellType cell_2;
  DistanceMapCellType cell_3;

  queue.push(DistanceMapSearchCellType(&cell_1, cell_1_dist));
  queue.push(DistanceMapSearchCellType(&cell_2, cell_2_dist));
  queue.push(DistanceMapSearchCellType(&cell_3, cell_3_dist));

  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_1_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_2_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_3_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);

  queue.push(DistanceMapSearchCellType(&cell_3, cell_3_dist));
  queue.push(DistanceMapSearchCellType(&cell_2, cell_2_dist));
  queue.push(DistanceMapSearchCellType(&cell_1, cell_1_dist));

  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_1_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_2_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_3_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_nn_point_idx_cell, searchQueue)

TEST(data_structures_distance_map_nn_point_idx_cell, searchQueueNearLimits)
{
  using DistanceMapCellType = DistanceMapNNPointIdxCell;
  using DistanceMapSearchQueueCellType =
      DistanceMapNNPointIdxCellSearchWithDistFloatQueue;
  using DistanceMapSearchCellType = DistanceMapSearchQueueCellType::value_type;

  constexpr float cell_1_dist = std::numeric_limits<float>::max() - 10.0f;
  constexpr float cell_2_dist = std::numeric_limits<float>::max();
  constexpr float cell_3_dist = std::numeric_limits<float>::max() - 20.0f;

  DistanceMapNNPointIdxCellSearchWithDistFloatQueue queue;

  DistanceMapCellType cell_1;
  DistanceMapCellType cell_2;
  DistanceMapCellType cell_3;

  queue.push(DistanceMapSearchCellType(&cell_1, cell_1_dist));
  queue.push(DistanceMapSearchCellType(&cell_2, cell_2_dist));
  queue.push(DistanceMapSearchCellType(&cell_3, cell_3_dist));

  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_3_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_1_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_dist, cell_2_dist);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_nn_point_idx_cell,
   // searchQueueNearLimits)

TEST(data_structures_distance_map_nn_point_idx_cell, searchQueueSparse)
{
  using DistanceMapCellType = DistanceMapNNPointIdxCell;
  using DistanceMapSearchQueueCellType =
      DistanceMapSparseNNPointIdxCellSearchWithDistFloatQueue;
  using DistanceMapSearchCellType = DistanceMapSearchQueueCellType::value_type;

  DistanceMapSearchQueueCellType queue;

  constexpr float cell_1_dist = 0.1f;
  constexpr float cell_2_dist = 0.2f;
  constexpr float cell_3_dist = 0.3f;

  DistanceMapCellType cell_1;
  DistanceMapCellType cell_2;
  DistanceMapCellType cell_3;

  queue.push(DistanceMapSearchCellType(&cell_1, cell_1_dist));
  queue.push(DistanceMapSearchCellType(&cell_2, cell_2_dist));
  queue.push(DistanceMapSearchCellType(&cell_3, cell_3_dist));

  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_1_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_2_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_3_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);

  queue.push(DistanceMapSearchCellType(&cell_3, cell_3_dist));
  queue.push(DistanceMapSearchCellType(&cell_2, cell_2_dist));
  queue.push(DistanceMapSearchCellType(&cell_1, cell_1_dist));

  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_1_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_1);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_2_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_2);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_3_dist);
  EXPECT_TRUE(queue.top().m_cell == &cell_3);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_nn_point_idx_cell, searchQueueSparse)

TEST(data_structures_distance_map_nn_point_idx_cell,
     searchQueueSparseNearLimits)
{
  using DistanceMapCellType = DistanceMapNNPointIdxCell;
  using DistanceMapSearchQueueCellType =
      DistanceMapSparseNNPointIdxCellSearchWithDistFloatQueue;
  using DistanceMapSearchCellType = DistanceMapSearchQueueCellType::value_type;

  constexpr float cell_1_dist = std::numeric_limits<float>::max() - 10.0f;
  constexpr float cell_2_dist = std::numeric_limits<float>::max();
  constexpr float cell_3_dist = std::numeric_limits<float>::max() - 20.0f;

  DistanceMapSearchQueueCellType queue;

  DistanceMapCellType cell_1;
  DistanceMapCellType cell_2;
  DistanceMapCellType cell_3;

  queue.push(DistanceMapSearchCellType(&cell_1, cell_1_dist));
  queue.push(DistanceMapSearchCellType(&cell_2, cell_2_dist));
  queue.push(DistanceMapSearchCellType(&cell_3, cell_3_dist));

  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_3_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_1_dist);
  queue.pop();
  EXPECT_FLOAT_EQ(queue.top().m_cell_dist, cell_2_dist);
  queue.pop();

  ASSERT_EQ(queue.size(), 0);
}  // TEST(data_structures_distance_map_nn_point_idx_cell,
   // searchQueueSparseNearLimits)

}  // namespace testing
