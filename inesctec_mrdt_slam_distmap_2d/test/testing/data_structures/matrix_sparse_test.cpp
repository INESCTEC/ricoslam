#include <gtest/gtest.h>

#include <bitset>
#include <inesctec_mrdt_slam_distmap_2d/data_structures/matrix_sparse.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_matrix_sparse, MatrixSparseIndex)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      MatrixSparseIndexInt idx(r, c);
      EXPECT_EQ(idx.m_row, r);
      EXPECT_EQ(idx.m_col, c);
      EXPECT_TRUE(idx == idx);
    }
  }
}  // TEST(data_structures_matrix_sparse, MatrixSparseIndex)

TEST(data_structures_matrix_sparse, MatrixSparseHash)
{
  constexpr int rows = 50;
  constexpr int cols = 50;

  MatrixSparseHashInt hash_int;
  MatrixSparseHashUInt hash_uint;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      size_t cell_hash_int = hash_int(MatrixSparseIndexInt(r, c));
      size_t cell_hash_uint = hash_uint(MatrixSparseIndexUInt(r, c));

      for (int r1 = 0; r1 < rows; r1++)
      {
        for (int c1 = 0; c1 < cols; c1++)
        {
          if ((r1 == r) && (c1 == c))
          {
            continue;
          }
          EXPECT_FALSE(cell_hash_int == hash_int(MatrixSparseIndexInt(r1, c1)));
          EXPECT_FALSE(cell_hash_uint ==
                       hash_uint(MatrixSparseIndexUInt(r1, c1)));
        }
      }
    }
  }
}  // TEST(data_structures_matrix_sparse, MatrixSparseHash)

TEST(data_structures_matrix_sparse, MatrixSparseHashInteger)
{
  constexpr int rows = 50;
  constexpr int cols = 50;

  MatrixSparseHashInt hash_int;

  for (int r = -rows; r < rows; r++)
  {
    for (int c = -cols; c < cols; c++)
    {
      size_t cell_hash_int = hash_int(MatrixSparseIndexInt(r, c));

      for (int r1 = -rows; r1 < rows; r1++)
      {
        for (int c1 = -cols; c1 < cols; c1++)
        {
          if ((r1 == r) && (c1 == c))
          {
            continue;
          }
          EXPECT_FALSE(cell_hash_int == hash_int(MatrixSparseIndexInt(r1, c1)));
        }
      }
    }
  }
}  // TEST(data_structures_matrix_sparse, MatrixSparseHashInteger

TEST(data_structures_matrix_sparse, constructor)
{
  constexpr MatrixSparseFloat::IndexScalar rows = 10;
  constexpr MatrixSparseFloat::IndexScalar cols = 20;

  MatrixSparseFloat mat_1;

  EXPECT_EQ(mat_1.rows(), 0);
  EXPECT_EQ(mat_1.cols(), 0);
  EXPECT_FLOAT_EQ(mat_1.defaultValue(), 0.0f);
  EXPECT_EQ(mat_1.totalSize(), 0);
  EXPECT_EQ(mat_1.nonZeros(), 0);

  MatrixSparseDouble mat_2(rows, cols);

  EXPECT_EQ(mat_2.rows(), rows);
  EXPECT_EQ(mat_2.cols(), cols);
  EXPECT_FLOAT_EQ(mat_2.defaultValue(), 0);
  EXPECT_EQ(mat_2.totalSize(), rows * cols);
  EXPECT_EQ(mat_2.nonZeros(), 0);

  MatrixSparseDouble mat_3(rows, cols, 10);

  EXPECT_EQ(mat_3.rows(), rows);
  EXPECT_EQ(mat_3.cols(), cols);
  EXPECT_FLOAT_EQ(mat_3.defaultValue(), 10);
  EXPECT_EQ(mat_3.totalSize(), rows * cols);
  EXPECT_EQ(mat_3.nonZeros(), rows * cols);

  MatrixSparseDouble mat_4(mat_3);

  EXPECT_EQ(mat_3.rows(), mat_4.rows());
  EXPECT_EQ(mat_3.cols(), mat_4.cols());
  EXPECT_FLOAT_EQ(mat_3.defaultValue(), mat_4.defaultValue());
  EXPECT_EQ(mat_3.totalSize(), mat_4.totalSize());
  EXPECT_EQ(mat_3.nonZeros(), mat_4.nonZeros());
}  // TEST(data_structures_matrix_sparse, constructor)

TEST(data_structures_matrix_sparse, isOnBorder)
{
  constexpr MatrixSparseFloat::IndexScalar rows = 30;
  constexpr MatrixSparseFloat::IndexScalar cols = 50;

  MatrixSparseFloat mat(rows, cols);

  ASSERT_EQ(mat.rows(), rows);
  ASSERT_EQ(mat.cols(), cols);

  for (int r = 0; r < mat.rows(); r++)
  {
    EXPECT_TRUE(mat.isOnBorder(r, 0));
    EXPECT_TRUE(mat.isOnBorder(Eigen::Vector2i(r, 0)));
    EXPECT_TRUE(mat.isOnBorder(MatrixSparseFloat::Index(r, 0)));

    EXPECT_TRUE(mat.isOnBorder(r, cols - 1));
    EXPECT_TRUE(mat.isOnBorder(Eigen::Vector2i(r, cols - 1)));
    EXPECT_TRUE(mat.isOnBorder(MatrixSparseFloat::Index(r, cols - 1)));

    for (int c = 0; c < mat.cols(); c++)
    {
      EXPECT_TRUE(mat.isOnBorder(0, c));
      EXPECT_TRUE(mat.isOnBorder(Eigen::Vector2i(0, c)));
      EXPECT_TRUE(mat.isOnBorder(MatrixSparseFloat::Index(0, c)));

      EXPECT_TRUE(mat.isOnBorder(rows - 1, c));
      EXPECT_TRUE(mat.isOnBorder(Eigen::Vector2i(rows - 1, c)));
      EXPECT_TRUE(mat.isOnBorder(MatrixSparseFloat::Index(rows - 1, c)));

      if ((r < rows - 1) && (r > 0) && (c < cols - 1) && (c > 0))
      {
        EXPECT_FALSE(mat.isOnBorder(r, c));
        EXPECT_FALSE(mat.isOnBorder(Eigen::Vector2i(r, c)));
        EXPECT_FALSE(mat.isOnBorder(MatrixSparseFloat::Index(r, c)));
      }
    }
  }

  EXPECT_FALSE(mat.isOnBorder(-1, -1));
  EXPECT_FALSE(mat.isOnBorder(Eigen::Vector2i(-1, -1)));
  EXPECT_FALSE(mat.isOnBorder(MatrixSparseFloat::Index(-1, -1)));

  EXPECT_FALSE(mat.isOnBorder(rows, cols));
  EXPECT_FALSE(mat.isOnBorder(Eigen::Vector2i(rows, cols)));
  EXPECT_FALSE(mat.isOnBorder(MatrixSparseFloat::Index(rows, cols)));
}  // TEST(data_structures_matrix_sparse, isOnBorder)

TEST(data_structures_matrix_sparse, isInside)
{
  constexpr MatrixSparseFloat::IndexScalar rows = 30;
  constexpr MatrixSparseFloat::IndexScalar cols = 50;

  MatrixSparseFloat mat(rows, cols);

  ASSERT_EQ(mat.rows(), rows);
  ASSERT_EQ(mat.cols(), cols);

  for (int r = 0; r < mat.rows(); r++)
  {
    for (int c = 0; c < mat.cols(); c++)
    {
      EXPECT_TRUE(mat.isInside(r, c));
      EXPECT_TRUE(mat.isInside(Eigen::Vector2i(r, c)));
      EXPECT_TRUE(mat.isInside(MatrixSparseFloat::Index(r, c)));
    }
  }

  EXPECT_FALSE(mat.isInside(-1, -1));
  EXPECT_FALSE(mat.isInside(Eigen::Vector2i(-1, -1)));
  EXPECT_FALSE(mat.isInside(MatrixSparseFloat::Index(-1, -1)));

  EXPECT_FALSE(mat.isInside(rows, cols));
  EXPECT_FALSE(mat.isInside(Eigen::Vector2i(rows, cols)));
  EXPECT_FALSE(mat.isInside(MatrixSparseFloat::Index(rows, cols)));
}  // TEST(data_structures_matrix_sparse, isInside)

TEST(data_structures_matrix_sparse, coeff)
{
  constexpr MatrixSparseFloat::IndexScalar rows = 10;
  constexpr MatrixSparseFloat::IndexScalar cols = 20;

  MatrixSparseFloat mat(rows, cols);

  for (MatrixSparseFloat::IndexScalar r = 0; r < mat.rows(); r++)
  {
    for (MatrixSparseFloat::IndexScalar c = 0; c < mat.cols(); c++)
    {
      EXPECT_FLOAT_EQ(mat.coeff(r, c), 0.f);
      EXPECT_FLOAT_EQ(mat.coeff(r, c), mat.defaultValue());
    }
  }

  ASSERT_EQ(mat.size(), 0);
  ASSERT_EQ(mat.nonZeros(), 0);
  ASSERT_EQ(mat.totalSize(), rows * cols);

  mat[MatrixSparseFloat::Index(5, 5)] = 10.f;

  EXPECT_EQ(mat.size(), 1);
  EXPECT_EQ(mat.nonZeros(), 1);
  EXPECT_EQ(mat.totalSize(), rows * cols);

  for (MatrixSparseFloat::IndexScalar r = 0; r < mat.rows(); r++)
  {
    for (MatrixSparseFloat::IndexScalar c = 0; c < mat.cols(); c++)
    {
      if ((r == 5) && (c == 5))
      {
        EXPECT_FLOAT_EQ(mat.coeff(r, c), 10.f);
        EXPECT_FALSE(mat.coeff(r, c) == mat.defaultValue());
      }
      else
      {
        EXPECT_FLOAT_EQ(mat.coeff(r, c), 0.f);
        EXPECT_FLOAT_EQ(mat.coeff(r, c), mat.defaultValue());
      }
    }
  }

  mat.coeffRef(MatrixSparseFloat::Index(5, 5)) = 20.f;

  EXPECT_EQ(mat.size(), 1);
  EXPECT_EQ(mat.nonZeros(), 1);
  EXPECT_EQ(mat.totalSize(), rows * cols);

  EXPECT_FLOAT_EQ(mat.coeff(5, 5), 20.f);
  EXPECT_FLOAT_EQ(mat.coeff(MatrixSparseFloat::Index(5, 5)), 20.f);
  EXPECT_FLOAT_EQ(mat.coeff(Eigen::Vector2i(5, 5)), 20.f);

  MatrixSparseFloat test_mat(mat);

  EXPECT_EQ(test_mat.rows(), mat.rows());
  EXPECT_EQ(test_mat.cols(), mat.cols());
  EXPECT_FLOAT_EQ(test_mat.defaultValue(), mat.defaultValue());
  EXPECT_EQ(test_mat.size(), mat.size());
  EXPECT_EQ(test_mat.nonZeros(), mat.nonZeros());
  EXPECT_EQ(test_mat.totalSize(), mat.totalSize());
}  // TEST(data_structures_matrix_sparse, coeff)

TEST(data_structures_matrix_sparse, coeffRef)
{
  using Index = MatrixSparseFloat::Index;
  using IndexScalar = MatrixSparseFloat::IndexScalar;

  constexpr IndexScalar rows = 10;
  constexpr IndexScalar cols = 20;

  MatrixSparseFloat mat(rows, cols);

  float value = 0.f;

  for (IndexScalar r = 0; r < mat.rows(); r++)
  {
    for (IndexScalar c = 0; c < mat.cols(); c++)
    {
      EXPECT_NO_THROW(mat.coeff(r, c));
      EXPECT_FLOAT_EQ(mat.coeff(r, c), 0.f);
      EXPECT_FLOAT_EQ(mat.coeff(r, c), mat.defaultValue());

      size_t prev_size = mat.size();

      // now, by calling the coeff by reference, the element will be auto added
      mat.coeffRef(r, c) = value;

      EXPECT_EQ(mat.size(), prev_size + 1);

      EXPECT_FLOAT_EQ(mat.coeff(r, c), value);
      EXPECT_FLOAT_EQ(mat.coeff(Eigen::Vector2i(r, c)), value);
      EXPECT_FLOAT_EQ(mat.coeff(Index(r, c)), value);

      EXPECT_FLOAT_EQ(mat.coeffRef(r, c), value);
      EXPECT_FLOAT_EQ(mat.coeffRef(Eigen::Vector2i(r, c)), value);
      EXPECT_FLOAT_EQ(mat.coeffRef(Index(r, c)), value);

      EXPECT_EQ(mat.size(), prev_size + 1);

      ++value;
    }
  }

  EXPECT_EQ(mat.totalSize(), rows * cols);
  EXPECT_EQ(mat.size(), mat.totalSize());
  EXPECT_EQ(mat.nonZeros(), mat.totalSize());

  EXPECT_ANY_THROW(mat.coeffRef(rows, cols));
  EXPECT_ANY_THROW(mat.coeffRef(0, cols));
  EXPECT_ANY_THROW(
      mat.coeffRef(rows, 0));  // note -1 not valid in uint as IndexScalar

  MatrixSparseFloat test_mat(mat);

  EXPECT_EQ(test_mat.rows(), mat.rows());
  EXPECT_EQ(test_mat.cols(), mat.cols());
  EXPECT_FLOAT_EQ(test_mat.defaultValue(), mat.defaultValue());
  EXPECT_EQ(test_mat.size(), mat.size());
  EXPECT_EQ(test_mat.nonZeros(), mat.nonZeros());
  EXPECT_EQ(test_mat.totalSize(), mat.totalSize());
}  // TEST(data_structures_matrix_sparse, coeffRef)

TEST(data_structures_matrix_sparse, fill)
{
  // using Index       = MatrixSparseDouble::Index;
  using IndexScalar = MatrixSparseDouble::IndexScalar;

  constexpr IndexScalar rows = 10;
  constexpr IndexScalar cols = 20;

  MatrixSparseInt mat_int_i(rows, cols);
  MatrixSparseFloat mat_float_i(rows, cols);
  MatrixSparseDouble mat_double_i(rows, cols);

  ASSERT_EQ(mat_int_i.rows(), rows);
  ASSERT_EQ(mat_float_i.rows(), rows);
  ASSERT_EQ(mat_double_i.rows(), rows);
  ASSERT_EQ(mat_int_i.cols(), cols);
  ASSERT_EQ(mat_float_i.cols(), cols);
  ASSERT_EQ(mat_double_i.cols(), cols);

  mat_int_i.fill(5);
  mat_float_i.fill(6.7f);
  mat_double_i.fill(9.99);

  EXPECT_TRUE(mat_int_i.defaultValue() == 5);
  EXPECT_TRUE(mat_float_i.defaultValue() == 6.7f);
  EXPECT_TRUE(mat_double_i.defaultValue() == 9.99);

  EXPECT_EQ(mat_int_i.size(), 0);
  EXPECT_EQ(mat_float_i.size(), 0);
  EXPECT_EQ(mat_double_i.size(), 0);

  EXPECT_EQ(mat_int_i.nonDefaultValues(), 0);
  EXPECT_EQ(mat_float_i.nonDefaultValues(), 0);
  EXPECT_EQ(mat_double_i.nonDefaultValues(), 0);

  EXPECT_EQ(mat_int_i.nonZeros(), rows * cols);
  EXPECT_EQ(mat_float_i.nonZeros(), rows * cols);
  EXPECT_EQ(mat_double_i.nonZeros(), rows * cols);

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      EXPECT_TRUE(mat_int_i.coeff(r, c) == 5);
      EXPECT_TRUE(mat_float_i.coeff(r, c) == 6.7f);
      EXPECT_TRUE(mat_double_i.coeff(r, c) == 9.99);
    }
  }
}  // TEST(data_structures_matrix_sparse, fill)

TEST(data_structures_matrix_sparse, fillStruct)
{
  struct MatrixCell
  {
    float m_distance = 0;
    float m_weight = 0;

    MatrixCell() : m_distance(0), m_weight(0) {}
    MatrixCell(float init) : m_distance(init), m_weight(init) {}
    MatrixCell(float dist, float w) : m_distance(dist), m_weight(w) {}

    bool operator==(const MatrixCell& other) const
    {
      return ((this->m_distance == other.m_distance) &&
              (this->m_weight == other.m_weight));
    }
  };  // struct MatrixCell

  // using Index       = MatrixSparse_<MatrixCell>::Index;
  using IndexScalar = MatrixSparse_<MatrixCell>::IndexScalar;

  constexpr IndexScalar rows = 50;
  constexpr IndexScalar cols = 30;

  MatrixSparse_<MatrixCell> mat_struct(rows, cols);

  ASSERT_EQ(mat_struct.rows(), rows);
  ASSERT_EQ(mat_struct.cols(), cols);

  mat_struct.fill(MatrixCell(0.25f,  //!< m_distance
                             0.50f   //!< m_weight
                             ));

  MatrixCell default_cell = mat_struct.defaultValue();

  ASSERT_FLOAT_EQ(default_cell.m_distance, 0.25f);
  ASSERT_FLOAT_EQ(default_cell.m_weight, 0.50f);

  for (int r = 0; r < mat_struct.rows(); r++)
  {
    for (int c = 0; c < mat_struct.cols(); c++)
    {
      EXPECT_FLOAT_EQ(mat_struct.coeff(r, c).m_distance, 0.25f);
      EXPECT_FLOAT_EQ(mat_struct.coeff(r, c).m_weight, 0.50f);
    }
  }

  EXPECT_TRUE(mat_struct.defaultValue() == default_cell);
  EXPECT_EQ(mat_struct.size(), 0);
  EXPECT_EQ(mat_struct.nonDefaultValues(), 0);
  EXPECT_EQ(mat_struct.nonZeros(), rows * cols);
}  // TEST(data_structures_matrix_sparse, fillStruct)

TEST(data_structures_matrix_sparse, getSubPx)
{
  float interpolated_value_f;
  double interpolated_value_d;

  MatrixSparseFloat mat_float(3, 3);
  MatrixSparseDouble mat_double(3, 3);

  mat_float.coeffRef(0, 2) = 3;
  mat_float.coeffRef(1, 2) = 3;
  mat_float.coeffRef(2, 2) = 3;
  mat_float.coeffRef(0, 1) = 1;
  mat_float.coeffRef(1, 1) = 1;
  mat_float.coeffRef(2, 1) = 1;
  mat_float.coeffRef(0, 0) = -1;
  mat_float.coeffRef(1, 0) = -1;
  mat_float.coeffRef(2, 0) = -1;

  mat_double.coeffRef(0, 2) = 3;
  mat_double.coeffRef(1, 2) = 3;
  mat_double.coeffRef(2, 2) = 3;
  mat_double.coeffRef(0, 1) = 1;
  mat_double.coeffRef(1, 1) = 1;
  mat_double.coeffRef(2, 1) = 1;
  mat_double.coeffRef(0, 0) = -1;
  mat_double.coeffRef(1, 0) = -1;
  mat_double.coeffRef(2, 0) = -1;

  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.50));
  EXPECT_NEAR(interpolated_value_f, 0.00f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.00, 0.50));
  EXPECT_NEAR(interpolated_value_f, 0.00f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.25, 0.50));
  EXPECT_NEAR(interpolated_value_f, 0.00f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.75, 0.50));
  EXPECT_NEAR(interpolated_value_f, 0.00f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(1.00, 0.50));
  EXPECT_NEAR(interpolated_value_f, 0.00f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.00));
  EXPECT_NEAR(interpolated_value_f, -1.0f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.25));
  EXPECT_NEAR(interpolated_value_f, -0.5f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.75));
  EXPECT_NEAR(interpolated_value_f, 0.5f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 1.00));
  EXPECT_NEAR(interpolated_value_f, 1.0f, 1e-6);

  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.50));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.00, 0.50));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.25, 0.50));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.75, 0.50));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(1.00, 0.50));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.00));
  EXPECT_NEAR(interpolated_value_d, -1.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.25));
  EXPECT_NEAR(interpolated_value_d, -0.5, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.75));
  EXPECT_NEAR(interpolated_value_d, 0.5, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 1.00));
  EXPECT_NEAR(interpolated_value_d, 1.0, 1e-9);

  mat_float.coeffRef(0, 2) = -1;
  mat_float.coeffRef(1, 2) = 1;
  mat_float.coeffRef(2, 2) = 3;
  mat_float.coeffRef(0, 1) = -1;
  mat_float.coeffRef(1, 1) = 1;
  mat_float.coeffRef(2, 1) = 3;
  mat_float.coeffRef(0, 0) = -1;
  mat_float.coeffRef(1, 0) = 1;
  mat_float.coeffRef(2, 0) = -3;

  mat_double.coeffRef(0, 2) = -1;
  mat_double.coeffRef(1, 2) = 1;
  mat_double.coeffRef(2, 2) = 3;
  mat_double.coeffRef(0, 1) = -1;
  mat_double.coeffRef(1, 1) = 1;
  mat_double.coeffRef(2, 1) = 3;
  mat_double.coeffRef(0, 0) = -1;
  mat_double.coeffRef(1, 0) = 1;
  mat_double.coeffRef(2, 0) = 3;

  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.50));
  EXPECT_NEAR(interpolated_value_f, 0.0f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.00));
  EXPECT_NEAR(interpolated_value_f, 0.0f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.25));
  EXPECT_NEAR(interpolated_value_f, 0.0f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 0.75));
  EXPECT_NEAR(interpolated_value_f, 0.0f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.50, 1.00));
  EXPECT_NEAR(interpolated_value_f, 0.0f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.00, 0.50));
  EXPECT_NEAR(interpolated_value_f, -1.0f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.25, 0.50));
  EXPECT_NEAR(interpolated_value_f, -0.5f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(0.75, 0.50));
  EXPECT_NEAR(interpolated_value_f, 0.5f, 1e-6);
  mat_float.getSubPxValue(interpolated_value_f, Eigen::Vector2f(1.00, 0.50));
  EXPECT_NEAR(interpolated_value_f, 1.0f, 1e-6);

  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.50));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.00));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.25));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 0.75));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.50, 1.00));
  EXPECT_NEAR(interpolated_value_d, 0.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.00, 0.50));
  EXPECT_NEAR(interpolated_value_d, -1.0, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.25, 0.50));
  EXPECT_NEAR(interpolated_value_d, -0.5, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(0.75, 0.50));
  EXPECT_NEAR(interpolated_value_d, 0.5, 1e-9);
  mat_double.getSubPxValue(interpolated_value_d, Eigen::Vector2f(1.00, 0.50));
  EXPECT_NEAR(interpolated_value_d, 1.0, 1e-9);

  for (int idx_row = 0; idx_row < 3; idx_row++)
  {
    for (int idx_col = 0; idx_col < 3; idx_col++)
    {
      EXPECT_FALSE(mat_double.getSubPxValue(interpolated_value_d,
                                            Eigen::Vector2f(2, idx_col)));
    }
    EXPECT_FALSE(mat_double.getSubPxValue(interpolated_value_d,
                                          Eigen::Vector2f(idx_row, 2)));
  }
}  // TEST(data_structures_matrix_sparse, getSubPx)

TEST(data_structures_matrix_sparse, resize)
{
  using Index = MatrixSparseFloat::Index;
  using IndexScalar = MatrixSparseFloat::IndexScalar;

  constexpr IndexScalar rows = 100;
  constexpr IndexScalar cols = 200;

  MatrixSparseFloat mat;

  EXPECT_EQ(mat.rows(), 0);
  EXPECT_EQ(mat.cols(), 0);
  EXPECT_FLOAT_EQ(mat.defaultValue(), 0.0f);
  EXPECT_EQ(mat.totalSize(), 0);
  EXPECT_EQ(mat.nonZeros(), 0);

  mat.resize(rows, cols);

  EXPECT_EQ(mat.rows(), rows);
  EXPECT_EQ(mat.cols(), cols);
  EXPECT_FLOAT_EQ(mat.defaultValue(), 0.0f);
  EXPECT_EQ(mat.totalSize(), rows * cols);
  EXPECT_EQ(mat.nonZeros(), 0);
  EXPECT_EQ(mat.size(), 0);

  mat.coeffRef(Index(5, 8)) = -1.f;
  mat.coeffRef(Index(rows - 1, cols - 1)) = 1.f;
  mat.coeffRef(Index(rows - 2, cols - 1)) = 1.f;

  EXPECT_EQ(mat.rows(), rows);
  EXPECT_EQ(mat.cols(), cols);
  EXPECT_FLOAT_EQ(mat.defaultValue(), 0.0f);
  EXPECT_EQ(mat.totalSize(), rows * cols);
  EXPECT_EQ(mat.nonZeros(), 3);
  EXPECT_EQ(mat.size(), 3);

  mat.resize(rows / 2, cols / 2);

  EXPECT_EQ(mat.rows(), rows / 2);
  EXPECT_EQ(mat.cols(), cols / 2);
  EXPECT_FLOAT_EQ(mat.defaultValue(), 0.0f);
  EXPECT_EQ(mat.totalSize(), rows * cols / 4);
  EXPECT_EQ(mat.nonZeros(), 1);
  EXPECT_EQ(mat.size(), 1);
}  // TEST(data_structures_matrix_sparse, resize)

}  // namespace testing
