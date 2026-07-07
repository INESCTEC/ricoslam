#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/matrix_sparse_map.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_matrix_sparse_map, constructor)
{
  constexpr MatrixSparseMapIndex::Scalar rows = 10;
  constexpr MatrixSparseMapIndex::Scalar cols = 20;

  MatrixSparseMapFloat mat_1;

  EXPECT_EQ(mat_1.rows(), 1);  // to avoid having a boolean with matrix init...
  EXPECT_EQ(mat_1.cols(), 1);  // (and handling it that shiet...)
  EXPECT_FLOAT_EQ(mat_1.defaultValue(), 0.0f);
  EXPECT_EQ(mat_1.totalSize(), 1);
  EXPECT_EQ(mat_1.nonZeros(), 0);
  EXPECT_TRUE(mat_1.limInf() == MatrixSparseMapIndex(0, 0));
  EXPECT_TRUE(mat_1.limSup() == MatrixSparseMapIndex(0, 0));
  EXPECT_EQ(mat_1.limInfRows(), 0);
  EXPECT_EQ(mat_1.limInfCols(), 0);
  EXPECT_EQ(mat_1.limSupRows(), 0);
  EXPECT_EQ(mat_1.limSupCols(), 0);

  MatrixSparseMapFloat mat_2(rows, cols);

  EXPECT_EQ(mat_2.rows(), rows);
  EXPECT_EQ(mat_2.cols(), cols);
  EXPECT_FLOAT_EQ(mat_2.defaultValue(), 0);
  EXPECT_EQ(mat_2.totalSize(), rows * cols);
  EXPECT_EQ(mat_2.nonZeros(), 0);
  EXPECT_TRUE(mat_2.limInf() == MatrixSparseMapIndex(0, 0));
  EXPECT_TRUE(mat_2.limSup() == MatrixSparseMapIndex(rows - 1, cols - 1));
  EXPECT_EQ(mat_2.limInfRows(), 0);
  EXPECT_EQ(mat_2.limInfCols(), 0);
  EXPECT_EQ(mat_2.limSupRows(), rows - 1);
  EXPECT_EQ(mat_2.limSupCols(), cols - 1);

  MatrixSparseMapFloat mat_3(rows, cols, 10);

  EXPECT_EQ(mat_3.rows(), rows);
  EXPECT_EQ(mat_3.cols(), cols);
  EXPECT_FLOAT_EQ(mat_3.defaultValue(), 10);
  EXPECT_EQ(mat_3.totalSize(), rows * cols);
  EXPECT_EQ(mat_3.nonZeros(), rows * cols);
  EXPECT_TRUE(mat_3.limInf() == MatrixSparseMapIndex(0, 0));
  EXPECT_TRUE(mat_3.limSup() == MatrixSparseMapIndex(rows - 1, cols - 1));
  EXPECT_EQ(mat_3.limInfRows(), 0);
  EXPECT_EQ(mat_3.limInfCols(), 0);
  EXPECT_EQ(mat_3.limSupRows(), rows - 1);
  EXPECT_EQ(mat_3.limSupCols(), cols - 1);

  MatrixSparseMapFloat mat_4(mat_3);

  EXPECT_EQ(mat_3.rows(), mat_4.rows());
  EXPECT_EQ(mat_3.cols(), mat_4.cols());
  EXPECT_FLOAT_EQ(mat_3.defaultValue(), mat_4.defaultValue());
  EXPECT_EQ(mat_3.totalSize(), mat_4.totalSize());
  EXPECT_EQ(mat_3.nonZeros(), mat_4.nonZeros());
  EXPECT_TRUE(mat_3.limInf() == mat_4.limInf());
  EXPECT_TRUE(mat_3.limSup() == mat_4.limSup());
  EXPECT_EQ(mat_3.limInfRows(), mat_4.limInfRows());
  EXPECT_EQ(mat_3.limInfCols(), mat_4.limInfCols());
  EXPECT_EQ(mat_3.limSupRows(), mat_4.limSupRows());
  EXPECT_EQ(mat_3.limSupCols(), mat_4.limSupCols());

  MatrixSparseMapDouble mat_5(MatrixSparseMapIndex(-rows, -cols),
                              MatrixSparseMapIndex(rows, cols));

  EXPECT_EQ(mat_5.rows(), rows * 2 + 1);
  EXPECT_EQ(mat_5.cols(), cols * 2 + 1);
  EXPECT_FLOAT_EQ(mat_5.defaultValue(), 0);
  EXPECT_EQ(mat_5.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_EQ(mat_5.nonZeros(), 0);
  EXPECT_TRUE(mat_5.limInf() == MatrixSparseMapIndex(-rows, -cols));
  EXPECT_TRUE(mat_5.limSup() == MatrixSparseMapIndex(rows, cols));
  EXPECT_EQ(mat_5.limInfRows(), -rows);
  EXPECT_EQ(mat_5.limInfCols(), -cols);
  EXPECT_EQ(mat_5.limSupRows(), rows);
  EXPECT_EQ(mat_5.limSupCols(), cols);

  MatrixSparseMapDouble mat_6(MatrixSparseMapIndex(-rows, -cols),
                              MatrixSparseMapIndex(rows, cols), 10);

  EXPECT_EQ(mat_6.rows(), rows * 2 + 1);
  EXPECT_EQ(mat_6.cols(), cols * 2 + 1);
  EXPECT_FLOAT_EQ(mat_6.defaultValue(), 10);
  EXPECT_EQ(mat_6.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_EQ(mat_6.nonZeros(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_TRUE(mat_6.limInf() == MatrixSparseMapIndex(-rows, -cols));
  EXPECT_TRUE(mat_6.limSup() == MatrixSparseMapIndex(rows, cols));
  EXPECT_EQ(mat_6.limInfRows(), -rows);
  EXPECT_EQ(mat_6.limInfCols(), -cols);
  EXPECT_EQ(mat_6.limSupRows(), rows);
  EXPECT_EQ(mat_6.limSupCols(), cols);

  MatrixSparseMapDouble mat_7(mat_6);

  EXPECT_EQ(mat_6.rows(), mat_7.rows());
  EXPECT_EQ(mat_6.cols(), mat_7.cols());
  EXPECT_FLOAT_EQ(mat_6.defaultValue(), mat_7.defaultValue());
  EXPECT_EQ(mat_6.totalSize(), mat_7.totalSize());
  EXPECT_EQ(mat_6.nonZeros(), mat_7.nonZeros());
  EXPECT_TRUE(mat_6.limInf() == mat_7.limInf());
  EXPECT_TRUE(mat_6.limSup() == mat_7.limSup());
  EXPECT_EQ(mat_6.limInfRows(), mat_7.limInfRows());
  EXPECT_EQ(mat_6.limInfCols(), mat_7.limInfCols());
  EXPECT_EQ(mat_6.limSupRows(), mat_7.limSupRows());
  EXPECT_EQ(mat_6.limSupCols(), mat_7.limSupCols());
}  // TEST(data_structures_matrix_sparse_map, constructor)

TEST(data_structures_matrix_sparse_map, isOnBorder)
{
  using Index = MatrixSparseMapFloat::Index;

  constexpr Index::Scalar rows = 30;
  constexpr Index::Scalar cols = 50;

  MatrixSparseMapFloat mat_1(rows, cols);

  ASSERT_EQ(mat_1.rows(), rows);
  ASSERT_EQ(mat_1.cols(), cols);

  for (int r = 0; r < mat_1.rows(); r++)
  {
    EXPECT_TRUE(mat_1.isOnBorder(r, 0));
    EXPECT_TRUE(mat_1.isOnBorder(Eigen::Vector2i(r, 0)));
    EXPECT_TRUE(mat_1.isOnBorder(Index(r, 0)));

    EXPECT_TRUE(mat_1.isOnBorder(r, cols - 1));
    EXPECT_TRUE(mat_1.isOnBorder(Eigen::Vector2i(r, cols - 1)));
    EXPECT_TRUE(mat_1.isOnBorder(Index(r, cols - 1)));

    for (int c = 0; c < mat_1.cols(); c++)
    {
      EXPECT_TRUE(mat_1.isOnBorder(0, c));
      EXPECT_TRUE(mat_1.isOnBorder(Eigen::Vector2i(0, c)));
      EXPECT_TRUE(mat_1.isOnBorder(Index(0, c)));

      EXPECT_TRUE(mat_1.isOnBorder(rows - 1, c));
      EXPECT_TRUE(mat_1.isOnBorder(Eigen::Vector2i(rows - 1, c)));
      EXPECT_TRUE(mat_1.isOnBorder(Index(rows - 1, c)));

      if ((r < rows - 1) && (r > 0) && (c < cols - 1) && (c > 0))
      {
        EXPECT_FALSE(mat_1.isOnBorder(r, c));
        EXPECT_FALSE(mat_1.isOnBorder(Eigen::Vector2i(r, c)));
        EXPECT_FALSE(mat_1.isOnBorder(Index(r, c)));
      }
    }
  }

  EXPECT_FALSE(mat_1.isOnBorder(-1, -1));
  EXPECT_FALSE(mat_1.isOnBorder(Eigen::Vector2i(-1, -1)));
  EXPECT_FALSE(mat_1.isOnBorder(Index(-1, -1)));

  EXPECT_FALSE(mat_1.isOnBorder(rows, cols));
  EXPECT_FALSE(mat_1.isOnBorder(Eigen::Vector2i(rows, cols)));
  EXPECT_FALSE(mat_1.isOnBorder(Index(rows, cols)));

  MatrixSparseMapFloat mat_2(Index(-rows, -cols), Index(rows, cols));

  ASSERT_EQ(mat_2.rows(), rows * 2 + 1);
  ASSERT_EQ(mat_2.cols(), cols * 2 + 1);

  for (int r = -rows; r <= rows; r++)
  {
    EXPECT_TRUE(mat_2.isOnBorder(r, -cols));
    EXPECT_TRUE(mat_2.isOnBorder(Eigen::Vector2i(r, -cols)));
    EXPECT_TRUE(mat_2.isOnBorder(Index(r, -cols)));

    EXPECT_TRUE(mat_2.isOnBorder(r, cols));
    EXPECT_TRUE(mat_2.isOnBorder(Eigen::Vector2i(r, cols)));
    EXPECT_TRUE(mat_2.isOnBorder(Index(r, cols)));

    for (int c = -cols; c <= cols; c++)
    {
      EXPECT_TRUE(mat_2.isOnBorder(-rows, c));
      EXPECT_TRUE(mat_2.isOnBorder(Eigen::Vector2i(-rows, c)));
      EXPECT_TRUE(mat_2.isOnBorder(Index(-rows, c)));

      EXPECT_TRUE(mat_2.isOnBorder(rows, c));
      EXPECT_TRUE(mat_2.isOnBorder(Eigen::Vector2i(rows, c)));
      EXPECT_TRUE(mat_2.isOnBorder(Index(rows, c)));

      if ((r < rows) && (r > -rows) && (c < cols) && (c > -cols))
      {
        EXPECT_FALSE(mat_2.isOnBorder(r, c));
        EXPECT_FALSE(mat_2.isOnBorder(Eigen::Vector2i(r, c)));
        EXPECT_FALSE(mat_2.isOnBorder(Index(r, c)));
      }
    }
  }
}  // TEST(data_structures_matrix_sparse_map, isOnBorder)

TEST(data_structures_matrix_sparse_map, isInside)
{
  using Index = MatrixSparseMapFloat::Index;

  constexpr Index::Scalar rows = 30;
  constexpr Index::Scalar cols = 50;

  MatrixSparseMapFloat mat_1(rows, cols);

  ASSERT_EQ(mat_1.rows(), rows);
  ASSERT_EQ(mat_1.cols(), cols);

  for (int r = 0; r < mat_1.rows(); r++)
  {
    for (int c = 0; c < mat_1.cols(); c++)
    {
      EXPECT_TRUE(mat_1.isInside(r, c));
      EXPECT_TRUE(mat_1.isInside(Eigen::Vector2i(r, c)));
      EXPECT_TRUE(mat_1.isInside(Index(r, c)));
    }
  }

  EXPECT_FALSE(mat_1.isInside(-1, -1));
  EXPECT_FALSE(mat_1.isInside(Eigen::Vector2i(-1, -1)));
  EXPECT_FALSE(mat_1.isInside(Index(-1, -1)));

  EXPECT_FALSE(mat_1.isInside(rows, cols));
  EXPECT_FALSE(mat_1.isInside(Eigen::Vector2i(rows, cols)));
  EXPECT_FALSE(mat_1.isInside(Index(rows, cols)));

  MatrixSparseMapFloat mat_2(Index(-rows, -cols), Index(rows, cols));

  ASSERT_EQ(mat_2.rows(), rows * 2 + 1);
  ASSERT_EQ(mat_2.cols(), cols * 2 + 1);

  for (int r = -rows; r <= rows; r++)
  {
    for (int c = -cols; c <= cols; c++)
    {
      EXPECT_TRUE(mat_2.isInside(r, c));
      EXPECT_TRUE(mat_2.isInside(Eigen::Vector2i(r, c)));
      EXPECT_TRUE(mat_2.isInside(Index(r, c)));
    }
  }

  EXPECT_FALSE(mat_2.isInside(-rows - 1, -cols - 1));
  EXPECT_FALSE(mat_2.isInside(Eigen::Vector2i(-rows - 1, -cols - 1)));
  EXPECT_FALSE(mat_2.isInside(Index(-rows - 1, -cols - 1)));

  EXPECT_FALSE(mat_2.isInside(rows + 1, cols + 1));
  EXPECT_FALSE(mat_2.isInside(Eigen::Vector2i(rows + 1, cols + 1)));
  EXPECT_FALSE(mat_2.isInside(Index(rows + 1, cols + 1)));

}  // TEST(data_structures_matrix_sparse_map, isInside)

TEST(data_structures_matrix_sparse_map, coeff)
{
  using Index = MatrixSparseMapFloat::Index;

  constexpr Index::Scalar rows = 10;
  constexpr Index::Scalar cols = 20;

  MatrixSparseMapFloat mat(Index(-rows, -cols), Index(rows, cols));

  for (int r = -rows; r <= rows; r++)
  {
    for (int c = -cols; c <= cols; c++)
    {
      EXPECT_FLOAT_EQ(mat.coeff(r, c), 0.f);
      EXPECT_FLOAT_EQ(mat.coeff(r, c), mat.defaultValue());
    }
  }

  ASSERT_EQ(mat.size(), 0);
  ASSERT_EQ(mat.nonZeros(), 0);
  ASSERT_EQ(mat.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));

  mat[Index(5, 5)] = 10.f;

  EXPECT_EQ(mat.size(), 1);
  EXPECT_EQ(mat.nonZeros(), 1);
  EXPECT_EQ(mat.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));

  for (int r = -rows; r <= rows; r++)
  {
    for (int c = -cols; c <= cols; c++)
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

  mat.coeffRef(Index(5, 5)) = 20.f;

  EXPECT_EQ(mat.size(), 1);
  EXPECT_EQ(mat.nonZeros(), 1);
  EXPECT_EQ(mat.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));

  EXPECT_FLOAT_EQ(mat.coeff(5, 5), 20.f);
  EXPECT_FLOAT_EQ(mat.coeff(Index(5, 5)), 20.f);
  EXPECT_FLOAT_EQ(mat.coeff(Eigen::Vector2i(5, 5)), 20.f);

  MatrixSparseMapFloat test_mat(mat);

  EXPECT_EQ(test_mat.rows(), mat.rows());
  EXPECT_EQ(test_mat.cols(), mat.cols());
  EXPECT_FLOAT_EQ(test_mat.defaultValue(), mat.defaultValue());
  EXPECT_EQ(test_mat.size(), mat.size());
  EXPECT_EQ(test_mat.nonZeros(), mat.nonZeros());
  EXPECT_EQ(test_mat.totalSize(), mat.totalSize());
}  // TEST(data_structures_matrix_sparse_map, coeff)

TEST(data_structures_matrix_sparse_map, coeffRef)
{
  using Index = MatrixSparseMapFloat::Index;
  using IndexScalar = MatrixSparseMapFloat::IndexScalar;

  constexpr IndexScalar rows = 10;
  constexpr IndexScalar cols = 20;

  MatrixSparseMapFloat mat(Index(-rows, -cols), Index(rows, cols));

  float value = 0.f;

  for (IndexScalar r = -rows; r <= rows; r++)
  {
    for (IndexScalar c = -cols; c <= cols; c++)
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

  EXPECT_EQ(mat.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_EQ(mat.size(), mat.totalSize());
  EXPECT_EQ(mat.nonZeros(), mat.totalSize());

  EXPECT_ANY_THROW(mat.coeffRef(rows + 1, cols + 1));
  EXPECT_ANY_THROW(mat.coeffRef(0, cols + 1));
  EXPECT_ANY_THROW(mat.coeffRef(rows + 1, 0));

  EXPECT_ANY_THROW(mat.coeffRef(-rows - 1, -cols - 1));
  EXPECT_ANY_THROW(mat.coeffRef(0, -cols - 1));
  EXPECT_ANY_THROW(mat.coeffRef(-rows - 1, 0));

  MatrixSparseMapFloat test_mat(mat);

  EXPECT_EQ(test_mat.rows(), mat.rows());
  EXPECT_EQ(test_mat.cols(), mat.cols());
  EXPECT_FLOAT_EQ(test_mat.defaultValue(), mat.defaultValue());
  EXPECT_EQ(test_mat.size(), mat.size());
  EXPECT_EQ(test_mat.nonZeros(), mat.nonZeros());
  EXPECT_EQ(test_mat.totalSize(), mat.totalSize());
}  // TEST(data_structures_matrix_sparse_map, coeffRef)

TEST(data_structures_matrix_sparse_map, fill)
{
  using Index = MatrixSparseMapFloat::Index;
  using IndexScalar = MatrixSparseMapFloat::IndexScalar;

  constexpr IndexScalar rows = 10;
  constexpr IndexScalar cols = 20;

  MatrixSparseMapInt mat_int_i(Index(-rows, -cols), Index(rows, cols));
  MatrixSparseMapFloat mat_float_i(Index(-rows, -cols), Index(rows, cols));
  MatrixSparseMapDouble mat_double_i(Index(-rows, -cols), Index(rows, cols));

  ASSERT_EQ(mat_int_i.rows(), rows * 2 + 1);
  ASSERT_EQ(mat_float_i.rows(), rows * 2 + 1);
  ASSERT_EQ(mat_double_i.rows(), rows * 2 + 1);
  ASSERT_EQ(mat_int_i.cols(), cols * 2 + 1);
  ASSERT_EQ(mat_float_i.cols(), cols * 2 + 1);
  ASSERT_EQ(mat_double_i.cols(), cols * 2 + 1);

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

  EXPECT_EQ(mat_int_i.nonZeros(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_EQ(mat_float_i.nonZeros(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_EQ(mat_double_i.nonZeros(), (rows * 2 + 1) * (cols * 2 + 1));

  for (IndexScalar r = mat_int_i.limInfRows(); r <= mat_int_i.limSupRows(); r++)
  {
    for (IndexScalar c = mat_int_i.limInfCols(); c <= mat_int_i.limSupCols();
         c++)
    {
      EXPECT_TRUE(mat_int_i.coeff(r, c) == 5);
      EXPECT_TRUE(mat_float_i.coeff(r, c) == 6.7f);
      EXPECT_TRUE(mat_double_i.coeff(r, c) == 9.99);
    }
  }
}  // TEST(data_structures_matrix_sparse_map, fill)

TEST(data_structures_matrix_sparse_map, fillStruct)
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

  using Index = MatrixSparseMap_<MatrixCell>::Index;
  using IndexScalar = MatrixSparseMap_<MatrixCell>::IndexScalar;

  constexpr IndexScalar rows = 50;
  constexpr IndexScalar cols = 30;

  MatrixSparseMap_<MatrixCell> mat_struct(Index(-rows, -cols),
                                          Index(rows, cols));

  ASSERT_EQ(mat_struct.rows(), rows * 2 + 1);
  ASSERT_EQ(mat_struct.cols(), cols * 2 + 1);
  ASSERT_TRUE(mat_struct.limInf() == MatrixSparseMapIndex(-rows, -cols));
  ASSERT_TRUE(mat_struct.limSup() == MatrixSparseMapIndex(rows, cols));
  ASSERT_EQ(mat_struct.limInfRows(), -rows);
  ASSERT_EQ(mat_struct.limInfCols(), -cols);
  ASSERT_EQ(mat_struct.limSupRows(), rows);
  ASSERT_EQ(mat_struct.limSupCols(), cols);

  mat_struct.fill(MatrixCell(0.25f,  //!< m_distance
                             0.50f   //!< m_weight
                             ));

  MatrixCell default_cell = mat_struct.defaultValue();

  ASSERT_FLOAT_EQ(default_cell.m_distance, 0.25f);
  ASSERT_FLOAT_EQ(default_cell.m_weight, 0.50f);

  for (IndexScalar r = mat_struct.limInfRows(); r <= mat_struct.limSupRows();
       r++)
  {
    for (IndexScalar c = mat_struct.limInfCols(); c <= mat_struct.limSupCols();
         c++)
    {
      EXPECT_FLOAT_EQ(mat_struct.coeff(r, c).m_distance, 0.25f);
      EXPECT_FLOAT_EQ(mat_struct.coeff(r, c).m_weight, 0.50f);
    }
  }

  EXPECT_TRUE(mat_struct.defaultValue() == default_cell);
  EXPECT_EQ(mat_struct.size(), 0);
  EXPECT_EQ(mat_struct.nonDefaultValues(), 0);
  EXPECT_EQ(mat_struct.nonZeros(), (rows * 2 + 1) * (cols * 2 + 1));
}  // TEST(data_structures_matrix_sparse_map, fillStruct)

TEST(data_structures_matrix_sparse_map, resize)
{
  using Index = MatrixSparseMapFloat::Index;
  using IndexScalar = MatrixSparseMapFloat::IndexScalar;

  constexpr IndexScalar rows = 100;
  constexpr IndexScalar cols = 200;

  MatrixSparseMapFloat mat_1;

  EXPECT_EQ(mat_1.rows(), 1);
  EXPECT_EQ(mat_1.cols(), 1);
  EXPECT_FLOAT_EQ(mat_1.defaultValue(), 0.0f);
  EXPECT_EQ(mat_1.totalSize(), 1);
  EXPECT_EQ(mat_1.nonZeros(), 0);

  mat_1.resize(rows, cols);

  EXPECT_EQ(mat_1.rows(), rows);
  EXPECT_EQ(mat_1.cols(), cols);
  EXPECT_FLOAT_EQ(mat_1.defaultValue(), 0.0f);
  EXPECT_EQ(mat_1.totalSize(), rows * cols);
  EXPECT_EQ(mat_1.nonZeros(), 0);
  EXPECT_EQ(mat_1.size(), 0);

  mat_1.coeffRef(Index(5, 8)) = -1.f;
  mat_1.coeffRef(Index(rows - 1, cols - 1)) = 1.f;

  EXPECT_EQ(mat_1.rows(), rows);
  EXPECT_EQ(mat_1.cols(), cols);
  EXPECT_FLOAT_EQ(mat_1.defaultValue(), 0.0f);
  EXPECT_EQ(mat_1.totalSize(), rows * cols);
  EXPECT_EQ(mat_1.nonZeros(), 2);
  EXPECT_EQ(mat_1.size(), 2);

  mat_1.resize(rows / 2, cols / 2);

  EXPECT_EQ(mat_1.rows(), rows / 2);
  EXPECT_EQ(mat_1.cols(), cols / 2);
  EXPECT_FLOAT_EQ(mat_1.defaultValue(), 0.0f);
  EXPECT_EQ(mat_1.totalSize(), rows * cols / 4);
  EXPECT_EQ(mat_1.nonZeros(), 1);
  EXPECT_EQ(mat_1.size(), 1);

  MatrixSparseMapFloat mat_2;

  EXPECT_EQ(mat_2.rows(), 1);
  EXPECT_EQ(mat_2.cols(), 1);
  EXPECT_FLOAT_EQ(mat_2.defaultValue(), 0.0f);
  EXPECT_EQ(mat_2.totalSize(), 1);
  EXPECT_EQ(mat_2.nonZeros(), 0);

  mat_2.resize(Index(-rows, -cols), Index(rows, cols));

  EXPECT_EQ(mat_2.rows(), rows * 2 + 1);
  EXPECT_EQ(mat_2.cols(), cols * 2 + 1);
  EXPECT_FLOAT_EQ(mat_2.defaultValue(), 0.0f);
  EXPECT_EQ(mat_2.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_EQ(mat_2.nonZeros(), 0);
  EXPECT_EQ(mat_2.size(), 0);

  mat_2.coeffRef(Index(5, 8)) = -1.f;
  mat_2.coeffRef(Index(-5, -8)) = -1.f;
  mat_2.coeffRef(Index(rows - 1, cols - 1)) = 1.f;
  mat_2.coeffRef(Index(-rows + 1, -cols + 1)) = 1.f;

  EXPECT_EQ(mat_2.rows(), rows * 2 + 1);
  EXPECT_EQ(mat_2.cols(), cols * 2 + 1);
  EXPECT_FLOAT_EQ(mat_2.defaultValue(), 0.0f);
  EXPECT_EQ(mat_2.totalSize(), (rows * 2 + 1) * (cols * 2 + 1));
  EXPECT_EQ(mat_2.nonZeros(), 4);
  EXPECT_EQ(mat_2.size(), 4);

  mat_2.resize(Index(-rows / 2, -cols / 2), Index(rows / 2, cols / 2));

  EXPECT_EQ(mat_2.rows(), rows + 1);
  EXPECT_EQ(mat_2.cols(), cols + 1);
  EXPECT_FLOAT_EQ(mat_2.defaultValue(), 0.0f);
  EXPECT_EQ(mat_2.totalSize(), (rows + 1) * (cols + 1));
  EXPECT_EQ(mat_2.nonZeros(), 2);
  EXPECT_EQ(mat_2.size(), 2);

}  // TEST(data_structures_matrix_sparse_map, resize)

TEST(data_structures_matrix_sparse_map, getSubPx)
{
  float interpolated_value_f;
  double interpolated_value_d;

  MatrixSparseMapFloat mat_float(3, 3);
  MatrixSparseMapDouble mat_double(3, 3);

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
}  // TEST(data_structures_matrix_sparse_map, getSubPx)

}  // namespace testing
