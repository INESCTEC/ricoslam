#include <gtest/gtest.h>

#include <inesctec_mrdt_slam_distmap_2d/data_structures/matrix.hpp>

namespace testing
{

using namespace inesctec_mrdt_slam_distmap_2d;
using namespace data_structures;

TEST(data_structures_matrix, constructor)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  MatrixInt mat_int_i(rows, cols);
  MatrixFloat mat_float_i(rows, cols);
  MatrixDouble mat_double_i(rows, cols);

  ASSERT_EQ(mat_int_i.rows(), rows);
  ASSERT_EQ(mat_float_i.rows(), rows);
  ASSERT_EQ(mat_double_i.rows(), rows);
  ASSERT_EQ(mat_int_i.cols(), cols);
  ASSERT_EQ(mat_float_i.cols(), cols);
  ASSERT_EQ(mat_double_i.cols(), cols);

  int idx = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      mat_int_i(r, c) = idx;
      mat_float_i(r, c) = idx;
      mat_double_i(r, c) = idx;
      idx++;
    }
  }

  MatrixInt mat_int_f(mat_int_i);
  MatrixFloat mat_float_f(mat_float_i);
  MatrixDouble mat_double_f(mat_double_i);

  EXPECT_EQ(mat_int_i.rows(), mat_int_f.rows());
  EXPECT_EQ(mat_float_i.rows(), mat_float_f.rows());
  EXPECT_EQ(mat_double_i.rows(), mat_double_f.rows());
  EXPECT_EQ(mat_int_i.cols(), mat_int_f.cols());
  EXPECT_EQ(mat_float_i.cols(), mat_float_f.cols());
  EXPECT_EQ(mat_double_i.cols(), mat_double_f.cols());

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      EXPECT_TRUE(mat_int_i(r, c) == mat_int_f(r, c));
      EXPECT_TRUE(mat_float_i(r, c) == mat_float_f(r, c));
      EXPECT_TRUE(mat_double_i(r, c) == mat_double_f(r, c));
    }
  }

}  // TEST(data_structures_matrix, constructor)

TEST(data_structures_matrix, constructorStruct)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  struct MatrixCell
  {
    float m_distance;
    float m_weight;

    bool operator==(const MatrixCell& other)
    {
      return ((this->m_distance == other.m_distance) &&
              (this->m_weight == other.m_weight));
    }
  };

  Matrix_<MatrixCell> mat_struct_i(rows, cols);

  ASSERT_EQ(mat_struct_i.rows(), rows);
  ASSERT_EQ(mat_struct_i.cols(), cols);

  for (int r = 0; r < mat_struct_i.rows(); r++)
  {
    for (int c = 0; c < mat_struct_i.cols(); c++)
    {
      mat_struct_i(r, c).m_distance =
          std::sqrt(static_cast<float>(r * r + c * c));
      mat_struct_i(r, c).m_weight = 1.0f;
    }
  }

  Matrix_<MatrixCell> mat_struct_f(mat_struct_i);

  for (int r = 0; r < mat_struct_i.rows(); r++)
  {
    for (int c = 0; c < mat_struct_i.cols(); c++)
    {
      EXPECT_TRUE(mat_struct_f(r, c) == mat_struct_i(r, c));
    }
  }

}  // TEST(data_structures_matrix, constructorStruct)

TEST(data_structures_matrix, fill)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  MatrixInt mat_int(rows, cols);
  MatrixFloat mat_float(rows, cols);
  MatrixDouble mat_double(rows, cols);

  ASSERT_EQ(mat_int.rows(), rows);
  ASSERT_EQ(mat_float.rows(), rows);
  ASSERT_EQ(mat_double.rows(), rows);
  ASSERT_EQ(mat_int.cols(), cols);
  ASSERT_EQ(mat_float.cols(), cols);
  ASSERT_EQ(mat_double.cols(), cols);

  mat_int.fill(10);
  mat_float.fill(10.0f);
  mat_double.fill(10.0);

  for (int r = 0; r < 20; r++)
  {
    for (int c = 0; c < 30; c++)
    {
      EXPECT_EQ(mat_int(r, c), 10);
      EXPECT_FLOAT_EQ(mat_float(r, c), 10.0f);
      EXPECT_FLOAT_EQ(mat_double(r, c), 10.0);
    }
  }

  struct MatCellTest
  {
   public:

    int m_int;
    float m_float;

   public:

    MatCellTest() {}
    MatCellTest(int p_int, float p_float) : m_int(p_int), m_float(p_float) {}
  };

  Matrix_<MatCellTest> mat_struct(20, 30);

  mat_struct.fill(MatCellTest(-10, 33.3));

  for (int r = 0; r < 20; r++)
  {
    for (int c = 0; c < 30; c++)
    {
      EXPECT_EQ(mat_struct(r, c).m_int, -10);
      EXPECT_FLOAT_EQ(mat_struct(r, c).m_float, 33.3f);
    }
  }
}  // TEST(data_structures_matrix, fill)

TEST(data_structures_matrix, fillStruct)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  struct MatrixCell
  {
    float m_distance;
    float m_weight;
  };

  Matrix_<MatrixCell> mat_struct(rows, cols);

  ASSERT_EQ(mat_struct.rows(), rows);
  ASSERT_EQ(mat_struct.cols(), cols);

  mat_struct.fill(MatrixCell({
      0.25f,  //!< m_distance
      0.50f   //!< m_weight
  }));

  for (int r = 0; r < mat_struct.rows(); r++)
  {
    for (int c = 0; c < mat_struct.cols(); c++)
    {
      EXPECT_FLOAT_EQ(mat_struct(r, c).m_distance, 0.25f);
      EXPECT_FLOAT_EQ(mat_struct(r, c).m_weight, 0.50f);
    }
  }

}  // TEST(data_structures_matrix, fillStruct)

TEST(data_structures_matrix, at)
{
  int rows = 20, cols = 30;

  MatrixInt mat_int(rows, cols);
  MatrixFloat mat_float(rows, cols);
  MatrixDouble mat_double(rows, cols);

  int index = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      mat_int(r, c) = index;
      mat_float(r, c) = index;
      mat_double(r, c) = index;

      index++;
    }
  }

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      Eigen::Vector2i pos(r, c);

      EXPECT_TRUE(mat_int(r, c) == mat_int.at(r, c));
      EXPECT_TRUE(mat_float(r, c) == mat_float.at(r, c));
      EXPECT_TRUE(mat_double(r, c) == mat_double.at(r, c));

      EXPECT_TRUE(mat_int(r, c) == mat_int.at(pos));
      EXPECT_TRUE(mat_float(r, c) == mat_float.at(pos));
      EXPECT_TRUE(mat_double(r, c) == mat_double.at(pos));

      int v_int = mat_int.at(r, c);
      float v_float = mat_float.at(r, c);
      double v_double = mat_double.at(r, c);

      EXPECT_TRUE(mat_int(r, c) == v_int);
      EXPECT_TRUE(mat_float(r, c) == v_float);
      EXPECT_TRUE(mat_double(r, c) == v_double);

      v_int = mat_int.at(pos);
      v_float = mat_float.at(pos);
      v_double = mat_double.at(pos);

      EXPECT_TRUE(mat_int(r, c) == v_int);
      EXPECT_TRUE(mat_float(r, c) == v_float);
      EXPECT_TRUE(mat_double(r, c) == v_double);
    }
  }
}  // TEST(data_structures_matrix, at)

TEST(data_structures_matrix, atIdx)
{
  constexpr int rows = 3;
  constexpr int cols = 4;
  constexpr int length = rows * cols;

  Matrix_<float, DefaultCellTraits<float>, Eigen::RowMajor> mat_row_maj(rows,
                                                                        cols);
  Matrix_<float, DefaultCellTraits<float>, Eigen::ColMajor> mat_col_maj(rows,
                                                                        cols);

  mat_row_maj << 8, 2, 2, 9, 9, 1, 4, 4, 3, 5, 4, 5;

  mat_col_maj << 8, 2, 2, 9, 9, 1, 4, 4, 3, 5, 4, 5;

  float array_row_maj[] = {8, 2, 2, 9, 9, 1, 4, 4, 3, 5, 4, 5};
  float array_col_maj[] = {8, 9, 3, 2, 1, 5, 2, 4, 4, 9, 4, 5};

  for (int idx = 0; idx < length; idx++)
  {
    EXPECT_FLOAT_EQ(mat_row_maj.at(idx), array_row_maj[idx]);
    EXPECT_FLOAT_EQ(mat_col_maj.at(idx), array_col_maj[idx]);
  }

}  // TEST(data_structures_matrix, atIdx)

TEST(data_structures_matrix, indexAt)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  Matrix_<float, DefaultCellTraits<float>, Eigen::RowMajor> mat_row_maj(rows,
                                                                        cols);
  Matrix_<float, DefaultCellTraits<float>, Eigen::ColMajor> mat_col_maj(rows,
                                                                        cols);

  ASSERT_EQ(mat_row_maj.rows(), rows);
  ASSERT_EQ(mat_row_maj.cols(), cols);
  ASSERT_EQ(mat_col_maj.rows(), rows);
  ASSERT_EQ(mat_col_maj.cols(), cols);

  int idx = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      mat_row_maj(r, c) = idx;
      mat_col_maj(r, c) = idx;
      idx++;
    }
  }

  idx = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      std::ptrdiff_t idx_row_maj = &mat_row_maj(r, c) - &mat_row_maj(0, 0);
      std::ptrdiff_t idx_col_maj = &mat_col_maj(r, c) - &mat_col_maj(0, 0);

      EXPECT_EQ(static_cast<size_t>(idx_row_maj), mat_row_maj.indexAt(r, c));
      EXPECT_EQ(static_cast<size_t>(idx_col_maj), mat_col_maj.indexAt(r, c));

      EXPECT_FLOAT_EQ(mat_row_maj.at(static_cast<size_t>(idx_row_maj)),
                      mat_row_maj(r, c));
      EXPECT_FLOAT_EQ(mat_col_maj.at(static_cast<size_t>(idx_col_maj)),
                      mat_col_maj(r, c));
    }
  }

}  // TEST(data_structures_matrix, indexAt)

TEST(data_structures_matrix, size)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  MatrixInt mat_int(rows, cols);
  MatrixFloat mat_float(rows, cols);
  MatrixDouble mat_double(rows, cols);

  EXPECT_TRUE(mat_int.size() == rows * cols);
  EXPECT_TRUE(mat_float.size() == rows * cols);
  EXPECT_TRUE(mat_double.size() == rows * cols);

}  // TEST(data_structures_matrix, size)

TEST(data_structures_matrix, forIndexIterator)
{
  int rows = 20, cols = 30;

  MatrixInt mat_int(rows, cols);
  MatrixFloat mat_float(rows, cols);
  MatrixDouble mat_double(rows, cols);

  int index = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      mat_int(r, c) = index;
      mat_float(r, c) = index;
      mat_double(r, c) = index;

      index++;
    }
  }

  ASSERT_EQ(MatrixInt::kCellAlign, Eigen::RowMajor);
  ASSERT_EQ(MatrixFloat::kCellAlign, Eigen::RowMajor);
  ASSERT_EQ(MatrixDouble::kCellAlign, Eigen::RowMajor);

  for (long int idx = 0; idx < mat_int.size(); idx++)
  {
    EXPECT_TRUE(mat_int.at(idx) == static_cast<int>(idx));
  }
}  // TEST(data_structures_matrix, forIndexIterator)

TEST(data_structures_matrix, forStlIteratorMagic)
{
  int rows = 20, cols = 30;

  MatrixInt mat_int(rows, cols);
  MatrixFloat mat_float(rows, cols);
  MatrixDouble mat_double(rows, cols);

  int index = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      mat_int(r, c) = index;
      mat_float(r, c) = index;
      mat_double(r, c) = index;

      index++;
    }
  }

  int idx = 0;

  for (auto& v_int : mat_int)
  {
    EXPECT_TRUE(v_int == idx);
    idx++;
  }

  idx = 0;

  for (auto& v_float : mat_float)
  {
    EXPECT_TRUE(v_float == idx);
    idx++;
  }

  idx = 0;

  for (auto& v_double : mat_double)
  {
    EXPECT_TRUE(v_double == idx);
    idx++;
  }

  struct TestStruct
  {
    int m_test_v1 = 0;
    int m_test_v2 = 0;
  };

  Matrix_<TestStruct> test_with_structs(rows, cols);

  index = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      test_with_structs(r, c).m_test_v1 = index;
      test_with_structs(r, c).m_test_v2 =
          static_cast<int>(test_with_structs.size()) - index - 1;

      index++;
    }
  }

  idx = 0;

  for (auto& v_test : test_with_structs)
  {
    EXPECT_TRUE(v_test.m_test_v1 == idx);
    EXPECT_TRUE(v_test.m_test_v2 ==
                static_cast<int>(test_with_structs.size()) - idx - 1);
    idx++;
  }
}  // TEST(data_structures_matrix, forStlIteratorMagic)

TEST(data_structures_matrix, getCellPos)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  Matrix_<float, DefaultCellTraits<float>, Eigen::RowMajor> mat_row_maj(rows,
                                                                        cols);
  Matrix_<float, DefaultCellTraits<float>, Eigen::ColMajor> mat_col_maj(rows,
                                                                        cols);

  ASSERT_EQ(mat_row_maj.rows(), rows);
  ASSERT_EQ(mat_row_maj.cols(), cols);
  ASSERT_EQ(mat_col_maj.rows(), rows);
  ASSERT_EQ(mat_col_maj.cols(), cols);

  int idx = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      mat_row_maj(r, c) = idx;
      mat_col_maj(r, c) = idx;
      idx++;
    }
  }

  idx = 0;

  for (int r = 0; r < rows; r++)
  {
    for (int c = 0; c < cols; c++)
    {
      Eigen::Vector2i cell_pos_row_maj =
          mat_row_maj.getCellPos(&mat_row_maj(r, c));
      Eigen::Vector2i cell_pos_col_maj =
          mat_col_maj.getCellPos(&mat_col_maj(r, c));

      EXPECT_FLOAT_EQ(mat_row_maj.at(cell_pos_row_maj), mat_row_maj(r, c));
      EXPECT_FLOAT_EQ(mat_col_maj.at(cell_pos_col_maj), mat_col_maj(r, c));

      EXPECT_EQ(r, cell_pos_row_maj.x());
      EXPECT_EQ(r, cell_pos_col_maj.x());

      EXPECT_EQ(c, cell_pos_row_maj.y());
      EXPECT_EQ(c, cell_pos_col_maj.y());
    }
  }

}  // TEST(data_structures_matrix, getCellPos)

TEST(data_structures_matrix, getSubPxValue)
{
  float interpolated_value_f;
  double interpolated_value_d;

  MatrixFloat mat_float;
  MatrixDouble mat_double;

  mat_float.resize(3, 3);
  mat_double.resize(3, 3);

  mat_float(0, 2) = 3;
  mat_float(1, 2) = 3;
  mat_float(2, 2) = 3;
  mat_float(0, 1) = 1;
  mat_float(1, 1) = 1;
  mat_float(2, 1) = 1;
  mat_float(0, 0) = -1;
  mat_float(1, 0) = -1;
  mat_float(2, 0) = -1;

  mat_double(0, 2) = 3;
  mat_double(1, 2) = 3;
  mat_double(2, 2) = 3;
  mat_double(0, 1) = 1;
  mat_double(1, 1) = 1;
  mat_double(2, 1) = 1;
  mat_double(0, 0) = -1;
  mat_double(1, 0) = -1;
  mat_double(2, 0) = -1;

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

  mat_float(0, 2) = -1;
  mat_float(1, 2) = 1;
  mat_float(2, 2) = 3;
  mat_float(0, 1) = -1;
  mat_float(1, 1) = 1;
  mat_float(2, 1) = 3;
  mat_float(0, 0) = -1;
  mat_float(1, 0) = 1;
  mat_float(2, 0) = -3;

  mat_double(0, 2) = -1;
  mat_double(1, 2) = 1;
  mat_double(2, 2) = 3;
  mat_double(0, 1) = -1;
  mat_double(1, 1) = 1;
  mat_double(2, 1) = 3;
  mat_double(0, 0) = -1;
  mat_double(1, 0) = 1;
  mat_double(2, 0) = 3;

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
}  // TEST(data_structures_matrix, getSubPxValue)

TEST(data_structures_matrix, isOnBorder)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  MatrixFloat mat;

  mat.resize(rows, cols);

  ASSERT_EQ(mat.rows(), rows);
  ASSERT_EQ(mat.cols(), cols);

  for (int r = 0; r < mat.rows(); r++)
  {
    EXPECT_TRUE(mat.isOnBorder(r, 0));
    EXPECT_TRUE(mat.isOnBorder(r, cols - 1));

    for (int c = 0; c < mat.cols(); c++)
    {
      EXPECT_TRUE(mat.isOnBorder(0, c));
      EXPECT_TRUE(mat.isOnBorder(rows - 1, c));

      if ((r < rows - 1) && (r > 0) && (c < cols - 1) && (c > 0))
      {
        EXPECT_FALSE(mat.isOnBorder(r, c));
      }
    }
  }

  EXPECT_FALSE(mat.isOnBorder(-1, -1));
  EXPECT_FALSE(mat.isOnBorder(rows, cols));

}  // TEST(data_structures_matrix, isOnBorder)

TEST(data_structures_matrix, isInside)
{
  constexpr int rows = 20;
  constexpr int cols = 30;

  MatrixFloat mat;

  mat.resize(rows, cols);

  ASSERT_EQ(mat.rows(), rows);
  ASSERT_EQ(mat.cols(), cols);

  for (int r = 0; r < mat.rows(); r++)
  {
    for (int c = 0; c < mat.cols(); c++)
    {
      EXPECT_TRUE(mat.isInside(r, c));
      EXPECT_TRUE(mat.isInside(Eigen::Vector2i(r, c)));
    }
  }

  EXPECT_FALSE(mat.isInside(-1, -1));
  EXPECT_FALSE(mat.isInside(rows, cols));

}  // TEST(data_structures_matrix, isInside)

}  // namespace testing
