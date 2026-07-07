#pragma once

#include <exception>
#include <functional>
#include <iostream>
#include <unordered_map>

// Eigen
#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/matrix.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief generic index to identify a matrix cell as a key (e.g., for sparse
 *        representations)
 * @tparam Scalar_ base scalar type to index member variables (uint32_t, ...)
 */
template <typename Scalar_ = uint32_t>
struct MatrixSparseIndex_
{
 public:

  using ThisType = MatrixSparseIndex_<Scalar_>;
  using Scalar = Scalar_;

  //! cell row index
  Scalar m_row = 0;

  //! cell column index
  Scalar m_col = 0;

 public:

  /**
   * @brief Construct a new MatrixSparseIndex_<Scalar_> object
   *        (default constructor)
   */
  MatrixSparseIndex_() : m_row(0), m_col(0) {}

  /**
   * @brief Construct a new MatrixSparseIndex_<Scalar_> object with row and
   *        column indexation
   * @param[in] row cell row index
   * @param[in] col cell column index
   */
  MatrixSparseIndex_(Scalar row, Scalar col) : m_row(row), m_col(col) {}

  /**
   * @brief Construct a new MatrixSparseIndex_<Scalar_> object with Eigen-based
   *        row and column indexation
   * @param[in] pos cell index position in a Eigen vector representation
   */
  MatrixSparseIndex_(const Eigen::Vector2i& pos)
      : m_row(static_cast<Scalar>(pos.x())), m_col(static_cast<Scalar>(pos.y()))
  {
  }

  /**
   * @brief Equality comparison operation (e.g., required for hash tables
   *        definition to compare different keys of cells)
   * @param[in] other another key of a matrix cell
   * @return true keys correspond to the same cell in the matrix
   * @return false otherwise
   */
  inline bool operator==(const ThisType& other) const
  {
    return ((this->m_row == other.m_row) && (this->m_col == other.m_col));
  }

};  // struct MatrixSparseIndex_<Scalar_>

/**
 * @brief hash function operator for the matrix cell's index key
 * @tparam IndexType_ matrix cell index type
 */
template <typename IndexType_ = MatrixSparseIndex_<uint32_t>>
struct MatrixSparseHash_
{
 public:

  using ThisType = MatrixSparseHash_<IndexType_>;
  using Index = IndexType_;
  using Scalar = typename IndexType_::Scalar;

 public:

  /**
   * @brief hash function operator based on OR and shift operation, to guarantee
   *        that no cell has the same key (thus, avoiding collisions)
   * @param[in] index matrix cell index
   * @return size_t hash function value
   */
  inline size_t operator()(const Index& index) const
  {
    return (static_cast<size_t>(index.m_row) << 32) |
           (static_cast<size_t>(index.m_col) & 0x00000000FFFFFFFF);
  }

};  // struct MatrixSparseHash_<IndexType_>

/**
 * @brief generic 2D sparse matrix class based on C++ STL hash map
 *        (unordered_map)
 * @tparam CellType_ matrix cell type
 * @tparam CellTraits_ cell traits type (setZero, sumAndScale, ...)
 * @tparam IndexType_ matrix cell index type
 * @tparam HashType_ hash function to generate a key for the cell index
 */
template <typename CellType_,
          typename CellTraits_ = DefaultCellTraits<CellType_>,
          typename IndexType_ = MatrixSparseIndex_<int32_t>,
          typename HashType_ = MatrixSparseHash_<MatrixSparseIndex_<int32_t>>>
struct MatrixSparse_
    : public std::unordered_map<IndexType_, CellType_, HashType_>
{
 public:

  using BaseType = std::unordered_map<IndexType_, CellType_, HashType_>;
  using ThisType = MatrixSparse_<CellType_, CellTraits_, IndexType_, HashType_>;
  using CellType = CellType_;
  using CellTraits = CellTraits_;

  using Index = IndexType_;
  using IndexScalar = typename IndexType_::Scalar;

  using Hash = HashType_;

 protected:

  //! number of rows of the sparse matrix
  IndexScalar m_rows_ = 0;

  //! number of columns of the sparse matrix
  IndexScalar m_cols_ = 0;

  //! default cell value for the sparse matrix
  CellType m_default_v_;

 public:

  /**
   * @brief Construct a new
   *        MatrixSparse_<CellType_, CellTraits_, IndexType_, HashType_> object
   */
  MatrixSparse_() : m_rows_(0), m_cols_(0)
  {
    CellTraits::setZero(m_default_v_);
  }

  /**
   * @brief Construct a new
   *        MatrixSparse_<CellType_, CellTraits_, IndexType_, HashType_> object
   *        defining its initial size
   * @param[in] rows number of rows
   * @param[in] cols number of cols
   */
  MatrixSparse_(IndexScalar rows, IndexScalar cols)
      : m_rows_(rows), m_cols_(cols)
  {
    CellTraits::setZero(m_default_v_);
  }

  /**
   * @brief Construct a new
   *        MatrixSparse_<CellType_, CellTraits_, IndexType_, HashType_> object
   *        defining its initial size and default value for the cells
   * @param[in] rows number of rows
   * @param[in] cols number of cols
   * @param[in] default_v cell default value (if not changed in the meantime)
   */
  MatrixSparse_(IndexScalar rows, IndexScalar cols, const CellType& default_v)
      : m_rows_(rows), m_cols_(cols), m_default_v_(default_v)
  {
  }

  /**
   * @brief Construct a new
   *        MatrixSparse_<CellType_, CellTraits_, IndexType_, HashType_> object
   *        based on another sparse matrix object
   * @param[in] other other sparse matrix object
   */
  MatrixSparse_(const ThisType& other)
      : BaseType(other),
        m_rows_(other.m_rows_),
        m_cols_(other.m_cols_),
        m_default_v_(other.m_default_v_)
  {
  }

  /**
   * @brief Get number of rows of the sparse matrix
   * @return IndexScalar number of rows
   */
  inline IndexScalar rows() const noexcept { return m_rows_; }

  /**
   * @brief Get number of columns of the sparse matrix
   * @return IndexScalar number of columns
   */
  inline IndexScalar cols() const noexcept { return m_cols_; }

  /**
   * @brief Get default value set for the sparse matrix cells
   *        (if the matrix element was not redefined / created in the hash map,
   *        the default value is the one considered for otehr
   *        operations / getters)
   * @return CellType default value cell
   */
  inline CellType defaultValue() const noexcept { return m_default_v_; }

  /**
   * @brief Get number of non-default value cells in the sparse matrix
   * @return IndexScalar number of non-default value cells
   */
  inline IndexScalar nonDefaultValues() const noexcept { return this->size(); }

  /**
   * @brief Get number of zero-value cells in the sparse matrix
   * @return IndexScalar number of zero-value cells in the sparse matrix
   */
  inline IndexScalar nonZeros() const noexcept
  {
    CellType zero_cell;
    CellTraits::setZero(zero_cell);

    if (m_default_v_ == zero_cell)
    {
      return this->size();
    }
    else
    {
      if (this->size() > 0)
      {
        IndexScalar num_nonzeros = 0;
        for (const auto& element : *this)
        {
          if (!(element.second == zero_cell))
          {
            ++num_nonzeros;
          }
        }
        return num_nonzeros;
      }
      else
      {
        return this->rows() * this->cols();
      }
    }
  }

  /**
   * @brief Total size (#rows x #columns) of the sparse matrix
   * @return IndexScalar total number of cells in the sparse matrix (may not be
   *                     allocated in the hash map!)
   */
  inline IndexScalar totalSize() const noexcept { return m_rows_ * m_cols_; }

  /**
   * @brief Evaluate if a cell index is inside the sparse matrix
   * @param[in] row row index of the cell
   * @param[in] col column index of the cell
   * @return true cell inside the sparse matrix limits
   * @return false otherwise
   */
  inline bool isInside(IndexScalar row, IndexScalar col) const
  {
    return ((row >= 0) && (col >= 0) && (row < this->rows()) &&
            (col < this->cols()));
  }

  /**
   * @brief Evaluate if a cell index is inside the sparse matrix
   * @param[in] index cell index
   * @return true cell inside the sparse matrix limits
   * @return false otherwise
   */
  inline bool isInside(const Index& index) const
  {
    return ((index.m_row >= 0) && (index.m_col >= 0) &&
            (index.m_row < this->rows()) && (index.m_col < this->cols()));
  }

  /**
   * @brief Evaluate if a cell index is inside the sparse matrix
   * @param[in] pos cell index (Eigen vector representation)
   * @return true cell inside the sparse matrix limits
   * @return false otherwise
   */
  inline bool isInside(const Eigen::Vector2i& pos) const
  {
    return ((static_cast<IndexScalar>(pos.x()) >= 0) &&
            (static_cast<IndexScalar>(pos.y()) >= 0) &&
            (static_cast<IndexScalar>(pos.x()) < this->rows()) &&
            (static_cast<IndexScalar>(pos.y()) < this->cols()));
  }

  /**
   * @brief Evaluate if a cell is on the border limits of a matrix
   * @param[in] row row index of the cell
   * @param[in] col column index of the cell
   * @return true cell on the border of the sparse matrix limits
   * @return false otherwise
   */
  inline bool isOnBorder(IndexScalar row, IndexScalar col) const
  {
    return ((row == 0) || (col == 0) || (row == this->rows() - 1) ||
            (col == this->cols() - 1));
  }

  /**
   * @brief Evaluate if a cell is on the border limits of a matrix
   * @param[in] index cell index
   * @return true cell on the border of the sparse matrix limits
   * @return false otherwise
   */
  inline bool isOnBorder(const Index& index) const
  {
    return ((index.m_row == 0) || (index.m_col == 0) ||
            (index.m_row == this->rows() - 1) ||
            (index.m_col == this->cols() - 1));
  }

  /**
   * @brief Evaluate if a cell is on the border limits of a matrix
   * @param[in] pos cell index (Eigen vector representation)
   * @return true cell on the border of the sparse matrix limits
   * @return false otherwise
   */
  inline bool isOnBorder(const Eigen::Vector2i& pos) const
  {
    return ((static_cast<IndexScalar>(pos.x()) == 0) ||
            (static_cast<IndexScalar>(pos.y()) == 0) ||
            (static_cast<IndexScalar>(pos.x()) == this->rows() - 1) ||
            (static_cast<IndexScalar>(pos.y()) == this->cols() - 1));
  }

  /**
   * @brief Get element at position indexed by row and column without creaating
   *        a new cell element in the hash map (const) (read-only)
   * @param[in] row row index of the cell
   * @param[in] col column index of the cell
   * @return const CellType& cell desired cell at position (row,col)
   */
  inline const CellType& coeff(IndexScalar row, IndexScalar col) const
  {
    if (!isInside(row, col))
    {
      throw std::out_of_range("MatrixSparse_::coeff | index outside matrix");
    }

    auto search = this->find(Index(row, col));

    return search == this->end() ? m_default_v_ : search->second;
  }

  /**
   * @brief Get element at position indexed by row and column without creaating
   *        a new cell element in the hash map (const) (read-only)
   * @param[in] index cell index
   * @return const CellType& cell desired cell at position
   *                         (index.m_row,index.m_col)
   */
  inline const CellType& coeff(const Index& index) const
  {
    if (!isInside(index))
    {
      throw std::out_of_range("MatrixSparse_::coeff | index outside matrix");
    }

    auto search = this->find(index);

    return search == this->end() ? m_default_v_ : search->second;
  }

  /**
   * @brief Get element at position indexed by row and column without creaating
   *        a new cell element in the hash map (const) (read-only)
   * @param[in] pos cell index (Eigen vector representation)
   * @return const CellType& cell desired cell at position (pos.x,pos.y)
   */
  inline const CellType& coeff(const Eigen::Vector2i& pos) const
  {
    if (!isInside(pos))
    {
      throw std::out_of_range("MatrixSparse_::coeff | index outside matrix");
    }

    auto search = this->find(Index(pos.x(), pos.y()));

    return search == this->end() ? m_default_v_ : search->second;
  }

  /**
   * @brief Get element at position indexed by row and column by reference
   *        (if element does not exist in the hash map, the cell is created!)
   * @param[in] row row index of the cell
   * @param[in] col column index of the cell
   * @return const CellType& cell desired cell at position (row,col)
   */
  inline CellType& coeffRef(IndexScalar row, IndexScalar col)
  {
    if (!isInside(row, col))
    {
      throw std::out_of_range("MatrixSparse_::coeff | index outside matrix");
    }

    return (*this)[Index(row, col)];
  }

  /**
   * @brief Get element at position indexed by row and column by reference
   *        (if element does not exist in the hash map, the cell is created!)
   * @param[in] index cell index
   * @return const CellType& cell desired cell at position
   *                         (index.m_row,index.m_col)
   */
  inline CellType& coeffRef(const Index& index)
  {
    if (!isInside(index))
    {
      throw std::out_of_range("MatrixSparse_::coeff | index outside matrix");
    }

    return (*this)[index];
  }

  /**
   * @brief Get element at position indexed by row and column by reference
   *        (if element does not exist in the hash map, the cell is created!)
   * @param[in] pos cell index (Eigen vector representation)
   * @return const CellType& cell desired cell at position (pos.x,pos.y)
   */
  inline CellType& coeffRef(const Eigen::Vector2i& pos)
  {
    if (!isInside(pos))
    {
      throw std::out_of_range("MatrixSparse_::coeff | index outside matrix");
    }

    return (*this)[Index(pos.x(), pos.y())];
  }

  /**
   * @brief Resize the sparse matrix (#rows, #columns) (if after resize there
   *        are cells outside the matrix, these are erased from the has table)
   * @param[in] rows number of rows
   * @param[in] cols number of columns
   */
  void resize(IndexScalar rows, IndexScalar cols)
  {
    bool erase_outside_cells = (rows < m_rows_) || (cols < m_cols_);

    m_rows_ = rows;
    m_cols_ = cols;

    if ((this->size() > 0) && (erase_outside_cells))
    {
      for (auto it = this->begin(); it != this->end();)
      {
        if (!this->isInside(it->first))
        {
          it = this->erase(it);
        }
        else
        {
          ++it;
        }
      }
    }
  }

  /**
   * @brief Fill the sparse matrix with a certain value
   *        (indeed, the hash map is cleared and the default value becomes the
   *        one specified as argument in the fill operation)
   * @param[in] value cell value
   */
  void fill(const CellType& value) noexcept
  {
    m_default_v_ = value;
    this->clear();
  }

  /**
   * @brief Get the subpixel value using bilinear interpolation
   * @tparam Scalar_ scalar type for interpolation
   * @param[out] value interpolated value based on the bilinear interpolation
   * @param[in] sub_px position in the matrix to interpolate the subpixel value
   *                   (row,col)
   * @return true interpolated point is inside the matrix
   * @return false not inside the matrix
   */
  template <typename Scalar_>
  bool getSubPxValue(CellType& value,
                     const Eigen::Matrix<Scalar_, 2, 1>& sub_px) const
  {
    if (!isInside(sub_px.x(), sub_px.y()))
    {
      return false;
    }

    int x0 = static_cast<int>(sub_px.x());
    int y0 = static_cast<int>(sub_px.y());
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (!isInside(x1, y1))
    {
      return false;
    }

    const Scalar_ dx0 = sub_px.x() - static_cast<Scalar_>(x0);
    const Scalar_ dy0 = sub_px.y() - static_cast<Scalar_>(y0);
    const Scalar_ dx1 = static_cast<Scalar_>(1.0) - dx0;
    const Scalar_ dy1 = static_cast<Scalar_>(1.0) - dy0;

    CellTraits::setZero(value);
    CellTraits::sumAndScale(value, this->coeff(x0, y0), dx1 * dy1);
    CellTraits::sumAndScale(value, this->coeff(x1, y0), dx0 * dy1);
    CellTraits::sumAndScale(value, this->coeff(x0, y1), dx1 * dy0);
    CellTraits::sumAndScale(value, this->coeff(x1, y1), dx0 * dy0);
    CellTraits::postInterpolate(value);

    return true;
  }

  /**
   * @brief Get the bilinear interpolation weights for a certain subpixel value
   * @tparam Scalar_ scalar type for interpolation
   * @param[out] wx0y0 contributing weight of cell (x0,y0) for subpixel value
   * @param[out] wx1y0 contributing weight of cell (x1,y0) for subpixel value
   * @param[out] wx0y1 contributing weight of cell (x0,y1) for subpixel value
   * @param[out] wx1y1 contributing weight of cell (x1,y1) for subpixel value
   * @param[in] sub_px position in the matrix to interpolate subpixel value
   *                   (row,col)
   * @return true interpolated point is inside the matrix
   * @return false not inside the matrix
   */
  template <typename Scalar_>
  bool getSubPxWeights(Scalar_& wx0y0, Scalar_& wx1y0, Scalar_& wx0y1,
                       Scalar_& wx1y1,
                       const Eigen::Matrix<Scalar_, 2, 1>& sub_px) const
  {
    if (!isInside(sub_px.x(), sub_px.y()))
    {
      return false;
    }

    int x0 = static_cast<int>(sub_px.x());
    int y0 = static_cast<int>(sub_px.y());
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (!isInside(x1, y1))
    {
      return false;
    }

    const Scalar_ dx0 = sub_px.x() - static_cast<Scalar_>(x0);
    const Scalar_ dy0 = sub_px.y() - static_cast<Scalar_>(y0);
    const Scalar_ dx1 = static_cast<Scalar_>(1.0) - dx0;
    const Scalar_ dy1 = static_cast<Scalar_>(1.0) - dy0;

    wx0y0 = dx1 * dy1;
    wx1y0 = dx0 * dy1;
    wx0y1 = dx1 * dy0;
    wx1y1 = dx0 * dy0;

    return true;
  }

  /**
   * @brief Get the subpixel value and the respective bilinear interpolation
   *        weights for a certain subpixel point
   * @tparam Scalar_ scalar type for interpolation
   * @param[out] value interpolated value based on the bilinear interpolation
   * @param[out] wx0y0 contributing weight of cell (x0,y0) for subpixel value
   * @param[out] wx1y0 contributing weight of cell (x1,y0) for subpixel value
   * @param[out] wx0y1 contributing weight of cell (x0,y1) for subpixel value
   * @param[out] wx1y1 contributing weight of cell (x1,y1) for subpixel value
   * @param[in] sub_px position in the matrix to interpolate subpixel value
   *                   (row,col)
   * @return true interpolated point is inside the matrix
   * @return false not inside the matrix
   */
  template <typename Scalar_>
  bool getSubPxAndWeights(CellType& value, Scalar_& wx0y0, Scalar_& wx1y0,
                          Scalar_& wx0y1, Scalar_& wx1y1,
                          const Eigen::Matrix<Scalar_, 2, 1>& sub_px) const
  {
    if (!isInside(sub_px.x(), sub_px.y()))
    {
      return false;
    }

    int x0 = static_cast<int>(sub_px.x());
    int y0 = static_cast<int>(sub_px.y());
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (!isInside(x1, y1))
    {
      return false;
    }

    const Scalar_ dx0 = sub_px.x() - static_cast<Scalar_>(x0);
    const Scalar_ dy0 = sub_px.y() - static_cast<Scalar_>(y0);
    const Scalar_ dx1 = static_cast<Scalar_>(1.0) - dx0;
    const Scalar_ dy1 = static_cast<Scalar_>(1.0) - dy0;

    wx0y0 = dx1 * dy1;
    wx1y0 = dx0 * dy1;
    wx0y1 = dx1 * dy0;
    wx1y1 = dx0 * dy0;

    CellTraits::setZero(value);
    CellTraits::sumAndScale(value, this->coeff(x0, y0), wx0y0);
    CellTraits::sumAndScale(value, this->coeff(x1, y0), wx1y0);
    CellTraits::sumAndScale(value, this->coeff(x0, y1), wx0y1);
    CellTraits::sumAndScale(value, this->coeff(x1, y1), wx1y1);
    CellTraits::postInterpolate(value);

    return true;
  }

};  // struct MatrixSparse_<CellType_, CellTraits_, IndexType_, HashType_>

using MatrixSparseIndexUInt = MatrixSparseIndex_<uint32_t>;
using MatrixSparseIndexInt = MatrixSparseIndex_<int32_t>;

using MatrixSparseHashUInt = MatrixSparseHash_<MatrixSparseIndexUInt>;
using MatrixSparseHashInt = MatrixSparseHash_<MatrixSparseIndexInt>;

using MatrixSparseUInt8 = MatrixSparse_<uint8_t>;
using MatrixSparseChar = MatrixSparse_<char>;
using MatrixSparseUChar = MatrixSparse_<unsigned char>;
using MatrixSparseInt = MatrixSparse_<int>;
using MatrixSparseFloat = MatrixSparse_<float>;
using MatrixSparseDouble = MatrixSparse_<double>;

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
