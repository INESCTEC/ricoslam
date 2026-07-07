#pragma once

#include <exception>
#include <functional>
#include <iostream>
#include <unordered_map>

// Eigen
#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/matrix_sparse.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

using MatrixSparseMapIndex = MatrixSparseIndex_<int32_t>;
using MatrixSparseMapHash = MatrixSparseHash_<MatrixSparseIndex_<int32_t>>;

/**
 * @brief generic 2D matrix sparse map class implementation based on C++ STL
 *        hash map (unordered_map)
 * This sparse map allows to have negative indexes. Instead of a matrix being
 * defined by its number of rows and columns, it is delimited by a inferior and
 * superior limits (kind of a bottom left and top right points).
 * @tparam CellType_ matrix cell type
 * @tparam CellTraits_ cell traits type (setZero, sumAndScale, ...)
 */
template <typename CellType_,
          typename CellTraits_ = DefaultCellTraits<CellType_>>
struct MatrixSparseMap_
    : std::unordered_map<MatrixSparseMapIndex, CellType_, MatrixSparseMapHash>
{
 public:

  using BaseType =
      std::unordered_map<MatrixSparseMapIndex, CellType_, MatrixSparseMapHash>;
  using ThisType = MatrixSparseMap_<CellType_, CellTraits_>;
  using CellType = CellType_;
  using CellTraits = CellTraits_;

  using Index = MatrixSparseMapIndex;
  using IndexScalar = typename Index::Scalar;

  using Hash = MatrixSparseMapHash;

 protected:

  //! matrix index limit inferior
  Index m_lim_inf_ = Index(0, 0);

  //! matrix index limit superior
  Index m_lim_sup_ = Index(0, 0);

  //! default cell value for the sparse matrix
  CellType m_default_v_;

 public:

  /**
   * @brief Construct a new MatrixSparseMap_<CellType_, CellTraits_> object
   *        (default constructor)
   */
  MatrixSparseMap_() : m_lim_inf_(Index(0, 0)), m_lim_sup_(Index(0, 0))
  {
    CellTraits::setZero(m_default_v_);
  }

  /**
   * @brief Construct a new MatrixSparseMap_<CellType_, CellTraits_> object
   *        defining its initial size based on matrix index limits
   * @param[in] lim_inf matrix index limit superior
   * @param[in] lim_sup matrix index limit superior
   */
  MatrixSparseMap_(const Index& lim_inf, const Index& lim_sup)
      : m_lim_inf_(lim_inf), m_lim_sup_(lim_sup)
  {
    CellTraits::setZero(m_default_v_);
  }

  /**
   * @brief Construct a new MatrixSparseMap_<CellType_, CellTraits_> object
   *        defining its initial size based on matrix index limits and default
   *        value for the cells
   * @param[in] lim_inf matrix index limit superior
   * @param[in] lim_sup matrix index limit superior
   * @param[in] default_v cell default value (if not changed in the meantime)
   */
  MatrixSparseMap_(const Index& lim_inf, const Index& lim_sup,
                   const CellType& default_v)
      : m_lim_inf_(lim_inf), m_lim_sup_(lim_sup), m_default_v_(default_v)
  {
  }

  /**
   * @brief Construct a new MatrixSparseMap_<CellType_, CellTraits_> object
   *        defining its initial size based on number of rows and columns
   *        (assuming matrix index limit inferior as (0,0))
   * @param[in] rows number of rows
   * @param[in] cols number of cols
   */
  MatrixSparseMap_(IndexScalar rows, IndexScalar cols)
      : m_lim_inf_(Index(0, 0)), m_lim_sup_(Index(rows - 1, cols - 1))
  {
    CellTraits::setZero(m_default_v_);
  }

  /**
   * @brief Construct a new MatrixSparseMap_<CellType_, CellTraits_> object
   *        defining its initial size based on number of rows and columns
   *        (assuming matrix index limit inferior as (0,0)) and default
   *        value for the cells
   * @param[in] rows number of rows
   * @param[in] cols number of cols
   * @param[in] default_v cell default value (if not changed in the meantime)
   */
  MatrixSparseMap_(IndexScalar rows, IndexScalar cols,
                   const CellType& default_v)
      : m_lim_inf_(Index(0, 0)),
        m_lim_sup_(Index(rows - 1, cols - 1)),
        m_default_v_(default_v)
  {
  }

  /**
   * @brief Get number of rows of the sparse matrix map
   * @return IndexScalar number of rows
   */
  inline IndexScalar rows() const noexcept
  {
    return m_lim_sup_.m_row - m_lim_inf_.m_row + 1;
  }

  /**
   * @brief Get number of columns of the sparse matrix map
   * @return IndexScalar number of columns
   */
  inline IndexScalar cols() const noexcept
  {
    return m_lim_sup_.m_col - m_lim_inf_.m_col + 1;
  }

  /**
   * @brief Get matrix index limit inferior of the sparse matrix map
   * @return Index matrix index limit inferior
   */
  inline Index limInf() const noexcept { return m_lim_inf_; }

  /**
   * @brief Get matrix index limit superior of the sparse matrix map
   * @return Index matrix index limit superior
   */
  inline Index limSup() const noexcept { return m_lim_sup_; }

  /**
   * @brief Get matrix index limit inferior on the rows direction of the sparse
   *        matrix map
   * @return IndexScalar row matrix index limit inferior
   */
  inline IndexScalar limInfRows() const noexcept { return m_lim_inf_.m_row; }

  /**
   * @brief Get matrix index limit inferior on the columns direction of the
   *        sparse matrix map
   * @return IndexScalar column matrix index limit inferior
   */
  inline IndexScalar limInfCols() const noexcept { return m_lim_inf_.m_col; }

  /**
   * @brief Get matrix index limit superior on the rows direction of the sparse
   *        matrix map
   * @return IndexScalar row matrix index limit superior
   */
  inline IndexScalar limSupRows() const noexcept { return m_lim_sup_.m_row; }

  /**
   * @brief Get matrix index limit superior on the columns direction of the
   *        sparse matrix map
   * @return IndexScalar column matrix index limit superior
   */
  inline IndexScalar limSupCols() const noexcept { return m_lim_sup_.m_col; }

  /**
   * @brief Get default value set for the sparse matrix map cells
   *        (if the matrix element was not redefined / created in the hash map,
   *        the default value is the one considered for otehr
   *        operations / getters)
   * @return CellType default value cell
   */
  inline CellType defaultValue() const noexcept { return m_default_v_; }

  /**
   * @brief Get number of non-default value cells in the sparse matrix map
   * @return IndexScalar number of non-default value cells
   */
  inline IndexScalar nonDefaultValues() const noexcept { return this->size(); }

  /**
   * @brief Get number of zero-value cells in the sparse matrix map
   * @return IndexScalar number of zero-value cells in the sparse matrix map
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
   * @brief Total size (width x height, based on the index limits) of the
   *        sparse matrix map
   * @return IndexScalar total number of cells in the sparse matrix map
   *                     (may not be allocated in the hash map!)
   */
  inline IndexScalar totalSize() const noexcept
  {
    return this->rows() * this->cols();
  }

  /**
   * @brief Evaluate if a cell index is inside the sparse matrix map
   * @param[in] row row index of the cell
   * @param[in] col column index of the cell
   * @return true cell inside the sparse matrix map limits
   * @return false otherwise
   */
  inline bool isInside(IndexScalar row, IndexScalar col) const
  {
    return ((row >= m_lim_inf_.m_row) && (col >= m_lim_inf_.m_col) &&
            (row <= m_lim_sup_.m_row) && (col <= m_lim_sup_.m_col));
  }

  /**
   * @brief Evaluate if a cell index is inside the sparse matrix map
   * @param[in] index cell index
   * @return true cell inside the sparse matrix map limits
   * @return false otherwise
   */
  inline bool isInside(const Index& index) const
  {
    return ((index.m_row >= m_lim_inf_.m_row) &&
            (index.m_col >= m_lim_inf_.m_col) &&
            (index.m_row <= m_lim_sup_.m_row) &&
            (index.m_col <= m_lim_sup_.m_col));
  }

  /**
   * @brief Evaluate if a cell index is inside the sparse matrix map
   * @param[in] pos cell index (Eigen vector representation)
   * @return true cell inside the sparse matrix map limits
   * @return false otherwise
   */
  inline bool isInside(const Eigen::Vector2i& pos) const
  {
    return ((static_cast<IndexScalar>(pos.x()) >= m_lim_inf_.m_row) &&
            (static_cast<IndexScalar>(pos.y()) >= m_lim_inf_.m_col) &&
            (static_cast<IndexScalar>(pos.x()) <= m_lim_sup_.m_row) &&
            (static_cast<IndexScalar>(pos.y()) <= m_lim_sup_.m_col));
  }

  /**
   * @brief Evaluate if a cell is on the border limits of a sparse matrix map
   * @param[in] row row index of the cell
   * @param[in] col column index of the cell
   * @return true cell on the border of the sparse matrix map limits
   * @return false otherwise
   */
  inline bool isOnBorder(IndexScalar row, IndexScalar col) const
  {
    return ((row == m_lim_inf_.m_row) || (col == m_lim_inf_.m_col) ||
            (row == m_lim_sup_.m_row) || (col == m_lim_sup_.m_col));
  }

  /**
   * @brief Evaluate if a cell is on the border limits of a sparse matrix map
   * @param[in] index cell index
   * @return true cell on the border of the sparse matrix map limits
   * @return false otherwise
   */
  inline bool isOnBorder(const Index& index) const
  {
    return ((index.m_row == m_lim_inf_.m_row) ||
            (index.m_col == m_lim_inf_.m_col) ||
            (index.m_row == m_lim_sup_.m_row) ||
            (index.m_col == m_lim_sup_.m_col));
  }

  /**
   * @brief Evaluate if a cell is on the border limits of a sparse matrix map
   * @param[in] pos cell index (Eigen vector representation)
   * @return true cell on the border of the sparse matrix map limits
   * @return false otherwise
   */
  inline bool isOnBorder(const Eigen::Vector2i& pos) const
  {
    return ((static_cast<IndexScalar>(pos.x()) == m_lim_inf_.m_row) ||
            (static_cast<IndexScalar>(pos.y()) == m_lim_inf_.m_col) ||
            (static_cast<IndexScalar>(pos.x()) == m_lim_sup_.m_row) ||
            (static_cast<IndexScalar>(pos.y()) == m_lim_sup_.m_col));
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
      throw std::out_of_range("MatrixSparseMap_::coeff | index outside matrix");
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
      throw std::out_of_range("MatrixSparseMap_::coeff | index outside matrix");
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
      throw std::out_of_range("MatrixSparseMap_::coeff | index outside matrix");
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
      throw std::out_of_range("MatrixSparseMap_::coeff | index outside matrix");
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
      throw std::out_of_range("MatrixSparseMap_::coeff | index outside matrix");
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
      throw std::out_of_range("MatrixSparseMap_::coeff | index outside matrix");
    }

    return (*this)[Index(pos.x(), pos.y())];
  }

  /**
   * @brief Resize the sparse map matrix (#rows, #columns) (if after resize
   *        there are cells outside the matrix, these are erased from the has
   *        table)
   * @param[in] rows number of rows
   * @param[in] cols number of columns
   */
  void resize(IndexScalar rows, IndexScalar cols)
  {
    bool erase_outside_cells =
        (m_lim_inf_.m_row < 0) || (m_lim_inf_.m_col < 0) ||
        (m_lim_sup_.m_row > rows - 1) || (m_lim_sup_.m_col > cols - 1);

    m_lim_inf_ = Index(0, 0);
    m_lim_sup_ = Index(rows - 1, cols - 1);

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
   * @brief Resize the sparse map matrix (limit inferior and superior index
   *        sizes) (if after resize there are cells outside the matrix, these
   *        are erased from the has table)
   * @param[in] lim_inf matrix index limit superior
   * @param[in] lim_sup matrix index limit superior
   */
  void resize(const Index& lim_inf, const Index& lim_sup)
  {
    bool erase_outside_cells = (m_lim_inf_.m_row < lim_inf.m_row) ||
                               (m_lim_inf_.m_col < lim_inf.m_col) ||
                               (m_lim_sup_.m_row > lim_sup.m_row) ||
                               (m_lim_sup_.m_col > lim_sup.m_col);

    m_lim_inf_ = lim_inf;
    m_lim_sup_ = lim_sup;

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

};  // struct MatrixSparseMap_<CellType_, CellTraits_>

using MatrixSparseMapUInt8 = MatrixSparseMap_<uint8_t>;
using MatrixSparseMapChar = MatrixSparseMap_<char>;
using MatrixSparseMapUChar = MatrixSparseMap_<unsigned char>;
using MatrixSparseMapInt = MatrixSparseMap_<int>;
using MatrixSparseMapFloat = MatrixSparseMap_<float>;
using MatrixSparseMapDouble = MatrixSparseMap_<double>;

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
