#pragma once

#include <iterator>
#include <vector>

// Eigen
#include <Eigen/StdVector>

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief default cell traits (useful to make mathematical operations with cells
 *        that may be a structure)
 * @tparam CellType_ cell type of the matrix
 */
template <typename CellType_>
struct DefaultCellTraits
{
 public:

  /**
   * @brief set to zero the destination cell
   * @param[in,out] dest destination cell
   */
  inline static void setZero(CellType_& dest) { dest = 0; }

  /**
   * @brief sum and scale the source cell into the current value of the
   *        destination cell
   * @tparam Scalar_ scalar type
   * @param[in,out] dest destination cell
   * @param[in] src source cell
   * @param[in] scale scalar scale
   */
  template <typename Scalar_>
  inline static void sumAndScale(CellType_& dest, const CellType_& src,
                                 Scalar_ scale)
  {
    dest += src * scale;
  }

  /**
   * @brief helper function in case of an operation posterior to the
   *        interpolation step (e.g., using the getSubPxValue implementation of
   *        the bilinear interpolation)
   * @param[in,out] dest destination cell
   */
  inline static void postInterpolate(CellType_& dest __attribute__((unused))) {}

};  // DefaultCellTraits<CellType_>

/**
 * @brief generic 2D matrix class
 * @tparam CellType_ cell type of the matrix
 * @tparam CellTraits_ cell traits type
 * @tparam CellAlign_ alignment of the matrix in memory (row / column majorwise)
 */
template <typename CellType_,
          typename CellTraits_ = DefaultCellTraits<CellType_>,
          Eigen::StorageOptions CellAlign_ = Eigen::RowMajor>
class Matrix_ : public Eigen::Matrix<CellType_, Eigen::Dynamic, Eigen::Dynamic,
                                     CellAlign_>
{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ThisType = Matrix_<CellType_, CellTraits_, CellAlign_>;
  using BaseType =
      Eigen::Matrix<CellType_, Eigen::Dynamic, Eigen::Dynamic, CellAlign_>;

  using CellType = CellType_;
  using CellTraits = CellTraits_;

  static constexpr int kCellAlign = CellAlign_;

  /**
   * @brief default constructor
   */
  Matrix_() : BaseType() {}

  /**
   * @brief construct a new object and resize it to #rows and #cols
   * @param[in] rows number of rows
   * @param[in] cols number of cols
   */
  Matrix_(size_t rows, size_t cols) : BaseType(rows, cols) {}

  /**
   * @brief fill the matrix with the same value
   * @param[in] cell_value desired cell value to fill the matrix with
   */
  void fill(const CellType& cell_value)
  {
    for (int r = 0; r < this->rows(); r++)
    {
      for (int c = 0; c < this->cols(); c++)
      {
        (*this)(r, c) = cell_value;
      }
    }
  }

  /**
   * @brief get element at position indexed by row and column
   * @param[in] r row index
   * @param[in] c column index
   * @return CellType& cell desired cell at position (row,column)
   */
  inline CellType& at(std::size_t r, std::size_t c) { return (*this)(r, c); }

  /**
   * @brief get element at position indexed by row and column (const)
   * @param[in] r row index
   * @param[in] c column index
   * @return CellType& cell desired cell at position (row,column)
   */
  inline const CellType& at(std::size_t r, std::size_t c) const
  {
    return (*this)(r, c);
  }

  /**
   * @brief get element at position indexed by Eigen::Vector2i
   * @param[in] pos cell position
   * @return CellType& cell desired cell set by Eigen::Vector2i
   */
  inline CellType& at(const Eigen::Vector2i& pos)
  {
    return (*this)(pos(0), pos(1));
  }

  /**
   * @brief get element at position indexed by Eigen::Vector2i (const)
   * @param[in] pos cell position
   * @return CellType& cell desired cell set by Eigen::Vector2i
   */
  inline const CellType& at(const Eigen::Vector2i& pos) const
  {
    return (*this)(pos(0), pos(1));
  }

  /**
   * @brief get element at position indexed by the global position in the data
   *        array (is dependent on if the matrix is organized as RowMajor or
   *        ColMajor)
   * @param[in] idx cell index in the data array
   * @return CellType& cell desired cell at idx position in the data array
   */
  inline CellType& at(std::size_t idx) { return this->data()[idx]; }

  /**
   * @brief get element at position indexed by the global position in the data
   *        array (is dependent on if the matrix is organized as RowMajor or
   *        ColMajor) (const)
   * @param[in] idx cell index in the data array
   * @return CellType& cell desired cell at idx position in the data array
   */
  inline const CellType& at(const std::size_t idx) const
  {
    return this->data()[idx];
  }

  /**
   * @brief compute the index in the data array (inside the Matrix Eigen class)
   *        correspondent to the index position (row, column)
   * @param[in] r row index
   * @param[in] c column index
   * @return size_t index of the cell (row,column) in the data internal array
   */
  inline size_t indexAt(std::size_t r, std::size_t c) const
  {
    switch (kCellAlign)
    {
      case Eigen::ColMajor:
        return c * this->rows() + r;
      default:
        return r * this->cols() + c;
    }
  }

  /**
   * @brief get the position of the cell in the matrix
   * @param[in] cell cell
   * @return Eigen::Vector2i cell position
   */
  inline Eigen::Vector2i getCellPos(const CellType* cell) const
  {
    std::ptrdiff_t index = cell - &(*this)(0, 0);

    assert((index >= 0) && (static_cast<int>(index) <
                            static_cast<int>(this->rows() * this->cols())));

    switch (kCellAlign)
    {
      case Eigen::ColMajor:
      {
        std::div_t divresult =
            div(static_cast<int>(index), static_cast<int>(this->rows()));
        return Eigen::Vector2i(divresult.rem, divresult.quot);
      }
      default:
      {
        std::div_t divresult =
            div(static_cast<int>(index), static_cast<int>(this->cols()));
        return Eigen::Vector2i(divresult.quot, divresult.rem);
      }
    }
  }

  /**
   * @brief get the subpixel value using bilinear interpolation
   * @tparam Scalar_ scalar type for interpolation
   * @param[out] value interpolated value based on the bilinear
   *                                interpolation
   * @param[in] sub_px position in the matrix to interpolate the
   *                            subpixel value (row,col)
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
    CellTraits::sumAndScale(value, (*this)(x0, y0), dx1 * dy1);
    CellTraits::sumAndScale(value, (*this)(x1, y0), dx0 * dy1);
    CellTraits::sumAndScale(value, (*this)(x0, y1), dx1 * dy0);
    CellTraits::sumAndScale(value, (*this)(x1, y1), dx0 * dy0);
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
    CellTraits::sumAndScale(value, (*this)(x0, y0), wx0y0);
    CellTraits::sumAndScale(value, (*this)(x1, y0), wx1y0);
    CellTraits::sumAndScale(value, (*this)(x0, y1), wx0y1);
    CellTraits::sumAndScale(value, (*this)(x1, y1), wx1y1);
    CellTraits::postInterpolate(value);

    return true;
  }

  /**
   * @brief check if a cell is on the border of the matrix
   * @param[in] row row index in the matrix
   * @param[in] col column index in the matrix
   * @return true cell is on the border
   * @return false not on the border
   */
  inline bool isOnBorder(int row, int col) const
  {
    return ((row == 0) || (col == 0) || (row == this->rows() - 1) ||
            (col == this->cols() - 1));
  }

  /**
   * @brief check if a cell is on the border of the matrix
   * @param[in] pos index point (row,col) in the matrix
   * @return true cell is on the border
   * @return false not on the border
   */
  inline bool isOnBorder(const Eigen::Vector2i& pos) const
  {
    return isOnBorder(pos(0), pos(1));
  }

  /**
   * @brief check if a cell is inside the matrix
   * @param[in] row row index in the matrix
   * @param[in] col column index in the matrix
   * @return true cell is inside the matrix
   * @return false not inside the matrix
   */
  inline bool isInside(int row, int col) const
  {
    return ((row >= 0) && (col >= 0) && (row < this->rows()) &&
            (col < this->cols()));
  }

  /**
   * @brief check if a cell is inside the matrix
   * @param[in] pos index point (row,col) in the matrix
   * @return true cell is inside the matrix
   * @return false not inside the matrix
   */
  inline bool isInside(const Eigen::Vector2i& pos) const
  {
    return isInside(pos(0), pos(1));
  }

 public:

  /**
   * @brief STL compatible forward iterator for the Matrix_ class
   * source:
   * https://internalpointers.com/post/writing-custom-iterators-modern-cpp
   */
  struct Iterator
  {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = CellType;
    using pointer = CellType*;
    using reference = CellType&;

   private:

    //! internal pointer of the iterator
    pointer m_ptr;

   public:

    /**
     * @brief construct a new iterator object based on a pointer to a CellType
     *        element
     *
     * @param[in] ptr pointer to a CellType element
     */
    Iterator(pointer ptr) : m_ptr(ptr) {}

    reference operator*() const { return *m_ptr; }

    pointer operator->() { return m_ptr; }

    Iterator& operator++()
    {
      m_ptr++;
      return *this;
    }

    Iterator operator++(int)
    {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }

    friend bool operator==(const Iterator& a, const Iterator& b)
    {
      return a.m_ptr == b.m_ptr;
    };

    friend bool operator!=(const Iterator& a, const Iterator& b)
    {
      return a.m_ptr != b.m_ptr;
    };

  };  // struct Iterator

 public:

  /**
   * @brief begin utility method for the iterator (compatible w/ for(auto& : ))
   * @return Iterator iterator set to the first element in the matrix (0,0)
   */
  Iterator begin() { return Iterator(&(*this)(0, 0)); }

  /**
   * @brief end utility method for the iterator (compatible w/ for(auto& : ))
   * @return Iterator iterator set to the last element in the matrix
   *         (rows-1,cols-1)
   */
  Iterator end()
  {
    return Iterator(&(*this)(this->rows() - 1, this->cols() - 1));
  }

};  // class Matrix_<CellType_, CellTraits_, CellAlign_>

using MatrixUInt8 = Matrix_<uint8_t>;
using MatrixChar = Matrix_<char>;
using MatrixUChar = Matrix_<unsigned char>;
using MatrixInt = Matrix_<int>;
using MatrixFloat = Matrix_<float>;
using MatrixDouble = Matrix_<double>;

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
