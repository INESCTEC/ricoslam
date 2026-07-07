#pragma once

#include <limits>
#include <queue>

// Eigen
#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/matrix_sparse.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief default operation traits for a distance map cell (w/ distance and
 *        first -order derivative parameters)
 * @note traits API compatible with the Matrix data structure (e.g. interpolate)
 * @tparam CellType_ distance map cell type
 */
template <typename CellType_>
struct DefaultDistanceMapCellTraits_
{
 public:

  using ThisType = DefaultDistanceMapCellTraits_<CellType_>;
  using CellType = CellType_;

 public:

  /**
   * @brief set to zero the destination cell
   * @param[in,out] dest destination cell
   */
  inline static void setZero(CellType& dest)
  {
    dest.m_dist = 0;
    dest.m_dist_d.setZero();
  }

  /**
   * @brief sum and scale the source cell into the current value of the
   *        destination cell
   * @param[in,out] dest destination cell
   * @param[in] src source cell
   * @param[in] scale scalar scale
   */
  inline static void sumAndScale(CellType& dest, const CellType& src,
                                 float scale)
  {
    dest.m_dist += src.m_dist * scale;
    dest.m_dist_d += src.m_dist_d * scale;
  }

  /**
   * @brief helper function in case of an operation posterior to the
   *        interpolation step (e.g., using the getSubPxValue implementation of
   *        the bilinear interpolation)
   * @param[in,out] dest destination cell
   */
  inline static void postInterpolate(CellType& /* dest */) {}

};  // struct DefaultDistanceMapCellTraits_<CellType_>

/**
 * @brief operation traits for a distance map cell with both first and
 *        second-order derivatives
 * @note traits API compatible with the Matrix data structure (e.g. interpolate)
 * @tparam CellType_ distance map cell type
 */
template <typename CellType_>
struct DistanceMapWith2ndDerivCellTraits_
{
 public:

  using ThisType = DistanceMapWith2ndDerivCellTraits_<CellType_>;
  using CellType = CellType_;

 public:

  /**
   * @brief set to zero the destination cell
   * @param[in,out] dest destination cell
   */
  inline static void setZero(CellType& dest)
  {
    dest.m_dist = 0;
    dest.m_dist_d.setZero();
    dest.m_dist_d2.setZero();
  }

  /**
   * @brief sum and scale the source cell into the current value of the
   *        destination cell
   * @param[in,out] dest destination cell
   * @param[in] src source cell
   * @param[in] scale scalar scale
   */
  inline static void sumAndScale(CellType& dest, const CellType& src,
                                 float scale)
  {
    dest.m_dist += src.m_dist * scale;
    dest.m_dist_d += src.m_dist_d * scale;
    dest.m_dist_d2 += src.m_dist_d2 * scale;
  }

  /**
   * @brief helper function in case of an operation posterior to the
   *        interpolation step (e.g., using the getSubPxValue implementation of
   *        the bilinear interpolation)
   * @param[in,out] dest destination cell
   */
  inline static void postInterpolate(CellType& /* dest */) {}

};  // struct DistanceMapWith2ndDerivCellTraits_<CellType_>

/**
 * @brief operation traits for a distance map cell to be compatible with the API
 *        from the Matrix data structure with EMPTY / NON-EXISTENT operations
 * @tparam CellType_ distance map cell type
 */
template <typename CellType_>
struct DistanceMapEmptyOperationsCellTraits_
{
 public:

  using ThisType = DistanceMapEmptyOperationsCellTraits_<CellType_>;
  using CellType = CellType_;

 public:

  /**
   * @brief set to zero the destination cell (...does not do anything...)
   * @param[in,out] dest destination cell
   */
  inline static void setZero(CellType& /* dest */) {}

  /**
   * @brief sum and scale the source cell into the current value of the
   *        destination cell (...does not do anything...)
   * @param[in,out] dest destination cell
   * @param[in] src source cell
   * @param[in] scale scalar scale
   */
  inline static void sumAndScale(CellType& /* dest */,
                                 const CellType& /* src */, float /* scale */)
  {
  }

  /**
   * @brief helper function in case of an operation posterior to the
   *        interpolation step (e.g., using the getSubPxValue implementation of
   *        the bilinear interpolation) (...does not do anything...)
   * @param[in,out] dest destination cell
   */
  inline static void postInterpolate(CellType& /* dest */) {}

};  // struct DistanceMapEmptyOperationsCellTraits_<CellType_>

template <typename Scalar_>
struct DistanceMapCell_
{
 public:

  using ThisType = DistanceMapCell_<Scalar_>;
  using Scalar = Scalar_;
  using NNIdxType = int32_t;
  using Index = MatrixSparseIndexInt;
  using IndexHash = MatrixSparseHash_<Index>;
  using Traits = DefaultDistanceMapCellTraits_<ThisType>;

  using VectorType = Eigen::Matrix<Scalar, 1, 2>;
  using MatrixType = void;

 public:

  NNIdxType m_nn_idx = -1;  //!< original point cloud index (nearest neighbor)

  Scalar m_dist;        //!< distance computed by the distance map (m)
  VectorType m_dist_d;  //!< distance first derivative along x and y (d/dx,d/dy)

 public:

  /**
   * @brief Construct a new DistanceMapCell_<Scalar_> object
   * @param[in] dist distance computed by the distance map (m)
   * @param[in] nn_idx original point cloud index (nearest neighbor)
   * @param[in] dist_d distance first derivative along x and y directions
   *                   (d/dx,d/dy)
   */
  DistanceMapCell_(Scalar dist = std::numeric_limits<Scalar>::max(),
                   NNIdxType nn_idx = -1,
                   VectorType dist_d = VectorType::Zero())
      : m_nn_idx(nn_idx), m_dist(dist), m_dist_d(dist_d)
  {
  }

  /**
   * @brief Construct a new DistanceMapCell_<Scalar_> object
   * @param[in] other distance map initialization cell
   */
  DistanceMapCell_(const ThisType& other)
      : m_nn_idx(other.m_nn_idx), m_dist(other.m_dist), m_dist_d(other.m_dist_d)
  {
  }

  /**
   * @brief copy method from an existent distance map cell
   * @param[in] other distance map cell
   * @return ThisType& the cell itself
   */
  inline ThisType& operator=(const ThisType& other)
  {
    this->m_nn_idx = other.m_nn_idx;
    this->m_dist = other.m_dist;
    this->m_dist_d = other.m_dist_d;

    return *this;
  }

  /**
   * @brief compasion cell operator based on the distance value (e.g., for
   *        breadth search distance map computation with priority queues)
   * @param[in] e another distance map cell
   * @return true other cell with lowest distance to neaarest neighbor
   * @return false otherwise
   */
  inline bool operator<(const ThisType& e) const { return e.m_dist < m_dist; }

};  // struct DistanceMapCell_<Scalar_>;

template <typename Scalar_>
struct DistanceMapWith2ndDerivCell_
{
 public:

  using ThisType = DistanceMapWith2ndDerivCell_<Scalar_>;
  using Scalar = Scalar_;
  using NNIdxType = int32_t;
  using Index = MatrixSparseIndexInt;
  using IndexHash = MatrixSparseHash_<Index>;
  using Traits = DistanceMapWith2ndDerivCellTraits_<ThisType>;

  using VectorType = Eigen::Matrix<Scalar, 1, 2>;
  using MatrixType = Eigen::Matrix<Scalar, 2, 2>;

 public:

  NNIdxType m_nn_idx = -1;  //!< original point cloud index (nearest neighbor)

  Scalar m_dist;        //!< distance computed by the distance map (m)
  VectorType m_dist_d;  //!< distance first derivative along x and y (d/dx,d/dy)
  MatrixType m_dist_d2;  //!< second distance derivative
                         //!< d²(d(.))/dxdx d²(d(.))/dxdy ;
                         //!< d²(d(.))/dydx d²(d(.))/dydy

 public:

  /**
   * @brief Construct a new DistanceMapWith2ndDerivCell_<Scalar_> object
   * @param[in] dist distance computed by the distance map (m)
   * @param[in] dist_d first distance derivative (d/dx; d/dy)
   * @param[in] dist_d2 second distance derivative
   *                    d²(d(.))/dxdx d²(d(.))/dxdy ;
   *                    d²(d(.))/dydx d²(d(.))/dydy
   *                    over NORMALIZED first derivative
   * @param[in] nn_idx original point cloud index (nearest neighbor)
   */
  DistanceMapWith2ndDerivCell_(Scalar dist = std::numeric_limits<Scalar>::max(),
                               NNIdxType nn_idx = -1,
                               VectorType dist_d = VectorType::Zero(),
                               MatrixType dist_d2 = MatrixType::Zero())
      : m_nn_idx(nn_idx), m_dist(dist), m_dist_d(dist_d), m_dist_d2(dist_d2)
  {
  }

  /**
   * @brief Construct a new DistanceMapWith2ndDerivCell_<Scalar_> object
   * @param[in] other distance map initialization cell
   */
  DistanceMapWith2ndDerivCell_(const ThisType& other)
      : m_nn_idx(other.m_nn_idx),
        m_dist(other.m_dist),
        m_dist_d(other.m_dist_d),
        m_dist_d2(other.m_dist_d2)
  {
  }

  /**
   * @brief copy method from an existent distance map cell
   * @param[in] other distance map cell
   * @return ThisType& the cell itself
   */
  inline ThisType& operator=(const ThisType& other)
  {
    this->m_nn_idx = other.m_nn_idx;
    this->m_dist = other.m_dist;
    this->m_dist_d = other.m_dist_d;
    this->m_dist_d2 = other.m_dist_d2;

    return *this;
  }

  /**
   * @brief compasion cell operator based on the distance value (e.g., for
   *        breadth search distance map computation with priority queues)
   * @param[in] e another distance map cell
   * @return true other cell with lowest distance to neaarest neighbor
   * @return false otherwise
   */
  inline bool operator<(const ThisType& e) const { return e.m_dist < m_dist; }

};  // struct DistanceMapWith2ndDerivCell_<Scalar_>

struct DistanceMapNNPointIdxCell
{
 public:

  using ThisType = DistanceMapNNPointIdxCell;
  using Scalar = void;
  using Index = MatrixSparseIndexInt;
  using IndexScalar = typename Index::Scalar;
  using IndexHash = MatrixSparseHash_<Index>;
  using Traits = DistanceMapEmptyOperationsCellTraits_<ThisType>;

  using VectorType = void;
  using MatrixType = void;

 public:

  IndexScalar m_nn_idx = -1;  //!< original point cloud index (nearest neighbor)

 public:

  /**
   * @brief Construct a new DistanceMapNNPointIdxCell object
   * @param[in] nn_idx original point cloud index (nearest neighbor)
   */
  DistanceMapNNPointIdxCell(IndexScalar nn_idx = -1) : m_nn_idx(nn_idx) {}

  /**
   * @brief Construct a new DistanceMapNNPointIdxCell object
   * @param[in] other distance map initialization cell
   */
  DistanceMapNNPointIdxCell(const ThisType& other) : m_nn_idx(other.m_nn_idx) {}

  /**
   * @brief copy method from an existent distance map cell
   * @param[in] other distance map cell
   * @return ThisType& the cell itself
   */
  inline ThisType& operator=(const ThisType& other)
  {
    this->m_nn_idx = other.m_nn_idx;

    return *this;
  }

};  // struct DistanceMapNNPointIdxCell

/**
 * @brief distance map search cell (only has the pointer to the original cell,
 *        assuming that the distance map container is able to compute the cell
 *        index based on the cell's pointer)
 * @tparam CellType_ distance map cell type
 */
template <typename CellType_>
struct DistanceMapSearchCell_
{
 public:

  using ThisType = DistanceMapSearchCell_<CellType_>;
  using CellType = CellType_;

 public:

  CellType* m_cell = nullptr;  //!< distance map cell pointer to the original
                               //!< cell in the map container

 public:

  /**
   * @brief Construct a new DistanceMapSearchCell_<CellType_> object
   *        (default constructor)
   * @param[in] cell pointer to the original cell in the map container
   */
  DistanceMapSearchCell_(CellType* cell = nullptr) : m_cell(cell) {}

  /**
   * @brief Construct a new DistanceMapSearchCell_<CellType_> object
   * @param[in] cell distance map cell
   */
  DistanceMapSearchCell_(CellType& cell) : m_cell(&cell) {}

  /**
   * @brief Construct a new DistanceMapSearchCell_<CellType_> object
   * @param[in] other copy constructor
   */
  DistanceMapSearchCell_(const ThisType& other) : m_cell(other.m_cell) {}

  /**
   * @brief compasion cell operator based on the distance value (e.g., for
   *        breadth search distance map computation with priority queues)
   * @param[in] e another distance map search cell
   * @return true other cell with lowest distance to nearest neighbor
   * @return false otherwise
   */
  inline bool operator<(const ThisType& e) const
  {
    return e.m_cell->m_dist < m_cell->m_dist;
  }

};  // struct DistanceMapSearchCell_<CellType_>

/**
 * @brief distance map search cell with internal distance field (instead of
 *        getting the latter from the cell itself)
 * @tparam CellType_ distance map cell type
 * @tparam DistScalar_ distance field scalar type
 */
template <typename CellType_, typename DistScalar_>
struct DistanceMapSearchCellWithDist_
{
 public:

  using ThisType = DistanceMapSearchCellWithDist_<CellType_, DistScalar_>;
  using CellType = CellType_;
  using Scalar = DistScalar_;

 public:

  CellType* m_cell = nullptr;  //!< distance map cell pointer to the original
                               //!< cell in the map container
  Scalar m_dist;  //!< distance associated to the cell (any dimensions)

 public:

  /**
   * @brief Construct a new DistanceMapSearchCellWithDist_<CellType_> object
   *        (default constructor)
   * @param[in] cell pointer to the original cell in the map container
   * @param[in] dist distance of the original cell to the closest neighbor (m)
   */
  DistanceMapSearchCellWithDist_(
      CellType* cell = nullptr,
      Scalar dist = std::numeric_limits<Scalar>::max())
      : m_cell(cell), m_dist(dist)
  {
  }

  /**
   * @brief Construct a new DistanceMapSearchCellWithDist_<CellType_> object
   * @param[in] cell distance map cell
   * @param[in] dist distance of the original cell to the closest neighbor (m)
   */
  DistanceMapSearchCellWithDist_(
      CellType& cell, Scalar dist = std::numeric_limits<Scalar>::max())
      : m_cell(&cell), m_dist(dist)
  {
  }

  /**
   * @brief Construct a new DistanceMapSearchCellWithDist_<CellType_> object
   * @param[in] other copy constructor
   */
  DistanceMapSearchCellWithDist_(const ThisType& other)
      : m_cell(other.m_cell), m_dist(other.m_dist)
  {
  }

  /**
   * @brief compasion cell operator based on the distance value (e.g., for
   *        breadth search distance map computation with priority queues)
   * @param[in] e another distance map search cell
   * @return true other cell with lowest distance to nearest neighbor
   * @return false otherwise
   */
  inline bool operator<(const ThisType& e) const
  {
    return e.m_dist < this->m_dist;
  }

};  // struct DistanceMapSearchCellWithDist_<CellType_>

/**
 * @brief specific distance map sparse search cell for sparse distance maps
 *        (in the sparse representation, the hash map container through the
 *        std::unordered_map implementation does not provide the CELL >>> KEY
 *        mapping required to use a standard DistanceMapSearchCell_<CellType_>)
 * @tparam CellType_ distance map sparse cell type
 */
template <typename CellType_>
struct DistanceMapSparseSearchCell_
{
 public:

  using ThisType = DistanceMapSparseSearchCell_<CellType_>;
  using CellType = CellType_;
  using CellIndex = typename CellType::Index;
  using CellIndexScalar = typename CellIndex::Scalar;

 public:

  CellType* m_cell = nullptr;  //!< distance map cell pointer to the original
                               //!< cell in the map container
  CellIndex m_cell_idx;  //!< matrix cell index of the current cell in the queue
  CellIndex m_parent_idx;  //!< matrix cell index of the current cell's parent
                           //!< (closest neighbour)

 public:

  /**
   * @brief Construct a new DistanceMapSparseSearchCell_<CellType_> object
   *        (default constructor)
   * @param[in] cell distance map cell pointer to the original cell
   * @param[in] cell_idx matrix cell index of the current cell in the queue
   * @param[in] parent_idx matrix cell index of the current cell's closest
   *                       neighbour
   */
  DistanceMapSparseSearchCell_(CellType* cell = nullptr,
                               CellIndex cell_idx = CellIndex(
                                   std::numeric_limits<CellIndexScalar>::max(),
                                   std::numeric_limits<CellIndexScalar>::max()),
                               CellIndex parent_idx = CellIndex(
                                   std::numeric_limits<CellIndexScalar>::max(),
                                   std::numeric_limits<CellIndexScalar>::max()))
      : m_cell(cell), m_cell_idx(cell_idx), m_parent_idx(parent_idx)
  {
  }

  /**
   * @brief Construct a new DistanceMapSparseSearchCell_<CellType_> object
   *        (default constructor)
   * @param[in] cell distance map cell
   * @param[in] cell_idx matrix cell index of the current cell in the queue
   * @param[in] parent_idx matrix cell index of the current cell's closest
   *                       neighbour
   */
  DistanceMapSparseSearchCell_(CellType& cell,
                               CellIndex cell_idx = CellIndex(
                                   std::numeric_limits<CellIndexScalar>::max(),
                                   std::numeric_limits<CellIndexScalar>::max()),
                               CellIndex parent_idx = CellIndex(
                                   std::numeric_limits<CellIndexScalar>::max(),
                                   std::numeric_limits<CellIndexScalar>::max()))
      : m_cell(&cell), m_cell_idx(cell_idx), m_parent_idx(parent_idx)
  {
  }

  /**
   * @brief Construct a new DistanceMapSparseSearchCell_<CellType_> object
   * @param[in] other copy constructor
   */
  DistanceMapSparseSearchCell_(const ThisType& other)
      : m_cell(other.m_cell),
        m_cell_idx(other.m_cell_idx),
        m_parent_idx(other.m_parent_idx)
  {
  }

  /**
   * @brief compasion cell operator based on the distance value (e.g., for
   *        breadth search distance map computation with priority queues)
   * @param[in] e another distance map sparse search cell
   * @return true other cell with lowest distance to nearest neighbor
   * @return false otherwise
   */
  inline bool operator<(const ThisType& e) const
  {
    return e.m_cell->m_dist < m_cell->m_dist;
  }

};  // struct DistanceMapSparseSearchCell_<CellType_>

/**
 * @brief specific distance map sparse search cell for sparse distance maps
 *        (in the sparse representation, the hash map container through the
 *        std::unordered_map implementation does not provide the CELL >>> KEY
 *        mapping required to use a standard DistanceMapSearchCell_<CellType_>)
 * @tparam CellType_ distance map sparse cell type
 * @tparam DistScalar_ distance field scalar type
 */
template <typename CellType_, typename DistScalar_>
struct DistanceMapSparseSearchCellWithDist_
{
 public:

  using ThisType = DistanceMapSparseSearchCellWithDist_<CellType_, DistScalar_>;
  using CellType = CellType_;
  using CellIndex = typename CellType::Index;
  using CellIndexScalar = typename CellIndex::Scalar;
  using CellScalar = DistScalar_;

 public:

  CellType* m_cell = nullptr;  //!< distance map cell pointer to the original
                               //!< cell in the map container
  CellScalar m_cell_dist;  //!< distance associated to the cell (any dimensions)
  CellIndex m_cell_idx;  //!< matrix cell index of the current cell in the queue
  CellIndex m_parent_idx;  //!< matrix cell index of the current cell's parent
                           //!< (closest neighbour)

 public:

  /**
   * @brief Construct a new
   *        DistanceMapSparseSearchCellWithDist_<CellType_, DistScalar_> object
   *        (default constructor)
   * @param[in] cell distance map cell pointer to the original cell
   * @param[in] cell_dist distance of the cell to the closest neighbor point (m)
   * @param[in] cell_idx matrix cell index of the current cell in the queue
   * @param[in] parent_idx matrix cell index of the current cell's closest
   *                       neighbour
   */
  DistanceMapSparseSearchCellWithDist_(
      CellType* cell = nullptr,
      CellScalar cell_dist = std::numeric_limits<CellScalar>::max(),
      CellIndex cell_idx =
          CellIndex(std::numeric_limits<CellIndexScalar>::max(),
                    std::numeric_limits<CellIndexScalar>::max()),
      CellIndex parent_idx =
          CellIndex(std::numeric_limits<CellIndexScalar>::max(),
                    std::numeric_limits<CellIndexScalar>::max()))
      : m_cell(cell),
        m_cell_dist(cell_dist),
        m_cell_idx(cell_idx),
        m_parent_idx(parent_idx)
  {
  }

  /**
   * @brief Construct a new
   *        DistanceMapSparseSearchCellWithDist_<CellType_, DistScalar_> object
   *        (default constructor)
   * @param[in] cell distance map cell
   * @param[in] cell_dist distance of the cell to the closest neighbor point (m)
   * @param[in] cell_idx matrix cell index of the current cell in the queue
   * @param[in] parent_idx matrix cell index of the current cell's closest
   *                       neighbour
   */
  DistanceMapSparseSearchCellWithDist_(
      CellType& cell,
      CellScalar cell_dist = std::numeric_limits<CellScalar>::max(),
      CellIndex cell_idx =
          CellIndex(std::numeric_limits<CellIndexScalar>::max(),
                    std::numeric_limits<CellIndexScalar>::max()),
      CellIndex parent_idx =
          CellIndex(std::numeric_limits<CellIndexScalar>::max(),
                    std::numeric_limits<CellIndexScalar>::max()))
      : m_cell(&cell),
        m_cell_dist(cell_dist),
        m_cell_idx(cell_idx),
        m_parent_idx(parent_idx)
  {
  }

  /**
   * @brief Construct a new
   *        DistanceMapSparseSearchCellWithDist_<CellType_, DistScalar_> object
   * @param[in] other copy constructor
   */
  DistanceMapSparseSearchCellWithDist_(const ThisType& other)
      : m_cell(other.m_cell),
        m_cell_dist(other.m_cell_dist),
        m_cell_idx(other.m_cell_idx),
        m_parent_idx(other.m_parent_idx)
  {
  }

  /**
   * @brief comparison cell operator based on the distance value (e.g., for
   *        breadth search distance map computation with priority queues)
   * @param[in] e another distance map sparse search cell
   * @return true other cell with lowest distance to nearest neighbor
   * @return false otherwise
   */
  inline bool operator<(const ThisType& e) const
  {
    return e.m_cell_dist < this->m_cell_dist;
  }

};  // struct DistanceMapSparseSearchCellWithDist_<CellType_, DistScalar_>

using DistanceMapCellFloat = DistanceMapCell_<float>;
using DistanceMapCellDouble = DistanceMapCell_<double>;

using DistanceMapWith2ndDerivCellFloat = DistanceMapWith2ndDerivCell_<float>;
using DistanceMapWith2ndDerivCellDouble = DistanceMapWith2ndDerivCell_<double>;

template <typename CellType_>
using DistanceMapCellSearchQueue_ =
    std::priority_queue<DistanceMapSearchCell_<CellType_>>;

template <typename CellType_>
using DistanceMapSparseCellSearchQueue_ =
    std::priority_queue<DistanceMapSparseSearchCell_<CellType_>>;

template <typename CellType_, typename DistScalar_>
using DistanceMapCellSearchWithDistQueue_ =
    std::priority_queue<DistanceMapSearchCellWithDist_<CellType_, DistScalar_>>;

template <typename CellType_, typename DistScalar_>
using DistanceMapSparseCellSearchWithDistQueue_ = std::priority_queue<
    DistanceMapSparseSearchCellWithDist_<CellType_, DistScalar_>>;

using DistanceMapCellFloatSearchQueue =
    DistanceMapCellSearchQueue_<DistanceMapCellFloat>;
using DistanceMapCellDoubleSearchQueue =
    DistanceMapCellSearchQueue_<DistanceMapCellDouble>;

using DistanceMapSparseCellFloatSearchQueue =
    DistanceMapSparseCellSearchQueue_<DistanceMapCellFloat>;
using DistanceMapSparseCellDoubleSearchQueue =
    DistanceMapSparseCellSearchQueue_<DistanceMapCellDouble>;

using DistanceMapWith2ndDerivCellFloatSearchQueue =
    DistanceMapCellSearchQueue_<DistanceMapWith2ndDerivCellFloat>;
using DistanceMapWith2ndDerivCellDoubleSearchQueue =
    DistanceMapCellSearchQueue_<DistanceMapWith2ndDerivCellDouble>;

using DistanceMapSparseWith2ndDerivCellFloatSearchQueue =
    DistanceMapSparseCellSearchQueue_<DistanceMapWith2ndDerivCellFloat>;
using DistanceMapSparseWith2ndDerivCellDoubleSearchQueue =
    DistanceMapSparseCellSearchQueue_<DistanceMapWith2ndDerivCellDouble>;

using DistanceMapNNPointIdxCellSearchWithDistFloatQueue =
    DistanceMapCellSearchWithDistQueue_<DistanceMapNNPointIdxCell, float>;
using DistanceMapNNPointIdxCellSearchWithDistDoubleQueue =
    DistanceMapCellSearchWithDistQueue_<DistanceMapNNPointIdxCell, double>;

using DistanceMapSparseNNPointIdxCellSearchWithDistFloatQueue =
    DistanceMapSparseCellSearchWithDistQueue_<DistanceMapNNPointIdxCell, float>;
using DistanceMapSparseNNPointIdxCellSearchWithDistDoubleQueue =
    DistanceMapSparseCellSearchWithDistQueue_<DistanceMapNNPointIdxCell,
                                              double>;

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
