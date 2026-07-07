#pragma once

#include <limits>
#include <memory>
#include <set>
#include <type_traits>
#include <vector>

// Eigen
#include <Eigen/Core>

// SRRG Software
#include <srrg2_core/srrg_boss/object_data.h>
#include <srrg2_core/srrg_boss/serializable.h>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_cell.hpp"
#include "inesctec_mrdt_slam_distmap_2d/data_structures/distance_map_defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/data_structures/image_defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/data_structures/matrix.hpp"
#include "inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_types.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

/**
 * @brief distance map template class to compute the distance and respective
 *        derivatives to the closest nearborhood cells based on a set of goals
 *        (dense matrix map structure = Eigen matrix)
 * @tparam CellType_ distance map cell type
 */
template <typename CellType_>
class DistanceMapStatic_ : public srrg2_core::Serializable
{
 public:

  using ThisType = DistanceMapStatic_<CellType_>;
  using BaseType = srrg2_core::Serializable;

  using CellType = CellType_;
  using CellIndex = typename CellType::Index;
  using CellIndexHash = typename CellType::IndexHash;
  using CellIndexScalar = typename CellIndex::Scalar;
  using CellScalar = typename CellType::Scalar;
  using CellTraits = typename CellType::Traits;
  using CellVectorType = typename CellType::VectorType;
  using CellMatrixType = typename CellType::MatrixType;
  using CellNNIdxType = typename CellType::NNIdxType;

  using CellTypeStd = DistanceMapCell_<CellScalar>;
  using CellTypeWith2ndDeriv = DistanceMapWith2ndDerivCell_<CellScalar>;

  using Scalar = CellScalar;
  using PointVectorType = geometry::Vector2_<Scalar>;

  using MapType = Matrix_<CellType, CellTraits>;
  using MapTypePtr = std::unique_ptr<MapType>;

  using CellSearchQueueType = DistanceMapCellSearchQueue_<CellType>;
  using CellSearchQueueTypeCell = typename CellSearchQueueType::value_type;

 public:

  enum Channel  //!< selector for channel to create OpenCV compatible images
  {
    kDist,       //!< distance channel (m)
    kDistDx,     //!< first derivative along x (d(d(.))/dx)
    kDistDy,     //!< first derivative along y (d(d(.))/dy)
    kDistD2Dxx,  //!< second derivative over NORMALIZED first derivative
                 //!< d²(d(.))/dxdx (0,0)
    kDistD2Dxy,  //!< second derivative over NORMALIZED first derivative
                 //!< d²(d(.))/dxdy (0,1)
    kDistD2Dyx,  //!< second derivative over NORMALIZED first derivative
                 //!< d²(d(.))/dydx (1,0)
    kDistD2Dyy   //!< second derivative over NORMALIZED first derivative
                 //!< d²(d(.))/dydy (1,1)
  };  // enum Channel

 protected:

  Scalar m_res_;       //!< resolution of the distance map (m/px)
  Scalar m_map_size_;  //!< fixate the size (square width / 2) of the distance
                       //!< map (m) (-1 if you want to assume the maximum range
                       //!< in the point cloud used to setGoals)
  Scalar m_max_dist_;  //!< maximum distance considered to compute distance map
                       //!< (m) (-1 if you want to compute whole distance map)

  MapTypePtr m_map_;  //!< distance map container (smart pointer to allow the
                      //!< deallocation of the memory with the distance map's
                      //!< free method)

  bool m_compute_all_map_ =
      false;  //!< internal variable that defines if the distance map will be
              //!< computed for all its matrix allocation or only up to
              //!< m_max_dist_ to the closest point

  PointVectorType
      m_pc_min_;  //!< point coordinates in global coordinate frame
                  //!< corresponding to (0,0) in mat required to convert global
                  //!< / sensor coordinates into local / pixel ones

  CellSearchQueueType
      m_queue_;  //!< priority queue used to compute the distance map

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct a new DistanceMapStatic_<CellType_>
   *        object (default initialization of the map unique pointer - you will
   *        be able to deallocate the map memory later on...)
   */
  DistanceMapStatic_()
  {
    m_map_ = std::make_unique<MapType>();
  }  // DistanceMapStatic_::DistanceMapStatic_()

  /**
   * @brief find neighbors around a cell in the distance map (N8 neighborhood)
   * @param[out] neighbors static vector of 8 elements (may not be all
   *                       initialized) to return the neighbors around the cell
   * @param[in] r row index of the current cell
   * @param[in] c column index of the current cell
   * @return int number of neighbors found
   */
  inline int findNeighbors(CellType** neighbors, int r, int c)
  {
    int rmin = r - 1 < 0 ? 0 : r - 1;
    int rmax = r + 1 > m_map_->rows() - 1 ? m_map_->rows() - 1 : r + 1;

    int cmin = c - 1 < 0 ? 0 : c - 1;
    int cmax = c + 1 > m_map_->cols() - 1 ? m_map_->cols() - 1 : c + 1;

    int k = 0;

    for (int rr = rmin; rr <= rmax; rr++)
    {
      for (int cc = cmin; cc <= cmax; cc++)
      {
        if ((rr != r) || (cc != c))
        {
          neighbors[k] = &(*m_map_)(rr, cc);
          k++;
        }
      }
    }

    return k;
  }  // inline int DistanceMapStatic_::findNeighbors(CellType**, int, int)

  /**
   * @brief set the goal points to compute the distance map
   * @tparam PointCloudType point cloud type
   * @param[in] pc point cloud (m)
   * @param[in] res resolution for the map (m/px)
   * @param[in] map_size fixate the size (square width / 2) of the distance map
   *                     (m) (-1 if you want to assume the maximum range in the
   *                     point cloud used to setGoals)
   * @param[in] max_dist maximum distance considered to compute the distance map
   *                     (m) (-1 if you want to compute the whole distance
   *                     map)
   */
  template <typename PointCloudType>
  void setGoals(const PointCloudType& pc, Scalar res,
                Scalar map_size = Scalar(-1.0), Scalar max_dist = Scalar(-1.0))
  {
    assert(res > 0 &&
           "DistanceMapStatic_::setGoals | "
           "resolution must be greater than 0 (m/px)");

    if (!m_map_)
    {
      m_map_ = std::make_unique<MapType>();
    }

    m_res_ = res;
    m_map_size_ = map_size;
    m_max_dist_ = max_dist;

    if ((pc.size() == 0) || (!pc.countStatus(pcl::PointStatus::kValid)))
    {
      m_map_->resize(0, 0);
      m_pc_min_.setZero();
      return;
    }

    PointVectorType pc_lim_min(std::numeric_limits<Scalar>::max(),
                               std::numeric_limits<Scalar>::max());
    PointVectorType pc_lim_max(-std::numeric_limits<Scalar>::max(),
                               -std::numeric_limits<Scalar>::max());

    if (m_map_size_ > 0)
    {
      pc_lim_min.x() = -m_map_size_;
      pc_lim_min.y() = -m_map_size_;
      pc_lim_max.x() = m_map_size_;
      pc_lim_max.y() = m_map_size_;
    }
    else
    {
      for (size_t idx = 0; idx < pc.size(); idx++)
      {
        if (pc[idx].m_status != pcl::PointStatus::kValid)
        {
          continue;
        }

        pc_lim_min.x() = std::min(pc_lim_min.x(), pc[idx].coordinates().x());
        pc_lim_min.y() = std::min(pc_lim_min.y(), pc[idx].coordinates().y());
        pc_lim_max.x() = std::max(pc_lim_max.x(), pc[idx].coordinates().x());
        pc_lim_max.y() = std::max(pc_lim_max.y(), pc[idx].coordinates().y());
      }
    }

    PointVectorType pc_lim_delta = pc_lim_max - pc_lim_min;

    m_compute_all_map_ = (m_max_dist_ <= 0);

    int rows, cols;

    if ((!m_compute_all_map_) && (m_map_size_ <= 0))
    {
      rows = static_cast<int>(
                 (pc_lim_delta.x() + m_max_dist_ * static_cast<Scalar>(2)) /
                 m_res_) +
             1;
      cols = static_cast<int>(
                 (pc_lim_delta.y() + m_max_dist_ * static_cast<Scalar>(2)) /
                 m_res_) +
             1;

      m_pc_min_.x() =
          pc_lim_min.x() - m_max_dist_;  // need this to account for max_dist!!!
      m_pc_min_.y() = pc_lim_min.y() - m_max_dist_;
    }
    else
    {
      rows = static_cast<int>((pc_lim_delta.x()) / m_res_) + 1;
      cols = static_cast<int>((pc_lim_delta.y()) / m_res_) + 1;

      m_pc_min_ = pc_lim_min;
    }

    m_map_->resize(rows, cols);

    if (!m_compute_all_map_)
    {
      m_map_->fill(CellType(m_max_dist_));
    }
    else
    {
      m_map_->fill(CellType());
    }

    m_queue_ = CellSearchQueueType();

    for (size_t idx = 0; idx < pc.size(); idx++)
    {
      if (pc[idx].m_status != pcl::PointStatus::kValid)
      {
        continue;
      }

      const Eigen::Vector2i map_idx = global2local(pc[idx].coordinates());

      if (!m_map_->isInside(map_idx))
      {
        continue;
      }

      (*m_map_)(map_idx.x(), map_idx.y()) =
          CellType(static_cast<Scalar>(0.0), idx);

      m_queue_.emplace(
          CellSearchQueueTypeCell(&(*m_map_)(map_idx.x(), map_idx.y())));
    }

    compute(pc);
  }  // void DistanceMapStatic_::setGoals(const PointCloudType&, Scalar, Scalar
     // = -1.0, Scalar = -1.0)

  /**
   * @brief convert point cloud point coordinates in the global frame to the
   *        distance map index coordinates (int index)
   *
   * This conversion method depends on the resolution of the distance map set
   * when the method setGoals is called, while also considering the maximum map
   * size range and maximum distance to compute the map (influence the size of
   * the map).
   *
   * @param[in] pt point in the global coordinate frame (m)
   * @return Eigen::Vector2i point in the distance map matrix (px)
   */
  inline Eigen::Vector2i global2local(const PointVectorType& pt) const
  {
    return Eigen::Vector2i(static_cast<int>((pt.x() - m_pc_min_.x()) / m_res_),
                           static_cast<int>((pt.y() - m_pc_min_.y()) / m_res_));
  }

  /**
   * @brief convert point cloud point coordinates in the global frame to the
   *        distance map index coordinates (float index)
   *
   * Differs from global2local by not making a static cast to int, enabling
   * future sub-pixelation using bilinear interpolation.
   *
   * @param[in] pt point in the global coordinate frame (m)
   * @return PointVectorType sub-pixel point in the distance map matrix (px)
   */
  inline const PointVectorType global2localSubPx(
      const PointVectorType& pt) const
  {
    return PointVectorType((pt.x() - m_pc_min_.x()) / m_res_,
                           (pt.y() - m_pc_min_.y()) / m_res_);
  }

  /**
   * @brief convert distance map index coordinates (row, column) to global cloud
   *        point coordinates
   * @param[in] index point in the distance map matrix - row, column (px)
   * @return const PointVectorType point coordinates in the global frame (m)
   */
  inline const PointVectorType local2global(const Eigen::Vector2i& index) const
  {
    return PointVectorType((index.x() * m_res_ + m_pc_min_.x()),
                           (index.y() * m_res_ + m_pc_min_.y()));
  }

  /**
   * @brief free the dynamic memory allocated to the distance map
   */
  inline void free()
  {
    m_map_.reset();
    m_queue_ = CellSearchQueueType();
  }

 protected:

  template <typename PointCloudType>
  void compute(const PointCloudType& pc)
  {
    CellType* neighbors[8];

    while (!m_queue_.empty())
    {
      CellType* current = m_queue_.top().m_cell;
      const Eigen::Vector2i map_idx_current = m_map_->getCellPos(current);

      m_queue_.pop();

      if (current->m_nn_idx < 0)
      {
        continue;
      }

      const Eigen::Vector2i map_idx_parent =
          global2local(pc.at(current->m_nn_idx).coordinates());

      int k =
          findNeighbors(neighbors, map_idx_current.x(), map_idx_current.y());

      for (int idx = 0; idx < k; idx++)
      {
        CellType* children = neighbors[idx];
        Eigen::Vector2i map_idx_children = m_map_->getCellPos(children);

        int r = map_idx_children.x();
        int c = map_idx_children.y();
        int dr = r - map_idx_parent.x();
        int dc = c - map_idx_parent.y();

        Scalar dist = std::sqrt(dr * dr + dc * dc) * m_res_;

        if (((dist < m_max_dist_) || (m_compute_all_map_)) &&
            (children->m_dist > dist))
        {
          children->m_nn_idx = current->m_nn_idx;
          children->m_dist = dist;

          m_queue_.emplace(CellSearchQueueTypeCell(children));
        }
      }
    }

    computeDistDx();
    computeDistDy();
    computeDistD2();
  }  // void DistanceMapStatic_::compute(const PointCloudType&)

  void computeDistDx()
  {
    for (int c = 0; c < m_map_->cols(); c++)
    {
      if (m_map_->rows() > 1)
      {
        (*m_map_)(0, c).m_dist_d(0) =
            (*m_map_)(1, c).m_dist - (*m_map_)(0, c).m_dist;

        (*m_map_)(m_map_->rows() - 1, c).m_dist_d(0) =
            (*m_map_)(m_map_->rows() - 1, c).m_dist -
            (*m_map_)(m_map_->rows() - 2, c).m_dist;

        (*m_map_)(0, c).m_dist_d(0) /= m_res_;
        (*m_map_)(m_map_->rows() - 1, c).m_dist_d(0) /= m_res_;
      }
      else
      {
        (*m_map_)(0, c).m_dist_d(0) = 0;
      }

      for (int r = 1; r < m_map_->rows() - 1; r++)
      {
        (*m_map_)(r, c).m_dist_d(0) =
            (*m_map_)(r + 1, c).m_dist - (*m_map_)(r - 1, c).m_dist;

        (*m_map_)(r, c).m_dist_d(0) *= 0.5 / m_res_;
      }
    }
  }  // void DistanceMapStatic_::computeDistDx()

  void computeDistDy()
  {
    for (int r = 0; r < m_map_->rows(); r++)
    {
      if (m_map_->cols() > 1)
      {
        (*m_map_)(r, 0).m_dist_d(1) =
            (*m_map_)(r, 1).m_dist - (*m_map_)(r, 0).m_dist;

        (*m_map_)(r, m_map_->cols() - 1).m_dist_d(1) =
            (*m_map_)(r, m_map_->cols() - 1).m_dist -
            (*m_map_)(r, m_map_->cols() - 2).m_dist;

        (*m_map_)(r, 0).m_dist_d(1) /= m_res_;
        (*m_map_)(r, m_map_->cols() - 1).m_dist_d(1) /= m_res_;
      }
      else
      {
        (*m_map_)(r, 0).m_dist_d(1) = 0;
      }

      for (int c = 1; c < m_map_->cols() - 1; c++)
      {
        (*m_map_)(r, c).m_dist_d(1) =
            (*m_map_)(r, c + 1).m_dist - (*m_map_)(r, c - 1).m_dist;

        (*m_map_)(r, c).m_dist_d(1) *= 0.5 / m_res_;
      }
    }
  }  // void DistanceMapStatic_::computeDistDy()

  template <typename T = CellMatrixType,
            std::enable_if_t<std::is_same<T, void>::value, bool> = 0>
  void computeDistD2()
  {
  }

  template <typename T = CellMatrixType,
            std::enable_if_t<!std::is_same<T, void>::value, bool> = 0>
  void computeDistD2()
  {
    for (int r = 0; r < m_map_->rows(); r++)
    {
      for (int c = 0; c < m_map_->cols(); c++)
      {
        CellType& cell = (*m_map_)(r, c);

        if (m_map_->isOnBorder(r, c))
        {
          cell.m_dist_d2.setZero();
          continue;
        }

        CellVectorType dist_d_dxdiff_1 = (*m_map_)(r + 1, c).m_dist_d;
        CellVectorType dist_d_dxdiff_0 = (*m_map_)(r - 1, c).m_dist_d;
        CellVectorType dist_d_dydiff_1 = (*m_map_)(r, c + 1).m_dist_d;
        CellVectorType dist_d_dydiff_0 = (*m_map_)(r, c - 1).m_dist_d;

        dist_d_dxdiff_1.normalize();
        dist_d_dxdiff_0.normalize();
        dist_d_dydiff_1.normalize();
        dist_d_dydiff_0.normalize();

        CellVectorType dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;
        CellVectorType dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

        cell.m_dist_d2 << dist_d_dxdiff(0), dist_d_dydiff(0), dist_d_dxdiff(1),
            dist_d_dydiff(1);

        cell.m_dist_d2 *= 0.5 / m_res_;
      }
    }
  }  // void DistanceMapStatic_::computeDistD2()

  /****************************************************************************
   * @brief OPENCV HELPER FUNCTIONS TO GENERATE IMAGES
   ****************************************************************************/

 public:

  void toImageIdx(RGBImage& img) const
  {
    size_t num_pts;

    std::set<int32_t> num_pts_set;

    for (int r = 0; r < m_map_->cols(); r++)
    {
      for (int c = 0; c < m_map_->rows(); c++)
      {
        if ((*m_map_)(c, r).m_nn_idx >= 0)
        {
          num_pts_set.emplace((*m_map_)(c, r).m_nn_idx);
        }
      }
    }

    num_pts = num_pts_set.size();

    img.create(m_map_->cols(), m_map_->rows());
    img = cv::Vec3b(0, 0, 0);

    for (int r = 0; r < m_map_->cols(); r++)
    {
      for (int c = 0; c < m_map_->rows(); c++)
      {
        int idx = (*m_map_)(c, r).m_nn_idx + 1;

        unsigned char nidx = static_cast<unsigned char>(std::round(
            255 * static_cast<Scalar>(idx) / static_cast<Scalar>(num_pts + 1)));

        img.at<cv::Vec3b>(r, c) = cv::Vec3b(nidx, nidx, nidx);
      }
    }
  }  // void DistanceMapStatic_::toImageIdx(RGBImage&) const

  template <typename PointCloudType>
  void toImageDist(RGBImage& img, const PointCloudType& pc)
  {
    toImage(img, Channel::kDist, pc);
  }

  template <typename PointCloudType>
  void toImageDistDx(RGBImage& img, const PointCloudType& pc)
  {
    toImage(img, Channel::kDistDx, pc);
  }

  template <typename PointCloudType>
  void toImageDistDy(RGBImage& img, const PointCloudType& pc)
  {
    toImage(img, Channel::kDistDy, pc);
  }

  template <typename PointCloudType>
  void toImageDistD2Dxx(RGBImage& img, const PointCloudType& pc)
  {
    toImage(img, Channel::kDistD2Dxx, pc);
  }
  template <typename PointCloudType>
  void toImageDistD2Dxy(RGBImage& img, const PointCloudType& pc)
  {
    toImage(img, Channel::kDistD2Dxy, pc);
  }
  template <typename PointCloudType>
  void toImageDistD2Dyx(RGBImage& img, const PointCloudType& pc)
  {
    toImage(img, Channel::kDistD2Dyx, pc);
  }
  template <typename PointCloudType>
  void toImageDistD2Dyy(RGBImage& img, const PointCloudType& pc)
  {
    toImage(img, Channel::kDistD2Dyy, pc);
  }

  /**
   * @brief draw the point cloud with circle markers onto an OpenCV RGB image
   * @tparam DrawPointCloudType_ point cloud type (only matters coordinates())
   * @param[in,out] img OpenCV RGB image
   * @param[in] pc point cloud (m)
   * @param[in] color color for the circle markers (OpenCV color)
   * @param[in] radius radius of the circle marker (px)
   */
  template <typename DrawPointCloudType_>
  void drawPoints(RGBImage& img, const DrawPointCloudType_& pc,
                  const cv::Scalar& color, int radius = 3) const
  {
    assert(static_cast<int>(m_map_->rows()) == img.cols);
    assert(static_cast<int>(m_map_->cols()) == img.rows);

    for (size_t idx = 0; idx < pc.size(); idx++)
    {
      const Eigen::Vector2i& map_idx = global2local(pc[idx].coordinates());

      if (!m_map_->isInside(map_idx))
      {
        continue;
      }

      cv::circle(img, cv::Point(map_idx.x(), map_idx.y()), radius, color);
    }
  }  // void DistanceMapStatic_::drawPoints<DrawPointCloudType_>(RGBImage&,
     // const DrawPointCloudType_&, const cv::Scalar&, int = 3) const

  /**
   * @brief draw the point cloud w.r.t. the distance map coordinates
   *        with circle markers onto an OpenCV RGB image
   * @tparam DrawPointCloudType_ point cloud type (only matters coordinates())
   * @param[in,out] img OpenCV RGB image
   * @param[in] pc point cloud in the distance map coordinates (px)
   * @param[in] color color for the circle markers (OpenCV color)
   * @param[in] radius radius of the circle marker (px)
   */
  template <typename DrawPointCloudType_>
  void drawPointsFromMapIdx(RGBImage& img, const DrawPointCloudType_& pc,
                            const cv::Scalar& color, int radius = 3) const
  {
    assert(static_cast<int>(m_map_->rows()) == img.cols);
    assert(static_cast<int>(m_map_->cols()) == img.rows);

    for (size_t idx = 0; idx < pc.size(); idx++)
    {
      Eigen::Vector2i map_idx(static_cast<int>(pc[idx].coordinates().x()),
                              static_cast<int>(pc[idx].coordinates().y()));

      if (!m_map_->isInside(map_idx))
      {
        continue;
      }

      cv::circle(img, cv::Point(map_idx.x(), map_idx.y()), radius, color);
    }
  }  // void
     // DistanceMapStatic_::drawPointsFromMapIdx<DrawPointCloudType_>(RGBImage&,
     // const DrawPointCloudType_&, const cv::Scalar&, int = 3) const

  /**
   * @brief draw the point cloud with a single pixel onto an OpenCV RGB image
   * @tparam DrawPointCloudType_ point cloud type (only matters coordinates())
   * @param[in,out] img OpenCV RGB image
   * @param[in] pc point cloud (m)
   * @param[in] color color for the pixel (OpenCV color)
   */
  template <typename DrawPointCloudType_>
  void drawPx(RGBImage& img, const DrawPointCloudType_& pc,
              const cv::Scalar& color) const
  {
    assert(static_cast<int>(m_map_->rows()) == img.cols);
    assert(static_cast<int>(m_map_->cols()) == img.rows);

    for (size_t idx = 0; idx < pc.size(); idx++)
    {
      const Eigen::Vector2i& map_idx = global2local(pc[idx].coordinates());

      if (!m_map_->isInside(map_idx))
      {
        continue;
      }

      img.at<cv::Vec3b>(map_idx.y(), map_idx.x()) =
          cv::Vec3b(color(0), color(1), color(2));
    }
  }  // void DistanceMapStatic_::drawPx<DrawPointCloudType_>(RGBImage&, const
     // DrawPointCloudType_&, const cv::Scalar&) const

  /**
   * @brief draw the point cloud w.r.t. the distance map coordinates
   *        with a single pixel onto an OpenCV RGB image
   * @tparam DrawPointCloudType_ point cloud type (only matters coordinates())
   * @param[in,out] img OpenCV RGB image
   * @param[in] pc point cloud in the distance map coordinates (px)
   * @param[in] color color for the pixel (OpenCV color)
   */
  template <typename DrawPointCloudType_>
  void drawPxFromMapIdx(RGBImage& img, const DrawPointCloudType_& pc,
                        const cv::Scalar& color) const
  {
    assert(static_cast<int>(m_map_->rows()) == img.cols);
    assert(static_cast<int>(m_map_->cols()) == img.rows);

    for (size_t idx = 0; idx < pc.size(); idx++)
    {
      Eigen::Vector2i map_idx(static_cast<int>(pc[idx].coordinates().x()),
                              static_cast<int>(pc[idx].coordinates().y()));

      if (!m_map_->isInside(map_idx))
      {
        continue;
      }

      img.at<cv::Vec3b>(map_idx.y(), map_idx.x()) =
          cv::Vec3b(color(0), color(1), color(2));
    }
  }  // void
     // DistanceMapStatic_::drawPxFromMapIdx<DrawPointCloudType_>(RGBImage&,
     // const DrawPointCloudType_&, const cv::Scalar&) const

  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<std::is_same<T, void>::value, bool> = 0>
  void toImage(RGBImage& img, Channel channel, const PointCloudType&)
  {
    img.create(m_map_->cols(), m_map_->rows());
    img = cv::Vec3b(0, 0, 0);

    Scalar nmin = std::numeric_limits<Scalar>::max();
    Scalar nmax = -std::numeric_limits<Scalar>::max();

    Scalar nmax_abs;

    auto get_channel_value = [this](int r, int c, Channel channel) -> Scalar
    {
      switch (channel)
      {
        case Channel::kDistDx:
        {
          return (*this->m_map_)(r, c).m_dist_d(0);
          break;
        }
        case Channel::kDistDy:
        {
          return (*this->m_map_)(r, c).m_dist_d(1);
          break;
        }
        case Channel::kDistD2Dxx:
        {
          if (m_map_->isOnBorder(r, c))
          {
            return Scalar(0);
          }

          CellVectorType dist_d_dxdiff_1 = (*m_map_)(r + 1, c).m_dist_d;
          CellVectorType dist_d_dxdiff_0 = (*m_map_)(r - 1, c).m_dist_d;

          dist_d_dxdiff_1.normalize();
          dist_d_dxdiff_0.normalize();

          CellVectorType dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;

          return dist_d_dxdiff(0) * 0.5 / m_res_;
          break;
        }
        case Channel::kDistD2Dxy:
        {
          if (m_map_->isOnBorder(r, c))
          {
            return Scalar(0);
          }

          CellVectorType dist_d_dydiff_1 = (*m_map_)(r, c + 1).m_dist_d;
          CellVectorType dist_d_dydiff_0 = (*m_map_)(r, c - 1).m_dist_d;

          dist_d_dydiff_1.normalize();
          dist_d_dydiff_0.normalize();

          CellVectorType dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

          return dist_d_dydiff(0) * 0.5 / m_res_;
          break;
        }
        case Channel::kDistD2Dyx:
        {
          if (m_map_->isOnBorder(r, c))
          {
            return Scalar(0);
          }

          CellVectorType dist_d_dxdiff_1 = (*m_map_)(r + 1, c).m_dist_d;
          CellVectorType dist_d_dxdiff_0 = (*m_map_)(r - 1, c).m_dist_d;

          dist_d_dxdiff_1.normalize();
          dist_d_dxdiff_0.normalize();

          CellVectorType dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;

          return dist_d_dxdiff(1) * 0.5 / m_res_;
          break;
        }
        case Channel::kDistD2Dyy:
        {
          if (m_map_->isOnBorder(r, c))
          {
            return Scalar(0);
          }

          CellVectorType dist_d_dydiff_1 = (*m_map_)(r, c + 1).m_dist_d;
          CellVectorType dist_d_dydiff_0 = (*m_map_)(r, c - 1).m_dist_d;

          dist_d_dydiff_1.normalize();
          dist_d_dydiff_0.normalize();

          CellVectorType dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

          return dist_d_dydiff(1) * 0.5 / m_res_;
          break;
        }
        default:
        {
          return (*this->m_map_)(r, c).m_dist;
          break;
        }
      }
    };

    for (int r = 0; r < m_map_->cols(); r++)
    {
      for (int c = 0; c < m_map_->rows(); c++)
      {
        const CellType& cell = (*m_map_)(c, r);

        if (cell.m_dist == std::numeric_limits<Scalar>::max())
        {
          continue;
        }

        nmin = std::min(nmin, get_channel_value(c, r, channel));
        nmax = std::max(nmax, get_channel_value(c, r, channel));
      }
    }

    nmax_abs = std::max(std::fabs(nmin), std::fabs(nmax));

    for (int r = 0; r < m_map_->cols(); r++)
    {
      for (int c = 0; c < m_map_->rows(); c++)
      {
        unsigned char v;
        Scalar value = get_channel_value(c, r, channel);

        if (value == 0)
        {
          img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
        }
        else if (value > 0)
        {
          v = static_cast<unsigned char>(std::round(255 * value / nmax_abs));

          img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, v, 0);
        }
        else
        {
          v = static_cast<unsigned char>(std::round(-255 * value / nmax_abs));

          img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, v);
        }
      }
    }
  }  // void toImage(RGBImage& img, Channel channel, const PointCloudType&)

  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<!std::is_same<T, void>::value, bool> = 0>
  void toImage(RGBImage& img, Channel channel, const PointCloudType&)
  {
    img.create(m_map_->cols(), m_map_->rows());
    img = cv::Vec3b(0, 0, 0);

    Scalar nmin = std::numeric_limits<Scalar>::max();
    Scalar nmax = -std::numeric_limits<Scalar>::max();

    Scalar nmax_abs;

    auto get_channel_value = [this](int r, int c, Channel channel) -> Scalar
    {
      switch (channel)
      {
        case Channel::kDistDx:
        {
          return (*this->m_map_)(r, c).m_dist_d(0);
          break;
        }
        case Channel::kDistDy:
        {
          return (*this->m_map_)(r, c).m_dist_d(1);
          break;
        }
        case Channel::kDistD2Dxx:
        {
          return (*this->m_map_)(r, c).m_dist_d2(0, 0);
          break;
        }
        case Channel::kDistD2Dxy:
        {
          return (*this->m_map_)(r, c).m_dist_d2(0, 1);
          break;
        }
        case Channel::kDistD2Dyx:
        {
          return (*this->m_map_)(r, c).m_dist_d2(1, 0);
          break;
        }
        case Channel::kDistD2Dyy:
        {
          return (*this->m_map_)(r, c).m_dist_d2(1, 1);
          break;
        }
        default:
        {
          return (*this->m_map_)(r, c).m_dist;
          break;
        }
      }
    };

    for (int r = 0; r < m_map_->cols(); r++)
    {
      for (int c = 0; c < m_map_->rows(); c++)
      {
        const CellType& cell = (*m_map_)(c, r);

        if (cell.m_dist == std::numeric_limits<Scalar>::max())
        {
          continue;
        }

        nmin = std::min(nmin, get_channel_value(c, r, channel));
        nmax = std::max(nmax, get_channel_value(c, r, channel));
      }
    }

    nmax_abs = std::max(std::fabs(nmin), std::fabs(nmax));

    for (int r = 0; r < m_map_->cols(); r++)
    {
      for (int c = 0; c < m_map_->rows(); c++)
      {
        unsigned char v;
        Scalar value = get_channel_value(c, r, channel);

        if (value == 0)
        {
          img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
        }
        else if (value > 0)
        {
          v = static_cast<unsigned char>(std::round(255 * value / nmax_abs));

          img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, v, 0);
        }
        else
        {
          v = static_cast<unsigned char>(std::round(-255 * value / nmax_abs));

          img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, v);
        }
      }
    }
  }  // void toImage(RGBImage& img, Channel channel, const PointCloudType&)

  /****************************************************************************
   * @brief GETTERS......
   ****************************************************************************/

 public:

  inline Scalar getRes() const { return m_res_; }
  inline Scalar getMapSize() const { return m_map_size_; }
  inline Scalar getMaxDist() const { return m_max_dist_; }
  inline ThisType* getDistanceMap() { return this; }
  inline MapType* getMap() const
  {
    if (m_map_)
    {
      return m_map_.get();
    }
    else
    {
      return nullptr;
    }
  }
  inline const PointVectorType& getPointCloudMin() const { return m_pc_min_; }

  /**
   * @brief get the type of distance map (if it has a lookup table with 2nd
   *        derivative or not)
   * @tparam T cell matrix type
   * @return DistanceMapImplType DistanceMapImplType::kStandard
   */
  template <typename T = CellMatrixType,
            std::enable_if_t<std::is_same<T, void>::value, bool> = 0>
  static inline DistanceMapImplType getType()
  {
    return DistanceMapImplType::kStandard;
  }

  /**
   * @brief get the type of distance map (if it has a lookup table with 2nd
   *        derivative or not)
   * @tparam T cell matrix type
   * @return DistanceMapImplType DistanceMapImplType::kWith2ndDeriv
   */
  template <typename T = CellMatrixType,
            std::enable_if_t<!std::is_same<T, void>::value, bool> = 0>
  static inline DistanceMapImplType getType()
  {
    return DistanceMapImplType::kWith2ndDeriv;
  }

  /**
   * @brief get the type of distance map (if it has a lookup table with 2nd
   *        derivative or not)
   * @tparam T cell matrix type
   * @return const char* type name of the distance map
   */
  template <typename T = CellMatrixType,
            std::enable_if_t<std::is_same<T, void>::value, bool> = 0>
  static inline const char* getTypeName()
  {
    return "DistanceMapStatic "
           "(dense matrix, lookup table with NN point parent idx, distance and "
           "1st derivative)";
  }

  /**
   * @brief get the type of distance map (if it has a lookup table with 2nd
   *        derivative or not)
   * @tparam T cell matrix type
   * @return const char* type name of the distance map
   */
  template <typename T = CellMatrixType,
            std::enable_if_t<!std::is_same<T, void>::value, bool> = 0>
  static inline const char* getTypeName()
  {
    return "DistanceMapStaticWith2ndDeriv "
           "(dense matrix, lookup table with NN point parent idx, distance and "
           "1st+2nd derivatives)";
  }

  inline bool getNN(CellNNIdxType& nn, const PointVectorType& pt) const
  {
    const Eigen::Vector2i& map_idx = global2local(pt);

    assert(m_map_ && "DistanceMapStatic_::getNN | map not created...");

    if (!m_map_->isInside(map_idx))
    {
      return false;
    }

    if ((*m_map_)(map_idx.x(), map_idx.y()).m_nn_idx < 0)
    {
      return false;
    }

    nn = (*m_map_)(map_idx.x(), map_idx.y()).m_nn_idx;

    return true;
  }

  inline bool getNN(CellNNIdxType& nn, const Eigen::Vector2i& pt) const
  {
    assert(m_map_ && "DistanceMapStatic_::getValue | map not created...");

    if (!m_map_->isInside(pt))
    {
      return false;
    }

    if ((*m_map_)(pt.x(), pt.y()).m_nn_idx < 0)
    {
      return false;
    }

    nn = (*m_map_)(pt.x(), pt.y()).m_nn_idx;

    return true;
  }

  /**
   * @brief get value method from the distance map based on a point in the
   *        sensor coordinate map
   * @tparam T cell type (its scalar type must be the same as the internal one
   *           of the distance map)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the global coordinate frame (m)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T, typename PointCloudType,
            std::enable_if_t<
                std::is_same<typename T::Scalar, CellScalar>::value, bool> = 0>
  inline bool getValue(T& value, const PointVectorType& pt,
                       const PointCloudType&) const
  {
    const Eigen::Vector2i& map_idx = global2local(pt);

    assert(m_map_ && "DistanceMapStatic_::getValue | map not created...");

    if (!m_map_->isInside(map_idx) || m_map_->isOnBorder(map_idx))
    {
      return false;
    }

    if ((*m_map_)(map_idx.x(), map_idx.y()).m_nn_idx < 0)
    {
      return false;
    }

    value.m_nn_idx = (*m_map_)(map_idx.x(), map_idx.y()).m_nn_idx;
    value.m_dist = (*m_map_)(map_idx.x(), map_idx.y()).m_dist;
    value.m_dist_d = (*m_map_)(map_idx.x(), map_idx.y()).m_dist_d;

    return true;
  }  // inline bool DistanceMapStatic_::getValue<T, PointCloudType>(T&, const
     // PointVectorType&, const PointCloudType&) const

  /**
   * @brief get value method from the distance map based on a point in the
   *        distance map matrix
   * @tparam T cell type (its scalar type must be the same as the internal one
   *           of the distance map)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the distance map matrix (px)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T, typename PointCloudType,
            std::enable_if_t<
                std::is_same<typename T::Scalar, CellScalar>::value, bool> = 0>
  inline bool getValue(T& value, const Eigen::Vector2i& pt,
                       const PointCloudType&) const
  {
    assert(m_map_ && "DistanceMapStatic_::getValue | map not created...");

    if (!m_map_->isInside(pt) || m_map_->isOnBorder(pt))
    {
      return false;
    }

    if ((*m_map_)(pt.x(), pt.y()).m_nn_idx < 0)
    {
      return false;
    }

    value.m_nn_idx = (*m_map_)(pt.x(), pt.y()).m_nn_idx;
    value.m_dist = (*m_map_)(pt.x(), pt.y()).m_dist;
    value.m_dist_d = (*m_map_)(pt.x(), pt.y()).m_dist_d;

    return true;
  }  // inline bool DistanceMapStatic_::getValue<T, PointCloudType>(T& value,
     // const Eigen::Vector2i&, const PointCloudType&) const

  /**
   * @brief get value method from the distance map based on a point in the
   *        sensor coordinate map, including the 2nd derivative matrix over the
   *        NORMALIZED 1st derivative vector
   * @note this is a template specialization that, in this case, the distance
   *       map container does not have the look-up table for the 2nd derivative
   *       over the NROMALIZED 1st-order derivative vector >>> thus, the second
   *       derivative allowing a manual computation of the 2nd derivative
   *       (to have a common API for the distance map matching formulation using
   *       the 2nd derivative)
   * @tparam T cell matrix type (to detect if cell type already has 2nd deriv)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the global coordinate frame (m)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<std::is_same<T, void>::value, bool> = 0>
  inline bool getValueWith2ndDeriv(CellTypeWith2ndDeriv& value,
                                   const PointVectorType& pt,
                                   const PointCloudType&) const
  {
    const Eigen::Vector2i& map_idx = global2local(pt);

    assert(m_map_ &&
           "DistanceMapStatic_::getValueWith2ndDeriv | map not created...");

    if (!m_map_->isInside(map_idx) || m_map_->isOnBorder(map_idx))
    {
      return false;
    }

    if ((*m_map_)(map_idx.x(), map_idx.y()).m_nn_idx < 0)
    {
      return false;
    }

    value.m_nn_idx = (*m_map_)(map_idx.x(), map_idx.y()).m_nn_idx;
    value.m_dist = (*m_map_)(map_idx.x(), map_idx.y()).m_dist;
    value.m_dist_d = (*m_map_)(map_idx.x(), map_idx.y()).m_dist_d;

    CellVectorType dist_d_dxdiff_1 =
        (*m_map_)(map_idx.x() + 1, map_idx.y()).m_dist_d;
    CellVectorType dist_d_dxdiff_0 =
        (*m_map_)(map_idx.x() - 1, map_idx.y()).m_dist_d;
    CellVectorType dist_d_dydiff_1 =
        (*m_map_)(map_idx.x(), map_idx.y() + 1).m_dist_d;
    CellVectorType dist_d_dydiff_0 =
        (*m_map_)(map_idx.x(), map_idx.y() - 1).m_dist_d;

    dist_d_dxdiff_1.normalize();
    dist_d_dxdiff_0.normalize();
    dist_d_dydiff_1.normalize();
    dist_d_dydiff_0.normalize();

    CellVectorType dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;
    CellVectorType dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

    value.m_dist_d2 << dist_d_dxdiff(0), dist_d_dydiff(0), dist_d_dxdiff(1),
        dist_d_dydiff(1);

    value.m_dist_d2 *= 0.5 / m_res_;

    return true;
  }  // inline bool getValueWith2ndDeriv<PointCloudType>(CellTypeWith2ndDeriv&,
     // const PointVectorType&, const PointCloudType&) const

  /**
   * @brief get value method from the distance map based on a point in the
   *        distance map matrix, including the 2nd derivative matrix over the
   *        NORMALIZED 1st derivative vector
   * @note this is a template specialization that, in this case, the distance
   *       map container does not have the look-up table for the 2nd derivative
   *       over the NROMALIZED 1st-order derivative vector >>> thus, the second
   *       derivative allowing a manual computation of the 2nd derivative
   *       (to have a common API for the distance map matching formulation using
   *       the 2nd derivative)
   * @tparam T cell matrix type (to detect if cell type already has 2nd deriv)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the distance map matrix (px)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<std::is_same<T, void>::value, bool> = 0>
  inline bool getValueWith2ndDeriv(CellTypeWith2ndDeriv& value,
                                   const Eigen::Vector2i& pt,
                                   const PointCloudType&) const
  {
    assert(m_map_ &&
           "DistanceMapStatic_::getValueWith2ndDeriv | map not created...");

    if (!m_map_->isInside(pt) || m_map_->isOnBorder(pt))
    {
      return false;
    }

    if ((*m_map_)(pt.x(), pt.y()).m_nn_idx < 0)
    {
      return false;
    }

    value.m_nn_idx = (*m_map_)(pt.x(), pt.y()).m_nn_idx;
    value.m_dist = (*m_map_)(pt.x(), pt.y()).m_dist;
    value.m_dist_d = (*m_map_)(pt.x(), pt.y()).m_dist_d;

    CellVectorType dist_d_dxdiff_1 = (*m_map_)(pt.x() + 1, pt.y()).m_dist_d;
    CellVectorType dist_d_dxdiff_0 = (*m_map_)(pt.x() - 1, pt.y()).m_dist_d;
    CellVectorType dist_d_dydiff_1 = (*m_map_)(pt.x(), pt.y() + 1).m_dist_d;
    CellVectorType dist_d_dydiff_0 = (*m_map_)(pt.x(), pt.y() - 1).m_dist_d;

    dist_d_dxdiff_1.normalize();
    dist_d_dxdiff_0.normalize();
    dist_d_dydiff_1.normalize();
    dist_d_dydiff_0.normalize();

    CellVectorType dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;
    CellVectorType dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

    value.m_dist_d2 << dist_d_dxdiff(0), dist_d_dydiff(0), dist_d_dxdiff(1),
        dist_d_dydiff(1);

    value.m_dist_d2 *= 0.5 / m_res_;

    return true;
  }  // inline bool getValueWith2ndDeriv<PointCloudType>(CellTypeWith2ndDeriv&,
     // const Eigen::Vector2i&, const PointCloudType&) const

  /**
   * @brief get value method from the distance map based on a point in the
   *        sensor coordinate map, including the 2nd derivative matrix over the
   *        NORMALIZED 1st derivative vector
   * @tparam T cell matrix type (to detect if cell type already has 2nd deriv)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the global coordinate frame (m)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<!std::is_same<T, void>::value, bool> = 0>
  inline bool getValueWith2ndDeriv(CellTypeWith2ndDeriv& value,
                                   const PointVectorType& pt,
                                   const PointCloudType&) const
  {
    const Eigen::Vector2i& map_idx = global2local(pt);

    assert(m_map_ &&
           "DistanceMapStatic_::getValueWith2ndDeriv | map not created...");

    if (!m_map_->isInside(map_idx) || m_map_->isOnBorder(map_idx))
    {
      return false;
    }

    if ((*m_map_)(map_idx.x(), map_idx.y()).m_nn_idx < 0)
    {
      return false;
    }

    value = (*m_map_)(map_idx.x(), map_idx.y());

    return true;
  }  // inline bool getValueWith2ndDeriv<PointCloudType>(CellTypeWith2ndDeriv&,
     // const PointVectorType&, const PointCloudType&) const

  /**
   * @brief get value method from the distance map based on a point in the
   *        distance map matrix, including the 2nd derivative matrix over the
   *        NORMALIZED 1st derivative vector
   * @tparam T cell matrix type (to detect if cell type already has 2nd deriv)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the distance map matrix (px)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<!std::is_same<T, void>::value, bool> = 0>
  inline bool getValueWith2ndDeriv(CellTypeWith2ndDeriv& value,
                                   const Eigen::Vector2i& pt,
                                   const PointCloudType&) const
  {
    assert(m_map_ &&
           "DistanceMapStatic_::getValueWith2ndDeriv | map not created...");

    if (!m_map_->isInside(pt) || m_map_->isOnBorder(pt))
    {
      return false;
    }

    if ((*m_map_)(pt.x(), pt.y()).m_nn_idx < 0)
    {
      return false;
    }

    value = (*m_map_)(pt.x(), pt.y());

    return true;
  }  // inline bool getValueWith2ndDeriv<PointCloudType>(CellTypeWith2ndDeriv&,
     // const Eigen::Vector2i&, const PointCloudType&) const

  /**
   * @brief get subpixelization value method from the distance map based on a
   *        point in the sensor coordinate map and on the bilinear interpolation
   * @tparam T cell type (its scalar type must be the same as the internal one
   *           of the distance map)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the global coordinate frame (m)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T, typename PointCloudType,
            std::enable_if_t<
                std::is_same<typename T::Scalar, CellScalar>::value, bool> = 0>
  inline bool getValueSubPx(T& value, const PointVectorType& pt,
                            const PointCloudType&) const
  {
    const PointVectorType& map_pt = global2localSubPx(pt);

    assert(m_map_ && "DistanceMapStatic_::getValueSubPx | map not created...");

    CellType cell;

    if (!m_map_->getSubPxValue(cell, map_pt))
    {
      return false;
    }

    if ((*m_map_)(static_cast<int>(map_pt.x()), static_cast<int>(map_pt.y()))
            .m_nn_idx < 0)
    {
      return false;
    }

    value.m_nn_idx =
        (*m_map_)(static_cast<int>(map_pt.x()), static_cast<int>(map_pt.y()))
            .m_nn_idx;
    value.m_dist = cell.m_dist;
    value.m_dist_d = cell.m_dist_d;

    return true;
  }  // inline bool DistanceMapStatic_::getValueSubPx<T, PointCloudType>(T&,
     // const PointVectorType&, const PointCloudType&) const

  /**
   * @brief get subpixelization value method from the distance map based on a
   *        point in the sensor coordinate map and on the bilinear
   *        interpolation, including the 2nd derivative matrix over the
   *        NORMALIZED 1st derivative vector
   * @note this is a template specialization that, in this case, the distance
   *       map container does not have the look-up table for the 2nd derivative
   *       over the NROMALIZED 1st-order derivative vector >>> thus, the second
   *       derivative allowing a manual computation of the 2nd derivative
   *       (to have a common API for the distance map matching formulation using
   *       the 2nd derivative)
   * @tparam T cell matrix type (to detect if cell type already has 2nd deriv)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the global coordinate frame (m)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<std::is_same<T, void>::value, bool> = 0>
  inline bool getValueSubPxWith2ndDeriv(CellTypeWith2ndDeriv& value,
                                        const PointVectorType& pt,
                                        const PointCloudType&) const
  {
    const PointVectorType& map_pt = global2localSubPx(pt);

    assert(m_map_ &&
           "DistanceMapStatic_::getValueSubPxWith2ndDeriv | "
           "map not created...");

    CellType cell;

    Scalar wx0y0;
    Scalar wx1y0;
    Scalar wx0y1;
    Scalar wx1y1;

    Eigen::Vector2i map_pt_idx_0(static_cast<int>(map_pt.x()),
                                 static_cast<int>(map_pt.y()));
    Eigen::Vector2i map_pt_idx_1(map_pt_idx_0.x() + 1, map_pt_idx_0.y() + 1);

    if (!m_map_->getSubPxAndWeights(cell, wx0y0, wx1y0, wx0y1, wx1y1, map_pt) ||
        m_map_->isOnBorder(map_pt_idx_0) || m_map_->isOnBorder(map_pt_idx_1))
    {
      return false;
    }

    int x0 = map_pt_idx_0.x();
    int y0 = map_pt_idx_0.y();

    if ((*m_map_)(x0, y0).m_nn_idx < 0)
    {
      return false;
    }

    value.m_nn_idx = (*m_map_)(x0, y0).m_nn_idx;
    value.m_dist = cell.m_dist;
    value.m_dist_d = cell.m_dist_d;

    int x1 = map_pt_idx_1.x();
    int y1 = map_pt_idx_1.y();

    CellVectorType dist_d_dxdiff_1;
    CellVectorType dist_d_dxdiff_0;
    CellVectorType dist_d_dydiff_1;
    CellVectorType dist_d_dydiff_0;

    CellVectorType dist_d_dxdiff;
    CellVectorType dist_d_dydiff;

    typename CellTypeWith2ndDeriv::MatrixType dist_d2_tmp;

    dist_d_dxdiff_1 = (*m_map_)(x0 + 1, y0).m_dist_d;
    dist_d_dxdiff_0 = (*m_map_)(x0 - 1, y0).m_dist_d;
    dist_d_dydiff_1 = (*m_map_)(x0, y0 + 1).m_dist_d;
    dist_d_dydiff_0 = (*m_map_)(x0, y0 - 1).m_dist_d;

    dist_d_dxdiff_1.normalize();
    dist_d_dxdiff_0.normalize();
    dist_d_dydiff_1.normalize();
    dist_d_dydiff_0.normalize();

    dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;
    dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

    dist_d2_tmp << dist_d_dxdiff(0), dist_d_dydiff(0), dist_d_dxdiff(1),
        dist_d_dydiff(1);

    dist_d2_tmp *= 0.5 / m_res_;

    value.m_dist_d2.setZero();

    value.m_dist_d2 += dist_d2_tmp * wx0y0;

    dist_d_dxdiff_1 = (*m_map_)(x1 + 1, y0).m_dist_d;
    dist_d_dxdiff_0 = (*m_map_)(x1 - 1, y0).m_dist_d;
    dist_d_dydiff_1 = (*m_map_)(x1, y0 + 1).m_dist_d;
    dist_d_dydiff_0 = (*m_map_)(x1, y0 - 1).m_dist_d;

    dist_d_dxdiff_1.normalize();
    dist_d_dxdiff_0.normalize();
    dist_d_dydiff_1.normalize();
    dist_d_dydiff_0.normalize();

    dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;
    dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

    dist_d2_tmp << dist_d_dxdiff(0), dist_d_dydiff(0), dist_d_dxdiff(1),
        dist_d_dydiff(1);

    dist_d2_tmp *= 0.5 / m_res_;

    value.m_dist_d2 += dist_d2_tmp * wx1y0;

    dist_d_dxdiff_1 = (*m_map_)(x0 + 1, y1).m_dist_d;
    dist_d_dxdiff_0 = (*m_map_)(x0 - 1, y1).m_dist_d;
    dist_d_dydiff_1 = (*m_map_)(x0, y1 + 1).m_dist_d;
    dist_d_dydiff_0 = (*m_map_)(x0, y1 - 1).m_dist_d;

    dist_d_dxdiff_1.normalize();
    dist_d_dxdiff_0.normalize();
    dist_d_dydiff_1.normalize();
    dist_d_dydiff_0.normalize();

    dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;
    dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

    dist_d2_tmp << dist_d_dxdiff(0), dist_d_dydiff(0), dist_d_dxdiff(1),
        dist_d_dydiff(1);

    dist_d2_tmp *= 0.5 / m_res_;

    value.m_dist_d2 += dist_d2_tmp * wx0y1;

    dist_d_dxdiff_1 = (*m_map_)(x1 + 1, y1).m_dist_d;
    dist_d_dxdiff_0 = (*m_map_)(x1 - 1, y1).m_dist_d;
    dist_d_dydiff_1 = (*m_map_)(x1, y1 + 1).m_dist_d;
    dist_d_dydiff_0 = (*m_map_)(x1, y1 - 1).m_dist_d;

    dist_d_dxdiff_1.normalize();
    dist_d_dxdiff_0.normalize();
    dist_d_dydiff_1.normalize();
    dist_d_dydiff_0.normalize();

    dist_d_dxdiff = dist_d_dxdiff_1 - dist_d_dxdiff_0;
    dist_d_dydiff = dist_d_dydiff_1 - dist_d_dydiff_0;

    dist_d2_tmp << dist_d_dxdiff(0), dist_d_dydiff(0), dist_d_dxdiff(1),
        dist_d_dydiff(1);

    dist_d2_tmp *= 0.5 / m_res_;

    value.m_dist_d2 += dist_d2_tmp * wx1y1;

    return true;
  }  // inline bool
     // DistanceMapStatic_::getValueSubPxWith2ndDeriv<PointCloudType>(
     // CellTypeWith2ndDeriv&, const PointVectorType&, const PointCloudType&)
     // const

  /**
   * @brief get subpixelization value method from the distance map based on a
   *        point in the sensor coordinate map and on the bilinear
   *        interpolation, including the 2nd derivative matrix over the
   *        NORMALIZED 1st derivative vector
   * @tparam T cell matrix type (to detect if cell type already has 2nd deriv)
   * @tparam PointCloudType point cloud type
   * @param[out] value value map cell
   * @param[in] pt point in the global coordinate frame (m)
   * @param[in] pc point cloud (m)
   * @return true valid value computation
   * @return false otherwise (e.g., outside the map, on the border,
   *               no index of the point cloud assigned)
   */
  template <typename T = CellMatrixType, typename PointCloudType,
            std::enable_if_t<!std::is_same<T, void>::value, bool> = 0>
  inline bool getValueSubPxWith2ndDeriv(CellTypeWith2ndDeriv& value,
                                        const PointVectorType& pt,
                                        const PointCloudType&) const
  {
    const PointVectorType& map_pt = global2localSubPx(pt);

    assert(
        m_map_ &&
        "DistanceMapStatic_::getValueSubPxWith2ndDeriv | map not created...");

    if (!m_map_->getSubPxValue(value, map_pt))
    {
      return false;
    }

    if ((*m_map_)(static_cast<int>(map_pt.x()), static_cast<int>(map_pt.y()))
            .m_nn_idx < 0)
    {
      return false;
    }

    return true;
  }  // inline bool
     // DistanceMapStatic_::getValueSubPxWith2ndDeriv<PointCloudType>(
     // CellTypeWith2ndDeriv&, const PointVectorType&, const PointCloudType&)
     // const

  /****************************************************************************
   * @brief SRRG SOFTWARE BOSS SERIALIZATION METHODS
   ****************************************************************************/

 public:

  void serialize(srrg2_core::ObjectData& data, srrg2_core::IdContext&) override
  {
    data.setDouble("res", m_res_);
    data.setDouble("map_size", m_map_size_);
    data.setDouble("max_dist", m_max_dist_);
  }  // void DistanceMapStatic_::serialize(srrg2_core::ObjectData&,
     // srrg2_core::IdContext&) override

  void deserialize(srrg2_core::ObjectData& data,
                   srrg2_core::IdContext&) override
  {
    m_res_ = static_cast<Scalar>(data.getDouble("res"));
    m_map_size_ = static_cast<Scalar>(data.getDouble("map_size"));
    m_max_dist_ = static_cast<Scalar>(data.getDouble("max_dist"));
  }  // void DistanceMapStatic_::deserialize(srrg2_core::ObjectData&,
     // srrg2_core::IdContext&) override

 public:

  /**
   * @brief used memory allocation estimation size without considering the
   *        possibly overallocation adopted by STL containers
   *        (size != capacity...)
   * @return size_t estimated allocated memory
   */
  size_t size() const
  {
    size_t distance_map_size = 0;

    distance_map_size += sizeof(ThisType);
    distance_map_size += sizeof(CellType) * m_map_->size();
    distance_map_size += sizeof(CellSearchQueueTypeCell) * m_queue_.size();

    return distance_map_size;
  }  // size_t DistanceMapStatic_::size() const

};  // class DistanceMapStatic_<CellType_> : public srrg2_core::Serializable

using DistanceMapStaticFloat = DistanceMapStatic_<DistanceMapCellFloat>;
using DistanceMapStaticDouble = DistanceMapStatic_<DistanceMapCellDouble>;

using DistanceMapStaticWith2ndDerivFloat =
    DistanceMapStatic_<DistanceMapWith2ndDerivCellFloat>;
using DistanceMapStaticWith2ndDerivDouble =
    DistanceMapStatic_<DistanceMapWith2ndDerivCellDouble>;

using DistanceMapStaticFloatPtr = std::shared_ptr<DistanceMapStaticFloat>;
using DistanceMapStaticDoublePtr = std::shared_ptr<DistanceMapStaticDouble>;

using DistanceMapStaticWith2ndDerivFloatPtr =
    std::shared_ptr<DistanceMapStaticWith2ndDerivFloat>;
using DistanceMapStaticWith2ndDerivDoublePtr =
    std::shared_ptr<DistanceMapStaticWith2ndDerivDouble>;

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
