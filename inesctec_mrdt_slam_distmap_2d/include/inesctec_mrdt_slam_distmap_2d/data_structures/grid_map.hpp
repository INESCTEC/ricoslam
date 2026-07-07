#pragma once

#include <Eigen/Geometry>
#include <cstdlib>
#include <limits>
#include <memory>
#include <utility>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/image_defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/data_structures/matrix.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace data_structures
{

using OccupancyGridMap = MatrixFloat;
using OccupancyImage = MatrixUInt8;
using FrequencyFloatGridMap = Matrix_<Eigen::Vector2f>;
using FrequencyIntGridMap = Matrix_<Eigen::Vector2i>;

using OccupancyGridMapPtr = std::shared_ptr<OccupancyGridMap>;
using OccupancyImagePtr = std::shared_ptr<OccupancyImage>;
using FrequencyFloatGridMapPtr = std::shared_ptr<FrequencyFloatGridMap>;
using FrequencyIntGridMapPtr = std::shared_ptr<FrequencyIntGridMap>;

/**
 * @brief header of the 2D grid map (origin, size, resolution) and utility
 *        functions (global2local, local2global, ...)
 */
class GridMap2DHeader
{
 public:

  using EstimateType = Eigen::Isometry2f;
  using IndexType = Eigen::Vector2i;
  using CoordinatesType = Eigen::Vector2f;

 protected:

  EstimateType m_origin_ = EstimateType::Identity();      //!< map w.r.t. world
  EstimateType m_origin_inv_ = EstimateType::Identity();  //!< world w.r.t. map
  IndexType m_size_ = IndexType::Zero();  //!< map size (width,height) (px)
  float m_res_ = 0.05f;                   //!< map resolution (m/px)
  float m_res_inv = 1 / 0.05f;            //!< map resolution inverse (px/m)

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief resize the grid map (must be overloaded by the grid map's
   *        implementation)
   * @param[in] size map size (width,height) (px)
   */
  virtual void resize(const IndexType& size) = 0;

  /**
   * @brief convert global to local point coordinates (based on the origin
   *        inverse SE2 coordinate transformation)
   * @param[in] pt point in the global coordinate frame (m)
   * @return CoordinatesType point in the local coordinate frame (m)
   */
  inline CoordinatesType global2local(const CoordinatesType& pt) const
  {
    return m_origin_inv_ * pt;
  }

  /**
   * @brief convert local to global point coordinates (based on the origin SE2
   *        coordinate transformation)
   * @param[in] pt point in the local coordinate frame (m)
   * @return CoordinatesType point in the global coordinate frame (m)
   */
  inline CoordinatesType local2global(const CoordinatesType& pt) const
  {
    return m_origin_ * pt;
  }

  /**
   * @brief convert local point to map sub-pixel coordinates
   * @param[in] pt point in the local coordinate frame (m)
   * @return CoordinatesType map coordinates ('float' px)
   */
  inline CoordinatesType local2FloatIdx(const CoordinatesType& pt) const
  {
    return pt * m_res_inv;
  }

  /**
   * @brief convert local point coordinates to map pixel coordinates (row, col)
   * @param[in] pt point in the local coordinate frame (m)
   * @return IndexType map coordinates (px)
   */
  inline IndexType local2Idx(const CoordinatesType& pt) const
  {
    return local2FloatIdx(pt).cast<int>();
  }

  /**
   * @brief convert map sub-pixel to local point coordinates
   * @param[in] idx map coordinates ('float' px)
   * @return CoordinatesType point in the local coordinate frame (m)
   */
  inline CoordinatesType floatIdx2local(const CoordinatesType& idx) const
  {
    return idx * m_res_;
  }

  /**
   * @brief convert map pixel (row, col) to local point coordinates
   * @param[in] idx map coordinates (px)
   * @return CoordinatesType point in the local coordinate frame (m)
   */
  inline CoordinatesType idx2local(const IndexType& idx) const
  {
    return floatIdx2local(idx.cast<float>());
  }

  /**
   * @brief convert global point to map sub-pixel coordinates
   * @param[in] pt point in the global coordinate frame (m)
   * @return CoordinatesType map coordinates ('float' px)
   */
  inline CoordinatesType global2FloatIdx(const CoordinatesType& pt) const
  {
    return local2FloatIdx(global2local(pt));
  }

  /**
   * @brief convert global point to map pixel coordinates (row, col)
   * @param[in] pt point in the global coordinate frame (m)
   * @return IndexType map coordinates (px)
   */
  inline IndexType global2Idx(const CoordinatesType& pt) const
  {
    return local2Idx(global2local(pt));
  }

  /**
   * @brief convert map sub-pixel to global point coordinates
   * @param[in] idx map coordinates ('float' px)
   * @return CoordinatesType point in the global coordinate frame (m)
   */
  inline CoordinatesType floatIdx2global(const CoordinatesType& idx) const
  {
    return local2global(floatIdx2local(idx));
  }

  /**
   * @brief convert map pixel (row, col) to global point coordinates
   * @param[in] idx map coordinates (px)
   * @return CoordinatesType point in the global coordinate frame (m)
   */
  inline CoordinatesType idx2global(const IndexType& idx) const
  {
    return local2global(idx2local(idx));
  }

  /**
   * @brief compute bounding box global dimensions of the map
   * @return CoordinatesType map size (width,height) (m)
   */
  inline CoordinatesType boundingBox() const { return idx2local(m_size_); }

  /**
   * @brief compute the global map coordinates of the grid map corner points
   *        ({0,0}, {width-1,0}, {width-1,height-1}, {0,height-1})
   * @param[in] pts map corners global map coordinates (m)
   */
  inline void corners(Eigen::Matrix<float, 2, 4>& pts) const
  {
    pts.col(0) = idx2global(IndexType{0, 0});
    pts.col(1) = idx2global(IndexType{m_size_.x() - 1, 0});
    pts.col(2) = idx2global(IndexType{m_size_.x() - 1, m_size_.y() - 1});
    pts.col(3) = idx2global(IndexType{0, m_size_.y() - 1});
  }

 public:

  inline void setOrigin(const EstimateType& origin)
  {
    m_origin_ = origin;
    m_origin_inv_ = origin.inverse();
  }

  inline void setRes(float res)
  {
    m_res_ = res;
    m_res_inv = 1 / res;
  }

  inline void setSize(const CoordinatesType& new_size)
  {
    m_size_ = local2Idx(new_size);
    resize(m_size_);
  }

  inline void setSize(const IndexType& new_size)
  {
    m_size_ = new_size;
    resize(m_size_);
  }

  inline const EstimateType& getOrigin() const { return m_origin_; }
  inline const EstimateType& getOriginInv() const { return m_origin_inv_; }
  inline const IndexType& getSize() const { return m_size_; }
  inline int getWidth() const { return m_size_.x(); }
  inline int getHeight() const { return m_size_.y(); }
  inline float getRes() const { return m_res_; }
  inline float getResInv() const { return m_res_inv; }
};  // class GridMap2DHeader

/**
 * @brief 2D occupancy grid map
 * @tparam OccupancyGridMapType_ 2D occupancy grid map type
 * @tparam OccupancyImageType_ 2D occupancy image type (free, occupied, unknown)
 * @tparam FrequencyGridMapType_ 2D frequency grid map type (#hits, #passes)
 */
template <typename OccupancyGridMapType_, typename OccupancyImageType_,
          typename FrequencyGridMapType_>
class GridMap2D_ : public GridMap2DHeader
{
 public:

  using OccupancyGridMapType = OccupancyGridMapType_;
  using OccupancyImageType = OccupancyImageType_;
  using FrequencyGridMapType = FrequencyGridMapType_;
  using OccupancyGridMapTypePtr = std::shared_ptr<OccupancyGridMapType>;
  using OccupancyImageTypePtr = std::shared_ptr<OccupancyImageType>;
  using FrequencyGridMapTypePtr = std::shared_ptr<FrequencyGridMapType>;

 protected:

  OccupancyGridMapTypePtr m_occ_ = std::make_shared<OccupancyGridMapType>();
  OccupancyImageTypePtr m_img_ = std::make_shared<OccupancyImageType>();
  FrequencyGridMapTypePtr m_freq_ = std::make_shared<FrequencyGridMapType>();

 public:

  /**
   * @brief default grid map 2D constructor (creates empty frequency and
   *        occupancy grid map, and occupancy image)
   */
  GridMap2D_() = default;

  /**
   * @brief grid map 2D constructor from external frequency and occupancy grid
   *        map, and occupancy image
   * @param[in] occ 2D occupancy grid map (or nullptr)
   * @param[in] img 2D occupancy image (or nullptr)
   * @param[in] freq 2D frequency grid map (or nullptr)
   */
  GridMap2D_(const OccupancyGridMapTypePtr& occ,
             const OccupancyImageTypePtr& img,
             const FrequencyGridMapTypePtr& freq)
      : m_occ_(occ), m_img_(img), m_freq_(freq)
  {
  }

  /**
   * @brief grid map 2D constructor from external frequency and occupancy grid
   *        map, and occupancy image
   * @param[in] occ 2D occupancy grid map (or nullptr)
   * @param[in] img 2D occupancy image (or nullptr)
   * @param[in] freq 2D frequency grid map (or nullptr)
   */
  GridMap2D_(OccupancyGridMapTypePtr&& occ, OccupancyImageTypePtr&& img,
             FrequencyGridMapTypePtr&& freq)
      : m_occ_(std::move(occ)), m_img_(std::move(img)), m_freq_(std::move(freq))
  {
  }

  /**
   * @brief resize grid map 2D with the same size for all maps initialized in
   *        the container
   * @param[in] new_size size (rows, columns)
   */
  virtual void resize(const IndexType& new_size)
  {
    if (m_occ_)
    {
      m_occ_->resize(new_size.x(), new_size.y());
    }

    if (m_img_)
    {
      m_img_->resize(new_size.x(), new_size.y());
    }

    if (m_freq_)
    {
      m_freq_->resize(new_size.x(), new_size.y());
    }
  }

 public:

  /**
   * @brief Convert the 2D occupancy grid map to a CV_32FC1 image, writing
   *        NaN for cells whose visit count is zero (never observed).
   *        Intended for lossless export (e.g. 32-bit float TIFF).
   * @param[in,out] img CV_32FC1 OpenCV image
   */
  void toOccupancyGridMapWithUnknown(FloatImage& img) const
  {
    if (!m_occ_)
    {
      return;
    }

    img.create(m_occ_->cols(), m_occ_->rows());

    const bool has_freq = static_cast<bool>(m_freq_);
    const float nan_val = std::numeric_limits<float>::quiet_NaN();

    for (int r = 0; r < img.rows; r++)
    {
      int rflip = img.rows - r - 1;

      for (int c = 0; c < img.cols; c++)
      {
        if (has_freq)
        {
          if ((*m_freq_)(c, r)(1) == 0)
          {
            img.at<float>(rflip, c) = nan_val;
          }
          else
          {
            img.at<float>(rflip, c) = (*m_occ_)(c, r);
          }
        }
        else
        {
          img.at<float>(rflip, c) = (*m_occ_)(c, r);
        }
      }
    }
  }

  /**
   * @brief convert the 2D occupancy grid map (considering the float continuous
   *        probability value for cells occupancy) to OpenCV format
   * @param[in,out] img unsigned char OpenCV image
   */
  void toImageOccupancyGridMap(UCharImage& img) const
  {
    if (!m_occ_)
    {
      return;
    }

    img.create(m_occ_->cols(), m_occ_->rows());

    for (int r = 0; r < img.rows; r++)
    {
      int rflip = img.rows - r - 1;

      for (int c = 0; c < img.cols; c++)
      {
        img.at<unsigned char>(rflip, c) =
            255 - static_cast<unsigned char>(std::round(255 * (*m_occ_)(c, r)));
      }
    }
  }

  /**
   * @brief convert the 2D occupancy grid image to OpenCV format
   * @param[in,out] img unsigned char OpenCV image
   */
  void toImageOccupancyGridImage(UCharImage& img) const
  {
    if (!m_img_)
    {
      return;
    }

    img.create(m_img_->cols(), m_img_->rows());

    for (int r = 0; r < img.rows; r++)
    {
      int rflip = img.rows - r - 1;

      for (int c = 0; c < img.cols; c++)
      {
        img.at<unsigned char>(rflip, c) = (*m_img_)(c, r);
      }
    }
  }

  /**
   * @brief compute the frequency map image into OpenCV format
   *        from the ratio of #hits / #passes, considering unknown as
   *        127 intensity value
   *        ( #hits / #passes = 1   >>> 0   / black intensity ;
   *          #hits / #passes = 0   >>> 255 / white intensity ;
   *          unknown         = 0.5 >>> 127 / gray intensity  )
   * @param[in,out] img unsigned char OpenCV image
   */
  void toImageFrequencyMap(UCharImage& img) const
  {
    if (!m_freq_)
    {
      return;
    }

    img.create(m_freq_->cols(), m_freq_->rows());

    for (int r = 0; r < img.rows; r++)
    {
      int rflip = img.rows - r - 1;

      for (int c = 0; c < img.cols; c++)
      {
        if ((*m_freq_)(c, r)(1) > 0)
        {
          img.at<unsigned char>(rflip, c) =
              255 - static_cast<unsigned char>(
                        std::round(255 * (*m_freq_)(c, r)(0) /
                                   static_cast<float>((*m_freq_)(c, r)(1))));
        }
        else
        {
          img.at<unsigned char>(rflip, c) = 127;
        }
      }
    }
  }

 public:

  inline OccupancyGridMapType* getOccupancyMap() const { return m_occ_.get(); }
  inline OccupancyImageType* getOccupancyImg() const { return m_img_.get(); }
  inline FrequencyGridMapType* getFrequencyMap() const { return m_freq_.get(); }

  inline const OccupancyGridMapTypePtr& getOccupancyMapPtr() const
  {
    return m_occ_;
  }
  inline const OccupancyImageTypePtr& getOccupancyImgPtr() const
  {
    return m_img_;
  }
  inline const FrequencyGridMapTypePtr& getFrequencyMapPtr() const
  {
    return m_freq_;
  }
};

}  // namespace data_structures

}  // namespace inesctec_mrdt_slam_distmap_2d
