#pragma once

#include <algorithm>
#include <vector>

#include "inesctec_mrdt_slam_distmap_2d/pcl/point_defs.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief point cloud container template class
 * @tparam ContainerType_ container type for the point cloud
 */
template <typename ContainerType_>
struct PointCloud_ : public ContainerType_
{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ThisType = PointCloud_<ContainerType_>;
  using BaseType = ContainerType_;

  using ContainerType = ContainerType_;
  using PointType = typename ContainerType::value_type;
  using PlainVectorType = typename PointType::VectorType;
  using Scalar = typename PlainVectorType::Scalar;

 public:

  static constexpr int PointDim = PointType::Dim;  //!< point dimension

 public:

  struct VoxelEntry
  {
   public:

    int m_key[PointDim];
    typename ContainerType::const_iterator m_it;

   public:

    /**
     * @brief operator < required for the std::sort algorithm
     *        (order the voxels by their keys for the ones with same keys to be
     *        grouped together)
     * @note the fact that if (m_key[dim] < other.m_key[dim]) returns true
     *       immediately is to speed up the sorting of a vector with voxel
     *       entries
     * @param[in] other other voxel entry
     * @return true if the current voxel entry has a key with any dimension
     *              lower than the other entry
     * @return false otherwise
     */
    inline bool operator<(const VoxelEntry& other) const
    {
      for (int dim = 0; dim < PointType::Dim; ++dim)
      {
        if (m_key[dim] < other.m_key[dim])
        {
          return true;
        }
        if (m_key[dim] > other.m_key[dim])
        {
          return false;
        }
      }

      return false;
    }

    /**
     * @brief set the key of the voxel entry
     *
     * key = scales_inv * pt dimension
     *
     * example:
     * - scale = 0.01 (voxel grid of 0.01m)
     * - pt = (0.051, 0.076)m
     *
     * - voxel key = (5,7) !!!!!
     *
     * @param[in] it constant iterator to the original point
     * @param[in] scales_inv scales inverted in each point coordinates dimension
     */
    inline void set(const typename ContainerType::const_iterator& it,
                    const PlainVectorType& scales_inv)
    {
      m_it = it;

      PlainVectorType plain(it->coordinates());

      for (int dim = 0; dim < PointType::Dim; ++dim)
      {
        m_key[dim] = static_cast<int>(scales_inv(dim) * plain(dim));
      }
    }

  };  // struct VoxelEntry

  struct VoxelEntryMove
  {
   public:

    int m_key[PointDim];
    PointType m_value;

   public:

    /**
     * @brief operator < required for the std::sort algorithm
     *        (order the voxels by their keys for the ones with same keys to be
     *        grouped together)
     * @note the fact that if (m_key[dim] < other.m_key[dim]) returns true
     *       immediately is to speed up the sorting of a vector with voxel
     *       entries
     * @param[in] other other voxel entry
     * @return true if the current voxel entry has a key with any dimension
     *              lower than the other entry
     * @return false otherwise
     */
    inline bool operator<(const VoxelEntryMove& other) const
    {
      for (int dim = 0; dim < PointType::Dim; ++dim)
      {
        if (m_key[dim] < other.m_key[dim])
        {
          return true;
        }
        if (m_key[dim] > other.m_key[dim])
        {
          return false;
        }
      }

      return false;
    }

    /**
     * @brief set the key of the voxel entry (std::move on the original point)
     *
     * key = scales_inv * pt dimension
     *
     * example:
     * - scale = 0.01 (voxel grid of 0.01m)
     * - pt = (0.051, 0.076)m
     *
     * - voxel key = (5,7) !!!!!
     *
     * @param[in] it constant iterator to the original point
     *               (std::move to voxel internal value)
     * @param[in] scales_inv scales inverted in each point coordinates dimension
     */
    inline void set(const typename ContainerType::const_iterator& it,
                    const PlainVectorType& scales_inv)
    {
      m_value = std::move(*it);

      const PlainVectorType& plain = m_value.coordinates();

      for (int dim = 0; dim < PointType::Dim; ++dim)
      {
        m_key[dim] = static_cast<int>(scales_inv(dim) * plain(dim));
      }
    }

  };  // struct VoxelEntry

 public:

  /**
   * @brief copy the point cloud into anouther container, providing the latter
   *        is iteratable
   * @tparam DestIteratorType_ destination iterator type
   * @param[in] dest_it iterator for the destination container
   * @param[in] keep_invalid keep invalid points in the output point cloud
   */
  template <typename DestIteratorType_>
  inline void copyTo(DestIteratorType_ dest_it, bool keep_invalid = false) const
  {
    for (const PointType& src_pt : *this)
    {
      if ((!keep_invalid) && !(src_pt.m_status == PointStatus::kValid))
      {
        continue;
      }

      *dest_it++ = src_pt;
    }
  }

  /**
   * @brief copy a specific field component from a point cloud into anouther
   *        container, providing the latter is iteratable
   * @tparam fieldIdx field index of the specific component in the point
   * @tparam DestIteratorType_ destination iterator type
   * @param[in] dest_it iterator for the destination container
   * @param[in] keep_invalid keep invalid points in the output point cloud
   */
  template <int fieldIdx, typename DestIteratorType_>
  inline void copyTo(DestIteratorType_ dest_it, bool keep_invalid = false) const
  {
    for (const PointType& src_pt : *this)
    {
      if ((!keep_invalid) && !(src_pt.m_status == PointStatus::kValid))
      {
        continue;
      }

      dest_it->template value<fieldIdx>() = src_pt.template value<fieldIdx>();

      dest_it++;
    }
  }

  /**
   * @brief count points in the cloud with a given PointStatus
   * @param[in] status point status
   * @return size_t number of points with m_status == status
   */
  inline size_t countStatus(PointStatus status) const
  {
    size_t count = 0;

    for (const PointType& src_pt : *this)
    {
      if (src_pt.m_status == status)
      {
        count++;
      }
    }

    return count;
  }

  /**
   * @brief set to zero the components of all the points present in the point
   *        cloud
   * @return ThisType& point cloud with the components of all points set to zero
   */
  inline ThisType& setZero()
  {
    for (PointType& p : *this)
    {
      p.m_status = PointStatus::kValid;
      p.setZero();
    }

    return *this;
  }

  /**
   * @brief set to zero to a specific component of all the points present in the
   *        point cloud
   * @tparam fieldIdx field index of the specific component in the point
   * @return ThisType& point cloud with a specific component of all points set
   * to zero
   */
  template <int fieldIdx>
  inline ThisType& setZero()
  {
    for (PointType& p : *this)
    {
      p.m_status = PointStatus::kValid;
      p.template setZero<fieldIdx>();
    }

    return *this;
  }

  /**
   * @brief normalize the components of all the points present in the point
   *        cloud
   * @return ThisType& point cloud with the components of all points normalized
   */
  inline ThisType& normalize()
  {
    for (PointType& p : *this)
    {
      p.normalize();
    }

    return *this;
  }

  /**
   * @brief normalize a specific component of all the points present in the
   *        point cloud
   * @tparam fieldIdx field index of the specific component in the point
   * @return ThisType& point cloud with a specific component of all points
   *               normalized
   */
  template <int fieldIdx>
  inline ThisType& normalize()
  {
    for (PointType& p : *this)
    {
      p.template normalize<fieldIdx>();
    }

    return *this;
  }

  /**
   * @brief transform in-place all the points in the point cloud
   * @tparam TransformType_ transformation type
   * @param[in] transform transformation
   * @return ThisType& transformed point cloud
   */
  template <typename TransformType_>
  inline ThisType& transformInPlace(const TransformType_& transform)
  {
    for (PointType& p : *this)
    {
      p.template transformInPlace<TransformType_>(transform);
    }

    return *this;
  }

  /**
   * @brief transform in-place of a specific component of all the points in the
   *        point cloud
   * @tparam fieldIdx field index of the specific component in the point
   * @tparam TransformType_ transformation type
   * @param[in] transform transformation
   * @return ThisType& transformed point cloud only on a specific component
   */
  template <int fieldIdx, typename TransformType_>
  inline ThisType& transformInPlace(const TransformType_& transform)
  {
    for (PointType& p : *this)
    {
      p.template transformInPlace<fieldIdx, TransformType_>(transform);
    }

    return *this;
  }

  /**
   * @brief transform the points present in the point cloud and copy them to a
   *        destination container (providing the latter is iteratable)
   * @tparam TransformType_ transformation type
   * @tparam DestIteratorType_ destination iterator type
   * @tparam suppress_invalid_pts boolean to suppress or not the invalid points
   *                              from the destination container
   * @param[in] dest_it iterator for the destination container
   * @param[in] transform transformation
   */
  template <typename TransformType_, typename DestIteratorType_,
            bool suppress_invalid_pts = true>
  inline void transform(DestIteratorType_ dest_it,
                        const TransformType_& transform) const
  {
    for (const PointType& src_pt : *this)
    {
      if ((src_pt.m_status != PointStatus::kValid))
      {
        if (!suppress_invalid_pts)
        {
          *dest_it++ = src_pt;
        }
        continue;
      }

      PointType dest_pt = src_pt.template transform<TransformType_>(transform);

      if ((dest_pt.m_status == PointStatus::kValid) || (!suppress_invalid_pts))
      {
        *dest_it++ = dest_pt;
      }
    }
  }

  /**
   * @brief transform a specific components of the points present in the point
   *        cloud and copy them to a destination container (providing the latter
   *        is iteratable)
   * @tparam fieldIdx field index of the specific component in the point
   * @tparam TransformType_ transformation type
   * @tparam DestIteratorType_ destination iterator type
   * @tparam suppress_invalid_pts boolean to suppress or not the invalid points
   *                              from the destination container
   * @param[in] dest_it iterator for the destination container
   * @param[in] transform transformation
   */
  template <int fieldIdx, typename TransformType_, typename DestIteratorType_,
            bool suppress_invalid_pts = true>
  inline void transform(DestIteratorType_ dest_it,
                        const TransformType_& transform) const
  {
    for (const PointType& src_pt : *this)
    {
      if ((src_pt.m_status != PointStatus::kValid))
      {
        if (!suppress_invalid_pts)
        {
          *dest_it++ = src_pt;
        }
        continue;
      }

      PointType dest_pt =
          src_pt.template transform<fieldIdx, TransformType_>(transform);

      if ((dest_pt.m_status == PointStatus::kValid) || (!suppress_invalid_pts))
      {
        *dest_it++ = dest_pt;
      }
    }
  }

  /**
   * @brief transform the points present in the point cloud and copy them to a
   *        destination container through return value
   * @tparam TransformType_ transformation type
   * @tparam suppress_invalid_pts boolean to suppress or not the invalid points
   *                              from the destination container
   * @param[in] transform transformation
   * @return ThisType transformed point cloud
   */
  template <typename TransformType_, bool suppress_invalid_pts = true>
  inline ThisType transform(const TransformType_& transform) const
  {
    ThisType returned;

    returned.resize(this->size());

    auto dest_it = returned.begin();

    for (const PointType& src_pt : *this)
    {
      if ((src_pt.m_status != PointStatus::kValid))
      {
        if (!suppress_invalid_pts)
        {
          *dest_it++ = src_pt;
        }
        continue;
      }

      PointType dest_pt = src_pt.template transform<TransformType_>(transform);

      if ((dest_pt.m_status == PointStatus::kValid) || (!suppress_invalid_pts))
      {
        *dest_it++ = dest_pt;
      }
    }

    return returned;
  }

  /**
   * @brief transform a specific component of the points present in the point
   *        cloud and copy them to a destination container through return value
   * @tparam fieldIdx field index of the specific component in the point
   * @tparam TransformType_ transformation type
   * @tparam suppress_invalid_pts boolean to suppress or not the invalid points
   *                              from the destination container
   * @param[in] transform transformation
   * @return ThisType transformed point cloud
   */
  template <int fieldIdx, typename TransformType_,
            bool suppress_invalid_pts = true>
  inline ThisType transform(const TransformType_& transform) const
  {
    ThisType returned;

    returned.resize(this->size());

    auto dest_it = returned.begin();

    for (const PointType& src_pt : *this)
    {
      if ((src_pt.m_status != PointStatus::kValid))
      {
        if (!suppress_invalid_pts)
        {
          *dest_it++ = src_pt;
        }
        continue;
      }

      PointType dest_pt =
          src_pt.template transform<fieldIdx, TransformType_>(transform);

      if ((dest_pt.m_status == PointStatus::kValid) || (!suppress_invalid_pts))
      {
        *dest_it++ = dest_pt;
      }
    }

    return returned;
  }

  /**
   * @brief voxelization method of a point cloud (different to voxelize of SRRG
   *        is the final automatic resize of the target container)
   * @param[out] target target point cloud container
   * @param[in] scales voxelization scales (with the possibility of having a
   *                   different scaling factor on each point dimension)
   */
  void voxelize(ThisType& target, const PlainVectorType& scales) const
  {
    std::vector<VoxelEntry> entries(this->size());

    PlainVectorType scales_inv;

    for (int dim = 0; dim < PointType::Dim; ++dim)
    {
      scales_inv[dim] = 0.f;

      if (scales[dim] > 0)
      {
        scales_inv[dim] = 1. / scales[dim];
      }
    }

    size_t num_pts = 0;

    for (typename ContainerType::const_iterator it = this->begin();
         it != this->end(); ++it)
    {
      if (it->m_status != PointStatus::kValid)
      {
        continue;
      }

      entries[num_pts++].set(it, scales_inv);
    }

    if (!num_pts)
    {
      return;
    }

    entries.resize(num_pts);

    std::sort(entries.begin(), entries.end());

    PointType acc_pt = *(entries[0].m_it);
    int acc_num = 1;

    target.resize(this->size());

    typename ContainerType::iterator target_it = target.begin();

    for (size_t idx = 1; idx < num_pts; ++idx)
    {
      const VoxelEntry& entry = entries[idx];
      const PointType& point = *(entry.m_it);

      if (entries[idx - 1] < entry)
      {
        if (acc_num)
        {
          acc_pt *= (1.f / acc_num);

          if (acc_pt.m_status == PointStatus::kValid)
          {
            *target_it++ = acc_pt;
          }
        }

        acc_pt = point;
        acc_num = 1;

        continue;
      }

      acc_pt += point;
      ++acc_num;
    }

    if (acc_num)
    {
      acc_pt *= (1.f / acc_num);

      if (acc_pt.m_status == PointStatus::kValid)
      {
        *target_it++ = acc_pt;
      }
    }

    target.resize(std::distance(target.begin(), target_it));
  }

  /**
   * @brief voxelization method of a point cloud (different to voxelize of SRRG
   *        is the final automatic resize of the target container)
   * @param[out] target target point cloud container
   * @param[in] scale voxelization scale (same scaling factor on each point
   *                  dimension)
   */
  inline void voxelize(ThisType& target, const Scalar& scale) const
  {
    this->voxelize(target, PlainVectorType::Constant(scale));
  }

  /**
   * @brief voxelization method of a point cloud (additional utility method to
   *        voxelize inplace, instead of having a target container, with
   *        automatic resize)
   * @param[in] scales voxelization scales (with the possibility of having a
   *                   different scaling factor on each point dimension)
   */
  void voxelize(const PlainVectorType& scales)
  {
    std::vector<VoxelEntryMove> entries(this->size());

    PlainVectorType scales_inv;

    for (int dim = 0; dim < PointType::Dim; ++dim)
    {
      scales_inv[dim] = 0.f;

      if (scales[dim] > 0)
      {
        scales_inv[dim] = 1. / scales[dim];
      }
    }

    size_t num_pts = 0;

    for (typename ContainerType::const_iterator it = this->begin();
         it != this->end(); ++it)
    {
      if (it->m_status != PointStatus::kValid)
      {
        continue;
      }

      entries[num_pts++].set(it, scales_inv);
    }

    if (!num_pts)
    {
      this->clear();
      return;
    }

    entries.resize(num_pts);

    std::sort(entries.begin(), entries.end());

    PointType acc_pt = entries[0].m_value;
    int acc_num = 1;

    typename ContainerType::iterator target_it = this->begin();

    for (size_t idx = 1; idx < num_pts; ++idx)
    {
      const VoxelEntryMove& entry = entries[idx];
      const PointType& point = entry.m_value;

      if (entries[idx - 1] < entry)
      {
        if (acc_num)
        {
          acc_pt *= (1.f / acc_num);

          if (acc_pt.m_status == PointStatus::kValid)
          {
            *target_it++ = acc_pt;
          }
        }

        acc_pt = point;
        acc_num = 1;

        continue;
      }

      acc_pt += point;
      ++acc_num;
    }

    if (acc_num)
    {
      acc_pt *= (1.f / acc_num);

      if (acc_pt.m_status == PointStatus::kValid)
      {
        *target_it++ = acc_pt;
      }
    }

    this->resize(std::distance(this->begin(), target_it));
  }

  /**
   * @brief voxelization method of a point cloud (additional utility method to
   *        voxelize inplace, instead of having a target container, with
   *        automatic resize)
   * @param[in] scale voxelization scale (same scaling factor on each point
   *                  dimension)
   */
  inline void voxelize(const Scalar& scale)
  {
    this->voxelize(PlainVectorType::Constant(scale));
  }

  /**
   * @brief voxelization method of a specific component of a point cloud to
   *        possibly another type of point cloud
   * @tparam fieldIdx field index of the specific component in the point
   * @tparam OtherPointCloudType target point cloud type
   * @param[out] target target point cloud container
   * @param[in] scales voxelization scales (with the possibility of having a
   *                   different scaling factor on each point dimension)
   */
  template <int fieldIdx, typename OtherPointCloudType>
  void voxelize(OtherPointCloudType& target,
                const PlainVectorType& scales) const
  {
    std::vector<VoxelEntry> entries(this->size());

    PlainVectorType scales_inv;

    for (int dim = 0; dim < PointType::Dim; ++dim)
    {
      scales_inv[dim] = 0.f;

      if (scales[dim] > 0)
      {
        scales_inv[dim] = 1. / scales[dim];
      }
    }

    size_t num_pts = 0;

    for (typename ContainerType::const_iterator it = this->begin();
         it != this->end(); ++it)
    {
      if (it->m_status != PointStatus::kValid)
      {
        continue;
      }

      entries[num_pts++].set(it, scales_inv);
    }

    if (!num_pts)
    {
      return;
    }

    entries.resize(num_pts);

    std::sort(entries.begin(), entries.end());

    PointType acc_pt = *(entries[0].m_it);
    int acc_num = 1;

    target.resize(this->size());

    typename OtherPointCloudType::iterator target_it = target.begin();

    for (size_t idx = 1; idx < num_pts; ++idx)
    {
      const VoxelEntry& entry = entries[idx];
      const PointType& point = *(entry.m_it);

      if (entries[idx - 1] < entry)
      {
        if (acc_num)
        {
          acc_pt *= (1.f / acc_num);

          if (acc_pt.m_status == PointStatus::kValid)
          {
            target_it->template value<fieldIdx>() =
                acc_pt.template value<fieldIdx>();
            target_it++;
          }
        }

        acc_pt = point;
        acc_num = 1;

        continue;
      }

      acc_pt += point;
      ++acc_num;
    }

    if (acc_num)
    {
      acc_pt *= (1.f / acc_num);

      if (acc_pt.m_status == PointStatus::kValid)
      {
        target_it->template value<fieldIdx>() =
            acc_pt.template value<fieldIdx>();
        target_it++;
      }
    }

    target.resize(std::distance(target.begin(), target_it));
  }

  /**
   * @brief voxelization method of a specific component of a point cloud to
   *        possibly another type of point cloud
   * @tparam fieldIdx field index of the specific component in the point
   * @tparam OtherPointCloudType target point cloud type
   * @param[out] target target point cloud container
   * @param[in] scale voxelization scale (same scaling factor on each point
   *                  dimension)
   */
  template <int fieldIdx, typename OtherPointCloudType>
  inline void voxelize(OtherPointCloudType& target, const Scalar& scale) const
  {
    this->template voxelize<fieldIdx>(target, PlainVectorType::Constant(scale));
  }

  /**
   * @brief voxelization method of a specific component of a point cloud
   *        (additional utility method to voxelize inplace, instead of having a
   *        target container, with automatic resize)
   * @tparam fieldIdx field index of the specific component in the point
   * @param[in] scales voxelization scales (with the possibility of having a
   *                   different scaling factor on each point dimension)
   */
  template <int fieldIdx>
  void voxelize(const PlainVectorType& scales)
  {
    std::vector<VoxelEntryMove> entries(this->size());

    PlainVectorType scales_inv;

    for (int dim = 0; dim < PointType::Dim; ++dim)
    {
      scales_inv[dim] = 0.f;

      if (scales[dim] > 0)
      {
        scales_inv[dim] = 1. / scales[dim];
      }
    }

    size_t num_pts = 0;

    for (typename ContainerType::const_iterator it = this->begin();
         it != this->end(); ++it)
    {
      if (it->m_status != PointStatus::kValid)
      {
        continue;
      }

      entries[num_pts++].set(it, scales_inv);
    }

    if (!num_pts)
    {
      this->clear();
      return;
    }

    entries.resize(num_pts);

    std::sort(entries.begin(), entries.end());

    PointType acc_pt = entries[0].m_value;
    int acc_num = 1;

    typename ContainerType::iterator target_it = this->begin();

    for (size_t idx = 1; idx < num_pts; ++idx)
    {
      const VoxelEntryMove& entry = entries[idx];
      const PointType& point = entry.m_value;

      if (entries[idx - 1] < entry)
      {
        if (acc_num)
        {
          acc_pt *= (1.f / acc_num);

          if (acc_pt.m_status == PointStatus::kValid)
          {
            target_it->template value<fieldIdx>() =
                acc_pt.template value<fieldIdx>();
            target_it++;
          }
        }

        acc_pt = point;
        acc_num = 1;

        continue;
      }

      acc_pt += point;
      ++acc_num;
    }

    if (acc_num)
    {
      acc_pt *= (1.f / acc_num);

      if (acc_pt.m_status == PointStatus::kValid)
      {
        target_it->template value<fieldIdx>() =
            acc_pt.template value<fieldIdx>();
        target_it++;
      }
    }

    this->resize(std::distance(this->begin(), target_it));
  }

  /**
   * @brief voxelization method of a specific component of a point cloud
   *        (additional utility method to voxelize inplace, instead of having a
   *        target container, with automatic resize)
   * @tparam fieldIdx field index of the specific component in the point
   * @param[in] scale voxelization scale (same scaling factor on each point
   *                  dimension)
   */
  template <int fieldIdx>
  inline void voxelize(const Scalar& scale)
  {
    this->template voxelize<fieldIdx>(PlainVectorType::Constant(scale));
  }

  /**
   * @brief put the point cloud into a string output stream
   * @param[in] os string output stream
   * @return std::ostream& same string output stream
   */
  inline std::ostream& toStream(std::ostream& os) const
  {
    int idx = 0;
    for (typename ContainerType::const_iterator it = this->begin();
         it != this->end(); it++)
    {
      os << "[" << idx++ << "]:";
      it->toStream(os) << " ";
    }
    return os;
  }

};  // struct PointCloud_<ContainerType_> : public ContainerType_

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
