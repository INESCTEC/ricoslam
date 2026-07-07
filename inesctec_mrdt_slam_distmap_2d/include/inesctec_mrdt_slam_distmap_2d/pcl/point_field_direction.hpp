#pragma once

#include <cmath>
#include <ostream>

#include "inesctec_mrdt_slam_distmap_2d/geometry/defs.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_defs.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief traits for direction fields in points classes
 * @note direction fields do not require euclideantoPolar nor polarToEuclidean
 *       transformations
 * @note direction fields are ONLY AFFECTED by the rotation of a SE(.)
 *       transformation
 * @note PointFieldType::RowsAtCompileTime is defined for Eigen matrix / vectors
 *       types to know the number of rows at compile time
 * @note PointFieldType::Scalar is also a typedef definition in Eigen matrix /
 *       vector types to know the base scalar type of the objects
 * @tparam PointFieldType_ point field type (e.g., Eigen::Vector3f) of the
 *                         direction component
 */
template <typename PointFieldType_>
struct PointFieldDirectionTraits_
{
 public:

  using ThisType = PointFieldDirectionTraits_<PointFieldType_>;
  using PointFieldType = PointFieldType_;

 public:

  static constexpr int Dim = PointFieldType::RowsAtCompileTime;

 public:

  inline static void setZero(PointFieldType& dest) { dest.setZero(); }

  inline static void copyTo(PointFieldType& dest, const PointFieldType& src)
  {
    for (int idx = 0; idx < Dim; idx++)
    {
      dest(idx) = src(idx);
    }
  }

  inline static void add(PointFieldType& dest, const PointFieldType& src1,
                         const PointFieldType& src2)
  {
    dest = src1 + src2;
  }

  inline static void addInPlace(PointFieldType& dest, const PointFieldType& src)
  {
    dest += src;
  }

  template <typename Scalar>
  inline static void scale(PointFieldType& dest, const PointFieldType& src,
                           const Scalar& s)
  {
    dest = src * s;
  }

  template <typename Scalar>
  inline static void scaleInPlace(PointFieldType& dest, const Scalar& s)
  {
    dest *= s;
  }

  template <typename Scalar>
  inline static void addAndScaleInPlace(PointFieldType& dest,
                                        const PointFieldType& src,
                                        const Scalar& s)
  {
    dest += src * s;
  }

  inline static PointStatus normalize(PointFieldType& dest)
  {
    dest.normalize();
    return PointStatus::kValid;
  }

  template <typename TransformType_>
  inline static PointStatus transform(PointFieldType& dest,
                                      const PointFieldType& src,
                                      const TransformType_& transform)
  {
    dest = transform.linear() * src;
    return PointStatus::kValid;
  }

  template <typename TransformType_>
  inline static PointStatus transformInPlace(PointFieldType& dest,
                                             const TransformType_& transform)
  {
    dest = transform.linear() * dest;
    return PointStatus::kValid;
  }

  static inline void euclideanToPolar(PointFieldType& dest,
                                      const PointFieldType& src)
  {
    dest = src;
  }

  static inline void polarToEuclidean(PointFieldType& dest,
                                      const PointFieldType& src)
  {
    dest = src;
  }

  static inline void euclideanToPolar(PointFieldType& dest) {}

  static inline void polarToEuclidean(PointFieldType& dest) {}

  static inline std::ostream& toStream(std::ostream& os,
                                       const PointFieldType& src)
  {
    os << "( " << src.transpose() << " )";
    return os;
  }

};  // struct PointFieldDirectionTraits_<PointFieldType_>

/**
 * @brief direction field definition for points classes
 * @tparam Scalar_ scalar type of the point field
 * @tparam Dim_ point field dimension
 */
template <typename Scalar_, int Dim_>
struct PointFieldDirection_
{
 public:

  using ThisType = PointFieldDirection_<Scalar_, Dim_>;
  using FieldType = geometry::VectorN_<Scalar_, Dim_>;
  using TraitsType = PointFieldDirectionTraits_<FieldType>;

  using Scalar = Scalar_;

 public:

  static constexpr int Dim = Dim_;

  FieldType m_value;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointFieldDirection_() { TraitsType::setZero(m_value); }

  PointFieldDirection_(const FieldType& field) : m_value(field) {}

};  // struct PointFieldDirection_<Scalar_, Dim_>

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
