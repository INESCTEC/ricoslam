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
 * @brief traits for coordinates fields in points classes
 * @note PointFieldType::RowsAtCompileTime is defined for Eigen matrix / vectors
 *       types to know the number of rows at compile time
 * @note PointFieldType::Scalar is also a typedef definition in Eigen matrix /
 *       vector types to know the base scalar type of the objects
 *
 * EUCLIDEAN <<< >>> POLAR COORDINATES TRANSFORMATION
 * 2D:
 * - polar: (theta, range)
 *   - theta = atan2(y,x)
 *   - range = norm(x,y)
 * - euclidean: (x, y)
 *   - x = range * cos(theta)
 *   - y = range * sin(theta)
 * 3D:
 * - polar: (azimuth, polar, range)
 *   - azimuth = atan2(y,x)
 *   - polar   = atan2(z,norm(x,y))
 *   - range   = norm(x,y,z)
 * - euclidean: (x, y, z)
 *   - x = range * cos(polar) * cos(azimuth)
 *   - y = range * cos(polar) * sin(azimuth)
 *   - z = range * sin(polar)
 *
 * @tparam PointFieldType_ point field type (e.g., Eigen::Vector3f) of the
 *                         coordinates
 */
template <typename PointFieldType_>
struct PointFieldCoordinatesTraits_
{
 public:

  using ThisType = PointFieldCoordinatesTraits_<PointFieldType_>;
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
    dest = transform * src;
    return PointStatus::kValid;
  }

  template <typename TransformType_>
  inline static PointStatus transformInPlace(PointFieldType& dest,
                                             const TransformType_& transform)
  {
    dest = transform * dest;
    return PointStatus::kValid;
  }

  static inline void euclideanToPolar(PointFieldType& dest,
                                      const PointFieldType& src)
  {
    if (Dim == 2)
    {
      dest(0) = atan2(src(1), src(0));
      dest(1) = src.norm();
      return;
    }
    if (Dim == 3)
    {
      dest(0) = atan2(src(1), src(0));
      dest(1) = atan2(src(2), src.head(2).norm());
      dest(2) = src.norm();
      return;
    }
  }

  static inline void polarToEuclidean(PointFieldType& dest,
                                      const PointFieldType& src)
  {
    if (Dim == 2)
    {
      dest(0) = src(1) * cos(src(0));
      dest(1) = src(1) * sin(src(0));
      return;
    }
    if (Dim == 3)
    {
      float c1 = cos(src(1));

      dest(0) = src(2) * c1 * cos(src(0));
      dest(1) = src(2) * c1 * sin(src(0));
      dest(2) = src(2) * sin(src(1));
      return;
    }
  }

  static inline void euclideanToPolar(PointFieldType& dest)
  {
    PointFieldType aux;
    euclideanToPolar(aux, dest);
    dest = aux;
  }

  static inline void polarToEuclidean(PointFieldType& dest)
  {
    PointFieldType aux;
    polarToEuclidean(aux, dest);
    dest = aux;
  }

  static inline std::ostream& toStream(std::ostream& os,
                                       const PointFieldType& src)
  {
    os << "( " << src.transpose() << " )";
    return os;
  }

};  // struct PointFieldCoordinatesTraits_<PointFieldType_>

/**
 * @brief coordinates field definition for points classes
 * @tparam Scalar_ scalar type of the point field
 * @tparam Dim_ point field dimension
 */
template <typename Scalar_, int Dim_>
struct PointFieldCoordinates_
{
 public:

  using ThisType = PointFieldCoordinates_<Scalar_, Dim_>;
  using FieldType = geometry::VectorN_<Scalar_, Dim_>;
  using TraitsType = PointFieldCoordinatesTraits_<FieldType>;

  using Scalar = Scalar_;

 public:

  static constexpr int Dim = Dim_;

  FieldType m_value;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointFieldCoordinates_() = default;
  PointFieldCoordinates_(const FieldType& field) : m_value(field) {}

};  // struct PointFieldCoordinates_<Scalar_, Dim_>

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
