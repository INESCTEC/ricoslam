#pragma once

#include <ostream>

// Eigen
#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/field_pack.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_fields.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_ops.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief point base definition (other types may derived from this one)
 * @tparam GeometricDim_ geometry dimension of the point
 * @tparam PointFieldsType_ variadic types of the point fields
 */
template <int GeometricDim_, typename... PointFieldsType_>
struct PointBase_ : public data_structures::FieldPack_<PointFieldsType_...>
{
 public:

  using ThisType = PointBase_<GeometricDim_, PointFieldsType_...>;
  using BaseType = data_structures::FieldPack_<PointFieldsType_...>;

  template <int idx>
  using ValueTypeAt =
      typename BaseType::template FieldPackItemTypeAt<idx>::FieldType;

  template <int idx>
  using TraitsTypeAt =
      typename BaseType::template FieldPackItemTypeAt<idx>::TraitsType;

 public:

  static constexpr int kGeometricDim =
      GeometricDim_;  //!< geometric dimension of the point coordinates

 public:

  PointStatus m_status = PointStatus::kValid;  //!< status of the point

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointBase_() = default;
  PointBase_(const BaseType& base) : BaseType(base) {}
  PointBase_(const PointFieldsType_&... fields) : BaseType(fields...) {}

  template <int fieldIdx>
  inline ValueTypeAt<fieldIdx>& value()
  {
    return BaseType::template field<fieldIdx>().m_value;
  }

  template <int fieldIdx>
  inline const ValueTypeAt<fieldIdx>& value() const
  {
    return BaseType::template field<fieldIdx>().m_value;
  }

  inline ThisType& setZero()
  {
    PointOps_<ThisType, BaseType::kNumFields - 1>::setZero(*this);
    return *this;
  }

  template <int fieldIdx>
  inline ThisType& setZero()
  {
    TraitsTypeAt<fieldIdx>::setZero(value<fieldIdx>());
    return *this;
  }

  inline ThisType& normalize()
  {
    PointOps_<ThisType, BaseType::kNumFields - 1>::normalize(*this);
    return *this;
  }

  template <int fieldIdx>
  inline ThisType& normalize()
  {
    TraitsTypeAt<fieldIdx>::normalize(value<fieldIdx>());
    return *this;
  }

  inline ThisType& copyFrom(const ThisType& src)
  {
    PointOps_<ThisType, BaseType::kNumFields - 1>::copyTo(*this, src);
    return *this;
  }

  template <int fieldIdx>
  inline ThisType& copyFrom(const ThisType& src)
  {
    TraitsTypeAt<fieldIdx>::copyTo(value<fieldIdx>(),
                                   src.template value<fieldIdx>());
    return *this;
  }

  inline ThisType& operator+=(const ThisType& other)
  {
    PointOps_<ThisType, BaseType::kNumFields - 1>::addInPlace(*this, other);
    return *this;
  }

  template <int fieldIdx>
  inline ThisType& operator+=(const ThisType& other)
  {
    TraitsTypeAt<fieldIdx>::addInPlace(value<fieldIdx>(),
                                       other.template value<fieldIdx>());
    return *this;
  }

  inline ThisType operator+(const ThisType& other) const
  {
    ThisType returned;
    PointOps_<ThisType, BaseType::kNumFields - 1>::add(returned, *this, other);
    return returned;
  }

  template <int fieldIdx>
  inline ThisType operator+(const ThisType& other) const
  {
    ThisType returned = *this;
    TraitsTypeAt<fieldIdx>::add(returned.template value<fieldIdx>(),
                                value<fieldIdx>(),
                                other.template value<fieldIdx>());
    return returned;
  }

  template <typename Scalar>
  inline ThisType& operator*=(const Scalar& s)
  {
    PointOps_<ThisType, BaseType::kNumFields - 1>::scaleInPlace(*this, s);
    return *this;
  }

  template <int fieldIdx, typename Scalar>
  inline ThisType& operator*=(const Scalar& s)
  {
    TraitsTypeAt<fieldIdx>::scaleInPlace(value<fieldIdx>(), s);
    return *this;
  }

  template <typename Scalar>
  inline ThisType operator*(const Scalar& s) const
  {
    ThisType returned;
    PointOps_<ThisType, BaseType::kNumFields - 1>::scale(returned, *this, s);
    return returned;
  }

  template <int fieldIdx, typename Scalar>
  inline ThisType operator*(const Scalar& s) const
  {
    ThisType returned = *this;
    TraitsTypeAt<fieldIdx>::scale(returned.template value<fieldIdx>(),
                                  value<fieldIdx>(), s);
    return returned;
  }

  inline ThisType& operator-=(const ThisType& other)
  {
    PointOps_<ThisType, BaseType::kNumFields - 1>::addAndScaleInPlace(
        *this, other, -1.0);
    return *this;
  }

  template <int fieldIdx>
  inline ThisType& operator-=(const ThisType& other)
  {
    TraitsTypeAt<fieldIdx>::addAndScaleInPlace(
        value<fieldIdx>(), other.template value<fieldIdx>(), -1.0);
    return *this;
  }

  inline ThisType operator-(const ThisType& other) const
  {
    ThisType returned;
    PointOps_<ThisType, BaseType::kNumFields - 1>::add(returned, *this,
                                                       other * -1.0);
    return returned;
  }

  template <int fieldIdx>
  inline ThisType operator-(const ThisType& other) const
  {
    ThisType returned = *this;
    TraitsTypeAt<fieldIdx>::add(returned.template value<fieldIdx>(),
                                value<fieldIdx>(),
                                other.template value<fieldIdx>() * -1.0);
    return returned;
  }

  template <typename TransformType>
  inline ThisType transform(const TransformType& transform) const
  {
    ThisType returned = *this;
    PointOps_<ThisType, BaseType::kNumFields - 1>::template transform<
        TransformType>(returned, *this, transform);
    return returned;
  }

  template <int fieldIdx, typename TransformType>
  inline ThisType transform(const TransformType& transform) const
  {
    ThisType returned = *this;
    TraitsTypeAt<fieldIdx>::template transform<TransformType>(
        returned.template value<fieldIdx>(), value<fieldIdx>(), transform);
    return returned;
  }

  template <typename TransformType>
  inline ThisType& transformInPlace(const TransformType& transform)
  {
    PointOps_<ThisType, BaseType::kNumFields - 1>::template transformInPlace<
        TransformType>(*this, transform);
    return *this;
  }

  template <int fieldIdx, typename TransformType>
  inline ThisType& transformInPlace(const TransformType& transform)
  {
    TraitsTypeAt<fieldIdx>::template transformInPlace<TransformType>(
        value<fieldIdx>(), transform);
    return *this;
  }

  inline ThisType euclideanToPolar() const
  {
    ThisType returned;
    PointOps_<ThisType, BaseType::kNumFields - 1>::euclideanToPolar(returned,
                                                                    *this);
    return returned;
  }

  template <int fieldIdx>
  inline ThisType euclideanToPolar() const
  {
    ThisType returned = *this;
    TraitsTypeAt<fieldIdx>::euclideanToPolar(
        returned.template value<fieldIdx>(), value<fieldIdx>());
    return returned;
  }

  inline ThisType polarToEuclidean() const
  {
    ThisType returned;
    PointOps_<ThisType, BaseType::kNumFields - 1>::polarToEuclidean(returned,
                                                                    *this);
    return returned;
  }

  template <int fieldIdx>
  inline ThisType polarToEuclidean() const
  {
    ThisType returned = *this;
    TraitsTypeAt<fieldIdx>::polarToEuclidean(
        returned.template value<fieldIdx>(), value<fieldIdx>());
    return returned;
  }

  inline std::ostream& toStream(std::ostream& os) const
  {
    os << "{ ";
    PointOps_<ThisType, BaseType::kNumFields - 1>::toStream(os, *this);
    os << "}";
    return os;
  }

};  // struct PointBase_<GeometricDim_, PointFieldsType_...> : public
    // data_structures::FieldPack_<PointFieldsType_...>

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
