#pragma once

#include <ostream>

// Eigen
#include <Eigen/Core>

#include "inesctec_mrdt_slam_distmap_2d/data_structures/field_pack_derived.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_fields.hpp"
#include "inesctec_mrdt_slam_distmap_2d/pcl/point_ops.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief derived point base definition (other types may derived from this one
 *        and add components to it)
 * @note the methods have the exact same code as PointBase_, only difference is
 *       on the ValueTypeAt and TraitsTypeAt, given that these definitions
 *       require the unrolled index from the FieldPackDerived class...
 * @tparam PointBaseType_ point base type
 * @tparam PointFieldsType_ variadic types of the point fields
 */
template <typename PointBaseType_, typename... PointFieldsType_>
struct PointDerived_
    : data_structures::FieldPackDerived_<PointBaseType_, PointFieldsType_...>
{
 public:

  using ThisType = PointDerived_<PointBaseType_, PointFieldsType_...>;
  using BaseType =
      data_structures::FieldPackDerived_<PointBaseType_, PointFieldsType_...>;

  using PointBaseType = PointBaseType_;

  template <int idx>
  using ValueTypeAt =
      typename BaseType::template FieldPackItemTypeAt<idx>::FieldType;

  template <int idx>
  using TraitsTypeAt =
      typename BaseType::template FieldPackItemTypeAt<idx>::TraitsType;

 public:

  PointStatus m_status = PointStatus::kValid;  //!< status of the point

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointDerived_() = default;
  PointDerived_(const BaseType& base) : BaseType(base) {}
  PointDerived_(const PointBaseType& base, const PointFieldsType_&... fields)
      : BaseType(base, fields...)
  {
  }

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

};  // struct PointDerived_<PointBaseType_, PointFieldsType_...> :
    // data_structures::FieldPackDerived_<PointBaseType_, PointFieldsType_...>

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
