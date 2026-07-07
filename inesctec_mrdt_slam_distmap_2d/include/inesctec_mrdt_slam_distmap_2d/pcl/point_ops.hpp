#pragma once

#include "inesctec_mrdt_slam_distmap_2d/pcl/point_defs.hpp"

namespace inesctec_mrdt_slam_distmap_2d
{

namespace pcl
{

/**
 * @brief generic point operations (variadic recursive template definition)
 * @tparam PointType_ point type
 * @tparam fieldIdx field index of the point type
 */
template <typename PointType_, int fieldIdx = PointType_::kNumFields - 1>
struct PointOps_
{
 public:

  using ThisType = PointOps_<PointType_, fieldIdx>;
  using PointType = PointType_;

 public:

  inline static void setZero(PointType& dest)
  {
    PointOps_<PointType_, fieldIdx - 1>::setZero(dest);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::setZero(dest.template value<fieldIdx>());
  }

  inline static void copyTo(PointType& dest, const PointType& src)
  {
    PointOps_<PointType_, fieldIdx - 1>::copyTo(dest, src);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::copyTo(dest.template value<fieldIdx>(),
                        src.template value<fieldIdx>());
  }

  inline static void add(PointType& dest, const PointType& src1,
                         const PointType& src2)
  {
    PointOps_<PointType_, fieldIdx - 1>::add(dest, src1, src2);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::add(dest.template value<fieldIdx>(),
                     src1.template value<fieldIdx>(),
                     src2.template value<fieldIdx>());
  }

  inline static void addInPlace(PointType& dest, const PointType& src)
  {
    PointOps_<PointType_, fieldIdx - 1>::addInPlace(dest, src);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::addInPlace(dest.template value<fieldIdx>(),
                            src.template value<fieldIdx>());
  }

  template <typename Scalar>
  inline static void scale(PointType& dest, const PointType& src,
                           const Scalar& s)
  {
    PointOps_<PointType_, fieldIdx - 1>::scale(dest, src, s);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::scale(dest.template value<fieldIdx>(),
                       src.template value<fieldIdx>(), s);
  }

  template <typename Scalar>
  inline static void scaleInPlace(PointType& dest, const Scalar& s)
  {
    PointOps_<PointType_, fieldIdx - 1>::scaleInPlace(dest, s);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::scaleInPlace(dest.template value<fieldIdx>(), s);
  }

  template <typename Scalar>
  inline static void addAndScaleInPlace(PointType& dest, const PointType& src,
                                        const Scalar& s)
  {
    PointOps_<PointType_, fieldIdx - 1>::addAndScaleInPlace(dest, src, s);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::addAndScaleInPlace(dest.template value<fieldIdx>(),
                                    src.template value<fieldIdx>(), s);
  }

  inline static void normalize(PointType& dest)
  {
    PointOps_<PointType_, fieldIdx - 1>::normalize(dest);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::normalize(dest.template value<fieldIdx>());
  }

  template <typename TransformType>
  inline static void transform(PointType& dest, const PointType& src,
                               const TransformType& transform)
  {
    PointOps_<PointType_, fieldIdx - 1>::transform(dest, src, transform);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::transform(dest.template value<fieldIdx>(),
                           src.template value<fieldIdx>(), transform);
  }

  template <typename TransformType>
  inline static void transformInPlace(PointType& dest,
                                      const TransformType& transform)
  {
    PointOps_<PointType_, fieldIdx - 1>::transformInPlace(dest, transform);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::transformInPlace(dest.template value<fieldIdx>(), transform);
  }

  inline static void euclideanToPolar(PointType& dest, const PointType& src)
  {
    PointOps_<PointType_, fieldIdx - 1>::euclideanToPolar(dest, src);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::euclideanToPolar(dest.template value<fieldIdx>(),
                                  src.template value<fieldIdx>());
  }

  inline static void polarToEuclidean(PointType& dest, const PointType& src)
  {
    PointOps_<PointType_, fieldIdx - 1>::polarToEuclidean(dest, src);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::polarToEuclidean(dest.template value<fieldIdx>(),
                                  src.template value<fieldIdx>());
  }

  inline static void toStream(std::ostream& os, const PointType& src)
  {
    PointOps_<PointType_, fieldIdx - 1>::toStream(os, src);

    using FieldTraits = typename PointType::template TraitsTypeAt<fieldIdx>;

    FieldTraits::toStream(os, src.template value<fieldIdx>());

    os << " ";
  }

};  // struct PointOps_<PointType_>

/**
 * @brief generic point operations (base class for recursive definition)
 * @tparam PointType_ point type
 */
template <typename PointType_>
struct PointOps_<PointType_, 0>
{
 public:

  using ThisType = PointOps_<PointType_>;
  using PointType = PointType_;

 public:

  inline static void setZero(PointType& dest)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::setZero(dest.template value<0>());
  }

  inline static void copyTo(PointType& dest, const PointType& src)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::copyTo(dest.template value<0>(), src.template value<0>());
  }

  inline static void add(PointType& dest, const PointType& src1,
                         const PointType& src2)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::add(dest.template value<0>(), src1.template value<0>(),
                     src2.template value<0>());
  }

  inline static void addInPlace(PointType& dest, const PointType& src)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::addInPlace(dest.template value<0>(), src.template value<0>());
  }

  template <typename Scalar>
  inline static void scale(PointType& dest, const PointType& src,
                           const Scalar& s)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::scale(dest.template value<0>(), src.template value<0>(), s);
  }

  template <typename Scalar>
  inline static void scaleInPlace(PointType& dest, const Scalar& s)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::scaleInPlace(dest.template value<0>(), s);
  }

  template <typename Scalar>
  inline static void addAndScaleInPlace(PointType& dest, const PointType& src,
                                        const Scalar& s)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::addAndScaleInPlace(dest.template value<0>(),
                                    src.template value<0>(), s);
  }

  inline static void normalize(PointType& dest)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::normalize(dest.template value<0>());
  }

  template <typename TransformType>
  inline static void transform(PointType& dest, const PointType& src,
                               const TransformType& transform)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::transform(dest.template value<0>(), src.template value<0>(),
                           transform);
  }

  template <typename TransformType>
  inline static void transformInPlace(PointType& dest,
                                      const TransformType& transform)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::transformInPlace(dest.template value<0>(), transform);
  }

  inline static void euclideanToPolar(PointType& dest, const PointType& src)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::euclideanToPolar(dest.template value<0>(),
                                  src.template value<0>());
  }

  inline static void polarToEuclidean(PointType& dest, const PointType& src)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::polarToEuclidean(dest.template value<0>(),
                                  src.template value<0>());
  }

  inline static void toStream(std::ostream& os, const PointType& src)
  {
    using FieldTraits = typename PointType::template TraitsTypeAt<0>;

    FieldTraits::toStream(os, src.template value<0>());

    os << " ";
  }

};  // struct PointOps_<PointType_, 0>

}  // namespace pcl

}  // namespace inesctec_mrdt_slam_distmap_2d
